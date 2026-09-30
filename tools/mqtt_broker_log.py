#!/usr/bin/env python3
"""Capture the MQTT broker's log and extract the lines about one device.

Mosquitto keeps no log history reachable over MQTT: `$SYS/broker/log/#` only delivers lines
logged while a subscriber is connected. So getting "the broker log since X" takes two steps:

    # 1. keep a recorder running (background/tmux) — appends everything it hears to a raw file
    tools/mqtt_broker_log.py record --device esp32-OT-MQTT-sensor-98a316fffe8e2674

    # 2. any time later: device-filtered, human-readable log from an optional start date-time
    tools/mqtt_broker_log.py extract --device esp32-OT-MQTT-sensor-98a316fffe8e2674 \\
        2026-09-30T05:45:00Z -o logs/2026-09-30__some_test__mqtt.log

The broker must have `log_dest topic` and log_type error/warning/notice/information/subscribe/
unsubscribe enabled (HA Mosquitto add-on customize include). DEBUG lines (CONNACK, PUBLISH,
PUBACK) never go to the log topic. Broker address/port/credentials come from secrets.yaml,
exactly as ota_push.py connects (including its ota_push_* overrides).

Raw file lines (one per event, receive time in UTC):
    2026-09-30T14:23:56.656Z LOG N 2026-09-30 19:23:56: New client connected from ...
    2026-09-30T14:23:57.290Z - q1 state b'{"t":17.4, ...}'      (R = retained copy)
    2026-09-30T14:23:57.701Z # watcher (re)connected rc=Success
`extract` also reads the older time-only raw format (HH:MM:SS.mmm prefix) written by the
first ad-hoc watcher; dates are recovered from the broker's own timestamps on LOG lines.

Extracted lines keep: broker log lines naming the device's client ids (ESP32_xxxxxx, learned
from its subscribes, and <device>-ota), its topics, or its IPv6 addresses (learned from its
CONNECTs — so bare TCP accepts show up too); for IPv4 only the exact addr:port of a device
connection; plus broker-wide persistence and all W/E lines; every message on the device's
topics; and the recorder's own (re)connects, since broker lines are lost while it is offline.
"""

import argparse
import ast
import re
import sys
import time
from datetime import date, datetime, timedelta, timezone
from pathlib import Path

import ota_push

DEFAULT_RAW = ota_push.PROJECT_ROOT / "logs" / "mqtt_broker_raw.log"
DAY = timedelta(days=1)
HALF_DAY = timedelta(hours=12)

ISO_TS = re.compile(r"^(\d{4}-\d\d-\d\dT\d\d:\d\d:\d\d\.\d{3})Z (.*)$")
LEGACY_TS = re.compile(r"^(\d\d:\d\d:\d\d\.\d{3}) (.*)$")
LOG_LINE = re.compile(r"^LOG (\S+) (\d{4}-\d\d-\d\d \d\d:\d\d:\d\d): (.*)$")
MSG_LINE = re.compile(r"^([R-]) (q\d) (\S+) (b['\"].*['\"])$")
CONNECTED = re.compile(r"New client connected from (\S+):(\d+) as (\S+) ")
ESP32_ID = re.compile(r"^ESP32_[0-9a-fA-F]{6}$")
SUBSCRIBE = re.compile(r"^(\S+) \d+ (\S+)$")
BROKER_WIDE = re.compile(r"Saving in-memory database|mosquitto version|Opening ipv|Config loaded"
                         r"|exiting|Terminating|restart", re.I)


def utc_now() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z"


def default_client_id(device: str) -> str:
    """Firmware's id (mqtt_sender.cpp): "ESP32_{:02x}{:02X}{:02X}" of the last 3 MAC bytes."""
    tail = device[-6:]
    return f"ESP32_{tail[0:2].lower()}{tail[2:6].upper()}"


# --- record -------------------------------------------------------------------------------------

def record(args: argparse.Namespace) -> None:
    dev = args.device
    topics = [(f"{dev}/state", 1), (f"{dev}/ota/#", 1), (f"{dev}/cfg/#", 1), ("$SYS/broker/log/#", 0)]
    args.raw.parent.mkdir(parents=True, exist_ok=True)
    out = args.raw.open("a", buffering=1, encoding="utf-8")

    def emit(text: str) -> None:
        out.write(f"{utc_now()} {text}\n")

    def on_message(_c, _u, msg):
        if "/ota/image/" in msg.topic:
            return  # 8 KiB binary chunks; the broker's SUBSCRIBE lines already show the progress
        if msg.topic.startswith("$SYS/broker/log/"):
            emit(f"LOG {msg.topic[16:]} {msg.payload.decode(errors='replace')}")
            return
        flag = "R" if msg.retain else "-"
        emit(f"{flag} q{msg.qos} {msg.topic[len(dev) + 1:]} {msg.payload[:240]!r}")

    def on_connect(c, _u, _f, rc, _p=None):
        emit(f"# watcher (re)connected rc={rc}")
        c.subscribe(topics)  # also after a broker restart

    def on_disconnect(_c, _u, flags, rc, _p=None):
        emit(f"# watcher disconnected rc={rc} flags={flags}")

    client = ota_push.connect(ota_push.read_secrets(args.secrets))
    client.on_message = on_message
    client.on_disconnect = on_disconnect
    client.on_connect = on_connect
    client.subscribe(topics)  # connect() already ran loop_start(), so the first CONNACK may be gone
    print(f"recording to {args.raw} (Ctrl-C to stop)", file=sys.stderr)
    try:
        while True:
            time.sleep(3600)
    except KeyboardInterrupt:
        pass


# --- extract ------------------------------------------------------------------------------------

def parse_raw(lines) -> list[tuple[datetime, str]]:
    """(UTC receive time, rest of line) for every raw line, either format."""
    events: list[tuple[datetime, str]] = []
    day: date | None = None
    prev_t: datetime | None = None
    pending: list[tuple[str, str]] = []  # legacy lines seen before the first date anchor

    def legacy_dt(t: str) -> datetime:
        return datetime.combine(day, datetime.strptime(t, "%H:%M:%S.%f").time(), timezone.utc)

    for line in lines:
        line = line.rstrip("\n")
        m = ISO_TS.match(line)
        if m:
            events.append((datetime.fromisoformat(m.group(1)).replace(tzinfo=timezone.utc), m.group(2)))
            continue
        m = LEGACY_TS.match(line)
        if not m:
            continue
        t, rest = m.group(1), m.group(2)
        lm = LOG_LINE.match(rest)
        if lm:
            # Re-anchor the date on every broker line: its local timestamp minus the whole-hour
            # offset between broker local time and our UTC receive time.
            broker = datetime.strptime(lm.group(2), "%Y-%m-%d %H:%M:%S").replace(tzinfo=timezone.utc)
            recv = datetime.strptime(t, "%H:%M:%S.%f")
            skew = (broker - broker.replace(hour=recv.hour, minute=recv.minute, second=recv.second)).total_seconds()
            offset_h = round((skew % 86400) / 3600) % 24
            if offset_h > 12:
                offset_h -= 24
            utc_est = broker - timedelta(hours=offset_h)
            cand = datetime.combine(utc_est.date(), recv.time(), timezone.utc)
            if cand - utc_est > HALF_DAY:
                cand -= DAY
            elif utc_est - cand > HALF_DAY:
                cand += DAY
            if day is None:
                # back-date the lines before this anchor, walking over midnight rollovers
                d = cand.date()
                back: list[tuple[datetime, str]] = []
                nxt = recv.time()
                for pt, prest in reversed(pending):
                    ptime = datetime.strptime(pt, "%H:%M:%S.%f").time()
                    if ptime > nxt and (datetime.combine(d, ptime) - datetime.combine(d, nxt)) > HALF_DAY:
                        d -= DAY
                    back.append((datetime.combine(d, ptime, timezone.utc), prest))
                    nxt = ptime
                events.extend(reversed(back))
                pending.clear()
            day = cand.date()
            prev_t = cand
            events.append((cand, rest))
            continue
        if day is None:
            pending.append((t, rest))
            continue
        dt = legacy_dt(t)
        if prev_t is not None and prev_t - dt > HALF_DAY:
            day += DAY
            dt += DAY
        prev_t = dt
        events.append((dt, rest))
    if pending:
        sys.exit("legacy raw file has no broker LOG line to anchor its dates on")
    return events


def fmt_ts(dt: datetime) -> str:
    return dt.strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z"


def learn_device(events, device: str, client_ids: set[str]) -> tuple[set[str], set[str]]:
    """Client ids that subscribe to the device's topics, and the addresses the device connects from."""
    for _dt, rest in events:
        lm = LOG_LINE.match(rest)
        if lm and lm.group(1) == "M/subscribe":
            sm = SUBSCRIBE.match(lm.group(3))
            # HA and the tools subscribe to these topics too; only the firmware uses ESP32_xxxxxx
            if sm and sm.group(2).startswith(device + "/") and ESP32_ID.match(sm.group(1)):
                client_ids.add(sm.group(1))
    v6_addrs: set[str] = set()
    v4_endpoints: set[str] = set()
    for _dt, rest in events:
        lm = LOG_LINE.match(rest)
        cm = CONNECTED.search(lm.group(3)) if lm else None
        if cm and (cm.group(3) in client_ids or cm.group(3).startswith(device)):
            if ":" in cm.group(1):
                v6_addrs.add(cm.group(1))
            else:
                v4_endpoints.add(f"{cm.group(1)}:{cm.group(2)}")
    return v6_addrs, v4_endpoints


def format_events(events, device: str, client_ids: set[str], since: datetime | None,
                  until: datetime | None) -> tuple[list[str], set[str], set[str]]:
    v6_addrs, v4_endpoints = learn_device(events, device, client_ids)
    needles = client_ids | {device} | {a + ":" for a in v6_addrs} | v4_endpoints
    out: list[str] = []
    for dt, rest in events:
        if (since and dt < since) or (until and dt >= until):
            continue
        ts = fmt_ts(dt)
        if rest.startswith("# watcher"):
            out.append(f"{ts} WATCHER  {rest[2:]}")
            continue
        lm = LOG_LINE.match(rest)
        if lm:
            lvl, msg = lm.group(1), lm.group(3)
            if lvl in ("W", "E") or BROKER_WIDE.search(msg) or any(n in msg for n in needles):
                out.append(f"{ts} BROKER   {lvl:<13} {msg}")
            continue
        mm = MSG_LINE.match(rest)
        if mm:
            kind = "RETAINED" if mm.group(1) == "R" else "MSG     "
            payload = ast.literal_eval(mm.group(4)).decode(errors="replace")
            out.append(f"{ts} {kind} {mm.group(2)} {mm.group(3)} {payload}".rstrip())
    return out, v6_addrs, v4_endpoints


def parse_when(text: str) -> datetime:
    dt = datetime.fromisoformat(text.strip().replace(" ", "T"))
    return dt.replace(tzinfo=timezone.utc) if dt.tzinfo is None else dt.astimezone(timezone.utc)


def extract(args: argparse.Namespace) -> None:
    events = parse_raw(args.raw.open(encoding="utf-8", errors="replace"))
    client_ids = set(args.client_id) if args.client_id else {default_client_id(args.device)}
    lines, v6_addrs, v4_endpoints = format_events(events, args.device, client_ids, args.since, args.until)
    first = lines[0][:24] if lines else "-"
    last = lines[-1][:24] if lines else "-"
    header = [
        f"# MQTT broker log for {' / '.join(sorted(client_ids))} / {args.device}, {first} → {last}",
        f"# Source: tools/mqtt_broker_log.py extract from {args.raw.name}"
        f" (since {fmt_ts(args.since) if args.since else 'start of recording'}"
        f"{', until ' + fmt_ts(args.until) if args.until else ''}).",
        "#   Recorder subscribed to $SYS/broker/log/# plus <device_id>/state, /ota/#, /cfg/#; timestamps are",
        "#   its receive time in UTC. DEBUG lines (CONNACK, PUBLISH, PUBACK) never reach the log topic;",
        "#   ota/image/* payloads are not recorded (the -ota client's SUBSCRIBE lines show chunk progress).",
        "# Line kinds:",
        "#   BROKER   <level> — broker log line naming the device's client ids, topics or addresses, plus",
        "#            broker-wide persistence/restart lines and all W/E lines. Other clients' connects dropped.",
        "#   MSG      — a message on the device's topics, as delivered to the recorder (live forward).",
        "#   RETAINED — the broker's retained copies, replayed each time the recorder (re)connected.",
        "#   WATCHER  — the recorder's own connection; broker lines between a disconnect and reconnect are lost.",
        f"# Device IPv6 addresses: {', '.join(sorted(v6_addrs)) or 'none'}",
    ]
    if v4_endpoints:
        header.append(f"# Device IPv4 endpoints: {', '.join(sorted(v4_endpoints))}")
    text = "\n".join(header + lines) + "\n"
    if args.output:
        args.output.write_text(text, encoding="utf-8")
        print(f"wrote {len(lines)} lines to {args.output}", file=sys.stderr)
    else:
        sys.stdout.write(text)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name in ("record", "extract"):
        p = sub.add_parser(name)
        p.add_argument("--device", required=True,
                       help="device id, e.g. esp32-OT-MQTT-sensor-98a316fffe8e2674 (topic prefix)")
        p.add_argument("--raw", type=Path, default=DEFAULT_RAW,
                       help=f"raw capture file (default {DEFAULT_RAW.relative_to(ota_push.PROJECT_ROOT)})")
        if name == "record":
            p.add_argument("--secrets", type=Path, default=ota_push.PROJECT_ROOT / "secrets.yaml",
                           help="secrets.yaml to read the broker address/credentials from")
        else:
            p.add_argument("since", nargs="?", type=parse_when,
                           help="start date-time, ISO 8601, UTC unless it carries an offset "
                                "(e.g. 2026-09-30T05:45:00Z); default: start of the recording")
            p.add_argument("--until", type=parse_when, help="end date-time (exclusive), same format")
            p.add_argument("--client-id", action="append",
                           help="device MQTT client id (repeatable); default derived from --device, "
                                "ids subscribing to the device's topics are added automatically")
            p.add_argument("-o", "--output", type=Path, help="output file (default stdout)")
    args = ap.parse_args()
    if args.cmd == "record":
        record(args)
    else:
        extract(args)


if __name__ == "__main__":
    main()
