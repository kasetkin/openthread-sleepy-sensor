#!/usr/bin/env python3
"""Per-close MQTT close-tail report from an OTBR (ot-agent) log.

Checks the fix in docs/plan_mqtt_close_tail.html against the border router's own view of the
child's traffic. Each MQTT close is found by the device's 62-byte TCP uplink (40 B IPv6 + 20 B
TCP + the 2-byte MQTT DISCONNECT; keepalive is off, so no PINGREQ shares that size). For every
close it prints when the broker's FIN and final ACK reached the child, whether the device
retransmitted a segment, whether a frame was delivered twice, and whether the device sent
anything after the last delivery (an RST, which is what a clean close must not have).

Usage: tools/otbr_close_tail.py OTBR_LOG [--child 0x2401]
The child's RLOC16 is the "to:0x…" of any "Sent IPv6 TCP msg" to the device; it changes only
when the device re-attaches to a different parent.
"""

import argparse
import re
from datetime import datetime, timedelta

WINDOW_S = 75.0   # longer than the 70 s idle poll, so a frame held for it is still counted
POLL_WINDOW_S = 1.0

TCP_RE = re.compile(r"^(\d\d:\d\d:\d\d\.\d+) .*?(Prepping indir tx|Sent|Received) IPv6 TCP msg, "
                    r"len:(\d+), chksum:(\w+).*?(?:to|from):(0x[0-9a-f]+)")
POLL_RE = re.compile(r"^(\d\d:\d\d:\d\d\.\d+) .*Rx data poll, src:(0x[0-9a-f]+), qed_msgs:(\d+)")


_day = timedelta(0)
_prev = None


def ts(s):
    """Parses the agent's HH:MM:SS.mmm uptime stamp, starting a new "day" whenever the clock
    goes backwards -- an agent restart (HA restart or host reboot) resets it to 00:00, possibly
    only minutes after the previous session's last line, and a capture can hold several
    sessions. The 10 s slack tolerates the agent's own out-of-order lines."""
    global _day, _prev
    t = datetime.strptime(s, "%H:%M:%S.%f")
    if _prev is not None and t + _day < _prev - timedelta(seconds=10):
        _day += timedelta(days=1)
    _prev = t + _day
    return _prev


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("log")
    ap.add_argument("--child", default="0x2401", help="device RLOC16 (default: %(default)s)")
    args = ap.parse_args()

    uplinks = []      # (time, len, chksum) device -> router
    deliveries = []   # (time, len, chksum) router -> device
    polls = []        # (time, queued messages)
    with open(args.log, errors="replace") as f:
        for line in f:
            m = TCP_RE.match(line)
            if m and m[5] == args.child:
                rec = (ts(m[1]), int(m[3]), m[4])
                if m[2] == "Received":
                    uplinks.append(rec)
                elif m[2] == "Sent":
                    deliveries.append(rec)
                continue
            m = POLL_RE.match(line)
            if m and m[2] == args.child:
                polls.append((ts(m[1]), int(m[3])))

    closes = [t for t, n, _ in uplinks if n == 62]
    print(f"{len(closes)} closes for child {args.child}\n")
    print("close             FIN in   last in  last out  polls/empty  retx  dup  after-last")
    for i, t0 in enumerate(closes):
        end = closes[i + 1] if i + 1 < len(closes) else None

        def inside(t, t0=t0, end=end):
            return 0 < (t - t0).total_seconds() < WINDOW_S and (end is None or t < end)

        down = [r for r in deliveries if inside(r[0])]
        up = [r for r in uplinks if inside(r[0])]
        rel = lambda t, t0=t0: f"+{(t - t0).total_seconds():.3f}"
        first_in = rel(down[0][0]) if down else "none"
        last_in = rel(down[-1][0]) if down else "none"
        last_out = rel(up[-1][0]) if up else "none"
        burst = [q for t, q in polls if 0 < (t - t0).total_seconds() < POLL_WINDOW_S]
        retx = len(up) - len({c for _, _, c in up})
        dup = len(down) - len({c for _, _, c in down})
        after = sum(1 for r in up if down and r[0] > down[-1][0])
        print(f"{t0:%d %H:%M:%S.%f}"[:-3].ljust(18), first_in.ljust(8), last_in.ljust(8), last_out.ljust(9),
              f"{len(burst)}/{sum(1 for q in burst if q == 0)}".ljust(12), str(retx).ljust(5),
              str(dup).ljust(4), after)


if __name__ == "__main__":
    main()
