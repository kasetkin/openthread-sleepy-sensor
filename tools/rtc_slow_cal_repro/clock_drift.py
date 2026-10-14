#!/usr/bin/env python3
# /// script
# requires-python = ">=3.10"
# dependencies = ["pyserial>=3.5"]
# ///
"""Measure how fast esp_timer runs in light sleep, against the host clock.

    # 1. capture: every line gets the host's arrival time; CLK lines also print a running fit
    clock_drift.py log /dev/ttyUSB0 -o rtc_cal.log

    # 2. any time later (or on a copy while 1 runs): rate per segment and per mode
    clock_drift.py fit rtc_cal.log

Capture on UART0 (an external USB-UART adapter), not USB-Serial-JTAG: the latter drops out in
light sleep and can deliver buffered lines late.

The reference is the host's CLOCK_MONOTONIC. NTP keeps its frequency right (slews it) but never
steps it, so the host should be NTP-synced and must not suspend during a capture.

Captured line format:   <UTC ISO time> <host monotonic ns> <device line>
Device CLK line:        CLK n=<wakes> t_us=<esp_timer> mode=<stock|fix-emul|fix-idf> cal=<Q19 period>
"""

import argparse
import datetime
import math
import re
import sys
import time
from dataclasses import dataclass, field

CLK_RE = re.compile(r"CLK n=(\d+) t_us=(-?\d+) mode=(\S+) cal=(\d+)")
RTC_CLK_CAL_FRACT = 19


@dataclass
class Point:
    wall: str
    host_s: float
    dev_s: float
    n: int
    cal: int


@dataclass
class Segment:
    mode: str
    points: list[Point] = field(default_factory=list)


def parse_clk(text: str) -> tuple[int, float, str, int] | None:
    m = CLK_RE.search(text)
    if m is None:
        return None
    return int(m[1]), int(m[2]) / 1e6, m[3], int(m[4])


def cal_khz(cal: float) -> float:
    return 1e3 * (1 << RTC_CLK_CAL_FRACT) / cal if cal else 0.0


def fit(points: list[Point]) -> tuple[float, float, float]:
    """Least-squares device time against host time: (ppm, standard error ppm, residual rms ms).

    Positive ppm = the device clock runs fast."""
    x0 = points[0].host_s
    y0 = points[0].dev_s
    xs = [p.host_s - x0 for p in points]
    ys = [p.dev_s - y0 for p in points]
    k = len(points)
    mx = sum(xs) / k
    my = sum(ys) / k
    sxx = sum((x - mx) ** 2 for x in xs)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    slope = sxy / sxx
    residuals = [y - my - slope * (x - mx) for x, y in zip(xs, ys)]
    sse = sum(r * r for r in residuals)
    se = math.sqrt(sse / (k - 2) / sxx) if k > 2 else math.nan
    return (slope - 1) * 1e6, se * 1e6, math.sqrt(sse / k) * 1e3


def split_segments(lines) -> list[Segment]:
    """One segment per run of CLK lines with the same mode and no reset in between."""
    segments: list[Segment] = []
    for line in lines:
        parts = line.rstrip("\n").split(" ", 2)
        if len(parts) < 3:
            continue
        host_s = int(parts[1]) / 1e9
        if parts[2].startswith("REPRO "):
            segments.append(Segment(mode=""))  # boot: never join across it
            continue
        clk = parse_clk(parts[2])
        if clk is None:
            continue
        n, dev_s, mode, cal = clk
        cur = segments[-1] if segments else None
        if cur is None or cur.mode not in ("", mode) or (cur.points and (n <= cur.points[-1].n or dev_s <= cur.points[-1].dev_s)):
            cur = Segment(mode=mode)
            segments.append(cur)
        cur.mode = mode
        cur.points.append(Point(parts[0], host_s, dev_s, n, cal))
    return [s for s in segments if s.points]


def cmd_log(args) -> None:
    import serial  # only the capture needs pyserial

    port = serial.Serial()
    port.port = args.port
    port.baudrate = args.baud
    # Block until a whole line is in: readline() with a timeout returns a partial line when the
    # timeout runs out mid-line, which splits a CLK line and stamps both halves wrong.
    port.timeout = None
    # Don't toggle the adapter's control lines on open: on some boards they reset the chip.
    port.dtr = False
    port.rts = False
    port.open()

    seg = Segment(mode="")
    with open(args.output, "a", encoding="utf-8") as out:
        print(f"capturing {args.port} @ {args.baud} -> {args.output}", file=sys.stderr)
        while True:
            raw = port.readline()
            mono_ns = time.monotonic_ns()
            wall = datetime.datetime.now(datetime.UTC).isoformat(timespec="milliseconds")
            text = raw.decode("utf-8", errors="replace").rstrip("\r\n")
            out.write(f"{wall} {mono_ns} {text}\n")
            out.flush()

            clk = parse_clk(text)
            if clk is None:
                print(f"{wall} {text}", flush=True)
                if text.startswith("REPRO "):
                    seg = Segment(mode="")
                continue
            n, dev_s, mode, cal = clk
            if seg.mode not in ("", mode) or (seg.points and n <= seg.points[-1].n):
                seg = Segment(mode=mode)
            seg.mode = mode
            seg.points.append(Point(wall, mono_ns / 1e9, dev_s, n, cal))
            span_min = (seg.points[-1].host_s - seg.points[0].host_s) / 60
            if len(seg.points) >= 3:
                ppm, se, _ = fit(seg.points)
                rate = f"{ppm:+8.1f} ±{se:5.1f} ppm over {span_min:5.1f} min"
            else:
                rate = "(fit after 3 lines)"
            print(f"{wall} {mode:8} n={n:7} {cal_khz(cal):8.3f} kHz  {rate}", flush=True)


def cmd_fit(args) -> None:
    with open(args.file, encoding="utf-8") as f:
        segments = split_segments(f)

    print(f"{'mode':8} {'start (UTC host)':>16} {'min':>6} {'lines':>5} {'ppm':>9} {'±SE':>6} {'rms ms':>6} {'cal kHz':>9}")
    per_mode: dict[str, list[tuple[float, float]]] = {}
    for seg in segments:
        pts = [p for p in seg.points if p.host_s - seg.points[0].host_s >= args.skip]
        if len(pts) < 3:
            continue
        ppm, se, rms = fit(pts)
        minutes = (pts[-1].host_s - pts[0].host_s) / 60
        mean_cal = sum(p.cal for p in pts) / len(pts)
        start = pts[0].wall[5:16].replace("T", " ")
        print(f"{seg.mode:8} {start:>16} {minutes:6.1f} {len(pts):5} {ppm:+9.1f} {se:6.1f} {rms:6.2f} {cal_khz(mean_cal):9.3f}")
        per_mode.setdefault(seg.mode, []).append((ppm, minutes))

    print()
    for mode, rows in per_mode.items():
        total = sum(m for _, m in rows)
        mean = sum(p * m for p, m in rows) / total
        print(f"{mode:8} {len(rows):3} segments, {total:7.1f} min, duration-weighted {mean:+8.1f} ppm")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_log = sub.add_parser("log", help="capture UART lines with host timestamps")
    p_log.add_argument("port")
    p_log.add_argument("-o", "--output", required=True)
    p_log.add_argument("-b", "--baud", type=int, default=115200)
    p_log.set_defaults(func=cmd_log)

    p_fit = sub.add_parser("fit", help="fit the clock rate per segment and per mode")
    p_fit.add_argument("file")
    p_fit.add_argument("--skip", type=float, default=0, help="seconds to drop at the start of each segment")
    p_fit.set_defaults(func=cmd_fit)

    args = parser.parse_args()
    try:
        args.func(args)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
