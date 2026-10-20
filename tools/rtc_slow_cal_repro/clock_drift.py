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
Device boot banner:     REPRO idf=<version> mode=<stock|fix-idf-<MIN_SLP_MS>> wake_ms=.. busy_ms=.. ...
Device report line:     C n=.. t=.. c=.. [d=..] r=.. s=.. k=.. [o=..]
                        (fields are described in main/rtc_slow_cal_repro.c; the mode comes from the banner)
Older captures:         CLK n=<wakes> t_us=<esp_timer> mode=<mode> cal=<Q19 period>
                            [die_cold=<C after wake> die_warm=<C after the busy time>]
"""

import argparse
import datetime
import math
import re
import sys
import time
from dataclasses import dataclass, field

CLK_RE = re.compile(r"CLK n=(\d+) t_us=(-?\d+) mode=(\S+) cal=(\d+)(?: die_cold=(\S+) die_warm=(\S+))?")
SHORT_RE = re.compile(r"^C ((?:\w+=-?\d+ ?)+)$")
BANNER_MODE_RE = re.compile(r"\bmode=(\S+)")
RTC_CLK_CAL_FRACT = 19


@dataclass
class Clk:
    """One report line. mode is None in the short format: it comes from the boot banner."""

    n: int
    dev_s: float
    mode: str | None
    cal: int
    die_cold: float = math.nan
    die_warm: float = math.nan
    slept_us: int | None = None
    sleeps: int | None = None
    overruns: int = 0


@dataclass
class Point:
    wall: str
    host_s: float
    dev_s: float
    n: int
    cal: int
    die_cold: float = math.nan
    die_warm: float = math.nan
    slept_us: int | None = None
    sleeps: int | None = None
    overruns: int = 0

    @classmethod
    def from_clk(cls, wall: str, host_s: float, clk: Clk) -> "Point":
        return cls(wall, host_s, clk.dev_s, clk.n, clk.cal, clk.die_cold, clk.die_warm, clk.slept_us, clk.sleeps, clk.overruns)


@dataclass
class Segment:
    mode: str
    points: list[Point] = field(default_factory=list)


def parse_clk(text: str) -> Clk | None:
    m = SHORT_RE.match(text)
    if m is not None:
        f = dict(tok.split("=", 1) for tok in m[1].split())
        if not {"n", "t", "c"} <= f.keys():
            return None
        die = int(f["d"]) / 10 if "d" in f else math.nan  # left out if the sensor failed
        slept_us = int(f["s"]) if "s" in f else None
        sleeps = int(f["k"]) if "k" in f else None
        return Clk(int(f["n"]), int(f["t"]) / 1e6, None, int(f["c"]), die, math.nan, slept_us, sleeps, int(f.get("o", 0)))
    m = CLK_RE.search(text)
    if m is None:
        return None
    # die_* are absent in captures from before they were added, and "nan" if the sensor failed.
    die_cold = float(m[5]) if m[5] else math.nan
    die_warm = float(m[6]) if m[6] else math.nan
    return Clk(int(m[1]), int(m[2]) / 1e6, m[3], int(m[4]), die_cold, die_warm)


def banner_mode(text: str) -> str | None:
    """The mode of a boot banner line ("" if it names none), or None if it is not a banner.

    Also matches older banners that went through ESP_LOGW and carry its prefix."""
    if "REPRO " not in text:
        return None
    m = BANNER_MODE_RE.search(text)
    return m[1] if m else ""


def cal_khz(cal: float) -> float:
    return 1e3 * (1 << RTC_CLK_CAL_FRACT) / cal if cal else 0.0


def mean_finite(values) -> float:
    vals = [v for v in values if math.isfinite(v)]
    return sum(vals) / len(vals) if vals else math.nan


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
        boot_mode = banner_mode(parts[2])
        if boot_mode is not None:
            segments.append(Segment(mode=boot_mode))  # boot: never join across it
            continue
        clk = parse_clk(parts[2])
        if clk is None:
            continue
        cur = segments[-1] if segments else None
        # A short line has no mode: it runs the firmware of the last banner. A reset without a
        # banner (lost on the wire) is still the same firmware.
        mode = clk.mode or (cur.mode if cur is not None and cur.mode else "?")
        if cur is None or cur.mode not in ("", mode) or (cur.points and (clk.n <= cur.points[-1].n or clk.dev_s <= cur.points[-1].dev_s)):
            cur = Segment(mode=mode)
            segments.append(cur)
        cur.mode = mode
        cur.points.append(Point.from_clk(parts[0], host_s, clk))
    return [s for s in segments if s.points]


def sleep_share(points: list[Point]) -> float:
    """Share of the device time spent in light sleep, from the lines after the first (each line
    counts the sleeps since the line before it). NaN for captures without the s= field."""
    slept = [p.slept_us for p in points[1:]]
    if not slept or None in slept:
        return math.nan
    span_s = points[-1].dev_s - points[0].dev_s
    return sum(slept) / 1e6 / span_s if span_s > 0 else math.nan


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
                boot_mode = banner_mode(text)
                if boot_mode is not None:
                    seg = Segment(mode=boot_mode)
                continue
            mode = clk.mode or seg.mode or "?"
            if seg.mode not in ("", mode) or (seg.points and clk.n <= seg.points[-1].n):
                seg = Segment(mode=mode)
            seg.mode = mode
            seg.points.append(Point.from_clk(wall, mono_ns / 1e9, clk))
            span_min = (seg.points[-1].host_s - seg.points[0].host_s) / 60
            if len(seg.points) >= 3:
                ppm, se, _ = fit(seg.points)
                rate = f"{ppm:+8.1f} ±{se:5.1f} ppm over {span_min:5.1f} min"
            else:
                rate = "(fit after 3 lines)"
            overrun = f"  OVERRUN x{clk.overruns}" if clk.overruns else ""
            print(f"{wall} {mode:12} n={clk.n:7} {cal_khz(clk.cal):8.3f} kHz  {clk.die_cold:6.2f} C  {rate}{overrun}", flush=True)


def cmd_fit(args) -> None:
    with open(args.file, encoding="utf-8") as f:
        segments = split_segments(f)

    print(
        f"{'mode':12} {'start (UTC host)':>16} {'min':>6} {'lines':>5} {'ppm':>9} {'±SE':>6} {'rms ms':>6} {'cal kHz':>9} {'die C':>6} {'warm-up':>7} {'sleep%':>6} {'ovr':>4}"
    )
    per_mode: dict[str, list[tuple[float, float]]] = {}
    for seg in segments:
        pts = [p for p in seg.points if p.host_s - seg.points[0].host_s >= args.skip]
        if len(pts) < 3:
            continue
        ppm, se, rms = fit(pts)
        minutes = (pts[-1].host_s - pts[0].host_s) / 60
        mean_cal = sum(p.cal for p in pts) / len(pts)
        die_cold = mean_finite(p.die_cold for p in pts)
        warm_up = mean_finite(p.die_warm - p.die_cold for p in pts)
        start = pts[0].wall[5:16].replace("T", " ")
        share = 100 * sleep_share(pts)
        overruns = pts[-1].overruns - pts[0].overruns
        print(
            f"{seg.mode:12} {start:>16} {minutes:6.1f} {len(pts):5} {ppm:+9.1f} {se:6.1f} {rms:6.2f} {cal_khz(mean_cal):9.3f} {die_cold:6.2f} {warm_up:+7.2f} {share:6.1f} {overruns:4}"
        )
        per_mode.setdefault(seg.mode, []).append((ppm, minutes))

    print()
    for mode, rows in per_mode.items():
        total = sum(m for _, m in rows)
        mean = sum(p * m for p, m in rows) / total
        print(f"{mode:12} {len(rows):3} segments, {total:7.1f} min, duration-weighted {mean:+8.1f} ppm")


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
