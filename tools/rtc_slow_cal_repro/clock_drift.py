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
import itertools
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
        f"{'mode':12} {'start (UTC host)':>16} {'min':>6} {'lines':>5} {'ppm':>9} {'±SE':>6} {'rms ms':>6} {'cal kHz':>9} {'die C min..max':>14} {'mean':>5} {'warm-up':>7} {'sleep%':>6} {'ovr':>4}"
    )
    per_mode: dict[str, list[tuple[float, float]]] = {}
    for seg in segments:
        pts = window(seg, args.skip, args.length)
        if len(pts) < 3:
            continue
        ppm, se, rms = fit(pts)
        minutes = (pts[-1].host_s - pts[0].host_s) / 60
        mean_cal = sum(p.cal for p in pts) / len(pts)
        die_cold = mean_finite(p.die_cold for p in pts)
        dies = [p.die_cold for p in pts if math.isfinite(p.die_cold)]
        die_range = f"{min(dies):5.1f}..{max(dies):5.1f}" if dies else "-"
        warm_up = mean_finite(p.die_warm - p.die_cold for p in pts)
        start = pts[0].wall[5:16].replace("T", " ")
        share = 100 * sleep_share(pts)
        overruns = pts[-1].overruns - pts[0].overruns
        print(
            f"{seg.mode:12} {start:>16} {minutes:6.1f} {len(pts):5} {ppm:+9.1f} {se:6.1f} {rms:6.2f} {cal_khz(mean_cal):9.3f} {die_range:>14} {die_cold:5.1f} {warm_up:+7.2f} {share:6.1f} {overruns:4}"
        )
        per_mode.setdefault(seg.mode, []).append((ppm, minutes))

    print()
    for mode, rows in per_mode.items():
        total = sum(m for _, m in rows)
        mean = sum(p * m for p, m in rows) / total
        print(f"{mode:12} {len(rows):3} segments, {total:7.1f} min, duration-weighted {mean:+8.1f} ppm")


@dataclass
class Sleep:
    """One light sleep, from two consecutive report lines (the one after it and the one before)."""

    slept_s: float  # its length (s= of the line after it)
    prev_s: float  # the light sleep before it, which decides where its RTC_SLOW period came from
    error_s: float  # device minus host time over the interval holding it: only the sleep is RC-timed
    host_s: float  # host time of that interval
    cal: int  # the period it was timed with (c= of the line after it)
    post_wake: bool | None  # period from the reading after the previous wake; None = too close to call
    rc_local_ppm: float = math.nan  # RC frequency at its calibration against the neighbouring sleeps' (+ = faster = colder)


# The firmware compares the pure RTC sleep time with the threshold, s= also holds the sleep entry and
# exit, so a previous sleep this close to the threshold can't be assigned to either side.
SOURCE_GUARD_US = 2000
# Sleeps on each side whose mean period is the local reference for one period (about +-40 s).
RC_NEIGHBOURS = 30


def mode_threshold_us(mode: str) -> float | None:
    """CONFIG_..._MIN_SLP_MS of a mode in us, None for stock (every sleep timed by the entry reading)."""
    m = re.fullmatch(r"fix-\w+?(?:-(\d+))?", mode)
    if m is None:
        return None
    return int(m[1]) * 1000 if m[1] else 50_000  # fix-idf / fix-emul before the option: 50 ms


def collect_sleeps(pts: list[Point], threshold_us: float | None) -> list[Sleep]:
    """Sleeps whose own and previous line are consecutive, each with exactly one light sleep."""
    sleeps = []
    for before, prev, cur in zip(pts, pts[1:], pts[2:]):
        if cur.n != prev.n + 1 or prev.n != before.n + 1 or cur.sleeps != 1 or prev.sleeps != 1:
            continue
        if threshold_us is None:
            post_wake = False
        elif abs(prev.slept_us - threshold_us) < SOURCE_GUARD_US:
            post_wake = None
        else:
            post_wake = prev.slept_us > threshold_us
        host_s = cur.host_s - prev.host_s
        error_s = (cur.dev_s - prev.dev_s) - host_s
        sleeps.append(Sleep(cur.slept_us / 1e6, prev.slept_us / 1e6, error_s, host_s, cur.cal, post_wake))
    # The die (and the RC with it) drifts by many degrees within a run, far more than the effect of a
    # warm or cold reading, so each period is compared with the mean of its neighbours in time.
    for i, x in enumerate(sleeps):
        lo = max(0, i - RC_NEIGHBOURS)
        near = [y.cal for y in sleeps[lo : i + RC_NEIGHBOURS + 1]]
        local = (sum(near) - x.cal) / (len(near) - 1)
        x.rc_local_ppm = (local / x.cal - 1) * 1e6
    return sleeps


def log_bins(per_decade: int) -> list[tuple[float, float, str]]:
    """Sleep-length bins from 1 ms to 10 s (the last one open-ended), as (low s, high s, label)."""

    def label(sec: float) -> str:
        return f"{sec * 1e3:.3g}ms" if sec < 1 else f"{sec:.3g}s"

    edges = [10 ** (-3 + i / per_decade) for i in range(4 * per_decade + 1)]
    bins = [(lo, hi, f"{label(lo)}-{label(hi)}") for lo, hi in itertools.pairwise(edges)]
    bins[-1] = (bins[-1][0], math.inf, bins[-1][2])
    bins[0] = (0.0, bins[0][1], bins[0][2])
    return bins


def sleep_rate(sleeps: list[Sleep]) -> tuple[float, float]:
    """Clock error over these sleeps, ppm of their sleep time, and its standard error."""
    total = sum(x.slept_s for x in sleeps)
    if len(sleeps) < 3 or total <= 0:
        return math.nan, math.nan
    rate = sum(x.error_s for x in sleeps) / total
    resid = [x.error_s - rate * x.slept_s for x in sleeps]
    var = sum(r * r for r in resid) / (len(sleeps) - 1)
    return rate * 1e6, math.sqrt(len(sleeps) * var) / total * 1e6


def mean_se(values: list[float]) -> tuple[float, float]:
    if len(values) < 3:
        return math.nan, math.nan
    mean = sum(values) / len(values)
    sd = math.sqrt(sum((v - mean) ** 2 for v in values) / (len(values) - 1))
    return mean, sd / math.sqrt(len(values))


def per_sleep_us(sleeps: list[Sleep]) -> tuple[float, float]:
    """Mean clock error per sleep, us, and its standard error: the honest unit for short sleeps,
    where a fixed per-sleep offset divided by a tiny sleep length gives meaningless ppm."""
    mean, se = mean_se([x.error_s for x in sleeps])
    return mean * 1e6, se * 1e6


def rc_offset(sleeps: list[Sleep]) -> tuple[float, float]:
    """RC frequency when these sleeps' periods were measured, ppm against their neighbours in time
    (positive = faster = colder die), and its standard error."""
    return mean_se([x.rc_local_ppm for x in sleeps])


def pm(value: float, se: float) -> str:
    return "-" if not math.isfinite(value) else f"{value:+.0f}±{se:.0f}"


def share(part: list[Sleep], cond) -> str:
    known = [x for x in part if x.post_wake is not None]
    return f"{100 * sum(cond(x) for x in known) / len(known):.0f}" if known else "-"


def cmd_bins(args) -> None:
    with open(args.file, encoding="utf-8") as f:
        segments = split_segments(f)
    bins = log_bins(args.per_decade)
    runs: list[tuple[str, list[Sleep]]] = []

    for seg in segments:
        pts = window(seg, args.skip, args.length)
        threshold_us = mode_threshold_us(seg.mode)
        sleeps = collect_sleeps(pts, threshold_us)
        if len(sleeps) < 10:
            continue
        name = f"{seg.mode}#{len(runs) + 1}"
        runs.append((name, sleeps))
        dies = [p.die_cold for p in pts if math.isfinite(p.die_cold)]
        die = f"{min(dies):.1f}..{max(dies):.1f} C (mean {sum(dies) / len(dies):.1f})" if dies else "-"
        total_host = sum(x.host_s for x in sleeps)
        run_ppm = sum(x.error_s for x in sleeps) / total_host * 1e6
        sleep_ppm, sleep_se = sleep_rate(sleeps)
        threshold = "none (stock: entry reading)" if threshold_us is None else f"{threshold_us / 1000:g} ms"
        print(f"== {name}  {pts[0].wall[5:16].replace('T', ' ')} UTC, {len(sleeps)} sleeps, threshold {threshold}, die {die}")
        print(f"   run: {run_ppm:+.1f} ppm of wall time (fit-comparable), {pm(sleep_ppm, sleep_se)} ppm of sleep time")
        print("   by the sleep's own length: its clock error, and what it adds to the run's ppm")
        print(f"   {'sleep':14} {'n':>5} {'time%':>6} {'post-wake%':>10} {'us per sleep':>13} {'ppm of sleep':>14} {'adds ppm':>9}")
        total_sleep = sum(x.slept_s for x in sleeps)
        for lo, hi, label in bins:
            part = [x for x in sleeps if lo <= x.slept_s < hi]
            if not part:
                continue
            rate, se = sleep_rate(part)
            adds = sum(x.error_s for x in part) / total_host * 1e6
            time_share = 100 * sum(x.slept_s for x in part) / total_sleep
            us, us_se = per_sleep_us(part)
            print(f"   {label:14} {len(part):5} {time_share:6.1f} {share(part, lambda x: x.post_wake):>10} {pm(us, us_se):>13} {pm(rate, se):>14} {adds:+9.1f}")
        print(f"   by the previous sleep's length (it decides the period source): RC at calibration vs its neighbours, and the clock error of sleeps >= {args.min_own:g} s")
        print(f"   {'prev sleep':14} {'n':>5} {'post-wake%':>10} {'RC ppm':>10} {'n>=':>5} {'ppm of sleep':>14}")
        for lo, hi, label in bins:
            part = [x for x in sleeps if lo <= x.prev_s < hi]
            if not part:
                continue
            rc, rc_se = rc_offset(part)
            long = [x for x in part if x.slept_s >= args.min_own]
            rate, se = sleep_rate(long)
            print(f"   {label:14} {len(part):5} {share(part, lambda x: x.post_wake):>10} {pm(rc, rc_se):>10} {len(long):5} {pm(rate, se):>14}")
        print()

    if not runs:
        print("no usable sleeps: bins needs report lines with s= and k= (captures from 2026-10-20 on)")
        return
    if len(runs) < 2:
        return
    width = max(14, *(len(name) + 1 for name, _ in runs))
    stock = [x for name, sleeps in runs if name.startswith("stock#") for x in sleeps]

    def pivot(title: str, key, select, value, vs_stock: bool = False) -> None:
        cols = [(name, sleeps) for name, sleeps in runs if not (vs_stock and name.startswith("stock#"))]
        if not cols or (vs_stock and not stock):
            return
        print(f"== {title}")
        print(f"   {'':14}" + "".join(f"{name:>{width}}" for name, _ in cols))
        for lo, hi, label in bins:
            ref = value([x for x in stock if lo <= key(x) < hi and select(x)]) if vs_stock else (0.0, 0.0)
            cells = []
            for _, sleeps in cols:
                v, se = value([x for x in sleeps if lo <= key(x) < hi and select(x)])
                cells.append(pm(v - ref[0], math.hypot(se, ref[1])))
            print(f"   {label:14}" + "".join(f"{c:>{width}}" for c in cells))
        print()

    own = lambda x: x.slept_s
    prev = lambda x: x.prev_s
    long_own = lambda x: x.slept_s >= args.min_own
    pivot("us of clock error per sleep, by the sleep's own length", own, lambda x: True, per_sleep_us)
    pivot(
        "difference to stock (all stock runs pooled), us per sleep, by the sleep's own length: the effect of the period source",
        own,
        lambda x: True,
        per_sleep_us,
        vs_stock=True,
    )
    pivot("ppm of sleep time, by the sleep's own length", own, lambda x: True, sleep_rate)
    pivot(f"ppm of sleep time of sleeps >= {args.min_own:g} s, by the previous sleep's length", prev, long_own, sleep_rate)
    pivot("RC frequency at calibration vs its neighbours in time, ppm (+ = colder die), by the previous sleep's length", prev, lambda x: True, rc_offset)


def window(seg: Segment, skip: float, length: float | None) -> list[Point]:
    """The points of a segment from skip to length seconds after its start (host time)."""
    t0 = seg.points[0].host_s
    return [p for p in seg.points if skip <= p.host_s - t0 and (length is None or p.host_s - t0 <= length)]


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
    p_fit.add_argument(
        "--length",
        type=float,
        default=None,
        help="use only the first LENGTH seconds of each segment (counted from its start, like --skip), so runs of different length cover the same sleeps",
    )
    p_fit.set_defaults(func=cmd_fit)

    p_bins = sub.add_parser("bins", help="clock error per sleep-length bin, by the sleep's own and the previous sleep's length")
    p_bins.add_argument("file")
    p_bins.add_argument("--skip", type=float, default=0, help="seconds to drop at the start of each segment")
    p_bins.add_argument("--length", type=float, default=None, help="use only the first LENGTH seconds of each segment")
    p_bins.add_argument("--per-decade", type=int, default=2, help="bins per decade of sleep length (default 2)")
    p_bins.add_argument("--min-own", type=float, default=1.0, help="shortest own sleep (s) for the by-previous-sleep clock error (default 1)")
    p_bins.set_defaults(func=cmd_bins)

    args = parser.parse_args()
    try:
        args.func(args)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
