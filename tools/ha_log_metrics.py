#!/usr/bin/env python3
"""Derive every power_model.py input from a Home Assistant history export.

Reconciling a measurement window against the model used to mean hand-extracting a dozen numbers
from a CSV, and this project's records show what that costs: figures land in notes without their
derivation, and windows turn out afterwards to have contained a reboot, an OTA, or a settings
change nobody checked for. This script does the extraction, prints every number with its
provenance attached, refuses to quote a window that fails a guard rail, and emits the exact
power_model.py command line that reproduces its own run.

    tools/ha_log_metrics.py --csv metrics_history_2026-08-29__to__2026-09-07.csv --run

Design notes worth knowing before reading the code, all derived in docs/plan_ha_log_metrics.md:

* **Integrate over the window; cadence is an OUTPUT.** Publishes fire on two different triggers --
  a safeguard timer and a dT/dRH threshold -- so the gap distribution is bimodal and its mean is a
  weather-dependent mixture weight, not a device setting. A window drifting from 60 % to 40 %
  safeguard-mode moves "cadence" ~10 % with no change in device behaviour. Totals do not care.
* **Anchor on the per-cycle MEASUREMENT entities only.** Constants like boot_count carry HA's
  export-boundary snapshot rows, which invent cycles: 3395 instead of 3390 on the reference window.
* **Forward-fill onto a cycle anchor, always.** It is exact reconstruction, not interpolation --
  see ha_history's docstring -- and skipping it biases counters by >50 %.
* **The LP period has two estimators and one known systematic.** Both are reported.
"""
import argparse
import statistics
import sys
from bisect import bisect_right
from collections import namedtuple

import ha_history as hh
import power_model as pm
from battery_power_from_history import decay_fit

# The series that carry a fresh value most cycles. This tuple IS the cycle clock -- see the module
# docstring. Guard-rail constants are read but never anchored on.
PER_CYCLE_ENTITIES = ("hp_awake_time", "radio_tx_time", "radio_rx_time", "heater_run_count",
                      "voltage", "battery", "temperature", "cca_failures")
GUARD_ENTITIES = ("boot_count", "tx_power_active", "sensor_samples")

# main/main.cpp: minutes_to_lp_cycles(1440, 20) with device_config.yaml's heater_period_minutes.
# The LP core increments cycles_since_heater every poll and zeroes it in the same poll that runs
# the heater, so the interval between two increments is exactly this many polls.
HEATER_TICKS_PER_STEP = 4320
MIN_HEATER_INCREMENTS = 3
# An off-schedule high-RH heater run (kHighRhThreshold 90 %RH held for heater_high_rh_trigger
# cycles) also zeroes that counter, making one interval short and the endpoint-span estimator
# silently wrong. Neither reference window trips it -- max RH 80.3 % and 72.0 % -- so this guard
# has no real-data coverage and exists on the strength of the firmware alone.
HEATER_INTERVAL_TOLERANCE = 0.01

LP_ESTIMATOR_DISAGREEMENT = 0.005  # 0.5 %
SAFEGUARD_BIN_S = 5.0
SAFEGUARD_POOL_S = 10.0
SAFEGUARD_PERCENTILE = 0.90
# Keep only heater increments whose ending gap clears the safeguard mode by this much. A bare
# "> mode" cut is too loose: window B's 357.0 s gap clears its 325.5 s mode but is a half-split
# stall, not a whole one.
HEATER_STALL_MARGIN_S = 50.0
OUTLIER_THRESHOLD_MS = 5000.0
MIN_WINDOW_DAYS = 5.0

Window = namedtuple("Window", "path start end seconds n_cycles cadence_s anchors table")
PhaseTotals = namedtuple("PhaseTotals", "tx_s rx_s cpu_s awake_s cycles negatives outlier_count "
                                        "outlier_s outlier_ua")
Safeguard = namedtuple("Safeguard", "mode_s share pool k")
LpPeriod = namedtuple("LpPeriod", "raw_s corrected_s stall_s increments uncertainty_frac "
                                  "cross_check_s disagreement k interval_raw_s "
                                  "interval_corrected_s")
Measured = namedtuple("Measured", "ua half_width_ua confidence mean_v pct_delta hours bootstrap")
Guard = namedtuple("Guard", "name ok fatal detail")
Findings = namedtuple("Findings", "window phases safeguard lp measured sensor_samples "
                                  "tx_power_dbm guards coverage")


# --- extraction ------------------------------------------------------------------------

def build_window(export, epsilon_s=hh.CYCLE_EPSILON_S, max_hold_s=hh.MAX_HOLD_S):
    """Anchors plus a cycle-anchored table of the metrics this script reads."""
    anchors = hh.cycle_anchors(export, PER_CYCLE_ENTITIES, epsilon_s)
    table = hh.cycle_table(export, anchors, PER_CYCLE_ENTITIES, epsilon_s, max_hold_s)
    seconds = (anchors[-1] - anchors[0]).total_seconds()
    return Window(path=export.path, start=anchors[0], end=anchors[-1], seconds=seconds,
                  n_cycles=len(anchors), cadence_s=seconds / (len(anchors) - 1),
                  anchors=anchors, table=table)


def phase_totals(window, outlier_ms=OUTLIER_THRESHOLD_MS, cpu_current_ma=None):
    """Per-cycle CPU residual (awake - tx - rx), clamped, then summed over the window.

    Per cycle and then summed, not totals differenced: a misaligned cycle produces a negative
    residual, and the count of those is the single most useful signal that the anchoring is wrong.
    Zero is the expected value and both reference windows give zero.
    """
    if cpu_current_ma is None:
        cpu_current_ma = (pm.CPU_CURRENT_MA_LOW + pm.CPU_CURRENT_MA_HIGH) / 2
    tx_ms = window.table.columns["radio_tx_time"].values
    rx_ms = window.table.columns["radio_rx_time"].values
    awake_ms = window.table.columns["hp_awake_time"].values
    tx_total = 0.0
    rx_total = 0.0
    cpu_total = 0.0
    awake_total = 0.0
    cycles = 0
    negatives = 0
    outlier_count = 0
    outlier_ms_total = 0.0
    for tx, rx, awake in zip(tx_ms, rx_ms, awake_ms):
        if tx is None or rx is None or awake is None:
            continue
        cpu = awake - tx - rx
        if cpu < 0.0:
            negatives += 1
            cpu = 0.0
        tx_total += tx
        rx_total += rx
        cpu_total += cpu
        awake_total += awake
        cycles += 1
        if awake > outlier_ms:
            outlier_count += 1
            outlier_ms_total += awake
    # Outliers are integrated separately so they are visible rather than buried. The current
    # assigned is the mid CPU figure, which is an ASSUMPTION -- a 41 s cycle is probably retry TX
    # at 305 mA or RX at 74 mA, not CPU-bound work.
    outlier_ua = outlier_ms_total / 1000.0 * cpu_current_ma * 1000.0 / window.seconds
    return PhaseTotals(tx_s=tx_total / 1000.0, rx_s=rx_total / 1000.0, cpu_s=cpu_total / 1000.0,
                       awake_s=awake_total / 1000.0, cycles=cycles, negatives=negatives,
                       outlier_count=outlier_count, outlier_s=outlier_ms_total / 1000.0,
                       outlier_ua=outlier_ua)


def safeguard_mode(window, bin_s=SAFEGUARD_BIN_S, pool_s=SAFEGUARD_POOL_S):
    """The safeguard-timer publish gap: densest bin, then everything pooled within +/-pool_s.

    Roughly 1900 samples on the reference window, against 8 heater intervals -- which is why this
    is worth having even though it is the less direct of the two LP-period estimators.
    """
    gaps = hh.anchor_gaps(window.anchors)
    histogram = hh.gap_histogram(window.anchors, bin_s)
    densest = max(histogram.items(), key=lambda item: item[1])[0]
    in_bin = [gap for gap in gaps if densest * bin_s <= gap < (densest + 1) * bin_s]
    mode_s = statistics.median(in_bin)
    pool = sorted(gap for gap in gaps if abs(gap - mode_s) <= pool_s)
    return Safeguard(mode_s=mode_s, share=len(pool) / len(gaps), pool=pool, k=None)


def heater_increments(export):
    return hh.monotone_increments(hh.series_for(export, "heater_run_count"))


def heater_interval_spread(increments):
    """(per-interval LP periods, max fractional deviation from their median).

    A deviation beyond HEATER_INTERVAL_TOLERANCE means one interval was not 4320 polls long --
    an off-schedule high-RH heater run -- and the endpoint-span estimator below is invalid.
    """
    periods = [(increments[i + 1][0] - increments[i][0]).total_seconds() / HEATER_TICKS_PER_STEP
               for i in range(len(increments) - 1)]
    if not periods:
        return [], 0.0
    median = statistics.median(periods)
    return periods, max(abs(period / median - 1.0) for period in periods)


def heater_stall_seconds(export, window, mode_s, margin_s=HEATER_STALL_MARGIN_S):
    """Median extra publish delay caused by the LP core's post-heater busy-wait.

    The LP core blocks for kRhSettleMs (90 s) plus a cooldown poll loop after each heater run, so
    the publish gap ENDING at the increment is stretched by exactly that. Use the gap ending at
    the increment, not the one starting at it: the count rides the same MQTT message as the sensor
    values, so the increment's timestamp IS an anchor, and taking the following gap yields the
    ordinary 303 s instead of the stalled 401 s.

    Only some increments are usable -- the heater perturbs T and RH, so the post-heater reading
    often trips a threshold and publishes early, splitting the stall across two gaps.
    """
    stalls = []
    for stamp, _ in heater_increments(export):
        # The increment rides the publish at its own anchor, so the gap that carried the stall is
        # the one ENDING at that anchor. bisect_right(...) - 1 is the anchor at or before the
        # increment, which is that anchor.
        index = bisect_right(window.anchors, stamp) - 1
        if index <= 0:
            continue
        gap = (window.anchors[index] - window.anchors[index - 1]).total_seconds()
        if gap > mode_s + margin_s:
            stalls.append(gap)
    if not stalls:
        return None, 0
    return statistics.median(stalls) - mode_s, len(stalls)


def resolve_lp_period(export, window, sensor_samples, safeguard, stall_s):
    """Both LP-period estimators, plus the heater estimator's known systematic.

    The primary estimator divides the first-to-last heater span by 4320 ticks per step. It is the
    more direct of the two, but it is NOT unbiased: exactly one of every 4320 polls is the heater
    poll, so the span is `4320 * P + stall` and the quotient overshoots by stall/4320 -- about
    +22 ms, +0.11 %, three times the quantization uncertainty. Both the raw and the
    stall-corrected figures are returned, and the printed uncertainty spans both, because the
    correction has a consequence worth seeing rather than hiding: it puts the ss=4 period 22 ms
    BELOW the model's own sleep+work floor, which implies an RTC_SLOW clock running ~0.11 % fast.

    The cross-check divides the safeguard-mode pool's p90 by the integer number of polls in it.
    p90 specifically, and empirically: the pool's median reads 0.36-0.40 % low in both reference
    windows because threshold-triggered publishes are always EARLY and contaminate the low side,
    while p95 and above read high because outage-stretched gaps contaminate the top.
    """
    increments = heater_increments(export)
    if len(increments) < 2:
        return None
    span_s = (increments[-1][0] - increments[0][0]).total_seconds()
    steps = HEATER_TICKS_PER_STEP * (len(increments) - 1)
    raw_s = span_s / steps
    corrected_s = raw_s - (stall_s / HEATER_TICKS_PER_STEP if stall_s else 0.0)
    # Each endpoint is quantized to the next successful publish; only the endpoint span averages
    # that down, which is why individual intervals scatter +/-0.4 % while this does not.
    quantization = window.cadence_s / steps / raw_s

    k = round(safeguard.mode_s / raw_s)
    cross_check_s = hh.percentile(safeguard.pool, SAFEGUARD_PERCENTILE) / k if k else None
    disagreement = (cross_check_s / raw_s - 1.0) if cross_check_s else None
    return LpPeriod(raw_s=raw_s, corrected_s=corrected_s, stall_s=stall_s,
                    increments=len(increments), uncertainty_frac=quantization,
                    cross_check_s=cross_check_s, disagreement=disagreement, k=k,
                    interval_raw_s=pm.lp_poll_interval_s(sensor_samples, raw_s),
                    interval_corrected_s=pm.lp_poll_interval_s(sensor_samples, corrected_s))


def measured_current(export):
    """Battery-decay current in uA with a bootstrap interval, delegating the fit to the sibling.

    Charge in, charge out: a battery-percentage decay is fraction-of-capacity per hour, so the
    current follows from mAh alone. No pack voltage enters -- reinstating one is the 2026-09-09
    unit bug, worth 6-7 %.
    """
    readings = hh.series_for(export, "voltage").samples
    decay = decay_fit(readings)
    # The SAME dwell weights the point estimate used -- an interval computed on a different
    # estimator than the number beside it is not an interval on that number.
    bootstrap = hh.slope_block_bootstrap(decay.hours, decay.percents, decay.weights)
    per_pp = pm.PACK_CAPACITY_MAH * 1000.0 / 100.0
    return Measured(ua=decay.mean_ma * 1000.0, half_width_ua=bootstrap.half_width * per_pp,
                    confidence=bootstrap.confidence, mean_v=decay.mean_v,
                    pct_delta=decay.pct_delta, hours=decay.total_hours, bootstrap=bootstrap)


def coverage_report(window):
    """{name: (sampled, held, missing, fraction)} -- a de-duplication rate, not a defect rate."""
    out = {}
    for name, column in window.table.columns.items():
        out[name] = (column.sampled, column.held, column.missing,
                     column.sampled / window.n_cycles)
    return out


# --- guard rails -----------------------------------------------------------------------

def check_guards(export, window, phases, lp, safeguard, findings_extras):
    """Every rail that has already invalidated a window in this project, in section 4's order."""
    guards = []

    def add(name, ok, fatal, detail):
        guards.append(Guard(name=name, ok=ok, fatal=fatal, detail=detail))

    for name in GUARD_ENTITIES:
        series = hh.series_for(export, name, required=False)
        if series is None:
            add(name, False, False, "entity absent from the export")
            continue
        value, complaint = hh.constant_value(series)
        add(name, complaint is None, True,
            complaint or f"constant at {value:g} across {len(series.samples)} rows")

    long_blackouts = []
    edge_blackouts = []
    for name in PER_CYCLE_ENTITIES + GUARD_ENTITIES:
        series = hh.series_for(export, name, required=False)
        if series is None:
            continue
        long_blackouts.extend(hh.blackouts_over(series, window.table.max_hold_s))
        edge_blackouts.extend(blackout for blackout in series.blackouts if blackout.edge)
    add("blackouts", not long_blackouts, True,
        f"{len(long_blackouts)} interior run(s) longer than the "
        f"{window.table.max_hold_s:.0f} s dwell cap"
        if long_blackouts else "no interior run longer than the dwell cap")
    add("edge blackouts", not edge_blackouts, False,
        f"{len(edge_blackouts)} run(s) anchored to an export boundary, longest "
        f"{max(blackout.seconds for blackout in edge_blackouts):.0f} s -- HA not yet knowing a "
        f"slow-changing value, not a device event"
        if edge_blackouts else "none")

    add("negative CPU residuals", phases.negatives == 0, True,
        f"{phases.negatives} cycle(s) where tx+rx exceeded hp_awake_time")

    days = window.seconds / 86400.0
    add("window length", days >= MIN_WINDOW_DAYS, False,
        f"{days:.2f} days (minimum {MIN_WINDOW_DAYS:g} for a quotable decay fit)")

    increments = heater_increments(export)
    add("heater increments", len(increments) >= MIN_HEATER_INCREMENTS, False,
        f"{len(increments)} increment(s); below {MIN_HEATER_INCREMENTS} the heater LP period "
        f"rests on a single interval, and below 2 there is none")
    periods, spread = heater_interval_spread(increments)
    add("heater interval spread", spread <= HEATER_INTERVAL_TOLERANCE, True,
        f"max deviation {spread * 100:.2f} % from the median interval"
        + ("" if spread <= HEATER_INTERVAL_TOLERANCE
           else " -- an off-schedule high-RH heater run would do this, and it invalidates the "
                "heater LP-period estimator"))

    if lp is not None:
        k_error = safeguard_k_error(lp, safeguard)
        add("safeguard k is integral", abs(k_error) <= 0.1, True,
            f"mode / period = {safeguard.mode_s / lp.raw_s:.3f} polls, "
            f"{k_error:+.3f} from the nearest integer ({lp.k})")
        if lp.disagreement is not None:
            add("LP estimators agree", abs(lp.disagreement) <= LP_ESTIMATOR_DISAGREEMENT, True,
                f"heater {lp.raw_s:.4f} s vs safeguard {lp.cross_check_s:.4f} s "
                f"({lp.disagreement * 100:+.3f} %)")
    else:
        # Fatal because the LP period is not a detail: through sensor_samples_extra_ua() it is a
        # ~147 uA term at ss=4. The safeguard-pool fallback docs/plan_ha_log_metrics.md section 4
        # describes is not implemented, so past --force the model runs on its NOMINAL period.
        add("LP period measured", False, True,
            "fewer than two heater_run_count increments (the heater runs once a day); "
            "--force runs the model on power_model's nominal period instead")

    for name, ok, detail in findings_extras:
        add(name, ok, False, detail)
    return guards


def safeguard_k_error(lp, safeguard):
    """How far mode/period sits from the nearest whole number of LP polls.

    The safeguard gap is (max_skip + 1) LP polls by construction, so this ratio must be an
    integer. When it is not, one of the two estimators is measuring something other than what it
    claims -- and through sensor_samples_extra_ua() the LP period is a ~147 uA term at ss=4,
    a quarter of the whole budget.
    """
    exact = safeguard.mode_s / lp.raw_s
    return exact - round(exact)


# --- orchestration ---------------------------------------------------------------------

def derive(export, epsilon_s=hh.CYCLE_EPSILON_S, max_hold_s=hh.MAX_HOLD_S,
           outlier_ms=OUTLIER_THRESHOLD_MS, sensor_samples=None, tx_power_dbm=None):
    """The whole extraction, printing nothing."""
    window = build_window(export, epsilon_s, max_hold_s)
    phases = phase_totals(window, outlier_ms)
    safeguard = safeguard_mode(window)

    if sensor_samples is None:
        value, _ = hh.constant_value(hh.series_for(export, "sensor_samples", required=False)
                                     or _empty_series("sensor_samples"))
        sensor_samples = int(value) if value is not None else 1
    if tx_power_dbm is None:
        value, _ = hh.constant_value(hh.series_for(export, "tx_power_active", required=False)
                                     or _empty_series("tx_power_active"))
        tx_power_dbm = value if value is not None else pm.TX_POWER_DBM

    stall_s, stall_n = heater_stall_seconds(export, window, safeguard.mode_s)
    lp = resolve_lp_period(export, window, sensor_samples, safeguard, stall_s)
    if lp is not None:
        exact_k = safeguard.mode_s / lp.raw_s
        safeguard = safeguard._replace(k=exact_k)

    measured = measured_current(export)

    extras = [("heater stall usable increments", stall_s is not None,
               f"{stall_n} increment(s) cleared mode + {HEATER_STALL_MARGIN_S:g} s"
               if stall_s is not None else "no increment's ending gap cleared the cut")]
    guards = check_guards(export, window, phases, lp, safeguard, extras)
    return Findings(window=window, phases=phases, safeguard=safeguard, lp=lp, measured=measured,
                    sensor_samples=sensor_samples,
                    tx_power_dbm=tx_power_dbm, guards=guards, coverage=coverage_report(window))


def _empty_series(name):
    return hh.Series(name=name, entity_id="", samples=[], blackouts=[], boundary_stamps=0,
                     non_numeric=0)


def model_phase_times(findings):
    """Per-cycle (tx, rx, cpu) in seconds -- totals over the SAME cycle count, per section 1.6."""
    cycles = findings.phases.cycles
    return (findings.phases.tx_s / cycles, findings.phases.rx_s / cycles,
            findings.phases.cpu_s / cycles)


def model_lp_period_s(findings, basis="raw"):
    """The LP period handed to the model, or None when the window could not measure one.

    None is only reachable past the fatal "LP period measured" guard rail (--force, --quiet), and
    power_model then falls back to its nominal period for the window's sensor_samples.
    """
    if findings.lp is None:
        return None
    return findings.lp.raw_s if basis == "raw" else findings.lp.corrected_s


def model_command_line(findings, basis="raw", capacity_mah=None):
    """The power_model.py invocation that reproduces this window. The reproducible artifact."""
    period_s = model_lp_period_s(findings, basis)
    tx_s, rx_s, cpu_s = model_phase_times(findings)
    parts = [
        "tools/power_model.py",
        f"--cadence {findings.window.cadence_s:.2f}",
        f"--sensor-samples {findings.sensor_samples}",
    ]
    # Left out rather than guessed, so the command line says as much as the window did.
    if period_s is not None:
        parts.append(f"--lp-poll-period {period_s:.4f}")
    parts += [
        f"--tx-power {findings.tx_power_dbm:g}",
        f"--phase-times {tx_s:.5f},{rx_s:.5f},{cpu_s:.5f}",
        f"--measured {findings.measured.ua:.0f}",
    ]
    if capacity_mah:
        parts.append(f"--capacity {capacity_mah:.0f}")
    return " \\\n    ".join(parts)


# --- reporting -------------------------------------------------------------------------

def print_guards(findings, forced=False, stream=sys.stdout):
    print("Guard rails", file=stream)
    for guard in findings.guards:
        if guard.ok:
            mark = "ok"
        elif forced:
            mark = "FORCED"
        elif guard.fatal:
            mark = "fail"
        else:
            mark = "warn"
        print(f"  [{mark:6s}] {guard.name}: {guard.detail}", file=stream)


def print_findings(findings, stream=sys.stdout):
    """Every derived line carries its provenance inline.

    Precision is typography here, not decoration. The phase totals are sums of logged values and
    are good to ~0.1 %, so they print to two decimals. The measured current carries a bootstrap
    interval of order +/-10 %, so it prints to three significant figures and never four -- quoting
    it more finely invites a future reader to chase differences that are noise.
    """
    window = findings.window
    phases = findings.phases
    print(f"Window: {window.start.isoformat()} -> {window.end.isoformat()}", file=stream)
    print(f"  {window.n_cycles} cycles over {window.seconds:.0f} s "
          f"({window.seconds / 86400.0:.2f} days)", file=stream)
    print(f"  cadence {window.cadence_s:.2f} s = T/(n-1)   "
          f"anchored on {len(PER_CYCLE_ENTITIES)} per-cycle measurement entities", file=stream)
    print(f"  safeguard mode {findings.safeguard.mode_s:.1f} s, "
          f"{findings.safeguard.share * 100:.0f} % of cycles; the rest are dT/dRH-triggered",
          file=stream)
    print("  NB cadence is an OUTPUT, and it is a weather-dependent mixture of those two "
          "triggers -- compare windows on the mode share too, not the mean alone.", file=stream)

    print(file=stream)
    print("Phase totals (forward-filled onto the cycle anchor, summed over the window)",
          file=stream)
    tx_s, rx_s, cpu_s = model_phase_times(findings)
    print(f"  radio TX  {phases.tx_s:9.2f} s   {tx_s * 1000:7.2f} ms/cycle", file=stream)
    print(f"  radio RX  {phases.rx_s:9.2f} s   {rx_s * 1000:7.2f} ms/cycle", file=stream)
    print(f"  CPU resid {phases.cpu_s:9.2f} s   {cpu_s * 1000:7.2f} ms/cycle   "
          f"(awake - tx - rx, per cycle then summed)", file=stream)
    print(f"  HP awake  {phases.awake_s:9.2f} s   "
          f"{phases.awake_s / window.seconds * 100:.3f} % of wall-clock", file=stream)
    print(f"  negative residuals: {phases.negatives}   cycles integrated: {phases.cycles}",
          file=stream)
    print(f"  outliers > {OUTLIER_THRESHOLD_MS / 1000:g} s: {phases.outlier_count} cycles / "
          f"{phases.outlier_s:.1f} s = {phases.outlier_s / phases.awake_s * 100:.1f} % of awake "
          f"(~{phases.outlier_ua:.0f} uA rail at the mid CPU current -- an ASSUMPTION, a 41 s "
          f"cycle is more likely radio than CPU)", file=stream)

    print(file=stream)
    print("Coverage (a de-duplication rate, not a defect rate)", file=stream)
    for name in PER_CYCLE_ENTITIES:
        sampled, held, missing, fraction = findings.coverage[name]
        print(f"  {name:16s} sampled {sampled:5d}/{window.n_cycles} ({fraction * 100:5.1f} %)   "
              f"held {held:5d}   missing {missing:4d}", file=stream)
    print(f"  sampled + held + missing == {window.n_cycles} for every row, so each filled cell is "
          f"auditable: `held` is exactly the count of HA-dropped repeats put back.", file=stream)
    print(f"  A series that changes far more slowly than the dwell cap ({window.table.max_hold_s:.0f} s) "
          f"shows most cycles as `missing` -- heater_run_count moves once a day and is here to "
          f"anchor cycles, not to be integrated. Only tx/rx/awake feed the totals above.",
          file=stream)

    print(file=stream)
    lp = findings.lp
    if lp is None:
        print("LP poll period: NOT MEASURABLE (fewer than two heater_run_count increments)",
              file=stream)
        print(f"  the model below runs on power_model's nominal "
              f"{pm.lp_poll_period_s(findings.sensor_samples):.4f} s for sensor_samples "
              f"{findings.sensor_samples}", file=stream)
    else:
        print("LP poll period", file=stream)
        print(f"  raw       {lp.raw_s:.4f} s  +/-{lp.uncertainty_frac * 100:.4f} % "
              f"(heater_run_count, {lp.increments} increments x {HEATER_TICKS_PER_STEP} ticks)",
              file=stream)
        print(f"  corrected {lp.corrected_s:.4f} s  (raw minus stall/{HEATER_TICKS_PER_STEP}; "
              f"each interval contains exactly one heater run, so the span is 4320*P + stall)",
              file=stream)
        print(f"  cross-check {lp.cross_check_s:.4f} s "
              f"(safeguard pool p{SAFEGUARD_PERCENTILE * 100:.0f} / k={lp.k}, "
              f"{len(findings.safeguard.pool)} samples), "
              f"disagreement {lp.disagreement * 100:+.3f} %", file=stream)
        print(f"  --lp-poll-period takes the period directly; the implied LP timer sleep is "
              f"{lp.interval_raw_s:.4f} s raw / {lp.interval_corrected_s:.4f} s corrected",
              file=stream)
        print(f"  heater busy-wait +{lp.stall_s:.0f} s "
              f"(kRhSettleMs 90 s + ~3 x kCooldownPollMs)" if lp.stall_s else
              "  heater busy-wait: not measurable in this window", file=stream)
        print("  The two figures bracket the answer: correcting the stall puts the period below "
              "the model's sleep+work floor, which implies a slightly fast RTC_SLOW clock.",
              file=stream)

    print(file=stream)
    measured = findings.measured
    print("Measured current (battery decay; charge in, charge out -- no pack voltage in it)",
          file=stream)
    print(f"  {measured.ua:.0f} uA +/-{measured.half_width_ua:.0f} uA "
          f"({measured.confidence * 100:.0f} % interval, "
          f"{measured.bootstrap.block_hours:g} h residual block bootstrap, "
          f"{measured.bootstrap.replications} replications, {measured.bootstrap.blocks} blocks, "
          f"seed {measured.bootstrap.seed})", file=stream)
    print(f"  from {measured.pct_delta:.2f} pp over {measured.hours:.2f} h at "
          f"{pm.PACK_CAPACITY_MAH:.0f} mAh; mean pack {measured.mean_v:.3f} V (reporting only)",
          file=stream)
    print(f"  operating point: sensor_samples {findings.sensor_samples}, "
          f"TX power {findings.tx_power_dbm:g} dBm", file=stream)


def report(findings, run_model=True, basis="raw", capacity_mah=None, forced=False,
           stream=sys.stdout):
    """Print the whole thing and return the exit code."""
    fatal = [guard for guard in findings.guards if not guard.ok and guard.fatal]
    warnings = [guard for guard in findings.guards if not guard.ok and not guard.fatal]

    if fatal and not forced:
        # No partial numbers to copy: a figure pasted out of a failing run is exactly how this
        # project's records acquired windows that were invalid all along.
        print_guards(findings, stream=stream)
        print(file=stream)
        print(f"REFUSING to quote this window: {len(fatal)} fatal guard rail(s). "
              f"Re-run with --force to override.", file=stream)
        return 2

    if forced and fatal:
        print("=" * 79, file=stream)
        print(f"FORCED past {len(fatal)} fatal guard rail(s). These numbers are an experiment, "
              "not a measurement.", file=stream)
        print("=" * 79, file=stream)
        print(file=stream)

    print_guards(findings, forced=forced and bool(fatal), stream=stream)
    print(file=stream)
    print_findings(findings, stream=stream)

    print(file=stream)
    print("Reproduce with:", file=stream)
    print(f"    {model_command_line(findings, basis, capacity_mah)}", file=stream)
    for guard in fatal if forced else []:
        print(f"    # FORCED: {guard.name} -- {guard.detail}", file=stream)

    if run_model:
        print(file=stream)
        print("-" * 79, file=stream)
        budget = pm.power_budget(
            findings.window.cadence_s,
            sensor_samples=findings.sensor_samples,
            lp_poll_period_sec=model_lp_period_s(findings, basis),
            capacity_mah=capacity_mah,
            measured_ua=findings.measured.ua,
            tx_power_dbm=findings.tx_power_dbm,
            phase_times_s=model_phase_times(findings),
        )
        pm.print_budget(budget)

        # The model's primitive is window totals; everything above was the per-cycle front end.
        # Running the primitive directly on the raw window is the honest form of the comparison,
        # and the small residual difference is worth printing rather than hiding, because it is
        # section 1.6's trap made visible: cadence divides T by the n-1 INTERVALS between anchors
        # while the phase means divide the totals by the n CYCLES, so the per-cycle form
        # reconstructs a window of cadence * n = T * n/(n-1), about 0.03 % long here.
        awake_s = findings.phases.tx_s + findings.phases.rx_s + findings.phases.cpu_s
        from_totals = pm.model_avg_current_ua(
            findings.window.seconds, findings.phases.tx_s, findings.phases.rx_s,
            findings.phases.cpu_s, None, findings.sensor_samples,
            model_lp_period_s(findings, basis), findings.tx_power_dbm)
        print(file=stream)
        print(f"Totals form (the model's primitive): {from_totals:.1f} uA "
              f"= {awake_s:.1f} s awake in {findings.window.seconds:.0f} s", file=stream)
        print(f"  vs {budget.mid_ua:.1f} uA from the per-cycle form above, a "
              f"{abs(from_totals - budget.mid_ua):.3f} uA gap: cadence is T/(n-1) over "
              f"{findings.window.n_cycles - 1} intervals while the phase means are totals/n over "
              f"{findings.phases.cycles} cycles. Same data, two denominators.", file=stream)

        # Available numerically because summarize() was split -- a human noticing this in 40 lines
        # of output is not a guard rail.
        if budget.residual_ua is not None and budget.residual_ua < 0:
            print(file=stream)
            print("GUARD: the self-discharge residual is negative, which is arithmetically "
                  "impossible rather than merely pessimistic -- see capacity_floor().",
                  file=stream)
            return 2

    if forced and fatal:
        print(file=stream)
        print("=" * 79, file=stream)
        print(f"FORCED past {len(fatal)} fatal guard rail(s). These numbers are an experiment, "
              "not a measurement.", file=stream)
        print("=" * 79, file=stream)
        return 3
    return 1 if warnings else 0


# --- CLI -------------------------------------------------------------------------------

def _build_parser():
    parser = argparse.ArgumentParser(
        prog="ha_log_metrics.py",
        description="Derive power_model.py inputs from a Home Assistant history export.")
    parser.add_argument("--csv", required=True, metavar="PATH",
                        help="HA history export, .csv or .csv.gz")
    parser.add_argument("--device", default=None, metavar="PREFIX",
                        help="override the inferred entity-name prefix")
    parser.add_argument("--sensor-samples", type=int, default=None, metavar="N",
                        help="override, when the number.* entity is absent from the export")
    parser.add_argument("--tx-power", type=float, default=None, metavar="DBM",
                        help="override, when tx_power_active is absent from the export")
    parser.add_argument("--capacity", type=float, default=None, metavar="MAH",
                        help="assumed pack capacity, passed through to the model")
    parser.add_argument("--cycle-epsilon", type=float, default=hh.CYCLE_EPSILON_S,
                        metavar="SECONDS", help="anchor cluster width (default: %(default)s)")
    parser.add_argument("--max-hold", type=float, default=hh.MAX_HOLD_S, metavar="SECONDS",
                        help="forward-fill dwell cap (default: %(default)s)")
    parser.add_argument("--outlier-threshold", type=float, default=OUTLIER_THRESHOLD_MS,
                        metavar="MS", help="per-cycle HP-awake outlier cut (default: %(default)s)")
    parser.add_argument("--lp-period-basis", choices=("raw", "corrected"), default="raw",
                        help="which LP-period estimate to feed the model; both are always "
                             "printed (default: %(default)s, for continuity with recorded figures)")
    parser.add_argument("--run", dest="run", action="store_true", default=True,
                        help="run power_model in-process against these findings (default)")
    parser.add_argument("--no-run", dest="run", action="store_false",
                        help="emit the command line without running the model")
    parser.add_argument("--list-entities", action="store_true",
                        help="print the export's short names and row counts, then exit")
    parser.add_argument("--force", action="store_true",
                        help="downgrade fatal guard rails to warnings; the run announces itself "
                             "as forced both before and after the numbers, and every violation is "
                             "echoed as a comment on the emitted command line")
    parser.add_argument("--quiet", action="store_true",
                        help="emit only the power_model.py command line, for scripting")
    return parser


def main(argv=None):
    args = _build_parser().parse_args(argv)
    export = hh.load_export(args.csv, args.device)

    if args.list_entities:
        print(f"{export.path}  device prefix {export.device_prefix!r}")
        for name in hh.short_names(export):
            series = export.series[name]
            print(f"  {name:28s} {len(series.samples):6d} numeric  "
                  f"{len(series.blackouts):3d} blackout  {series.non_numeric:4d} non-numeric  "
                  f"{series.boundary_stamps} boundary")
        return 0

    try:
        findings = derive(export, args.cycle_epsilon, args.max_hold, args.outlier_threshold,
                          args.sensor_samples, args.tx_power)
    except hh.AnchorError as error:
        # Same exit code as a failed guard rail: the data can't be quoted as asked, and nothing
        # was computed that could be copied out of a traceback.
        print(f"REFUSING to anchor cycles in this export: {error}")
        return 2
    if args.quiet:
        print(model_command_line(findings, args.lp_period_basis, args.capacity))
        return 0
    return report(findings, args.run, args.lp_period_basis, args.capacity, args.force)


if __name__ == "__main__":
    raise SystemExit(main())
