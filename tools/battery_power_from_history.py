#!/usr/bin/env python3
"""Estimate average power/current draw from a Home Assistant voltage history CSV export.

Reimplements the firmware's voltage->battery% curve (main/sensorstask.h) so the
percentages match what HA's own battery sensor shows, then fits % vs. elapsed
time across all samples (robust to the ~2 mV ADC quantization noise) to derive
mean mW / mA the same way docs/power-usage-from-usb.ods does it by hand.

Before trusting the result, confirm the export window is OTA-free (check the HA
update entity's history): an OTA's few high-power minutes are enough to skew a
multi-day average by 20%+, as happened on 2026-08-01ish (0.40-0.46 mA measured
with OTA activity inside the window vs. 0.335 mA on an OTA-free window covering
much of the same period).

If the export carries a temperature series alongside the voltage one, the fit is
also run with ambient temperature as a second regressor. That matters because the
pack's own open-circuit voltage moves with temperature, so a window whose ambient
temperature TRENDS (rather than merely oscillating) biases the discharge slope --
measured at up to 11% on the 2026-08-29->09-07 window, which cooled ~9 K end to end.
See temperature_corrected_fit() for the measured coefficients.

    tools/battery_power_from_history.py [--csv PATH]
"""
import argparse
import statistics
from bisect import bisect_right
from collections import namedtuple

from ha_history import load_series
from power_model import PACK_CAPACITY_MAH, PACK_NOMINAL_VOLTAGE

DEFAULT_CSV_PATH = "voltage_history.csv"

# Reject a temperature sample as a stale stand-in for a voltage sample beyond this gap.
# HA de-duplicates unchanged values, so an exact-timestamp join would drop almost every
# row -- the join has to be as-of (most recent sample at or before), not exact.
MAX_TEMPERATURE_STALENESS_S = 1800

# Below this many joined samples the second regressor is not worth fitting. Deliberately a
# strict ">" comparison, as it always has been -- see this module's history.
MIN_JOINED_SAMPLES = 100

# Etalon (reference) resting-voltage discharge curve of a textbook 4.20 V/cell 1S
# Li-ion, copied from main/sensorstask.h's BATTERY_CURVE.
BATTERY_CURVE = [
    (3270, 0), (3610, 5), (3690, 10), (3710, 15), (3730, 20), (3750, 25),
    (3770, 30), (3790, 35), (3800, 40), (3820, 45), (3840, 50), (3850, 55),
    (3870, 60), (3910, 65), (3950, 70), (3980, 75), (4020, 80), (4080, 85),
    (4110, 90), (4150, 95), (4200, 100),
]

# The pack constants live in power_model.py, which carries the capacity-parameter essay
# explaining why capacity is a parameter of the COMPARISON rather than a property of the device.
# Note which one is bound here: PACK_NOMINAL_VOLTAGE (3.7 V, a nameplate figure used only to quote
# pack energy in mWh), NOT power_model.PACK_MEAN_VOLTAGE (3.931 V, this pack's measured mean
# terminal voltage). This module's old `PACK_MEAN_VOLTAGE = 3.7` was the nominal misnamed, and
# that misnaming is exactly how a 3.7/mean_v factor got into the current calculation.
PACK_ENERGY_MWH = PACK_CAPACITY_MAH * PACK_NOMINAL_VOLTAGE  # 11840 mWh

Decay = namedtuple("Decay", "readings hours percents n t0 t1 total_hours first_v last_v mean_v "
                            "slope pct_start pct_end pct_delta mwh_used mean_mw mean_ma "
                            "runtime_days")
Joined = namedtuple("Joined", "hours percents temperatures")
TemperatureFit = namedtuple("TemperatureFit", "n slope coefficient residual_sd drift_k min_c "
                                              "max_c corrected_mw corrected_ma")


def etalon_soc(mv):
    if mv <= BATTERY_CURVE[0][0]:
        return float(BATTERY_CURVE[0][1])
    if mv >= BATTERY_CURVE[-1][0]:
        return float(BATTERY_CURVE[-1][1])
    for (lo_mv, lo_pct), (hi_mv, hi_pct) in zip(BATTERY_CURVE, BATTERY_CURVE[1:]):
        if lo_mv <= mv <= hi_mv:
            return lo_pct + (mv - lo_mv) * (hi_pct - lo_pct) / (hi_mv - lo_mv)


def voltage_to_percent(mv):
    # main/sensorstask.h's MIN_VOLTAGE/MAX_VOLTAGE pack anchors: they rescale the
    # generic etalon curve so 3300 mV -> 0% and 4120 mV -> 100% for this pack.
    soc_min = etalon_soc(3300)
    soc_max = etalon_soc(4120)
    normalized = (etalon_soc(mv) - soc_min) / (soc_max - soc_min) * 100.0
    return max(0.0, min(100.0, normalized))


def ordinary_least_squares(columns, y):
    """Multiple linear regression by Gaussian elimination on the normal equations.

    Hand-rolled rather than pulled in from numpy/scipy: this repo deliberately carries no
    third-party dependency for anything it can compute itself, and the design matrix here
    is at most 3 columns wide.
    """
    n = len(y)
    k = len(columns)
    a = [[sum(columns[i][r] * columns[j][r] for r in range(n)) for j in range(k)] for i in range(k)]
    b = [sum(columns[i][r] * y[r] for r in range(n)) for i in range(k)]
    for i in range(k):
        pivot = max(range(i, k), key=lambda r: abs(a[r][i]))
        a[i], a[pivot] = a[pivot], a[i]
        b[i], b[pivot] = b[pivot], b[i]
        for r in range(i + 1, k):
            factor = a[r][i] / a[i][i]
            for c in range(i, k):
                a[r][c] -= factor * a[i][c]
            b[r] -= factor * b[i]
    solution = [0.0] * k
    for i in reversed(range(k)):
        solution[i] = (b[i] - sum(a[i][j] * solution[j] for j in range(i + 1, k))) / a[i][i]
    residuals = [y[r] - sum(solution[i] * columns[i][r] for i in range(k)) for r in range(n)]
    return solution, residuals


def temperature_corrected_fit(hours, percents, temperatures):
    """Refit battery% against elapsed time AND ambient temperature.

    Returns (slope_pp_per_hour, coefficient_pp_per_kelvin, residual_sd).

    The coefficient is real and was measured on this device across five multi-day windows:
    +0.075, +0.083, +0.162, +0.255, +0.341 mV/K, correlating with mean pack voltage at
    r = -0.98 (n=5). That SoC dependence is why it is a battery effect (a cell's entropic
    coefficient dU/dT is SoC-dependent by definition) rather than ADC-reference or divider
    drift, which cannot know the pack's state of charge. Magnitudes are consistent with
    published full-cell entropic coefficients for graphite-anode cells.

    IMPORTANT -- this sharpens a single window's number; it does NOT explain the
    window-to-window spread. Applying it to all five windows moved the spread the wrong
    way (0.60-1.25 mA raw -> 0.54-1.26 mA corrected, sd 0.231 -> 0.245 mA). Do not reach
    for it as an explanation of the long-running residual drift.
    """
    ones = [1.0] * len(hours)
    (_, slope, coefficient), residuals = ordinary_least_squares([ones, hours, temperatures], percents)
    return slope, coefficient, statistics.pstdev(residuals)


def decay_fit(readings):
    """Fit battery % against elapsed hours and derive the window's mean power and current.

    Four details here move the last printed digit and are deliberate, not accidental:

    * `total_hours` is `hours[-1]`, the last elapsed-hours value, not a recomputed span.
    * `pct_start` is the regression INTERCEPT, not `percents[0]` -- the whole point is to be
      robust to the ~2 mV ADC quantization on the endpoints.
    * `mean_v` spans every reading, and the temperature-corrected fit below divides by this same
      `mean_v` even though it runs on a smaller joined subset.
    * `runtime_days` keeps the ENERGY form (`PACK_ENERGY_MWH / mean_mw`) because the 3.7 V
      cancels there exactly; it is unaffected by the current-side unit question.
    """
    t0 = readings[0][0]
    hours = [(ts - t0).total_seconds() / 3600.0 for ts, _ in readings]
    voltages = [v for _, v in readings]
    percents = [voltage_to_percent(v * 1000) for v in voltages]
    total_hours = hours[-1]

    fit = statistics.linear_regression(hours, percents)
    pct_start = fit.intercept
    pct_end = fit.intercept + fit.slope * total_hours
    pct_delta = pct_start - pct_end

    mean_v = statistics.mean(voltages)
    mwh_used = pct_delta / 100.0 * PACK_ENERGY_MWH
    mean_mw = mwh_used / total_hours
    mean_ma = mean_mw / mean_v
    return Decay(readings=readings, hours=hours, percents=percents, n=len(hours), t0=t0,
                 t1=readings[-1][0], total_hours=total_hours, first_v=voltages[0],
                 last_v=voltages[-1], mean_v=mean_v, slope=fit.slope, pct_start=pct_start,
                 pct_end=pct_end, pct_delta=pct_delta, mwh_used=mwh_used, mean_mw=mean_mw,
                 mean_ma=mean_ma, runtime_days=PACK_ENERGY_MWH / mean_mw / 24.0)


def join_temperatures(decay, temperature_readings, max_staleness_s=MAX_TEMPERATURE_STALENESS_S):
    """As-of join: the most recent temperature sample at or before each voltage sample.

    As-of and not exact-timestamp, because HA de-duplicates unchanged values and an exact join
    would drop almost every row. NB this is the right primitive here precisely BECAUSE the two
    series are independent of each other; for series co-published in one MQTT message an as-of
    join silently shifts them by a whole cycle.
    """
    stamps = [ts for ts, _ in temperature_readings]
    values = [v for _, v in temperature_readings]
    matched_hours = []
    matched_percents = []
    matched_temps = []
    for (timestamp, _), hour, percent in zip(decay.readings, decay.hours, decay.percents):
        i = bisect_right(stamps, timestamp) - 1
        if i < 0 or (timestamp - stamps[i]).total_seconds() > max_staleness_s:
            continue
        matched_hours.append(hour)
        matched_percents.append(percent)
        matched_temps.append(values[i])
    return Joined(hours=matched_hours, percents=matched_percents, temperatures=matched_temps)


def temperature_fit(decay, joined):
    """Run temperature_corrected_fit() over a joined subset and restate it as mW / mA."""
    corrected_slope, coefficient, residual_sd = temperature_corrected_fit(
        joined.hours, joined.percents, joined.temperatures)
    corrected_mw = -corrected_slope / 100.0 * PACK_ENERGY_MWH
    temp_fit = statistics.linear_regression(joined.hours, joined.temperatures)
    drift_k = temp_fit.slope * (joined.hours[-1] - joined.hours[0])
    return TemperatureFit(n=len(joined.hours), slope=corrected_slope, coefficient=coefficient,
                          residual_sd=residual_sd, drift_k=drift_k,
                          min_c=min(joined.temperatures), max_c=max(joined.temperatures),
                          corrected_mw=corrected_mw, corrected_ma=corrected_mw / decay.mean_v)


def print_decay(decay):
    print(f"Samples: {decay.n}")
    print(f"Window: {decay.t0.isoformat()} -> {decay.t1.isoformat()} ({decay.total_hours:.2f} h)")
    print(f"Voltage: {decay.first_v:.3f} V -> {decay.last_v:.3f} V (mean {decay.mean_v:.3f} V)")
    print(f"Battery %: {decay.pct_start:.2f}% -> {decay.pct_end:.2f}% "
          f"(fitted, delta {decay.pct_delta:.2f} pp)")
    print(f"Energy used: {decay.mwh_used:.2f} mWh")
    print(f"Average power: {decay.mean_mw:.3f} mW")
    print(f"Average current: {decay.mean_ma:.3f} mA")
    print(f"Implied full-pack runtime at this rate: {decay.runtime_days:.1f} days")


def print_temperature_correction(decay, fit):
    print()
    print(f"Temperature-corrected fit ({fit.n} joined samples):")
    print(f"  Ambient: {fit.min_c:.1f} - {fit.max_c:.1f} C, "
          f"trend {fit.drift_k:+.2f} K across the window")
    print(f"  Coefficient: {fit.coefficient:+.4f} pp/K (residual sd {fit.residual_sd:.3f} pp)")
    print(f"  Average current: {fit.corrected_ma:.3f} mA "
          f"({(fit.corrected_ma - decay.mean_ma) / decay.mean_ma * 100:+.1f}% vs uncorrected)")
    if abs(fit.drift_k) < 2.0:
        print("  NOTE: ambient barely trended here, so the correction is small by construction.")
    print("  The correction de-biases THIS window; it does not explain window-to-window")
    print("  spread (tested across five windows -- see temperature_corrected_fit's docstring).")


def _build_parser():
    parser = argparse.ArgumentParser(
        prog="battery_power_from_history.py",
        description="Average current from a Home Assistant battery-voltage history export.")
    parser.add_argument("--csv", default=DEFAULT_CSV_PATH, metavar="PATH",
                        help="HA history export, .csv or .csv.gz (default: %(default)s)")
    return parser


def main(argv=None):
    args = _build_parser().parse_args(argv)
    readings, temperature_readings = load_series(args.csv)
    decay = decay_fit(readings)
    print_decay(decay)

    if temperature_readings:
        joined = join_temperatures(decay, temperature_readings)
        if len(joined.hours) > MIN_JOINED_SAMPLES:
            print_temperature_correction(decay, temperature_fit(decay, joined))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
