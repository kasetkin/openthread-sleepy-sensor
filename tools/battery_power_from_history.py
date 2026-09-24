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

    tools/battery_power_from_history.py [--csv PATH]

CAUTION -- the "Average current" line changed on 2026-09-09 and is NOT comparable with any figure
recorded before then. It used to compute `mean_mw / mean_v`, where `mean_mw` was itself built from
`PACK_CAPACITY_MAH * 3.7`; expanded, that is

    printed  = dpct/100 * 3200 * 3.7 / (mean_v * hours)      # what it did
    correct  = dpct/100 * 3200       / hours                 # charge in, charge out

leaving a stray `3.7 / mean_v` that biased every reading LOW. This is the same error
power_model.runtime_days() records having fixed on its own side; it was still live here.

The correction is NOT a constant scaling -- it depends on the window's mean pack voltage, so it
shrinks as the pack discharges, and it does not cancel in a window-to-window difference:

    | window                                            | mean_v | was    | now    |
    | voltage_history.csv (2026-07-24 -> 07-28)         | 4.092  | 0.437  | 0.483  |
    | voltage_history_2026-07-26_2026-08-04.csv         | 4.083  | 0.463  | 0.511  |
    | metrics 2026-07-30 -> 08-04 (clean baseline)      | 4.076  | 0.3354 | 0.3695 |
    | metrics 2026-08-06 -> 08-10                       | 4.025  | 0.601  | 0.654  |
    | metrics 2026-08-10 -> 08-13                       | 4.010  | 0.709  | 0.769  |
    | metrics 2026-08-14 -> 08-18                       | 3.983  | 0.800  | 0.862  |
    | metrics 2026-08-21 -> 08-29 (sensor_samples=16)   | 3.968  | 1.169  | 1.253  |
    | metrics 2026-08-29 -> 09-07 (sensor_samples=4)    | 3.931  | 0.565  | 0.600  |

All in mA. Two consequences worth carrying forward. The clean baseline is 0.3695 mA, not 0.3354.
And the ss=16 minus ss=4 delta that LP_ACTIVE_CURRENT_UA was back-solved from moves 604 -> 653 uA,
+8% on that 9 mA guess -- so the back-solve is due a re-run, not just a relabel.

"Average power", "Energy used" and "Implied full-pack runtime" are all UNAFFECTED: the assumed
3.7 V cancels in the runtime expression, and the other two were always energy quantities.

Independent confirmation that the new number is the right one: fitting Home Assistant's own
`battery` percentage series for the 2026-08-29 -> 09-07 window -- no voltage curve, no pack
voltage, no energy conversion anywhere in the path -- gives 600.2 uA, against this script's
600.0 uA and the 564.7 uA it used to print.

CAUTION 2 -- every figure in the table above predates 2026-09-10, when the fit became
DWELL-WEIGHTED (see decay_fit and dwell_weights). That shift is small and does not disturb the
unit-fix argument those numbers were recorded to make, but the "now" column is no longer what
the script prints: the two windows with pinned regression values moved 0.600 -> 0.5946 mA
(-0.90 %) and 1.253 -> 1.259 mA (+0.45 %). The same battery%-direct cross-check quoted above,
re-run dwell-weighted, gives 594.8 uA against the script's 594.6.
"""
import argparse
import statistics
from collections import namedtuple

from ha_history import MAX_HOLD_S, load_series, weighted_least_squares_slope
from power_model import PACK_CAPACITY_MAH, PACK_NOMINAL_VOLTAGE

DEFAULT_CSV_PATH = "voltage_history.csv"

# Ceiling on how long one sample is credited with standing, for the dwell weighting below.
# Bound rather than dropped: across a real dropout the value did stand, we simply have no
# evidence for how long, and clamping keeps the sample while capping its leverage. Shares
# ha_history's hold cap so one number governs both.
MAX_DWELL_H = MAX_HOLD_S / 3600.0

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

Decay = namedtuple("Decay", "readings hours percents weights n t0 t1 total_hours first_v last_v "
                            "mean_v slope pct_start pct_end pct_delta mwh_used mean_mw mean_ma "
                            "runtime_days")


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


def dwell_weights(hours, max_dwell_h=MAX_DWELL_H):
    """How long each sample STOOD, in hours -- the weights for the decay fit.

    Home Assistant de-duplicates unchanged values, so a voltage row exists only where the
    voltage moved, and on this device it moves with the diurnal temperature swing: rows run
    ~4x denser at midday than before dawn (6.5/h at 07:00 vs 26/h at 10:00 on the
    2026-09-07 -> 09-09 window). An unweighted fit gives every row one vote and so leans on
    whichever hours were noisiest; weighting by dwell time is the continuous-time fit.

    The final sample has no successor, so its dwell is unobservable and it is credited with
    the median of the rest rather than being dropped -- dropping it would shorten the fitted
    span, which is the one thing the endpoint is actually needed for.
    """
    if len(hours) < 2:
        return [1.0] * len(hours)
    gaps = [min(b - a, max_dwell_h) for a, b in zip(hours, hours[1:])]
    gaps = [gap if gap > 0.0 else 0.0 for gap in gaps]
    positive = [gap for gap in gaps if gap > 0.0]
    return gaps + [statistics.median(positive) if positive else 1.0]


def decay_fit(readings):
    """Fit battery % against elapsed hours and derive the window's mean power and current.

    The fit is DWELL-WEIGHTED: each sample is weighted by how long its value stood, not counted
    once. See dwell_weights() for why -- HA's de-duplication makes the row density a function of
    the diurnal temperature swing, and an unweighted fit inherits that as a bias. Switched on
    2026-09-10; it moves a nine-day window ~0.5 % and a two-day one ~3 %, both well inside the
    bootstrap interval, so it is a correctness fix rather than a material change of answer. It
    is NOT a robustness fix: measured across rolling sub-windows the spread is unchanged, because
    the dominant error is the diurnal signal itself, which is a function of time and survives
    any reweighting.

    Four details here move the last printed digit and are deliberate, not accidental:

    * `total_hours` is `hours[-1]`, the last elapsed-hours value, not a recomputed span.
    * `pct_start` is the regression INTERCEPT, not `percents[0]` -- the whole point is to be
      robust to the ~2 mV ADC quantization on the endpoints.
    * `mean_v` spans every reading and is REPORTING ONLY: no current divides by it any more.
      That division is exactly what the 2026-09-09 unit fix removed, and reinstating it would
      put the bias straight back. Note it is deliberately left UNWEIGHTED -- it is a plain
      description of the samples, not an input to anything.
    * `runtime_days` keeps the ENERGY form (`PACK_ENERGY_MWH / mean_mw`) because the 3.7 V
      cancels there exactly -- `PACK_ENERGY_MWH / (pct_delta/100 * PACK_ENERGY_MWH / hours)`
      reduces to `hours / (pct_delta/100)`. It was never affected by the unit bug and must not be
      "made consistent" with the charge form; leave it alone.
    """
    t0 = readings[0][0]
    hours = [(ts - t0).total_seconds() / 3600.0 for ts, _ in readings]
    voltages = [v for _, v in readings]
    percents = [voltage_to_percent(v * 1000) for v in voltages]
    total_hours = hours[-1]
    weights = dwell_weights(hours)

    fit = weighted_least_squares_slope(hours, percents, weights)
    pct_start = fit.intercept
    pct_end = fit.intercept + fit.slope * total_hours
    pct_delta = pct_start - pct_end

    mean_v = statistics.mean(voltages)
    mwh_used = pct_delta / 100.0 * PACK_ENERGY_MWH
    mean_mw = mwh_used / total_hours
    # Charge in, charge out. A battery-percentage decay is a fraction-of-CAPACITY-per-hour
    # quantity, and capacity is in mAh -- so the current follows from mAh directly and no voltage
    # belongs in it. See the CAUTION block in this module's docstring for what this changed.
    mean_ma = pct_delta / 100.0 * PACK_CAPACITY_MAH / total_hours
    return Decay(readings=readings, hours=hours, percents=percents, weights=weights,
                 n=len(hours), t0=t0, t1=readings[-1][0], total_hours=total_hours,
                 first_v=voltages[0], last_v=voltages[-1], mean_v=mean_v, slope=fit.slope,
                 pct_start=pct_start, pct_end=pct_end, pct_delta=pct_delta, mwh_used=mwh_used,
                 mean_mw=mean_mw, mean_ma=mean_ma,
                 runtime_days=PACK_ENERGY_MWH / mean_mw / 24.0)


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


def _build_parser():
    parser = argparse.ArgumentParser(
        prog="battery_power_from_history.py",
        description="Average current from a Home Assistant battery-voltage history export.")
    parser.add_argument("--csv", default=DEFAULT_CSV_PATH, metavar="PATH",
                        help="HA history export, .csv or .csv.gz (default: %(default)s)")
    return parser


def main(argv=None):
    args = _build_parser().parse_args(argv)
    # load_series still returns a temperature series; this module no longer has a use for one.
    readings, _ = load_series(args.csv)
    decay = decay_fit(readings)
    print_decay(decay)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
