#!/usr/bin/env python3
"""Reconciled duty-cycle power model for the sleepy sensor, parameterized by cycle cadence.

Formalizes the "Full reconciled power model" from the 2026-08-05 light-sleep power
investigation (see project_light_sleep_power_investigation memory) as code instead of
one-off prose math, and generalizes it from a single hardcoded 290s example to any
cadence. Per-cycle awake-phase durations are treated as fixed (independent of cadence) --
validated by the 2026-08-14->08-18 measurement, where a ~172s actual cadence (vs the
~290s this model was calibrated against) still showed a normal per-cycle hp_awake_time,
confirming cadence changes wall-clock frequency, not per-cycle work.

All inputs are named flags; run with --help for the full list.

    tools/ha_log_metrics.py --csv metrics_history_2026-08-29__to__2026-09-07.csv --run

That is now the way to run this model against a real window: ha_log_metrics.py derives every flag
below from a Home Assistant export and prints the invocation it used. Driving it by hand still
works, and the flags mean the same thing:

    tools/power_model.py --cadence 234.34 --sensor-samples 4 --lp-poll-period 20.3442 \
                         --phase-times 0.09901,0.36425,0.49826 --measured 600

Pass --measured to reconcile against a voltage-decay result: the model then prints the implied
self-discharge residual and the smallest pack capacity that keeps the budget self-consistent.
--phase-times sets the per-cycle (tx, rx, cpu) durations explicitly.

NB: this used to take bare positional arguments (cadence, samples, poll interval, ...). Those no
longer work -- old invocations need the flags. --lp-poll-interval is likewise gone, replaced by
--lp-poll-period: it takes the LP core's real wake-to-wake period (sleep + work, which is what
heater_run_count measures) rather than the timer's sleep duration. Converting an old invocation
means ADDING lp_active_time_s(N) -- 11 ms at N=1, 344 ms at N=4, 1676 ms at N=16.

CAUTION -- three constants changed on 2026-09-09 and output is NOT comparable with figures
recorded before then: SLEEP_CURRENT_UA 28 -> 35 uA, LP_ACTIVE_CURRENT_UA None -> 9000 uA, and
TX current 153 mA (6 dBm, hardcoded) -> a TX_POWER_DBM lookup defaulting to 20 dBm. A runtime
bug was fixed at the same time (it mixed a 3.7 V nominal with a 3.983 V draw and understated
runtime by 7.6%), and the model now converts across the buck so its totals are battery-side and
directly comparable with a decay measurement.
"""
import argparse
import math
from collections import namedtuple

# Per-cycle awake-phase durations (seconds) and currents (mA), current conditions (6 dBm TX).
# Sources: 2026-08-05 5-phase breakdown + radio_tx_time/radio_rx_time (see memory above).
TX_TIME_S = 0.090

# C6 datasheet Table 5-9, "Current Consumption for 802.15.4 in Active Mode". Only these four
# points are published; anything between them is a linear interpolation, not a table value --
# tx_current_uncertainty_ma() below puts a number on how much that choice is worth.
TX_CURRENT_BY_DBM = {-15.0: 92.0, 0.0: 119.0, 12.0: 187.0, 20.0: 305.0}

# The runtime knob (runtime_config.cpp's cfg/tx_power_dbm) is an int32_t end to end, so integer
# dBm over the datasheet's own span is the model's entire input domain -- 36 values, which is why
# tx_current_table() can materialise all of them rather than interpolating on demand.
TX_POWER_MIN_DBM = -15
TX_POWER_MAX_DBM = 20

# The device's live tx_power_dbm. It ran at 6 dBm from 2026-08-05, was found back at 20 dBm by
# 2026-08-29, and the user has since decided to KEEP 20 dBm -- so 20 is the current baseline, not
# a bug to be corrected. NB: every calibration point recorded in project_power_estimates before
# 2026-09-09 was computed with a hardcoded 153 mA (6 dBm), so those figures are not comparable to
# output produced at this default.
#
# How much not comparable depends entirely on the cadence, and the figure below is quoted at the
# one the device actually runs -- 234.34 s, the measured 2026-08-29 -> 09-07 publish cadence, with
# that window's measured 99.01 ms of TX per cycle:
#
#     (305 - 153) mA * 0.09901 s / 234.34 s = +64.2 uA
#
# Name the cadence whenever this number is repeated. At the 290 s argparse default with the
# calibration TX_TIME_S it is +47.2 uA instead, and a reader who assumes the wrong one will
# "correct" a figure that was right.
TX_POWER_DBM = 20.0


def tx_current_ma(tx_power_dbm=None):
    """Radio TX current at a given TX power, linearly interpolated between datasheet points."""
    dbm = TX_POWER_DBM if tx_power_dbm is None else tx_power_dbm
    points = sorted(TX_CURRENT_BY_DBM.items())
    if dbm <= points[0][0]:
        return points[0][1]
    if dbm >= points[-1][0]:
        return points[-1][1]
    for (lo_dbm, lo_ma), (hi_dbm, hi_ma) in zip(points, points[1:]):
        if lo_dbm <= dbm <= hi_dbm:
            return lo_ma + (dbm - lo_dbm) * (hi_ma - lo_ma) / (hi_dbm - lo_dbm)


def tx_current_mw_basis_ma(tx_power_dbm=None):
    """The same interpolation done in LINEAR output power instead of dBm.

    Exists only to bound tx_current_ma()'s modelling error, never to replace it. A PA's drain
    current tracks output power, and output power is what dBm is the logarithm OF -- so
    interpolating linearly in dBm and interpolating linearly in mW are two defensible readings of
    the same four datasheet points, and they disagree by up to 25 mA (see
    tx_current_uncertainty_ma).

    NB: neither basis is globally right, and that is the point. Fitting only the 0 and 12 dBm
    anchors and extrapolating down to -15 gives 33.9 mA in the dBm basis and 114.6 mA in the mW
    basis, against a published 92.0 -- so the true curve is neither, and the piecewise structure
    is doing the real work. Both bases agree exactly at all four anchors; between them, treat the
    gap as an error bar rather than picking a winner.
    """
    dbm = TX_POWER_DBM if tx_power_dbm is None else tx_power_dbm
    points = sorted(TX_CURRENT_BY_DBM.items())
    if dbm <= points[0][0]:
        return points[0][1]
    if dbm >= points[-1][0]:
        return points[-1][1]
    milliwatts = 10.0 ** (dbm / 10.0)
    for (lo_dbm, lo_ma), (hi_dbm, hi_ma) in zip(points, points[1:]):
        if lo_dbm <= dbm <= hi_dbm:
            lo_mw = 10.0 ** (lo_dbm / 10.0)
            hi_mw = 10.0 ** (hi_dbm / 10.0)
            return lo_ma + (milliwatts - lo_mw) * (hi_ma - lo_ma) / (hi_mw - lo_mw)


def tx_current_uncertainty_ma(tx_power_dbm=None):
    """How much the interpolation BASIS is worth at this TX power, in mA. Zero at the anchors.

    This is the honest error bar on tx_current_ma() and it is not small: 20.3 mA at 6 dBm and
    25.4 mA at 16 dBm, against datasheet anchors that are exact. It matters here because the two
    TX powers this device has actually run at sit at opposite ends of it -- 20 dBm is a published
    anchor and carries none of this, while 6 dBm is the exact midpoint of the 0-12 dBm gap and so
    carries the maximum. Every power figure recorded in project_power_estimates before 2026-09-09
    was computed at 6 dBm, i.e. at the least certain point on this curve.

    Note this is a MODELLING spread, not a datasheet tolerance; the datasheet's own part-to-part
    variation is additional and unquantified here.
    """
    return abs(tx_current_ma(tx_power_dbm) - tx_current_mw_basis_ma(tx_power_dbm))


def tx_current_table(step_db=1):
    """Materialise the whole input domain as {dbm: (milliamps, provenance)}.

    provenance is "datasheet" for the four published points and "interpolated" for everything
    else. Enumerating all 36 integer values rather than interpolating on demand is not about
    precision -- it is the same function evaluated at integers, so the numbers are identical --
    it is so that a value used in a recorded result can be looked up and its provenance seen.
    Losing track of which TX power a figure assumed is a mistake this project has already made
    once: the four calibration points in SLEEP_CURRENT_UA's comment are correct only at 6 dBm and
    do not say so.
    """
    table = {}
    dbm = TX_POWER_MIN_DBM
    while dbm <= TX_POWER_MAX_DBM:
        provenance = "datasheet" if float(dbm) in TX_CURRENT_BY_DBM else "interpolated"
        table[dbm] = (tx_current_ma(dbm), provenance)
        dbm += step_db
    return table


def print_tx_current_table(step_db=1):
    """Print tx_current_table() with the per-row basis uncertainty alongside."""
    print(f"TX current vs TX power (C6 datasheet Table 5-9, {TX_POWER_MIN_DBM:+d} to "
          f"{TX_POWER_MAX_DBM:+d} dBm at {step_db} dB steps)")
    print(f"  {'dBm':>4s}  {'mA':>7s}  {'+/- mA':>7s}  source")
    for dbm, (milliamps, provenance) in sorted(tx_current_table(step_db).items()):
        spread = tx_current_uncertainty_ma(dbm)
        marker = "" if provenance == "datasheet" else f"  ({spread / milliamps * 100:.0f} %)"
        print(f"  {dbm:+4d}  {milliamps:7.1f}  {spread:7.1f}  {provenance}{marker}")
    print("  +/- mA is the spread between interpolating linearly in dBm (what the model uses)")
    print("  and linearly in mW -- a modelling choice, not a datasheet tolerance. Zero at the")
    print("  four published anchors, largest midway between them.")


RX_TIME_S = 0.217
RX_CURRENT_MA = 74.0  # C6 datasheet Table 5-9, fixed regardless of TX power

CPU_TIME_S = 0.461  # CPU-active, radio-idle residual (mostly mqtt_state_publish ACK wait)

# Per-cycle phase durations for the two reconciled windows live in PHASE_TIMES_WINDOW_A_S /
# _B_S beside REGRESSION_TABLE below. A --measured-phases flag holding a frozen snapshot of
# window A used to sit here; it was deleted on 2026-09-09 once ha_log_metrics.py could derive the
# figures, because a third copy is a third thing to keep in sync -- and it was already the wrong
# statistic, holding the mean-of-observed (0.09913, 0.36428, 0.49810) over 3390 cycles rather
# than the forward-filled (0.09901, 0.36425, 0.49826). HA de-duplication makes the observed rows
# a biased sample of cycles; see ha_history's docstring.
CPU_CURRENT_MA_LOW = 27.0  # C6 datasheet Table 5-10, clocks-disabled
CPU_CURRENT_MA_HIGH = 38.0  # C6 datasheet Table 5-10, clocks-enabled

# #todo DIRECT MEASUREMENT NEEDED -- 35 uA is a GUESS, not a measurement.
#
# Set to 35.0 on 2026-09-09 at the user's instruction. This is the ESP32-C6 datasheet's Table 5-11
# "CPU, wireless communication modules and peripherals are powered down, and all GPIOs are
# high-impedance" tier -- the better of its two light-sleep rows, and the one this firmware's
# sdkconfig should land in (CONFIG_PM_POWER_DOWN_PERIPHERAL_IN_LIGHT_SLEEP=y,
# CONFIG_PM_ESP_SLEEP_POWER_DOWN_CPU=y). The other tier, 180 uA, is ruled out by the component
# budget: at 180 uA it alone would cost ~161 uA at the battery, more than the whole unexplained
# remainder (see the power-budget report's Table 2).
#
# NOTE -- this REPLACED a back-solved 28.0 uA, which came from the 335.4 uA clean-baseline window
# rather than from a datasheet. So this edit swapped a data-derived estimate for a datasheet one;
# both are estimates, and the datasheet figure is the less device-specific of the two. The
# supporting back-solve spanned ~15-41 uA, so 28 and 35 both sit inside it.
#
# CONSEQUENCE: every calibration point recorded before 2026-09-09 was computed at 28 uA and now
# reads ~7 uA higher (290s: 182.4 -> 189.4; 600s: 102.6 -> 109.6; 900s: 77.8 -> 84.8; 1800s:
# 52.9 -> 59.9). That is the constant changing, not the model breaking -- but do not compare new
# output against those older figures without accounting for it.
#
# ALL FOUR OF THOSE POINTS ASSUME --tx-power 6, and they always did -- they predate the
# TX_POWER_DBM lookup, when 153 mA (6 dBm) was hardcoded. The module default is now 20 dBm, so
# re-running `power_model.py --cadence 290` prints 236.6 uA and reads as a regression against the
# 189.4 quoted here. It is not one: `--tx-power 6` still reproduces 189.4 exactly. This omission
# is why REGRESSION_TABLE below names a TX power on every single row.
#
# The datasheet tier also assumes "all GPIOs high-impedance" and says nothing about an LP core
# running its own timer and LP_I2C, which this firmware does continuously. If anything that makes
# 35 uA optimistic as a whole-chip sleep floor.
#
# To actually measure it: current probe on the 3.3 V rail with the radio detached and the LP core
# halted, or a long cadence (max_publish_gap_sec very high) window where the sleep term dominates.
SLEEP_CURRENT_UA = 35.0  # GUESS -- see #todo above

# Multi-sample averaging (components/lp_sensor_core/lp_core/main.cpp's measureAveraged(),
# device_config.yaml's sensor_samples). SLEEP_CURRENT_UA above stands for the whole-chip floor
# during a sensor_samples=1 poll cadence, so it already absorbs that much LP-active time -- hence
# sensor_samples_extra_ua() below models the DELTA against N=1, not the whole LP-active term.
#
# Per-poll timing mirrors lp_core/main.cpp: each measure() costs kMeasureDelayUs (10ms) plus its
# two LP_I2C transactions (~1ms), and measureAveraged() inserts kInterSampleDelayUs (100ms)
# between samples but not before the first. Both are ulp_lp_core_delay_us() busy-waits (a plain
# cycle-counter spin, see that file's header comment), so this is genuinely CPU-active time, not
# sleep. Keep in sync if either constant changes there.
LP_MEASURE_S = 0.011
LP_INTER_SAMPLE_S = 0.100
LP_EXTRA_S_PER_SAMPLE = LP_MEASURE_S + LP_INTER_SAMPLE_S  # cost of one sample beyond the first
# #todo DIRECT MEASUREMENT NEEDED -- 9 mA is a GUESS, not a measurement.
#
# Set to 9000 uA on 2026-09-09 at the user's instruction so the model stops silently omitting a
# term that is worth ~146 uA (roughly a quarter of the whole budget). Treat every figure this
# constant feeds as provisional until someone puts a current probe on the board.
#
# Where the 9 mA came from: differencing the sensor_samples=16 and =4 windows back-solves
# 9.4-10.1 mA, and netting the component budget against the measured total (see the power-budget
# report) admits 1.4-14.7 mA depending on what is assumed for the light-sleep floor and pack
# self-discharge. 9 mA sits in the overlap of those two ranges. That is a plausible value, not a
# determined one.
#
# Three reasons it is still soft, all of which survive pinning it:
#
# 1. It fails an independent cross-check. Fitting it to those two windows and applying it to the
#    2026-07-30 clean-baseline window (never used in the fit) leaves a +86.8 uA residual vs the
#    +181.3 uA the two fitted windows share -- not globally consistent. NB: those two sharing a
#    residual is an arithmetic identity (two points, one fitted parameter), NOT corroboration.
# 2. The leftover residual is not a constant systematic offset that could simply be subtracted.
#    Four windows ran at sensor_samples=1, where this term contributes nothing whatever its value,
#    and their residuals span 87-514 uA (2026-07-30 / 08-06 / 08-10 / 08-14 windows). The residual
#    drifts by ~430 uA between windows -- the same order as the ~600 uA effect being measured, so
#    any single before/after pair cannot separate the two.
# 3. The name misleads: ~9 mA is ~9x what a 20 MHz LP RISC-V core should draw alone (Table 5-10
#    puts the whole 160 MHz HP core at 27-38 mA), so whatever this captures is WHOLE-CHIP current
#    while the LP core is active -- LP_I2C clocking, the SHT4x mid-conversion, and any power
#    domains held up -- not the LP CPU's own draw.
#
# To actually measure it: a current probe on the 3.3 V rail, triggered on the LP wake, is the
# direct route. Failing that, alternate sensor_samples 16->1->16->1 across four adjacent multi-day
# windows -- repeating the contrast lets the drifting residual be differenced out, which a single
# before/after pair structurally cannot do.
LP_ACTIVE_CURRENT_UA = 9000.0  # GUESS -- see #todo above

# --- pack ------------------------------------------------------------------------------
# Capacity is a PARAMETER of the comparison, not a property of the device. It enters in
# exactly two places, in opposite directions, and getting that backwards is easy:
#
#   * NOTHING above depends on it. Every device term is a datasheet current (TX is 305 mA at
#     20 dBm whatever pack is fitted) times a measured duty cycle. model_avg_current_ua() is
#     capacity-independent, full stop.
#   * A voltage-decay MEASUREMENT is a fraction-of-pack-per-unit-time quantity, so the current
#     it reports scales linearly with whatever capacity you assume. 3200 -> 3100 mAh is -3.12%
#     on the measured current and 0% on the model.
#   * Implied runtime from a measurement is therefore capacity-INVARIANT (the assumed capacity
#     cancels), while implied runtime from this model scales linearly with it. Same words,
#     opposite behaviour, depending on which side produced the number.
#
# The practical consequence is capacity_floor() below: because the device side is fixed and
# absolute, a low enough assumed capacity makes the self-discharge residual negative, which is
# not pessimistic but arithmetically impossible.
#
# 3200 mAh is a nameplate figure and inexpensive 18650s routinely deliver less than rated even
# when new, so treat it as an upper estimate rather than a known quantity.
PACK_CAPACITY_MAH = 3200.0
PACK_NOMINAL_VOLTAGE = 3.7   # only used to quote pack energy in mWh
PACK_ENERGY_MWH = PACK_CAPACITY_MAH * PACK_NOMINAL_VOLTAGE
# Measured mean terminal voltage over the 2026-08-29 -> 09-07 window. Only affects the mW
# conversion; the uA figures are independent of it.
PACK_MEAN_VOLTAGE = 3.931

# Board-level terms that are real, well-anchored, and absent from the duty-cycle model above:
# the always-connected 200.2k+201.7k battery divider (~9.8 uA at 3.93 V, device_config.yaml)
# plus board static leakage (~7.3 uA: the XIAO's measured 14.3 uA deep-sleep total minus the
# C6's own 7 uA). Kept OUT of model_avg_current_ua() so its documented calibration points stay
# comparable, and added by device_total_ua() instead.
BOARD_FIXED_UA = 17.1

# --- rail -> battery -------------------------------------------------------------------
# Everything above is current drawn from the 3.3 V rail, because that is what the datasheets
# specify. A voltage-decay measurement is battery-side, and the SGM6029C buck sits between
# them, so the two are NOT directly comparable: I_batt = I_rail * 3.3 / V_batt / efficiency.
#
# Efficiency is from the SGM6029 datasheet's own "SGM6029C PSM Efficiency vs. Load Current"
# plot at VOUT = 3.3 V, interpolated between its 3.8 V and 4.2 V traces for a 3.931 V pack:
# 93.9% at 1 mA, 94.3% at 32.5 mA, 95.6% at 74 mA, 95.5% at 305 mA, 91.1% at 1 A. It is flat
# and high because at 3.931 V in and 3.3 V out the converter is barely stepping down. One
# average is good enough here -- the spread across our whole load range is under 2 points.
#
# At 3.931 V the net factor is 3.3/3.931/0.94 = 0.884, i.e. rail-side figures OVERSTATE
# battery current by about 13%. Power-save mode is confirmed by the measurement itself:
# forced-PWM would draw 8.5 mA of quiescent current, fifteen times the whole budget.
RAIL_VOLTAGE = 3.3
BUCK_EFFICIENCY = 0.94


def rail_to_battery_ua(rail_ua, efficiency=BUCK_EFFICIENCY, pack_voltage=None):
    """Convert 3.3 V rail current to the battery-side current a decay measurement would see."""
    return rail_ua * RAIL_VOLTAGE / (pack_voltage or PACK_MEAN_VOLTAGE) / efficiency

# --- pack self-discharge -------------------------------------------------------------
# The cell's own leakage. Every term above is something the DEVICE draws; this one is the
# battery losing charge on its own, and a voltage-decay measurement cannot tell the two
# apart -- so it has always been silently folded into this project's "unexplained residual"
# (see project_power_estimates). Published figures for an 18650 are 1-3 %/month at 25 C,
# which on this 3200 mAh pack is 44-131 uA: the same order as the whole rest of the budget.
#
# Deliberately None rather than a guessed number, same policy as LP_ACTIVE_CURRENT_UA: this
# pack has never been measured at rest, and picking 2 %/month would put a ~88 uA invention
# into the total where it would read as if it had been measured. Set it only from a real
# rest test (charge, disconnect the load, log open-circuit voltage for a week).
SELF_DISCHARGE_PCT_PER_MONTH_AT_25C = None

# Self-discharge follows Arrhenius kinetics -- leakage current doubles roughly every 15 K
# (Sensirion-style "doubles per 10 K" rules of thumb are for other mechanisms; the
# measured figure for Li-ion leakage is ~15 K). The device sits by a window, so its
# temperature swings far more than a cupboard-mounted sensor would.
SELF_DISCHARGE_DOUBLING_K = 15.0
SELF_DISCHARGE_REFERENCE_C = 25.0

# The SHT4x measures AIR, on the PCB; the cell is a ~45 g lump that damps and lags those
# swings. Applying the Arrhenius weighting to raw air temperature therefore overstates the
# excursions. One hour is a plausible first-order time constant for an 18650 in still air --
# it is an assumption, not a measurement, and the weighting is only mildly sensitive to it
# (the 2026-08-29->09-07 window's multiplier moves 0.928 -> 0.920 -> 0.910 for tau =
# 0 / 1 / 2 h), so it does not need to be exact.
BATTERY_THERMAL_TAU_H = 1.0


def lp_active_time_s(sensor_samples):
    """Wall-clock time the LP core stays awake per poll for `sensor_samples` readings."""
    samples = max(sensor_samples, 1)
    return samples * LP_MEASURE_S + (samples - 1) * LP_INTER_SAMPLE_S


# The LP timer's SLEEP duration, from device_config.yaml's lp_poll_interval_sec. It is NOT the
# wake-to-wake period: lp_sensor_core.c passes it as ulp_lp_core_cfg_t::lp_timer_sleep_duration_us,
# so the LP core sleeps that long AFTER main() returns and the period is sleep + work. Used only
# as the fallback when no measured period is supplied.
LP_POLL_INTERVAL_DEFAULT_S = 20.0


def lp_poll_period_s(sensor_samples, lp_poll_interval_sec=LP_POLL_INTERVAL_DEFAULT_S):
    """Predict the LP wake-to-wake period from the configured SLEEP duration: sleep + work.

    Prefer a MEASURED period wherever one exists -- this function is the fallback, and its ss=16
    prediction is known wrong. Measurement is possible without any battery data at all:
    heater_run_count advances once per exactly 4320 LP cycles (minutes_to_lp_cycles(1440, 20) in
    main/main.cpp), so heater wall-clock span / 4320 measures the period directly.

    * 2026-08-29->09-07 at sensor_samples=4: 20.3442 s measured vs 20.344 s predicted here.
    * 2026-08-21->08-29 at sensor_samples=16: 21.7868 s measured vs 21.676 s predicted, i.e.
      +110.8 ms/poll (+6.6%) unexplained -- suspiciously close to exactly one extra
      sample-iteration, and no positive-coefficient linear model fits both points at once.

    Treat high-sensor_samples predictions as a lower bound until that is resolved. NB the ss=4
    agreement is less clean than it looks: the measured span includes one heater run per interval,
    which biases it +stall/4320 = about +22 ms (+0.11%), so the raw 20.3442 s is a small overshoot
    sitting on top of what may be a slightly fast RTC_SLOW clock. See docs/plan_ha_log_metrics.md
    section 2.2.
    """
    return lp_poll_interval_sec + lp_active_time_s(sensor_samples)


def lp_poll_interval_s(sensor_samples, lp_poll_period_sec):
    """Inverse of lp_poll_period_s: recover the LP timer's sleep duration from the real period."""
    return lp_poll_period_sec - lp_active_time_s(sensor_samples)


def sensor_samples_extra_ua(sensor_samples, lp_poll_period_sec):
    """Extra average current (uA) from sensor_samples>1, or 0.0 if LP_ACTIVE_CURRENT_UA is
    still unverified (None) -- see its comment.

    `lp_poll_period_sec` is the real wake-to-wake period -- the quantity heater_run_count actually
    measures. It used to be the LP timer's sleep duration instead, which forced any caller holding
    a measured period to subtract lp_active_time_s() before passing it in, only for this function
    to add the same modelled term straight back. That round trip cancelled exactly, but only for a
    caller that performed the precise inverse; at sensor_samples=16, where lp_active_time_s() is
    known wrong by +110.8 ms, it silently baked that error into the input.

    Averaged over the LP poll period, not cadence_s: this recurs every LP poll, independent of the
    HP publish cadence. Since average current over one period is
    SLEEP_CURRENT_UA + (LP_ACTIVE_CURRENT_UA - SLEEP_CURRENT_UA) * duty, and SLEEP_CURRENT_UA was
    back-solved from sensor_samples=1 measurements, the term modeled here is the duty DELTA
    against N=1 -- otherwise the N=1 LP activity would be counted twice. Only that N=1 comparison
    point needs the sleep duration, and it is derived here rather than demanded of the caller.
    """
    if LP_ACTIVE_CURRENT_UA is None:
        return 0.0

    sleep_s = lp_poll_interval_s(sensor_samples, lp_poll_period_sec)

    def duty(samples):
        return lp_active_time_s(samples) / (sleep_s + lp_active_time_s(samples))

    return (LP_ACTIVE_CURRENT_UA - SLEEP_CURRENT_UA) * (duty(sensor_samples) - duty(1))


def arrhenius_multiplier(samples, tau_h=BATTERY_THERMAL_TAU_H):
    """Time-weighted self-discharge multiplier vs. a flat 25 C, from real temperature history.

    `samples` is a chronological [(datetime, temp_c)] list -- e.g. the `_temperature` rows of
    a Home Assistant export. Home Assistant de-duplicates unchanged values, so each sample is
    weighted by its DWELL time (how long it stood before the next one), not counted once:
    an unweighted mean over-represents hot afternoons, which generate far more samples than
    quiet nights. On the 2026-08-29->09-07 window that distinction alone moves the mean from
    24.5 C to 21.9 C.

    Returns 1.0 for a pack held at exactly 25 C, >1 hotter, <1 cooler.

    Because the weighting is convex, brief hot excursions cost more than their share of
    wall-clock time -- the gap between this figure and a naive 2**((mean_T-25)/D) is about
    +6% on that same window. Measured multipliers across five windows: 1.216, 1.209, 1.278,
    1.033, 0.920 -- i.e. self-discharge really does vary ~40% window to window, but at a
    2 %/month nominal that is only an ~31 uA swing against a ~650 uA spread in the measured
    totals, so it is a refinement, not the explanation for that spread.
    """
    if len(samples) < 2:
        return 1.0
    weighted = 0.0
    total_s = 0.0
    smoothed = samples[0][1]
    for (stamp, temp_c), (next_stamp, _) in zip(samples, samples[1:]):
        dwell_s = (next_stamp - stamp).total_seconds()
        # A gap this long means the export dropped out; carrying the old value across it
        # would invent temperature history that was never recorded.
        if not 0.0 < dwell_s < 7200.0:
            continue
        if tau_h > 0.0:
            smoothed += (1.0 - math.exp(-(dwell_s / 3600.0) / tau_h)) * (temp_c - smoothed)
        else:
            smoothed = temp_c
        weighted += 2.0 ** ((smoothed - SELF_DISCHARGE_REFERENCE_C) / SELF_DISCHARGE_DOUBLING_K) * dwell_s
        total_s += dwell_s
    return weighted / total_s if total_s else 1.0


def self_discharge_ua(temperature_multiplier=1.0):
    """Pack self-discharge in uA, or 0.0 while SELF_DISCHARGE_PCT_PER_MONTH_AT_25C is None."""
    if SELF_DISCHARGE_PCT_PER_MONTH_AT_25C is None:
        return 0.0
    hours_per_month = 730.5
    at_25c_ua = PACK_CAPACITY_MAH * SELF_DISCHARGE_PCT_PER_MONTH_AT_25C / 100.0 / hours_per_month * 1000.0
    return at_25c_ua * temperature_multiplier


def model_avg_current_ua(window_s, tx_s, rx_s, cpu_s, cpu_current_ma=None, sensor_samples=1,
                         lp_poll_period_sec=None, tx_power_dbm=None):
    """Average current (uA) over a whole window, from that window's TOTAL phase durations.

    Totals, not a per-cycle reconstruction, because totals are the form the measurement is in: a
    battery-decay result is sum(charge)/T, and matching the forms removes a class of error rather
    than bounding it. It also stops "cadence" -- which on this device is a weather-dependent
    mixture of safeguard-timer and threshold-triggered publishes, not a setting -- from being an
    input at all. cadence_current_ua() below is the per-cycle front end for hand use; the two are
    identical when `cadence := T/n` and `phase_times := totals/n` use the SAME n.

    Raises when the awake totals exceed the window. In the per-cycle form that was a pessimistic
    edge case worth clamping to zero sleep; in the totals form it is an extraction bug, and
    silently flooring it would hide the thing most worth seeing.

    lp_poll_period_sec is the LP core's real wake-to-wake period -- what heater_run_count measures
    -- not the timer's sleep duration. Left None it falls back to the period predicted from
    device_config.yaml's 20 s sleep at this sensor_samples.
    """
    if lp_poll_period_sec is None:
        lp_poll_period_sec = lp_poll_period_s(sensor_samples)
    if cpu_current_ma is None:
        cpu_current_ma = (CPU_CURRENT_MA_LOW + CPU_CURRENT_MA_HIGH) / 2
    awake_s = tx_s + rx_s + cpu_s
    sleep_s = window_s - awake_s
    if sleep_s < 0.0:
        raise ValueError(
            f"awake total {awake_s:.3f} s exceeds the {window_s:.3f} s window by "
            f"{-sleep_s:.3f} s -- that is an extraction bug, not a pessimistic edge case")
    awake_charge_ua_s = (
        tx_s * tx_current_ma(tx_power_dbm)
        + rx_s * RX_CURRENT_MA
        + cpu_s * cpu_current_ma
    ) * 1000  # mA*s -> uA*s
    sleep_contribution_ua = SLEEP_CURRENT_UA * sleep_s / window_s
    awake_contribution_ua = awake_charge_ua_s / window_s
    extra_ua = sensor_samples_extra_ua(sensor_samples, lp_poll_period_sec)
    # NB: self-discharge is deliberately NOT added here. This function models what the DEVICE
    # draws; self-discharge is the pack losing charge on its own. They are only summed when
    # comparing against a voltage-decay measurement, which sees both -- power_budget() does that
    # explicitly so the distinction stays visible.
    return sleep_contribution_ua + awake_contribution_ua + extra_ua


def cadence_current_ua(cadence_s, cpu_current_ma=None, sensor_samples=1, lp_poll_period_sec=None,
                       tx_power_dbm=None, phase_times_s=None):
    """Per-cycle front end for model_avg_current_ua: one publish every cadence_s seconds.

    Exactly the totals form with a window of one cycle, which is what makes the two equivalent.
    Kept because a back-of-envelope run wants to say "one cycle every 290 s", and because every
    figure this project recorded before 2026-09-09 was quoted that way.

    phase_times_s overrides the per-cycle (tx, rx, cpu) durations, which otherwise use the
    2026-08-05/06 calibration. Real windows measure noticeably more RX -- see
    PHASE_TIMES_WINDOW_A_S -- so pass measured durations to reproduce a window's component budget.
    """
    tx_s, rx_s, cpu_s = phase_times_s or (TX_TIME_S, RX_TIME_S, CPU_TIME_S)
    return model_avg_current_ua(cadence_s, tx_s, rx_s, cpu_s, cpu_current_ma, sensor_samples,
                                lp_poll_period_sec, tx_power_dbm)


def runtime_days(avg_current_ua, capacity_mah=None):
    """Days to flatten the pack at a constant average current.

    Previously computed as PACK_ENERGY_MWH / (uA * 3.983 V), which mixed a 3.7 V nominal for the
    energy available with a 3.983 V figure for the power drawn and understated runtime by
    3.983/3.7 = 7.6%. Charge in, charge out -- no voltage belongs in this at all.
    """
    return (capacity_mah or PACK_CAPACITY_MAH) / (avg_current_ua / 1000.0) / 24.0


def device_total_ua(cadence_s, sensor_samples=1, lp_poll_period_sec=None, cpu_current_ma=None,
                    tx_power_dbm=None, phase_times_s=None):
    """Everything the device draws, BATTERY-SIDE, ready to compare against a decay measurement.

    The duty-cycle model converted across the buck, plus the board terms (which sit on the
    battery directly and are not converted).

    KNOWN OMISSION: the LP core's busy-wait after each SHT4x heater run -- kRhSettleMs (90 s)
    plus a cooldown poll loop, both ulp_lp_core_delay_us() spins at full LP power. At roughly
    one run a day and the guessed 9 mA that is about 11 uA rail / 10 uA battery, which this
    function does not include. Its duration has never been measured, only estimated at ~110 s.
    """
    rail_ua = cadence_current_ua(cadence_s, cpu_current_ma, sensor_samples, lp_poll_period_sec,
                                 tx_power_dbm, phase_times_s)
    return rail_to_battery_ua(rail_ua) + BOARD_FIXED_UA


def implied_self_discharge(measured_ua, device_ua, capacity_mah=None):
    """Self-discharge as the residual of a voltage-decay measurement, in (uA, %/month).

    Returns a NEGATIVE current when the assumed capacity is too low for the device terms -- see
    capacity_floor(). The residual absorbs every error anywhere else in the budget and is
    amplified by measured/residual (about 6x on the 2026-08-29 window), so a plausible-looking
    answer here is weak evidence, not confirmation.
    """
    capacity_mah = capacity_mah or PACK_CAPACITY_MAH
    residual_ua = measured_ua - device_ua
    hours_per_month = 730.5
    pct_per_month = residual_ua / 1000.0 * hours_per_month / capacity_mah * 100.0
    return residual_ua, pct_per_month


def scale_measurement(measured_ua_at_nominal, capacity_mah, nominal_mah=None):
    """Restate a voltage-decay result under a different assumed capacity (it scales linearly)."""
    return measured_ua_at_nominal * capacity_mah / (nominal_mah or PACK_CAPACITY_MAH)


def capacity_floor(measured_ua_at_nominal, device_ua, self_discharge_floor_pct=0.0,
                   nominal_mah=None):
    """Smallest assumed capacity that keeps the budget self-consistent, in mAh.

    Solves  measured(C) - device >= C * floor_pct  for C, where measured(C) scales with C and
    device does not. Below the returned value the device alone draws more than the pack
    demonstrably lost, which is impossible rather than merely pessimistic.

    With self_discharge_floor_pct=0 this is the hard arithmetic wall; raising it to a plausible
    minimum self-discharge tightens it. Returns inf when even an infinite pack cannot satisfy the
    constraint -- which is itself informative: it means a pinned guess is too high, or the
    measurement is too low, or the assumed self-discharge floor is wrong.
    """
    nominal_mah = nominal_mah or PACK_CAPACITY_MAH
    slope = measured_ua_at_nominal / nominal_mah - self_discharge_floor_pct / 73.05
    return device_ua / slope if slope > 0 else float("inf")


# --- regression anchor -----------------------------------------------------------------
# What "unchanged behaviour" MEANS for this module, so a refactor has something to be checked
# against. Until this table existed the module had no such statement, and the only figures that
# looked like one -- the four calibration points in SLEEP_CURRENT_UA's comment -- silently assumed
# 6 dBm while the default moved to 20, so re-running them read as a regression.
#
# Hence: TX POWER IS NAMED ON EVERY ROW. Every other input is named too. A row that stops
# reproducing is a bug until someone proves otherwise and re-blesses it in the same commit that
# moves it, next to the reason.
#
# Per-cycle phase durations measured over the two reconciled windows, forward-filled onto the
# cycle anchor, NOT the mean of the surviving rows -- HA de-duplication makes those a biased
# sample of cycles (see docs/plan_ha_log_metrics.md section 1.5). ha_log_metrics.py derives these
# and its test suite asserts they still match this pair.
PHASE_TIMES_WINDOW_A_S = (0.09901, 0.36425, 0.49826)  # 2026-08-29 -> 09-07, sensor_samples=4
PHASE_TIMES_WINDOW_B_S = (0.09857, 0.31676, 0.50241)  # 2026-08-20 -> 08-29, sensor_samples=16

# label, cadence_s, sensor_samples, lp_poll_period_sec, tx_power_dbm, phase_times_s,
# then the expected mid / low / high model_avg_current_ua and device_total_ua, all in uA.
#
# The period column is the REAL wake-to-wake period. The windows A and B rows carry the values
# heater_run_count measured (20.3442 / 21.7868 s); everything else carries the period implied by
# device_config.yaml's 20 s sleep at that sensor_samples. When this table was first written the
# column held sleep durations (20.0000 / 20.0002 / 20.1108) -- the uA figures are unchanged,
# because period = sleep + lp_active_time_s(N) and that is exactly the conversion.
REGRESSION_TABLE = (
    ("historic calibration", 290.00, 1, 20.0110, 6.0, None, 189.4, 180.7, 198.2, 186.3),
    ("historic calibration", 600.00, 1, 20.0110, 6.0, None, 109.6, 105.4, 113.9, 115.0),
    ("historic calibration", 900.00, 1, 20.0110, 6.0, None, 84.8, 81.9, 87.6, 92.8),
    ("historic calibration", 1800.00, 1, 20.0110, 6.0, None, 59.9, 58.5, 61.3, 70.6),
    ("current default", 290.00, 1, 20.0110, 20.0, None, 236.6, 227.9, 245.3, 228.4),
    ("current default", 1800.00, 1, 20.0110, 20.0, None, 67.5, 66.1, 68.9, 77.4),
    ("window A (ss=4)", 234.34, 4, 20.3442, 20.0, PHASE_TIMES_WINDOW_A_S, 494.5, 482.8, 506.2, 458.7),
    ("window B (ss=16)", 239.15, 16, 21.7868, 20.0, PHASE_TIMES_WINDOW_B_S, 1011.6, 1000.1, 1023.2, 920.5),
    ("LP term off", 234.34, 1, 20.0112, 20.0, PHASE_TIMES_WINDOW_A_S, 347.8, 336.2, 359.5, 327.7),
)

# Supporting invariants, kept beside the table because they are the terms it is most sensitive to.
# sensor_samples_extra_ua is 30% of window A's modelled rail current and 68% of window B's, and
# rests entirely on the LP_ACTIVE_CURRENT_UA guess -- so if anything here drifts, it is this.
# (label, sensor_samples, lp_poll_period_sec, expected_extra_ua, expected_lp_poll_interval_s)
REGRESSION_INVARIANTS = (
    ("window A (ss=4)", 4, 20.3442, 146.66, 20.0002),
    ("window B (ss=16)", 16, 21.7868, 684.75, 20.1108),
    ("N=1 contributes nothing", 1, 20.0110, 0.00, 20.0000),
)


def check_regression_table(tolerance_ua=0.05, tolerance_s=0.0001):
    """Recompute REGRESSION_TABLE and REGRESSION_INVARIANTS; return a list of complaint strings.

    Empty list means the module still reproduces its anchor. Deliberately returns complaints
    rather than raising or printing, so callers can decide (--check-regression prints them, a
    test asserts on them).
    """
    complaints = []
    for row in REGRESSION_TABLE:
        label = row[0]
        cadence_s = row[1]
        sensor_samples = row[2]
        lp_poll_period_sec = row[3]
        tx_power_dbm = row[4]
        phase_times_s = row[5]
        expected = row[6:]
        actual = (
            cadence_current_ua(cadence_s, None, sensor_samples, lp_poll_period_sec,
                               tx_power_dbm, phase_times_s),
            cadence_current_ua(cadence_s, CPU_CURRENT_MA_LOW, sensor_samples,
                               lp_poll_period_sec, tx_power_dbm, phase_times_s),
            cadence_current_ua(cadence_s, CPU_CURRENT_MA_HIGH, sensor_samples,
                               lp_poll_period_sec, tx_power_dbm, phase_times_s),
            device_total_ua(cadence_s, sensor_samples, lp_poll_period_sec, None,
                            tx_power_dbm, phase_times_s),
        )
        names = ("mid", "low", "high", "device_total")
        for name, want, got in zip(names, expected, actual):
            if abs(got - want) > tolerance_ua:
                complaints.append(
                    f"{label} @ {cadence_s:g}s N={sensor_samples} {tx_power_dbm:g}dBm: "
                    f"{name} expected {want:.1f} uA, got {got:.1f} uA")

    for label, sensor_samples, lp_poll_period_sec, want_ua, want_interval_s in REGRESSION_INVARIANTS:
        got_ua = sensor_samples_extra_ua(sensor_samples, lp_poll_period_sec)
        if abs(got_ua - want_ua) > tolerance_ua:
            complaints.append(f"{label}: sensor_samples_extra_ua expected {want_ua:.2f} uA, "
                              f"got {got_ua:.2f} uA")
        got_interval_s = lp_poll_interval_s(sensor_samples, lp_poll_period_sec)
        if abs(got_interval_s - want_interval_s) > tolerance_s:
            complaints.append(f"{label}: lp_poll_interval_s expected {want_interval_s:.4f} s, "
                              f"got {got_interval_s:.4f} s")
        round_trip_s = lp_poll_period_s(sensor_samples, got_interval_s)
        if abs(round_trip_s - lp_poll_period_sec) > tolerance_s:
            complaints.append(f"{label}: period/interval round trip lost "
                              f"{abs(round_trip_s - lp_poll_period_sec):.6f} s")
    return complaints


def print_regression_table():
    """Print REGRESSION_TABLE recomputed, with a pass/fail verdict."""
    print("Regression anchor (see check_regression_table)")
    print(f"  {'case':22s}{'cadence':>9s}{'N':>3s}{'period':>10s}{'dBm':>5s}"
          f"{'mid':>9s}{'low':>9s}{'high':>9s}{'battery':>9s}")
    for row in REGRESSION_TABLE:
        mid = cadence_current_ua(row[1], None, row[2], row[3], row[4], row[5])
        low = cadence_current_ua(row[1], CPU_CURRENT_MA_LOW, row[2], row[3], row[4], row[5])
        high = cadence_current_ua(row[1], CPU_CURRENT_MA_HIGH, row[2], row[3], row[4], row[5])
        total = device_total_ua(row[1], row[2], row[3], None, row[4], row[5])
        print(f"  {row[0]:22s}{row[1]:9.2f}{row[2]:3d}{row[3]:10.4f}{row[4]:5.0f}"
              f"{mid:9.1f}{low:9.1f}{high:9.1f}{total:9.1f}")
    print("  mid/low/high are model_avg_current_ua on the 3V3 rail; battery is device_total_ua.")
    complaints = check_regression_table()
    if complaints:
        print(f"  FAILED -- {len(complaints)} deviation(s):")
        for complaint in complaints:
            print(f"    {complaint}")
    else:
        print("  OK -- all rows and invariants reproduce.")
    return complaints


Budget = namedtuple("Budget", "cadence_s sensor_samples lp_poll_period_sec lp_poll_interval_sec "
                              "tx_power_dbm phase_times_s capacity_mah mid_ua low_ua high_ua "
                              "extra_ua tx_current_ma tx_spread_ma tx_spread_ua battery_ua "
                              "board_ua device_ua self_discharge_ua temperature_multiplier "
                              "runtime_days measured_ua scaled_measured_ua residual_ua "
                              "residual_pct_per_month capacity_floors")

CAPACITY_FLOOR_STEPS_PCT = (0.0, 0.35, 1.0, 2.0)


def power_budget(cadence_s, sensor_samples=1, lp_poll_period_sec=None, temperature_multiplier=1.0,
                 capacity_mah=None, measured_ua=None, tx_power_dbm=None, phase_times_s=None):
    """Compute the whole budget and return it, printing nothing.

    Split out of summarize() so a caller can reconcile against these numbers directly instead of
    scraping them back out of 40 lines of text -- which is what ha_log_metrics.py's --run does.
    print_budget() below is the other half; summarize() is both, kept for the CLI.
    """
    if lp_poll_period_sec is None:
        lp_poll_period_sec = lp_poll_period_s(sensor_samples)
    capacity_mah = capacity_mah or PACK_CAPACITY_MAH
    mid = cadence_current_ua(cadence_s, None, sensor_samples, lp_poll_period_sec,
                             tx_power_dbm, phase_times_s)
    low = cadence_current_ua(cadence_s, CPU_CURRENT_MA_LOW, sensor_samples, lp_poll_period_sec,
                             tx_power_dbm, phase_times_s)
    high = cadence_current_ua(cadence_s, CPU_CURRENT_MA_HIGH, sensor_samples, lp_poll_period_sec,
                              tx_power_dbm, phase_times_s)
    tx_s = (phase_times_s or (TX_TIME_S, RX_TIME_S, CPU_TIME_S))[0]
    spread_ma = tx_current_uncertainty_ma(tx_power_dbm)
    battery_ua = rail_to_battery_ua(mid)
    device_ua = battery_ua + BOARD_FIXED_UA

    scaled = None
    residual_ua = None
    residual_pct = None
    floors = ()
    if measured_ua is not None:
        scaled = scale_measurement(measured_ua, capacity_mah)
        residual_ua, residual_pct = implied_self_discharge(scaled, device_ua, capacity_mah)
        floors = tuple((floor_pct, capacity_floor(measured_ua, device_ua, floor_pct))
                       for floor_pct in CAPACITY_FLOOR_STEPS_PCT)

    return Budget(
        cadence_s=cadence_s,
        sensor_samples=sensor_samples,
        lp_poll_period_sec=lp_poll_period_sec,
        lp_poll_interval_sec=lp_poll_interval_s(sensor_samples, lp_poll_period_sec),
        tx_power_dbm=TX_POWER_DBM if tx_power_dbm is None else tx_power_dbm,
        phase_times_s=phase_times_s or (TX_TIME_S, RX_TIME_S, CPU_TIME_S),
        capacity_mah=capacity_mah,
        mid_ua=mid, low_ua=low, high_ua=high,
        extra_ua=sensor_samples_extra_ua(sensor_samples, lp_poll_period_sec),
        tx_current_ma=tx_current_ma(tx_power_dbm),
        tx_spread_ma=spread_ma,
        tx_spread_ua=spread_ma * tx_s * 1000 / cadence_s,
        battery_ua=battery_ua,
        board_ua=BOARD_FIXED_UA,
        device_ua=device_ua,
        self_discharge_ua=self_discharge_ua(temperature_multiplier),
        temperature_multiplier=temperature_multiplier,
        runtime_days=runtime_days(mid, capacity_mah),
        measured_ua=measured_ua,
        scaled_measured_ua=scaled,
        residual_ua=residual_ua,
        residual_pct_per_month=residual_pct,
        capacity_floors=floors,
    )


def print_budget(budget):
    """Print a Budget. Pure presentation -- every number here came from power_budget()."""
    print(f"Cycle cadence: {budget.cadence_s:.1f} s   pack {budget.capacity_mah:.0f} mAh")
    if budget.sensor_samples != 1:
        print(f"Sensor samples: {budget.sensor_samples} "
              f"(poll period {budget.lp_poll_period_sec:.3f} s "
              f"= {budget.lp_poll_interval_sec:.3f} s sleep "
              f"+ {lp_active_time_s(budget.sensor_samples) * 1000:.0f} ms work)")
        if LP_ACTIVE_CURRENT_UA is None:
            print("  extra current from sensor_samples: NOT MODELED "
                  "(LP_ACTIVE_CURRENT_UA uncalibrated)")
        else:
            print(f"  of which sensor_samples extra: {budget.extra_ua:.1f} uA "
                  f"(LP_ACTIVE_CURRENT_UA = {LP_ACTIVE_CURRENT_UA / 1000:.1f} mA is a GUESS -- "
                  f"see its #todo)")
    print(f"Average current: {budget.mid_ua:.1f} uA "
          f"(range {budget.low_ua:.1f}-{budget.high_ua:.1f} uA)")
    print(f"Average power: {budget.mid_ua * PACK_MEAN_VOLTAGE / 1000:.3f} mW")
    print(f"Implied full-pack runtime: {budget.runtime_days:.1f} days")

    if SELF_DISCHARGE_PCT_PER_MONTH_AT_25C is None:
        print("Pack self-discharge: NOT MODELED (SELF_DISCHARGE_PCT_PER_MONTH_AT_25C "
              "uncalibrated; literature 1-3 %/month = 44-131 uA on this pack)")
        print("  -> a voltage-decay measurement sees device + self-discharge together, so it "
              "will read HIGHER than the figure above.")
    else:
        print(f"Pack self-discharge: {budget.self_discharge_ua:.1f} uA "
              f"({SELF_DISCHARGE_PCT_PER_MONTH_AT_25C:.1f} %/month at 25 C "
              f"x {budget.temperature_multiplier:.3f} temperature multiplier)")
        print(f"Device + self-discharge: {budget.mid_ua + budget.self_discharge_ua:.1f} uA "
              "(this is what a voltage-decay measurement should see)")

    print(f"TX power: {budget.tx_power_dbm:.0f} dBm -> {budget.tx_current_ma:.0f} mA")
    if budget.tx_spread_ma > 0.0:
        # Not a published anchor, so the number above is an interpolation whose basis is worth
        # something -- say how much rather than letting it read like a datasheet value.
        print(f"  NB: interpolated, not a datasheet point -- basis uncertainty "
              f"+/-{budget.tx_spread_ma:.0f} mA = +/-{budget.tx_spread_ua:.1f} uA at this cadence "
              f"(see tx_current_uncertainty_ma)")
    print(f"Across the buck: {budget.mid_ua:.1f} uA on 3V3 -> {budget.battery_ua:.1f} uA at the "
          f"battery (x{RAIL_VOLTAGE / PACK_MEAN_VOLTAGE / BUCK_EFFICIENCY:.3f})")
    print(f"Board fixed terms: {budget.board_ua:.1f} uA (battery divider + static leakage)")
    print(f"Device total, battery-side: {budget.device_ua:.1f} uA")

    if budget.measured_ua is None:
        return
    print()
    print(f"Reconciliation against a measured {budget.measured_ua:.1f} uA (quoted at "
          f"{PACK_CAPACITY_MAH:.0f} mAh):")
    if budget.capacity_mah != PACK_CAPACITY_MAH:
        print(f"  restated at {budget.capacity_mah:.0f} mAh: {budget.scaled_measured_ua:.1f} uA")
    print(f"  self-discharge residual: {budget.residual_ua:.1f} uA = "
          f"{budget.residual_pct_per_month:.2f} %/month")
    if budget.residual_ua < 0:
        print("  NEGATIVE -- this capacity is arithmetically impossible, not just pessimistic.")
    print(f"  measured runtime: "
          f"{runtime_days(budget.scaled_measured_ua, budget.capacity_mah):.1f} days "
          "(capacity-invariant -- the assumption cancels)")
    print("  smallest self-consistent capacity:")
    for floor_pct, floor_mah in budget.capacity_floors:
        label = "residual >= 0" if floor_pct == 0 else f"self-discharge >= {floor_pct:.2f} %/mo"
        if floor_mah == float("inf") or floor_mah > PACK_CAPACITY_MAH:
            print(f"    {label:28s} -> no solution at or below nominal")
        else:
            print(f"    {label:28s} -> {floor_mah:6.0f} mAh "
                  f"({(1 - floor_mah / PACK_CAPACITY_MAH) * 100:.1f} % degradation)")


def summarize(cadence_s, sensor_samples=1, lp_poll_period_sec=None, temperature_multiplier=1.0,
              capacity_mah=None, measured_ua=None, tx_power_dbm=None, phase_times_s=None):
    """Compute and print the budget. Thin wrapper; callers wanting the numbers use power_budget."""
    budget = power_budget(cadence_s, sensor_samples, lp_poll_period_sec, temperature_multiplier,
                          capacity_mah, measured_ua, tx_power_dbm, phase_times_s)
    print_budget(budget)
    return budget


def _phase_times(text):
    """Parse a "tx,rx,cpu" trio of per-cycle durations in seconds."""
    parts = [p.strip() for p in text.split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("expected three comma-separated seconds: tx,rx,cpu")
    try:
        return tuple(float(p) for p in parts)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"not a number: {exc}") from exc


def _build_parser():
    parser = argparse.ArgumentParser(
        prog="power_model.py",
        description="Duty-cycle power model for the OpenThread sleepy sensor.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "examples:\n"
            "  power_model.py --cadence 290 --tx-power 6\n"
            "  power_model.py --cadence 234.34 --sensor-samples 4 --lp-poll-period 20.3442 \\\n"
            "                 --phase-times 0.09901,0.36425,0.49826 --measured 600\n"
            "  power_model.py --cadence 239.15 --sensor-samples 16 --lp-poll-period 21.7868 \\\n"
            "                 --phase-times 0.09857,0.31676,0.50241 --measured 1253\n"
            "\n"
            "ha_log_metrics.py derives all of those from an HA export and prints the command.\n"
        ),
    )
    parser.add_argument("--cadence", type=float, default=290.0, metavar="SECONDS",
                        help="publish cycle period in seconds (default: %(default)s). A "
                             "convenience: the model's primitive is window totals, and "
                             "--window/--totals/--cycles below express a window directly")
    parser.add_argument("--window", type=float, default=None, metavar="SECONDS",
                        help="measurement window length; with --totals and --cycles this "
                             "replaces --cadence/--phase-times, and is the form a decay "
                             "measurement is actually in")
    parser.add_argument("--totals", type=_phase_times, default=None, metavar="TX,RX,CPU",
                        help="whole-window phase totals in seconds, to pair with --window")
    parser.add_argument("--cycles", type=int, default=None, metavar="N",
                        help="successful publishes in the window; only used to restate --window "
                             "and --totals as the per-cycle figures this prints")
    parser.add_argument("--sensor-samples", type=int, default=1, metavar="N",
                        help="SHT4x readings averaged per LP poll, device_config.yaml's "
                             "sensor_samples (default: %(default)s)")
    parser.add_argument("--lp-poll-period", type=float, default=None, metavar="SECONDS",
                        help="LP core wake-to-wake period, as heater_run_count measures it "
                             "(default: predicted from device_config.yaml's 20 s sleep at this "
                             "--sensor-samples). NB this REPLACED --lp-poll-interval, which took "
                             "the timer's sleep duration instead; pass sleep + work here")
    parser.add_argument("--tx-power", type=float, default=None, metavar="DBM",
                        help=f"radio TX power (default: {TX_POWER_DBM:g}, the device's live setting)")
    parser.add_argument("--capacity", type=float, default=None, metavar="MAH",
                        help=f"assumed pack capacity (default: {PACK_CAPACITY_MAH:g}); affects the "
                             "measurement side only, never the device model")
    parser.add_argument("--measured", type=float, default=None, metavar="UA",
                        help="measured average current from a voltage-decay run, quoted at the "
                             "nominal capacity; enables the reconciliation block")
    parser.add_argument("--temperature-multiplier", type=float, default=1.0, metavar="X",
                        help="Arrhenius self-discharge multiplier vs 25 C, from "
                             "arrhenius_multiplier() (default: %(default)s)")
    parser.add_argument("--list-tx-table", action="store_true",
                        help="print TX current for every integer dBm the runtime knob accepts, "
                             "with each row's provenance and interpolation-basis uncertainty, "
                             "then exit")
    parser.add_argument("--check-regression", action="store_true",
                        help="recompute REGRESSION_TABLE and its invariants, print them with a "
                             "verdict, then exit 0 if unchanged or 1 if any row moved")
    parser.add_argument("--phase-times", type=_phase_times, default=None, metavar="TX,RX,CPU",
                        help="explicit per-cycle phase durations in seconds; ha_log_metrics.py "
                             "derives these from a window (NB --measured-phases is gone -- it "
                             "held a frozen, and wrong, snapshot of one window)")
    return parser


if __name__ == "__main__":
    args = _build_parser().parse_args()
    if args.list_tx_table:
        print_tx_current_table()
        raise SystemExit(0)
    if args.check_regression:
        raise SystemExit(1 if print_regression_table() else 0)

    cadence_s = args.cadence
    phase_times_s = args.phase_times
    if args.window is not None:
        if args.totals is None or not args.cycles:
            raise SystemExit("--window needs --totals and --cycles")
        # Section 1.6's equivalence, applied once and in the only place it belongs: both
        # definitions must divide by the SAME cycle count or the two forms silently disagree.
        cadence_s = args.window / args.cycles
        phase_times_s = tuple(total / args.cycles for total in args.totals)
    elif args.totals is not None:
        raise SystemExit("--totals needs --window")

    try:
        summarize(
            cadence_s,
            sensor_samples=args.sensor_samples,
            lp_poll_period_sec=args.lp_poll_period,
            temperature_multiplier=args.temperature_multiplier,
            capacity_mah=args.capacity,
            measured_ua=args.measured,
            tx_power_dbm=args.tx_power,
            phase_times_s=phase_times_s,
        )
    except ValueError as exc:
        # The awake-exceeds-window check. A message, not a traceback: the input is wrong, not the
        # program.
        raise SystemExit(f"power_model.py: {exc}") from exc
