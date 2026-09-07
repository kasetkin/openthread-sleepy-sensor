#!/usr/bin/env python3
"""Reconciled duty-cycle power model for the sleepy sensor, parameterized by cycle cadence.

Formalizes the "Full reconciled power model" from the 2026-08-05 light-sleep power
investigation (see project_light_sleep_power_investigation memory) as code instead of
one-off prose math, and generalizes it from a single hardcoded 290s example to any
cadence. Per-cycle awake-phase durations are treated as fixed (independent of cadence) --
validated by the 2026-08-14->08-18 measurement, where a ~172s actual cadence (vs the
~290s this model was calibrated against) still showed a normal per-cycle hp_awake_time,
confirming cadence changes wall-clock frequency, not per-cycle work.

    tools/power_model.py [cadence_s] [sensor_samples] [lp_poll_interval_sec]
"""
import sys

# Per-cycle awake-phase durations (seconds) and currents (mA), current conditions (6 dBm TX).
# Sources: 2026-08-05 5-phase breakdown + radio_tx_time/radio_rx_time (see memory above).
TX_TIME_S = 0.090
TX_CURRENT_MA = 153.0  # interpolated from C6 datasheet Table 5-9 at 6 dBm

RX_TIME_S = 0.217
RX_CURRENT_MA = 74.0  # C6 datasheet Table 5-9, fixed regardless of TX power

CPU_TIME_S = 0.461  # CPU-active, radio-idle residual (mostly mqtt_state_publish ACK wait)
CPU_CURRENT_MA_LOW = 27.0  # C6 datasheet Table 5-10, clocks-disabled
CPU_CURRENT_MA_HIGH = 38.0  # C6 datasheet Table 5-10, clocks-enabled

SLEEP_CURRENT_UA = 28.0  # back-solved light-sleep baseline (not the datasheet's 180uA figure)

# Multi-sample averaging (components/lp_sensor_core/lp_core/main.cpp's measureAveraged(),
# device_config.yaml's sensor_samples). SLEEP_CURRENT_UA above was back-solved from real
# measurements taken when the LP core did exactly one read per poll (sensor_samples=1), so it
# already absorbs that much LP-active time -- hence sensor_samples_extra_ua() below models the
# DELTA against N=1, not the whole LP-active term.
#
# Per-poll timing mirrors lp_core/main.cpp: each measure() costs kMeasureDelayUs (10ms) plus its
# two LP_I2C transactions (~1ms), and measureAveraged() inserts kInterSampleDelayUs (100ms)
# between samples but not before the first. Both are ulp_lp_core_delay_us() busy-waits (a plain
# cycle-counter spin, see that file's header comment), so this is genuinely CPU-active time, not
# sleep. Keep in sync if either constant changes there.
LP_MEASURE_S = 0.011
LP_INTER_SAMPLE_S = 0.100
LP_EXTRA_S_PER_SAMPLE = LP_MEASURE_S + LP_INTER_SAMPLE_S  # cost of one sample beyond the first
# UNVERIFIED, deliberately None rather than a guessed number: this project has no hardware
# measurement of the LP core's own active current, and the ESP32-C6 datasheet's Table 5-11
# (Low-Power Modes) has no "LP CPU actively executing" row either -- only Light-sleep/Deep-
# sleep/RTC-timer-only/power-off, all bundled into SLEEP_CURRENT_UA above for N=1. Rather than
# invent a number that would look like a real measurement, model_avg_current_ua() below skips
# the extra-current term entirely (and summarize() says so) until this is back-solved from a
# real sensor_samples>1 window the same way SLEEP_CURRENT_UA was (see the sensor-samples-
# averaging plan's Verification section).
#
# Best estimate so far (2026-09-07, still NOT written in here): 9.4-10.1 mA, from differencing the
# ss=16 and ss=4 windows -- the spread is the unexplained ss=16 timing excess noted in
# lp_poll_period_s() propagating through. Three reasons it stays None:
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
# 3. The name would mislead: ~10 mA is ~10x what a 20 MHz LP RISC-V core should draw alone
#    (Table 5-10 puts the whole 160 MHz HP core at 27-38 mA), so whatever this coefficient captures
#    is WHOLE-CHIP current while the LP core is active -- LP_I2C clocking, the SHT4x mid-conversion,
#    and any power domains held up -- not the LP CPU's own draw. Rename it if it ever gets filled in.
#
# To actually calibrate: alternate sensor_samples 16->1->16->1 across four adjacent multi-day
# windows. Repeating the contrast lets the drifting residual be differenced out, which a single
# before/after pair structurally cannot do.
LP_ACTIVE_CURRENT_UA = None

PACK_CAPACITY_MAH = 3200
PACK_MEAN_VOLTAGE = 3.7
PACK_ENERGY_MWH = PACK_CAPACITY_MAH * PACK_MEAN_VOLTAGE


def lp_active_time_s(sensor_samples):
    """Wall-clock time the LP core stays awake per poll for `sensor_samples` readings."""
    samples = max(sensor_samples, 1)
    return samples * LP_MEASURE_S + (samples - 1) * LP_INTER_SAMPLE_S


def lp_poll_period_s(sensor_samples, lp_poll_interval_sec):
    """Real LP wake-to-wake period. lp_poll_interval_sec is the LP timer's SLEEP duration
    (lp_sensor_core.c passes it as ulp_lp_core_cfg_t::lp_timer_sleep_duration_us), so the LP core
    sleeps that long AFTER main() returns -- the period is sleep + work, and stretches with
    sensor_samples rather than staying fixed.

    Confirmed on hardware, independently of any battery data: heater_run_count advances once per
    exactly 4320 LP cycles (minutes_to_lp_cycles(1440, 20) in main/main.cpp), so heater wall-clock
    period / 4320 measures this directly. 2026-08-29->09-07 at sensor_samples=4 measured 20.3442 s
    vs 20.344 s predicted here -- 0.1%. The 2026-08-21->08-29 window at sensor_samples=16 measured
    21.7868 s vs 21.676 s predicted, i.e. +110.8 ms/poll (+6.6%) unexplained -- suspiciously close
    to exactly one extra sample-iteration, and no positive-coefficient linear model fits both
    points at once. Treat high sensor_samples predictions as a lower bound until that is resolved.
    """
    return lp_poll_interval_sec + lp_active_time_s(sensor_samples)


def sensor_samples_extra_ua(sensor_samples, lp_poll_interval_sec):
    """Extra average current (uA) from sensor_samples>1, or 0.0 if LP_ACTIVE_CURRENT_UA is
    still unverified (None) -- see its comment.

    Averaged over the LP poll period, not cadence_s: this recurs every LP poll, independent of the
    HP publish cadence. Since average current over one period is
    SLEEP_CURRENT_UA + (LP_ACTIVE_CURRENT_UA - SLEEP_CURRENT_UA) * duty, and SLEEP_CURRENT_UA was
    back-solved from sensor_samples=1 measurements, the term modeled here is the duty DELTA
    against N=1 -- otherwise the N=1 LP activity would be counted twice.
    """
    if LP_ACTIVE_CURRENT_UA is None:
        return 0.0

    def duty(samples):
        return lp_active_time_s(samples) / lp_poll_period_s(samples, lp_poll_interval_sec)

    return (LP_ACTIVE_CURRENT_UA - SLEEP_CURRENT_UA) * (duty(sensor_samples) - duty(1))


def model_avg_current_ua(cadence_s, cpu_current_ma=None, sensor_samples=1, lp_poll_interval_sec=20.0):
    """Average current (uA) for one publish cycle every cadence_s seconds."""
    if cpu_current_ma is None:
        cpu_current_ma = (CPU_CURRENT_MA_LOW + CPU_CURRENT_MA_HIGH) / 2
    awake_s = TX_TIME_S + RX_TIME_S + CPU_TIME_S
    awake_charge_ua_s = (
        TX_TIME_S * TX_CURRENT_MA
        + RX_TIME_S * RX_CURRENT_MA
        + CPU_TIME_S * cpu_current_ma
    ) * 1000  # mA*s -> uA*s
    sleep_s = max(cadence_s - awake_s, 0.0)
    sleep_contribution_ua = SLEEP_CURRENT_UA * sleep_s / cadence_s
    awake_contribution_ua = awake_charge_ua_s / cadence_s
    extra_ua = sensor_samples_extra_ua(sensor_samples, lp_poll_interval_sec)
    return sleep_contribution_ua + awake_contribution_ua + extra_ua


def summarize(cadence_s, sensor_samples=1, lp_poll_interval_sec=20.0):
    low = model_avg_current_ua(cadence_s, CPU_CURRENT_MA_LOW, sensor_samples, lp_poll_interval_sec)
    high = model_avg_current_ua(cadence_s, CPU_CURRENT_MA_HIGH, sensor_samples, lp_poll_interval_sec)
    mid = model_avg_current_ua(cadence_s, None, sensor_samples, lp_poll_interval_sec)
    mean_v = 3.983  # rough current pack voltage; only affects the mW/runtime conversion
    print(f"Cycle cadence: {cadence_s:.1f} s")
    if sensor_samples != 1:
        real_period = lp_poll_period_s(sensor_samples, lp_poll_interval_sec)
        print(f"Sensor samples: {sensor_samples} (poll sleep {lp_poll_interval_sec:.1f} s "
              f"+ {lp_active_time_s(sensor_samples) * 1000:.0f} ms work "
              f"= {real_period:.3f} s real period)")
        extra_ua = sensor_samples_extra_ua(sensor_samples, lp_poll_interval_sec)
        if LP_ACTIVE_CURRENT_UA is None:
            print(f"  extra current from sensor_samples: NOT MODELED (LP_ACTIVE_CURRENT_UA uncalibrated)")
        else:
            print(f"  of which sensor_samples extra: {extra_ua:.1f} uA")
    print(f"Average current: {mid:.1f} uA (range {low:.1f}-{high:.1f} uA)")
    print(f"Average power: {mid * mean_v / 1000:.3f} mW")
    runtime_days = PACK_ENERGY_MWH / (mid * mean_v / 1000) / 24.0
    print(f"Implied full-pack runtime: {runtime_days:.1f} days")


if __name__ == "__main__":
    cadence = float(sys.argv[1]) if len(sys.argv) > 1 else 290.0
    samples = int(sys.argv[2]) if len(sys.argv) > 2 else 1
    poll_interval = float(sys.argv[3]) if len(sys.argv) > 3 else 20.0
    summarize(cadence, samples, poll_interval)
