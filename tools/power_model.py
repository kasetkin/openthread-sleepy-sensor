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
# already absorbs that much LP-active time -- this models only the EXTRA active time from
# samples beyond the first. LP_EXTRA_S_PER_SAMPLE mirrors lp_core/main.cpp's
# kInterSampleDelayUs (100ms) + kMeasureDelayUs (10ms); keep the two in sync if either changes.
LP_EXTRA_S_PER_SAMPLE = 0.110
# UNVERIFIED, deliberately None rather than a guessed number: this project has no hardware
# measurement of the LP core's own active current, and the ESP32-C6 datasheet's Table 5-11
# (Low-Power Modes) has no "LP CPU actively executing" row either -- only Light-sleep/Deep-
# sleep/RTC-timer-only/power-off, all bundled into SLEEP_CURRENT_UA above for N=1. Rather than
# invent a number that would look like a real measurement, model_avg_current_ua() below skips
# the extra-current term entirely (and summarize() says so) until this is back-solved from a
# real sensor_samples>1 window the same way SLEEP_CURRENT_UA was (see the sensor-samples-
# averaging plan's Verification section).
LP_ACTIVE_CURRENT_UA = None

PACK_CAPACITY_MAH = 3200
PACK_MEAN_VOLTAGE = 3.7
PACK_ENERGY_MWH = PACK_CAPACITY_MAH * PACK_MEAN_VOLTAGE


def sensor_samples_extra_ua(sensor_samples, lp_poll_interval_sec):
    """Extra average current (uA) from sensor_samples>1, or 0.0 if LP_ACTIVE_CURRENT_UA is
    still unverified (None) -- see its comment. Averaged over lp_poll_interval_sec, not
    cadence_s: this recurs every LP poll, independent of the HP publish cadence."""
    extra_samples = max(sensor_samples - 1, 0)
    if extra_samples == 0 or LP_ACTIVE_CURRENT_UA is None:
        return 0.0
    return LP_ACTIVE_CURRENT_UA * extra_samples * LP_EXTRA_S_PER_SAMPLE / lp_poll_interval_sec


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
        print(f"Sensor samples: {sensor_samples} (poll interval {lp_poll_interval_sec:.1f} s)")
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
