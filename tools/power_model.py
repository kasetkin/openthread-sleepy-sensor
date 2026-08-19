#!/usr/bin/env python3
"""Reconciled duty-cycle power model for the sleepy sensor, parameterized by cycle cadence.

Formalizes the "Full reconciled power model" from the 2026-08-05 light-sleep power
investigation (see project_light_sleep_power_investigation memory) as code instead of
one-off prose math, and generalizes it from a single hardcoded 290s example to any
cadence. Per-cycle awake-phase durations are treated as fixed (independent of cadence) --
validated by the 2026-08-14->08-18 measurement, where a ~172s actual cadence (vs the
~290s this model was calibrated against) still showed a normal per-cycle hp_awake_time,
confirming cadence changes wall-clock frequency, not per-cycle work.

    tools/power_model.py [cadence_s]
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

PACK_CAPACITY_MAH = 3200
PACK_MEAN_VOLTAGE = 3.7
PACK_ENERGY_MWH = PACK_CAPACITY_MAH * PACK_MEAN_VOLTAGE


def model_avg_current_ua(cadence_s, cpu_current_ma=None):
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
    return sleep_contribution_ua + awake_contribution_ua


def summarize(cadence_s):
    low = model_avg_current_ua(cadence_s, CPU_CURRENT_MA_LOW)
    high = model_avg_current_ua(cadence_s, CPU_CURRENT_MA_HIGH)
    mid = model_avg_current_ua(cadence_s)
    mean_v = 3.983  # rough current pack voltage; only affects the mW/runtime conversion
    print(f"Cycle cadence: {cadence_s:.1f} s")
    print(f"Average current: {mid:.1f} uA (range {low:.1f}-{high:.1f} uA)")
    print(f"Average power: {mid * mean_v / 1000:.3f} mW")
    runtime_days = PACK_ENERGY_MWH / (mid * mean_v / 1000) / 24.0
    print(f"Implied full-pack runtime: {runtime_days:.1f} days")


if __name__ == "__main__":
    cadence = float(sys.argv[1]) if len(sys.argv) > 1 else 290.0
    summarize(cadence)
