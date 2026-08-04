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

    tools/battery_power_from_history.py
"""
import csv
import statistics
from datetime import datetime

CSV_PATH = "voltage_history.csv"

# Etalon (reference) resting-voltage discharge curve of a textbook 4.20 V/cell 1S
# Li-ion, copied from main/sensorstask.h's BATTERY_CURVE.
BATTERY_CURVE = [
    (3270, 0), (3610, 5), (3690, 10), (3710, 15), (3730, 20), (3750, 25),
    (3770, 30), (3790, 35), (3800, 40), (3820, 45), (3840, 50), (3850, 55),
    (3870, 60), (3910, 65), (3950, 70), (3980, 75), (4020, 80), (4080, 85),
    (4110, 90), (4150, 95), (4200, 100),
]

PACK_CAPACITY_MAH = 3200
PACK_MEAN_VOLTAGE = 3.7
PACK_ENERGY_MWH = PACK_CAPACITY_MAH * PACK_MEAN_VOLTAGE # 11840  # 3200 mAh * 3.7 V nominal


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


readings = []
with open(CSV_PATH, newline="") as f:
    for row in csv.DictReader(f):
        try:
            voltage = float(row["state"])
        except ValueError:
            continue  # e.g. "unavailable"
        timestamp = datetime.fromisoformat(row["last_changed"].replace("Z", "+00:00"))
        readings.append((timestamp, voltage))
readings.sort(key=lambda r: r[0])

t0 = readings[0][0]
hours = [(ts - t0).total_seconds() / 3600.0 for ts, _ in readings]
voltages = [v for _, v in readings]
percents = [voltage_to_percent(v * 1000) for v in voltages]
total_hours = hours[-1]

n = len(hours)
slope, intercept = statistics.linear_regression(hours, percents)

pct_start = intercept
pct_end = intercept + slope * total_hours
pct_delta = pct_start - pct_end

mean_v = statistics.mean(voltages)
mwh_used = pct_delta / 100.0 * PACK_ENERGY_MWH
mean_mw = mwh_used / total_hours
mean_ma = mean_mw / mean_v
runtime_days = PACK_ENERGY_MWH / mean_mw / 24.0

print(f"Samples: {n}")
print(f"Window: {t0.isoformat()} -> {readings[-1][0].isoformat()} ({total_hours:.2f} h)")
print(f"Voltage: {voltages[0]:.3f} V -> {voltages[-1]:.3f} V (mean {mean_v:.3f} V)")
print(f"Battery %: {pct_start:.2f}% -> {pct_end:.2f}% (fitted, delta {pct_delta:.2f} pp)")
print(f"Energy used: {mwh_used:.2f} mWh")
print(f"Average power: {mean_mw:.3f} mW")
print(f"Average current: {mean_ma:.3f} mA")
print(f"Implied full-pack runtime at this rate: {runtime_days:.1f} days")
