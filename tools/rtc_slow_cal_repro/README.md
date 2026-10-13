# RTC_SLOW light-sleep calibration reproducer

With automatic light sleep and the internal RC oscillator as RTC_SLOW, ESP-IDF calibrates RTC_SLOW
in `sleep_low_power_clock_calibration()` (`components/esp_hw_support/sleep_modes.c`), on the way
*into* each light sleep. At that moment the die is still warm from the active phase. The RC has a
strong negative temperature coefficient (about -1540 ppm/°C measured on an ESP32-C6), so it speeds
up as the die cools during the sleep. The sleep is then converted to time with a period that is too
long, and esp_timer and the RTC time run fast. The longer and hotter the active phase, the bigger
the error.

This app shows the effect against the host clock, and compares it with a reading taken right after
wake-up, while the die is still at its sleep temperature.

## What it does

- Automatic light sleep (`esp_pm_configure`, tickless idle), RTC_SLOW = internal RC, CPU fixed at
  160 MHz.
- Wakes every `REPRO_WAKE_PERIOD_MS` (default 1000 ms) and busy-loops the CPU for `REPRO_BUSY_MS`
  (default 50 ms) to warm the die, like a radio burst would.
- Every `REPRO_REPORT_EVERY` wakes (default 10) prints over UART0:

      CLK n=<wakes> t_us=<esp_timer_get_time()> mode=<stock|fix-emul|fix-idf> cal=<RTC_SLOW period, Q19 us>

- With `REPRO_EMULATE_CAL_AFTER_WAKEUP` (default on), the app wraps `rtc_clk_cal()` so that the
  light-sleep path uses one calibration taken right after the previous wake (≥ 50 ms of sleep)
  instead of measuring at entry. With `REPRO_MODE_SWITCH_PERIOD_S` (default 1800 s) it alternates
  between `stock` and `fix-emul`, so both modes run on the same board, at the same temperature,
  against the same reference, in one capture.

## Run it

Connect a USB-UART adapter to UART0 (on the Seeed XIAO ESP32C6: TX = D6 / GPIO16, plus GND). Don't
use USB-Serial-JTAG for the capture: it drops out in light sleep and can deliver buffered lines
late, which corrupts the timing.

```sh
idf.py -B build build flash          # flash over USB as usual
./clock_drift.py log /dev/ttyUSB0 -o rtc_cal.log    # leave running, ideally hours
./clock_drift.py fit rtc_cal.log     # any time, also on a copy while capturing
```

The capture host should be NTP-synced and must not suspend: the reference is its
`CLOCK_MONOTONIC`, which NTP slews but never steps. Positive ppm means the device clock runs fast.

`clock_drift.py` needs only Python 3.10+ and pyserial (PEP 723 header, so `uv run clock_drift.py`
works too).

## Options (`idf.py menuconfig` → RTC_SLOW calibration reproducer)

| Option | Default | Meaning |
|---|---|---|
| `REPRO_WAKE_PERIOD_MS` | 1000 | light sleep between wakes |
| `REPRO_BUSY_MS` | 50 | CPU busy time after each wake |
| `REPRO_REPORT_EVERY` | 10 | wakes per CLK line |
| `REPRO_EMULATE_CAL_AFTER_WAKEUP` | y | in-app emulation of the fix (turn off on an ESP-IDF that has it) |
| `REPRO_MODE_SWITCH_PERIOD_S` | 1800 | alternate stock / fix every N s, 0 = never |
| `REPRO_START_WITH_FIX` | n | mode at boot |
