# RTC_SLOW light-sleep calibration reproducer

With automatic light sleep and the internal RC oscillator as RTC_SLOW, ESP-IDF calibrates RTC_SLOW
in `sleep_low_power_clock_calibration()` (`components/esp_hw_support/sleep_modes.c`), on the way
*into* each light sleep. At that moment the die is still warm from the active phase. The RC has a
strong negative temperature coefficient (about -1540 ppm/°C measured on an ESP32-C6), so it speeds
up as the die cools during the sleep. The sleep is then converted to time with a period that is too
long, and esp_timer and the RTC time run fast. The longer and hotter the active phase, the bigger
the error.

This app measures that error against the host clock. Built with an ESP-IDF that has
`CONFIG_PM_LIGHTSLEEP_RTC_OSC_CAL_AFTER_WAKEUP` ([espressif/esp-idf#19250](https://github.com/espressif/esp-idf/pull/19250)),
it measures the fix the same way.

## What it does

- Automatic light sleep (`esp_pm_configure`, tickless idle), RTC_SLOW = internal RC, CPU fixed at
  160 MHz, 1 kHz FreeRTOS tick.
- Sleeps a random time, log-uniform between `REPRO_SLEEP_MIN_MS` and `REPRO_SLEEP_MAX_MS`
  (default 1 ms to 10 s, so each decade gets a quarter of the sleeps). The sequence comes from a
  fixed seed, so every run sees the same sleeps. Or sleeps a fixed `REPRO_WAKE_PERIOD_MS`.
- After each wake it stays active for a fixed window, its length taken in turn from `k_busy_ms` in
  `main/rtc_slow_cal_repro.c` (10, 10, 200, 200 ms; `{200}` is the MIN_SLP_MS sweep of
  2026-10-20): it prints a report line first, then busy-loops to the end of the window to warm the
  die, as a radio burst would.
- Around every light sleep it takes two RTC_SLOW readings of its own, like ESP-IDF's (10 RC
  cycles): one just before the sleep (the stock entry reading) and one right after it (the
  post-wake reading). Both go into the report line, so `clock_drift.py --replay` can compute the
  clock error of any calibration policy (stock or any `MIN_SLP_MS`) from one run.
- The console UART is flushed before each light sleep (not suspended), so a line is never held
  back over the next sleep.

Output over UART0, a boot banner and then one line per wake:

    REPRO idf=<version> mode=<stock|fix-idf-<MIN_SLP_MS>> sleep_ms=1..10000 seed=0x19250 busy_ms=10,10,200,200 report_every=1
    C n=1342 t=6989573048 c=3601859 d=260 r=5000 s=4998123 k=1 e=3601859 w=3600548 b=200

The fields of the `C` line are described at the top of `main/rtc_slow_cal_repro.c`.

## Run it

Connect a USB-UART adapter to UART0 (on the Seeed XIAO ESP32C6: TX = D6 / GPIO16, plus GND). Don't
use USB-Serial-JTAG for the capture: it drops out in light sleep.

```sh
idf.py -B build build flash          # while the app light-sleeps, enter download mode by hand: hold BOOT, tap RESET
./clock_drift.py log /dev/ttyUSB0 -o rtc_cal.log    # leave running, ideally hours
./clock_drift.py fit rtc_cal.log     # any time, also on a copy while capturing
```

The capture host should be NTP-synced and must not suspend: the reference is its
`CLOCK_MONOTONIC`, which NTP slews but never steps. Positive ppm means the device clock runs fast.
`fit` reports one row per boot / mode, so one capture can hold several firmwares.

`clock_drift.py` needs only Python 3.10+ and pyserial (PEP 723 header, so `uv run clock_drift.py`
works too).

`sweep.sh` builds one firmware per `CONFIG_PM_LIGHTSLEEP_RTC_OSC_CAL_AFTER_WAKEUP_MIN_SLP_MS` value
plus stock, flashes them, and runs the capture detached; see the comment at its top.

## Options (`idf.py menuconfig` → RTC_SLOW calibration reproducer)

| Option | Default | Meaning |
|---|---|---|
| `REPRO_SLEEP_LOG_UNIFORM` | y | random log-uniform sleeps; n = fixed `REPRO_WAKE_PERIOD_MS` |
| `REPRO_SLEEP_MIN_MS` / `REPRO_SLEEP_MAX_MS` | 1 / 10000 | range of the random sleeps |
| `REPRO_SEED` | 0x19250 | seed of the sleep sequence |
| `REPRO_WAKE_PERIOD_MS` | 5000 | fixed sleep, if not random |
| `REPRO_REPORT_EVERY` | 1 | wakes per report line |
