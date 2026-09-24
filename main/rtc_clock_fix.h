#pragma once

#include <cstdint>

#include "esp_err.h"

// Fix for the device clock running ~+670..+830 ppm fast in light sleep. esp_timer is advanced
// across every light sleep by RTC_SLOW ticks x an RTC_SLOW calibration, and ESP-IDF takes that
// calibration at sleep entry, i.e. right after whatever woke the core. RTC_SLOW (the internal RC)
// runs ~1540 ppm/C slower as the die warms, and the activity before a sleep warms it enough to
// read ~800 ppm slow. The calibration itself is exact (-3 +-4 ppm at 10 cycles, sandwiched between
// 1024-cycle ones); its moment is what's wrong.
//
// So calibrate RTC_SLOW in a light-sleep exit callback instead, right after the wake and before
// OpenThread turns the radio on (a CSL window opens >= 2.3 ms later), while the die is still at its
// sleeping temperature, and hand the mean of the last few such "cold" values to ESP-IDF's sleep path
// through a -Wl,--wrap=rtc_clk_cal. Same 10-cycle calibration, same cost, different moment. Measured
// against the border router's clock: +670..+830 ppm -> -48 +-0.7 ppm (2026-09-23, 19 h, 8 samples).
//
// ESP-IDF stores whatever the sleep path got as the global RTC_SLOW calibration
// (esp_clk_slowclk_cal_set() in sleep_modes.c), so everything else timed off RTC_SLOW gets the cold
// value too -- the LP core's wake timer, which reads it back every time it re-arms, among them. And
// every wake calibrates: each CSL window, lwIP's ~1 s timers, the 70 s idle data poll. So the value
// is as fresh as the wake cadence, whether CSL is on or not and however seldom data is published.
//
// A single calibration scatters by ~590 ppm. A moving average only smooths that over its own span --
// it preserves the sum of its inputs, so over tens of seconds the clock barely notices the count --
// while lagging behind a die that drifts, hence 8 by default.

// cfg/rtc_cal_samples: how many of the newest cold calibrations the sleep path averages. 0 leaves
// the sleep path to ESP-IDF's own calibration at sleep entry, untouched: no cold calibration taken.
inline constexpr uint32_t RTC_CAL_SAMPLES_MAX = 32;
inline constexpr uint32_t RTC_CAL_SAMPLES_DEFAULT = 8;

// cfg/rtc_cal_period_sec: how often the status task wakes the core, 0 = never. It logs the die
// temperature and the calibrations, and its own timer wake is a calibration like any other, so the
// period also caps how old the calibration can get should every other wake ever go away.
inline constexpr uint32_t RTC_CAL_PERIOD_SEC_FLOOR = 30;
inline constexpr uint32_t RTC_CAL_PERIOD_SEC_MAX = 3600;
inline constexpr uint32_t RTC_CAL_PERIOD_SEC_DEFAULT = 300;

// The values the setters below actually use for a requested one.
inline constexpr uint32_t rtc_cal_clamp_samples(uint32_t samples)
{
    return samples < RTC_CAL_SAMPLES_MAX ? samples : RTC_CAL_SAMPLES_MAX;
}
inline constexpr uint32_t rtc_cal_clamp_period_sec(uint32_t period_sec)
{
    if (period_sec == 0)
        return 0;
    if (period_sec < RTC_CAL_PERIOD_SEC_FLOOR)
        return RTC_CAL_PERIOD_SEC_FLOOR;
    return period_sec < RTC_CAL_PERIOD_SEC_MAX ? period_sec : RTC_CAL_PERIOD_SEC_MAX;
}

// Call once, after enableAutomaticLightSleep() and before any other light-sleep exit callback is
// registered, so the cold calibration is the first thing after every wake.
esp_err_t rtc_clock_fix_start(uint32_t samples, uint32_t period_sec);

// Live changes from runtime_config.cpp. A new sample count applies from the next sleep, with no
// refill: the newest RTC_CAL_SAMPLES_MAX cold values are kept whatever the count. A new period
// restarts the status task's wait.
void rtc_clock_fix_set_samples(uint32_t samples);
uint32_t rtc_clock_fix_samples();
void rtc_clock_fix_set_period_sec(uint32_t period_sec);
uint32_t rtc_clock_fix_period_sec();
