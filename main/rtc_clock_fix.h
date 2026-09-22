#pragma once

#include <cstdint>

#include "esp_err.h"

// Fix for the device clock running ~+670 ppm fast in light sleep (2026-09-22 captures, see
// rtc_cal_diag.h). esp_timer is advanced across every light sleep by RTC_SLOW ticks x an
// RTC_SLOW calibration, and ESP-IDF takes that calibration at sleep entry, i.e. right after
// whatever woke the core. RTC_SLOW (the internal RC) runs ~1900 ppm/C slower as the die warms,
// and a CSL receive window warms it enough to read ~+720 ppm slow within 5-20 ms. The
// calibration itself is exact (-8 +-8 ppm at 10 cycles, sandwiched between 1024-cycle ones);
// its moment is what's wrong.
//
// So calibrate RTC_SLOW in a light-sleep exit callback instead, right after the wake and before
// OpenThread turns the radio on (a CSL window opens >= 2.3 ms later), while the die is still at
// its sleeping temperature, and hand that to ESP-IDF's sleep path through a
// -Wl,--wrap=rtc_clk_cal. Same 10-cycle calibration, same cost, different moment. Both the latest
// such "cold" value and the mean of the last 32 are kept; RtcCalMode picks which one the sleep
// path uses. HA-tunable (cfg/rtc_cal_mode), taking effect at the next sleep.
//
// Every 5 minutes a status line logs the die temperature (die_temp.h), the cold calibrations and
// which calibration timed the sleeps since the previous line.

enum class RtcCalMode : uint8_t {
    EspIdf = 0,    // ESP-IDF's own calibration at sleep entry (the fast clock)
    ColdLast = 1,  // the latest calibration taken right after a wake
    ColdMean = 2,  // the mean of the last 32 of those
};

// Call once, after enableAutomaticLightSleep() and before any other light-sleep exit callback
// is registered, so the cold calibration runs first after every wake. `cal_mode` is an
// RtcCalMode; out-of-range values are clamped.
esp_err_t rtc_clock_fix_start(uint32_t cal_mode);

// Live mode change from runtime_config.cpp, applied from the next sleep on.
void rtc_clock_fix_set_cal_mode(uint32_t mode);
uint32_t rtc_clock_fix_cal_mode();

// For rtc_cal_diag: called from the wrap in the sleep path, with interrupts off, so it must be
// IRAM_ATTR and must not block. `used` is the period the sleep path got, `measured` the
// calibration ESP-IDF's own call made or 0 when a cold value was used instead.
using RtcSleepCalObserver = void (*)(uint32_t used, uint32_t measured);
void rtc_clock_fix_set_sleep_cal_observer(RtcSleepCalObserver observer);
