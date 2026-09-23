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
// such "cold" value and the mean of the last COLD_RING_SIZE are kept; RtcCalMode picks which one
// the sleep path uses. HA-tunable (cfg/rtc_cal_mode), taking effect at the next sleep.
//
// A single calibration scatters by ~920 ppm (2026-09-22 capture, 164 status lines), so the mean is
// the steadier of the two, but only over spans shorter than the ring: a moving average preserves
// the sum of its inputs, so over 10 s or more the accumulated clock error barely depends on the
// ring size at all. What does grow with it is the lag behind a drifting die -- 67 ppm at 32 samples
// at the fastest drift in that capture, against the -54 ppm the clock is still out by. Hence 8.
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
// IRAM_ATTR and must not block. `used` is the period the sleep path got and `cold` says whether
// that came from the ring. `measured` is ESP-IDF's own calibration at sleep entry, which is taken
// even when a cold value times the sleep -- only while an observer is registered, because its one
// remaining use is to show the diagnostic how much warmer the die reads there.
using RtcSleepCalObserver = void (*)(uint32_t used, uint32_t measured, bool cold);
void rtc_clock_fix_set_sleep_cal_observer(RtcSleepCalObserver observer);

// For rtc_cal_diag's cooling curve: called from the light-sleep exit callback once per sleep, with
// interrupts off, so it must not block. `cold` is the calibration just taken at the wake,
// `slept_us` the sleep it ended and `taken` whether it went into the ring. Registering an observer
// also makes the callback calibrate after sleeps too short to be worth ringing, since those are
// the ones that wake with the die still warm and so are what traces the curve.
//
// This callback is registered before any other light-sleep exit callback (see
// rtc_clock_fix_start), so the observer runs before them: rtc_cal_diag relies on that to pair each
// cold value with the sleep-entry calibration of the same sleep, which its own exit callback has
// not yet retired.
using RtcColdCalObserver = void (*)(int64_t slept_us, uint32_t cold, bool taken);
void rtc_clock_fix_set_cold_cal_observer(RtcColdCalObserver observer);
