#pragma once

#include <cstdint>
#include <string>

#include "esp_err.h"
#include "network_link.h"

// Fix for the device clock running ~+670 ppm fast in light sleep (2026-09-22 captures, see
// rtc_cal_diag.h). esp_timer is advanced across every light sleep by RTC_SLOW ticks x an
// RTC_SLOW calibration, and ESP-IDF takes that calibration at sleep entry, i.e. right after
// whatever woke the core. RTC_SLOW (the internal RC) runs ~1900 ppm/C slower as the die warms,
// and a CSL receive window warms it enough to read ~+720 ppm slow within 5-20 ms. The
// calibration itself is exact (-8 +-8 ppm at 10 cycles, sandwiched between 1024-cycle ones);
// its moment is what's wrong.
//
// Proposal A: calibrate RTC_SLOW in a light-sleep exit callback instead, right after the wake
// and before OpenThread turns the radio on (a CSL window opens >= 2.3 ms later), while the die
// is still at its sleeping temperature, and hand that to ESP-IDF's sleep path through a
// -Wl,--wrap=rtc_clk_cal. Same 10-cycle calibration, same cost, different moment. Both the
// latest such "cold" value and the mean of the last 32 are kept; RtcCalMode picks which one
// the sleep path uses.
//
// Proposal C: learn what is left from a time reference. About every 30 minutes, inside a
// publish window, a few SNTP exchanges with `ntp_server` measure the device clock against real
// time; over the interval since the previous sample that gives the clock's error, and so the
// correction ("trim") the sleep path's calibration needs. Both the latest interval's trim and
// the mean of the last 8 are kept; RtcTrimMode picks which one is applied.
//
// Every 5 minutes a status line logs the die temperature (die_temp.h), the cold calibrations and
// the trims; each trim estimate also logs the interval's mean die temperature, so a
// temperature-dependent trim can be judged later. Both modes are HA-tunable (cfg/rtc_cal_mode,
// cfg/rtc_trim_mode) and take effect at the next sleep.

enum class RtcCalMode : uint8_t {
    EspIdf = 0,    // ESP-IDF's own calibration at sleep entry (the fast clock)
    ColdLast = 1,  // the latest calibration taken right after a wake
    ColdMean = 2,  // the mean of the last 32 of those
};

enum class RtcTrimMode : uint8_t {
    Off = 0,
    NtpLast = 1,  // the trim the latest NTP interval asked for
    NtpMean = 2,  // the mean of the last 8 intervals'
};

struct RtcClockFixConfig {
    uint32_t cal_mode;  // RtcCalMode; out-of-range values are clamped
    uint32_t trim_mode;  // RtcTrimMode; out-of-range values are clamped
    // Literal IPv4 or IPv6 address (no DNS). An IPv4 server is reached through the Thread
    // network's NAT64 prefix. Empty disables the NTP samples, and with them any trim.
    std::string ntp_server;
};

// Call once, after enableAutomaticLightSleep() and before any other light-sleep exit callback
// is registered, so the cold calibration runs first after every wake.
esp_err_t rtc_clock_fix_start(const RtcClockFixConfig &config);

// Live mode changes from runtime_config.cpp, applied from the next sleep on. Changing the
// calibration mode also discards the trims learned so far: each was measured against the old
// calibration.
void rtc_clock_fix_set_cal_mode(uint32_t mode);
void rtc_clock_fix_set_trim_mode(uint32_t mode);
uint32_t rtc_clock_fix_cal_mode();
uint32_t rtc_clock_fix_trim_mode();

// Proposal C's NTP sample, from the MQTT task inside a publish window (fast polling, so the
// replies arrive promptly). Does nothing unless a sample is due and `budget_ms` covers a whole
// burst, so it never stretches a publish cycle past what the sensor task waits for.
void rtc_clock_fix_ntp_sample_if_due(const NetworkLink &link, uint32_t budget_ms);

// For rtc_cal_diag: called from the wrap in the sleep path, with interrupts off, so it must be
// IRAM_ATTR and must not block. `used` is the period the sleep path got, `measured` the
// calibration ESP-IDF's own call made or 0 when a cold value was used instead.
using RtcSleepCalObserver = void (*)(uint32_t used, uint32_t measured);
void rtc_clock_fix_set_sleep_cal_observer(RtcSleepCalObserver observer);
