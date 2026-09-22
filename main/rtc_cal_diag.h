#pragma once

#include "esp_err.h"

// Diagnostic for the device clock running ~+680..+830 ppm fast (measured against the border
// router's log and HA's server timestamps). Across a light sleep, esp_timer is advanced by
// RTC_SLOW ticks x the RTC_SLOW calibration, and ESP-IDF's light-sleep path recalibrates RTC_SLOW
// over only 10 cycles before every sleep (RTC_CLK_SRC_CAL_CYCLES in esp_hw_support/sleep_modes.c),
// while the core is awake. Two candidate causes: a counting offset in the 10-cycle calibration
// (one 40 MHz XTAL count is already ~340 ppm there), or RTC_SLOW itself running at a different
// rate asleep than awake. The first version's single run on 2026-09-22 saw RTC_SLOW slow down by
// ~870 ppm within half a second of staying awake, which also spoiled its sequential comparison.
//
// Every 5 minutes, the first time 5 minutes after start, a low-priority task holds a
// no-light-sleep lock for ~2 s and logs (tag rtc-cal, every ppm figure positive = a longer tick,
// i.e. esp_timer would run fast across a sleep):
//  - esp_timer and the RTC tick counter read together, so two runs and the border router's log
//    give the tick length RTC_SLOW really had between them, mostly asleep;
//  - a one-second timeline of 1024-cycle calibrations from the moment it woke, relative to its
//    settled tail, binned and raw;
//  - 10-, 32- and 100-cycle calibrations each sandwiched between two 1024-cycle ones, so a
//    counting offset shows up as a bias shrinking as 1/N and drift during the run cancels;
//  - the die temperature (die_temp.h) at wake, after the timeline and at the end, so the
//    timeline's drift can be set against the die warming up;
//  - the periods the sleep path used over its last 512 sleeps -- ESP-IDF's own calibration or a
//    cold one, see rtc_clock_fix.h, whose wrap of rtc_clk_cal() reports them here -- overall and
//    by how long the core had been awake before sleeping, plus the mean weighted by the sleep
//    each one timed.
//
// 2026-09-22 results: the 10-cycle calibration is exact; RTC_SLOW slows right after activity,
// which is what rtc_clock_fix.h fixes. Delete this module and its call in main.cpp once that fix
// is settled (rtc_clock_fix.cpp owns the --wrap).
esp_err_t rtc_cal_diag_start();
