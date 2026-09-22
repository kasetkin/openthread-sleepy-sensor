#pragma once

#include "esp_err.h"

// Diagnostic for the device clock running ~+830 ppm fast (measured 2026-09-22 against the
// border router's uptime and HA's server timestamps). Across a light sleep, esp_timer is advanced
// by RTC_SLOW ticks x the RTC_SLOW calibration, and ESP-IDF's light-sleep path recalibrates
// RTC_SLOW over only 10 cycles before every sleep (RTC_CLK_SRC_CAL_CYCLES in
// esp_hw_support/sleep_modes.c) -- where one 40 MHz XTAL count is already ~340 ppm, so a fixed
// counting offset of a couple of XTAL cycles would bias every calibration by hundreds of ppm.
//
// Every 30 minutes (first run a minute after start) a low-priority task, holding a
// no-light-sleep lock so the sleep path can't take the calibration hardware mid-measurement,
// calibrates RTC_SLOW at 10, 32, 100 and 1024 cycles and logs each mean against a 4096-cycle
// reference: a fixed offset shows up as a bias shrinking as 1/N, an oscillator that is simply
// different while asleep shows up as no bias at all. It also logs the mean of the 10-cycle values
// the sleep path itself got since the previous run, collected by a pass-through wrap of
// rtc_clk_cal() that changes nothing.
//
// Delete this module, its call in main.cpp and its --wrap flag in CMakeLists.txt once the question
// is settled.
esp_err_t rtc_cal_diag_start();
