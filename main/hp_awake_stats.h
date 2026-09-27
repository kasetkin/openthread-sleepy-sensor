#pragma once

#include <cstdint>

#include "esp_err.h"

// Tracks how much of each publish cycle the HP core spent NOT in light sleep, as a diagnostic
// for the ~335 uA vs ~35 uA "correct config" power investigation -- see the plan doc's
// quantitative model: Radio TX/RX time only measures the radio's own active sub-windows, not
// the surrounding CPU-active time (attach check, JSON encode, ACK wait, DFS ramp), which this
// fills in.
//
// Registers a light-sleep exit callback (via esp_pm_light_sleep_register_cbs(), which supports
// multiple independent registrations -- see lp_sensor_core.c for the sibling registration) to
// track total HP-core wall-clock time NOT spent in light sleep. Call once from main.cpp, right
// after enableAutomaticLightSleep() -- sleep time before this point is untracked, matching the
// same ordering constraint documented on that call.
esp_err_t hp_awake_stats_init();

// This period's awake time (wall-clock elapsed minus every reported light-sleep duration since
// the last call), in microseconds, then resets the accumulator -- same "per-cycle delta" shape
// as LinkStats's counter-derived fields. Call once per publish cycle.
uint32_t hp_awake_stats_get_and_reset_us();
