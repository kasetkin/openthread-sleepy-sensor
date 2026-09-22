#pragma once

#include <cstdint>
#include <optional>

// The ESP32-C6's own on-die temperature sensor, for the light-sleep clock work: RTC_SLOW (the
// internal RC every light sleep is timed with) moves ~1900 ppm per degree, so the die's
// temperature is what the clock's error should follow -- the SHT4x reads the air instead.
struct DieTemp {
    // The driver's calibrated reading, averaged over the samples. The driver truncates each
    // reading to a whole degree before its eFuse offset, so this is coarse in absolute terms.
    float celsius;
    // The sensor's raw output code, averaged: ~0.44 C per step (TEMPERATURE_SENSOR_LL_ADC_FACTOR)
    // within one measuring range, so changes between readings are finer than `celsius` shows.
    float raw;
};

// One raw step in degrees within a measuring range: TEMPERATURE_SENSOR_LL_ADC_FACTOR in ESP-IDF's
// esp_hal_ana_conv/esp32c6 temperature_sensor_ll.h.
inline constexpr float DIE_TEMP_RAW_STEP_C = 0.4386f;

// Installs the driver, averages `samples` readings and uninstalls it again, so the sensor
// doesn't keep its power domain from powering down in light sleep between readings. Takes a
// couple of milliseconds; safe to call from several tasks. nullopt if the driver fails.
std::optional<DieTemp> die_temp_read(uint32_t samples);
