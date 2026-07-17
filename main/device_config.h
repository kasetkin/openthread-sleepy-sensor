#pragma once

#include <cstddef>
#include <string_view>

// Per-device configuration (calibration offsets, publish cadence, heater schedule, battery
// ADC), embedded at build time (EMBED_TXTFILES ../device_config.yaml).
// Parsed with yaml_get_string() from secrets.h — same `key: "value"` syntax as secrets.yaml.

// Linker-generated symbols — file-private; callers use device_config_yaml() only.
extern "C" {
    extern const char _binary_device_config_yaml_start[];
    extern const char _binary_device_config_yaml_end[];
}

inline std::string_view device_config_yaml() {
    return {_binary_device_config_yaml_start,
            static_cast<size_t>(_binary_device_config_yaml_end - _binary_device_config_yaml_start)};
}
