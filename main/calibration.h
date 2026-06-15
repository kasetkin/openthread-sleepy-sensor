#pragma once

#include <cstddef>
#include <string_view>

// Per-device sensor calibration, embedded at build time (EMBED_TXTFILES ../calibration.txt).
// Parsed with yaml_get_string() from secrets.h — same `key: "value"` syntax as secrets.yaml.

// Linker-generated symbols — file-private; callers use calibration_txt() only.
extern "C" {
    extern const char _binary_calibration_txt_start[];
    extern const char _binary_calibration_txt_end[];
}

inline std::string_view calibration_txt() {
    return {_binary_calibration_txt_start,
            static_cast<size_t>(_binary_calibration_txt_end - _binary_calibration_txt_start)};
}
