#pragma once

#include <cstddef>
#include <string>
#include <string_view>

// Per-device sensor calibration, embedded at build time (EMBED_TXTFILES ../calibration.txt).
// Parsed with yaml_get_string() — a minimal `key: "value"` lookup (see calibration.txt.example).

// Linker-generated symbols — file-private; callers use calibration_txt() only.
extern "C" {
    extern const char _binary_calibration_txt_start[];
    extern const char _binary_calibration_txt_end[];
}

inline std::string_view calibration_txt() {
    return {_binary_calibration_txt_start,
            static_cast<size_t>(_binary_calibration_txt_end - _binary_calibration_txt_start)};
}

// Finds `key: "value"` in the embedded content and returns the quoted value.
// Returns "" if the key is not found.
inline std::string yaml_get_string(std::string_view content, std::string_view key) {
    std::string search{key};
    search += ": \"";
    const auto pos = content.find(search);
    if (pos == std::string_view::npos)
        return {};
    const auto val_start = pos + search.size();
    const auto val_end = content.find('"', val_start);
    if (val_end == std::string_view::npos)
        return {};
    return std::string{content.substr(val_start, val_end - val_start)};
}
