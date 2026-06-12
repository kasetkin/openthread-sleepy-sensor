#pragma once

#include <string>
#include <string_view>

// Linker-generated symbols — file-private; callers use secrets_yaml() only.
extern "C" {
    extern const char _binary_secrets_yaml_start[];
    extern const char _binary_secrets_yaml_end[];
}

inline std::string_view secrets_yaml() {
    return {_binary_secrets_yaml_start,
            static_cast<size_t>(_binary_secrets_yaml_end - _binary_secrets_yaml_start)};
}

// Finds `key: "value"` in the YAML content and returns the quoted value.
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
