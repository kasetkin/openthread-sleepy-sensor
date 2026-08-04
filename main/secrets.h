#pragma once

#include <string>
#include <string_view>
#include <optional>
#include <charconv>
#include "openthread/dataset.h"

// Linker-generated symbols — file-private; callers use secrets_yaml() only.
extern "C" {
    extern const char _binary_secrets_yaml_start[];
    extern const char _binary_secrets_yaml_end[];
}

inline std::string_view secrets_yaml() {
    return {_binary_secrets_yaml_start,
            static_cast<size_t>(_binary_secrets_yaml_end - _binary_secrets_yaml_start)};
}

// Finds `key: "value"` in the YAML content and returns the quoted value, skipping any
// match that's commented out (a '#' earlier on the same line) — otherwise a stray
// `#key: "old value"` left above the real line would silently shadow it.
// Returns "" if no uncommented match is found.
inline std::string yaml_get_string(std::string_view content, std::string_view key) {
    std::string search{key};
    search += ": \"";

    size_t search_from = 0;
    while (true) {
        const auto pos = content.find(search, search_from);
        if (pos == std::string_view::npos)
            return {};

        const auto line_start = content.rfind('\n', pos);
        const auto line_begin = (line_start == std::string_view::npos) ? 0 : line_start + 1;
        const auto hash_pos = content.find('#', line_begin);
        if (hash_pos != std::string_view::npos && hash_pos < pos) {
            search_from = pos + search.size();
            continue;  // this match is commented out; keep looking
        }

        const auto val_start = pos + search.size();
        const auto val_end = content.find('"', val_start);
        if (val_end == std::string_view::npos)
            return {};
        return std::string{content.substr(val_start, val_end - val_start)};
    }
}

// nullopt if `key` is missing from `content` or its value fails to parse -- distinguishable
// from a legitimately-configured 0, so a renamed/typo'd/dropped key in a hand-edited
// device_config.yaml doesn't silently masquerade as an intentional zero (see call sites in
// main.cpp, which log a warning and substitute an explicit default on nullopt).
inline std::optional<float> parse_as_float(std::string_view content, std::string_view key)
{
    const std::string s = yaml_get_string(content, key);
    if (s.empty())
        return std::nullopt;
    float value = 0.0f;
    const auto [ptr, ec] = std::from_chars(s.data(), s.data() + s.size(), value);
    if (ec != std::errc{} || ptr != s.data() + s.size())
        return std::nullopt;
    return value;
}

// See parse_as_float()'s comment -- same missing/malformed-vs-legitimate-zero distinction.
inline std::optional<uint32_t> parse_as_uint32(std::string_view content, std::string_view key)
{
    const std::string s = yaml_get_string(content, key);
    if (s.empty())
        return std::nullopt;
    uint32_t value = 0;
    const auto [ptr, ec] = std::from_chars(s.data(), s.data() + s.size(), value);
    if (ec != std::errc{} || ptr != s.data() + s.size())
        return std::nullopt;
    return value;
}

// See parse_as_float()'s comment -- same missing/malformed-vs-legitimate-value distinction.
// Signed counterpart to parse_as_uint32() -- needed for keys (like tx_power_dbm) whose value
// can legitimately be negative; std::from_chars handles the leading '-' natively.
inline std::optional<int32_t> parse_as_int32(std::string_view content, std::string_view key)
{
    const std::string s = yaml_get_string(content, key);
    if (s.empty())
        return std::nullopt;
    int32_t value = 0;
    const auto [ptr, ec] = std::from_chars(s.data(), s.data() + s.size(), value);
    if (ec != std::errc{} || ptr != s.data() + s.size())
        return std::nullopt;
    return value;
}

// See parse_as_float()'s comment -- same missing/malformed-vs-legitimate-value distinction.
// Accepts only the literal strings "true" and "false".
inline std::optional<bool> parse_as_bool(std::string_view content, std::string_view key)
{
    const std::string s = yaml_get_string(content, key);
    if (s == "true")
        return true;
    if (s == "false")
        return false;
    return std::nullopt;
}

inline uint8_t hex_nibble(char c)
{
    if (c >= '0' && c <= '9') return static_cast<uint8_t>(c - '0');
    if (c >= 'a' && c <= 'f') return static_cast<uint8_t>(c - 'a' + 10);
    if (c >= 'A' && c <= 'F') return static_cast<uint8_t>(c - 'A' + 10);
    return 0;
}

inline bool parse_dataset_tlvs(const std::string &hex, otOperationalDatasetTlvs &out)
{
    static const char *TAG = "secrets-parser";
    size_t hex_len = hex.size();

    if (hex_len == 0 || hex_len % 2 != 0 || hex_len / 2 > OT_OPERATIONAL_DATASET_MAX_LENGTH) {
        ESP_LOGE(TAG, "Invalid OT TLV hex string (len=%d)", static_cast<int>(hex_len));
        return false;
    }

    out = {};
    for (size_t i = 0; i < hex_len; i += 2)
        out.mTlvs[out.mLength++] = static_cast<uint8_t>(
            (hex_nibble(hex[i]) << 4) | hex_nibble(hex[i + 1]));

    ESP_LOGI(TAG, "Parsed %d bytes from OT TLV dataset", out.mLength);
    return true;
}