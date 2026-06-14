#pragma once

#include <esp_err.h>
#include <string>

/// intended only for ESP32-C6 from SeeedStudio

void enableRf(const bool enableRf);
void enableExtAntenna(const bool enableExtAnt);
void enableUserLED(const bool enableLED);
/// brief diagnostic flash: LED on for `onMs`, then off. Negligible power vs a held LED.
void blinkUserLED(const uint32_t onMs);

/// configure wakeup timer
[[nodiscard("device won't wake on timer if unchecked")]]
esp_err_t registerWakeupTimer(const uint64_t wakeupMicrosec);
esp_err_t registerWakeupTimer(int) = delete("duration must be uint32_t microseconds — negative values silently wrap to enormous sleep");

/// sleep for wakeupMicrosec from above + ?10ms? using esp_light_sleep
void correctLightSleep();

[[nodiscard("NVS unavailable if init failure ignored")]]
esp_err_t initNvsFlash();

/// for loggertask code migration, because it was written for Arduino
unsigned long millisFromStart();
/// emulate code from RTC.h
uint64_t getValidTime();

#include <charconv>
#include <optional>
#include <type_traits>

// Append any integer or float/double to `out` via to_chars — zero heap allocation.
// Buffer sizing (shortest round-trip, base 10):
//   uint64_t:    20 chars  (19 digits + sign)
//   double:      24 chars  (sign + 17 sig.digits + '.' + 'e' + sign + 3 exp.digits)
//   long double: 26 chars  (x86 80-bit, 18 sig.digits, exponent up to ±4932, 4 exp.digits)
// On ESP32-C6 long double == double (24 chars), but 28 = 26 + 2 keeps the template
// correct on any platform without wasting stack space.
template<typename T>
    requires std::is_arithmetic_v<T>
inline void appendNum(std::string& out, T value) noexcept {
    char buf[28];
    auto [ptr, ec] = std::to_chars(buf, buf + sizeof(buf), value);
    out.append(buf, ptr);
}

template<typename T>
    requires std::is_arithmetic_v<T>
inline void appendNum(std::string& out, std::optional<T> value) noexcept {
    if (!value.has_value()) {
        out.push_back('-');
        return;
    }

    char buf[28];
    auto [ptr, ec] = std::to_chars(buf, buf + sizeof(buf), value.value());
    out.append(buf, ptr);
}

inline void appendZeroPadded(std::string& out, int value, int width) noexcept {
    char buf[12];
    auto [ptr, ec] = std::to_chars(buf, buf + sizeof(buf), value);
    const int written = static_cast<int>(ptr - buf);
    if (written < width)
        out.append(static_cast<size_t>(width - written), '0');
    out.append(buf, ptr);
}

// esp_err_t initI2C();
/// ESP tasks
// void sht3xTask(void *pvParameters);
// void light_sleep_ble_sensor(void *args);
