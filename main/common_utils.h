#pragma once

#include <esp_err.h>
#include <string>

/// intended only for ESP32-C6 from SeeedStudio

void enableRf(const bool enableRf);
void enableExtAntenna(const bool enableExtAnt);
void enableUserLED(const bool enableLED);
/// brief diagnostic flash: LED on for `onMs`, then off. Negligible power vs a held LED.
void blinkUserLED(const uint32_t onMs, size_t count = 1);

/// Enables ESP-IDF's automatic, PM-lock-gated light sleep (esp_pm_configure() with
/// light_sleep_enable=true). Call once at boot, before starting OpenThread/Wi-Fi — mirrors
/// ESP-IDF's own ot_sleepy_device/light_sleep example. max==min keeps CPU pinned at
/// CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ (DFS doesn't help a radio-bound sleepy end device) while
/// still enabling the light-sleep gate. No app code needs to call esp_light_sleep_start()
/// itself afterward — FreeRTOS's tickless-idle idle task enters light sleep automatically
/// whenever no task is ready and no esp_pm lock (e.g. OpenThread's own radio-state lock, see
/// esp_openthread_sleep.c) is held.
[[nodiscard("light sleep silently stays disabled if the configure call is ignored")]]
esp_err_t enableAutomaticLightSleep();

[[nodiscard("NVS unavailable if init failure ignored")]]
esp_err_t initNvsFlash();

/// Reset-reason refinement pair. IDF reports both an OTA reboot and the LP-core-stall
/// supervisor's reboot (sensorstask.cpp) as plain ESP_RST_SW; only the latter is a failure
/// signal worth surfacing in HA (the supervisor reboots ONLY on a confirmed LP-core heartbeat
/// stall -- link-down and broker-unreachable cycles never reach it, see sensorstask.cpp). The
/// supervisor calls markLpStallReboot() right before its esp_restart(), leaving a magic word
/// in RTC (LP) RAM -- which survives a software reset -- and main.cpp's boot-time
/// reset-reason mapping calls consumeLpStallRebootMarker() (read-and-clear, so the refinement
/// applies to exactly one boot) to tell the two apart. Same hint technique esp_reset_reason()
/// itself uses.
void markLpStallReboot();
bool consumeLpStallRebootMarker();

/// Same technique, different cause: sensorstask.cpp's bad-OTA safety net reboots when the
/// running image is still unconfirmed (PENDING_VERIFY) after enough consecutive unhealthy
/// cycles -- a freshly-flashed image that can never confirm itself has no other way back, and
/// this reboot doubles as the rollback trigger. Mutually exclusive with the LP-stall marker
/// above at any single reboot (only one path's esp_restart() call ever actually runs).
void markOtaUnconfirmedReboot();
bool consumeOtaUnconfirmedRebootMarker();

/// for loggertask code migration, because it was written for Arduino
unsigned long millisFromStart();
/// emulate code from RTC.h
uint64_t getValidTime();

#include <charconv>
#include <optional>
#include <type_traits>
#include <array>
#include <string_view>
#include <cstdint>

// Parse a dotted-quad IPv4 ("a.b.c.d") into 4 octets; nullopt on any malformation.
std::optional<std::array<uint8_t, 4>> parseIpv4(std::string_view s);

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
