#pragma once

#include <string>
#include <string_view>
#include <optional>
#include <cstdint>

#include "network_link.h"

// Upper bound on MqttConfig::device_name (enforced by main.cpp before it ever reaches here) —
// the single source of truth both main.cpp's truncation and mqtt_sender.cpp's fixed-size
// topic/payload buffers are sized against, so the two can't silently drift apart.
inline constexpr size_t MQTT_MAX_DEVICE_NAME_LEN = 64;

// device_id = sanitised device_name + '-' + one 2-hex-digit pair per byte of the chip's factory
// MAC (main.cpp's addOTMacSuffix() reads an 8-byte IEEE 802.15.4 extended address via
// esp_read_mac()). Derived, not hand-counted, so it can't drift from that function's actual
// output shape; MQTT_MAC_ADDRESS_BYTES is also checked against addOTMacSuffix()'s mac[] array
// via a static_assert there, so the two stay in sync.
inline constexpr size_t MQTT_MAC_ADDRESS_BYTES = 8;
inline constexpr size_t MQTT_MAX_DEVICE_ID_LEN =
    MQTT_MAX_DEVICE_NAME_LEN + 1 /* '-' */ + 2 * MQTT_MAC_ADDRESS_BYTES;

// Longest string main.cpp's resetReasonString() can produce (and main.cpp truncates
// anything longer, mirroring the device_name bound above) — mqtt_sender.cpp's state-JSON
// buffer is sized against this, so the two can't silently drift apart.
inline constexpr size_t MQTT_MAX_RESET_REASON_LEN = sizeof("lp_stall_reboot") - 1;  // longest of resetReasonString()'s literals (main.cpp)

// Longest string NetworkLink::cslStatus() can produce (openthread_link.cpp/wifi_link.cpp) --
// same "buffer sizing can't drift from the actual literals" role as the bound above.
inline constexpr size_t MQTT_MAX_CSL_STATUS_LEN = sizeof("unsupported") - 1;

struct MqttConfig {
    std::string      broker_address; // literal IPv4 or IPv6 address, e.g. "192.168.77.250" or "fd12:3456:789a::10"
    uint16_t         port;       // MQTT port, e.g. 1883
    std::string      username;
    std::string      password;
    // unique id: sanitised device_name + '-' + 16 hex MAC digits (see main.cpp's
    // addOTMacSuffix()) -- at most MQTT_MAX_DEVICE_ID_LEN chars.
    std::string      device_id;
    std::string      device_name;  // human-readable name from secrets.yaml "device_name", capped to MQTT_MAX_DEVICE_NAME_LEN
    bool             use_tls = false;      // true: mqtts:// with server verification; false: plaintext mqtt://
    std::string      tls_ca_cert_b64;      // optional CA/leaf cert, base64 body only (no PEM markers/newlines);
                                            // empty => trust ESP-IDF's public CA bundle instead. Only used if use_tls.
    // HA sensors' expire_after (seconds without an update before HA shows "unavailable");
    // main.cpp derives it as 2 x SensorsTask::safeguardWakeSec(lp_poll_interval_sec,
    // max_skip_cycles). The LP core guarantees a publish at latest every
    // (max_skip_cycles + 1) polls, so two safeguard windows without data means delivery is
    // genuinely broken (and the reboot self-recovery typically lands well inside that
    // budget). 0 omits the field (entities then never expire, the pre-feature behaviour).
    uint32_t         expire_after_sec = 0;
    uint32_t         boot_count = 0;        // lifetime NVS boot counter, published as state key "bc"
    std::string      reset_reason = "unknown";  // last reboot cause, published as state key "rr";
                                                 // at most MQTT_MAX_RESET_REASON_LEN chars
};

// Call once before the sensor task starts. `link` must outlive the sensor task — it
// backs the transport-specific parts (broker URI, reachability, publish-window hooks).
void mqtt_sender_init(const MqttConfig &cfg, const NetworkLink *link);

// Trigger a connect → publish discovery + state → disconnect cycle.
// Safe to call from any task; non-blocking (the MQTT event loop does the work).
// battery_percent/battery_millivolts/heater_problem/heater_run_count all ride along on a
// temperature/humidity publish (extra state-JSON fields plus their one-time discovery
// configs) and never trigger a cycle by themselves: if both temperature and humidity are
// empty, nothing is published regardless of these. Pass both battery values or neither -- a
// lone one is ignored; heater_problem/heater_run_count are likewise set together or not at
// all (absent until the LP core has ever completed a heater run).
void mqtt_send_sensor_data(std::optional<float> temperature, std::optional<float> humidity,
                           std::optional<float> battery_percent = std::nullopt,
                           std::optional<int> battery_millivolts = std::nullopt,
                           std::optional<bool> heater_problem = std::nullopt,
                           std::optional<uint32_t> heater_run_count = std::nullopt);

// True while a publish cycle started by mqtt_send_sensor_data() is still in flight.
// Lets a caller tell "a publish is happening" apart from "nothing was sent this cycle".
bool mqtt_is_busy();

// Block until the in-flight publish cycle (if any) has finished, or timeout_ms elapses.
// Returns true if the sender is idle (cycle completed, or none was running), false on timeout.
// Call this before light-sleeping so sleep does not suspend the MQTT task/radio mid-publish.
bool mqtt_wait_for_idle(uint32_t timeout_ms);

// Outcome of the most recently finished publish cycle: true only when the broker connected AND
// ACKed the state message ("data actually reached HA"), false on any failure (no connect, no ACK,
// nothing to send, or an aborted cycle). Only meaningful once mqtt_wait_for_idle() reports idle.
bool mqtt_last_publish_succeeded();
