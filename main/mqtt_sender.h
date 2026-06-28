#pragma once

#include <string>
#include <string_view>
#include <optional>
#include <cstdint>

struct MqttConfig {
    std::string      ha_ipv4;    // fallback IPv4, e.g. "192.168.77.250"
    uint16_t         port;       // MQTT port, e.g. 1883
    std::string      username;
    std::string      password;
    std::string      device_id;    // unique id, e.g. "<device_name>_a1b2c3" (name + chip MAC suffix)
    std::string      device_name;  // human-readable name from secrets.yaml "device_name"
};

// Call once before the sensor task starts.
void mqtt_sender_init(const MqttConfig &cfg);

// Update the NAT64 prefix used to reach the IPv4 broker from Thread.
// prefix_bytes must point to 12 bytes (a /96 prefix, e.g. from otExternalRouteConfig).
// Call this when the Thread network data contains a nat64=1 route.
void mqtt_sender_set_nat64_prefix(const uint8_t *prefix_bytes);

// Trigger a connect → publish discovery + state → disconnect cycle.
// Safe to call from any task; non-blocking (the MQTT event loop does the work).
void mqtt_send_sensor_data(std::optional<float> temperature, std::optional<float> humidity);

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
