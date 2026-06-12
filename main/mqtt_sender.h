#pragma once

#include <string>
#include <string_view>
#include <cstdint>

struct MqttConfig {
    std::string      ha_ipv4;    // fallback IPv4, e.g. "192.168.77.250"
    uint16_t         port;       // MQTT port, e.g. 1883
    std::string      username;
    std::string      password;
    std::string_view device_id;  // compile-time constant, e.g. "esp32c6_sensor_1"
};

// Call once before the sensor task starts.
void mqtt_sender_init(const MqttConfig &cfg);

// Update the NAT64 prefix used to reach the IPv4 broker from Thread.
// prefix_bytes must point to 12 bytes (a /96 prefix, e.g. from otExternalRouteConfig).
// Call this when the Thread network data contains a nat64=1 route.
void mqtt_sender_set_nat64_prefix(const uint8_t *prefix_bytes);

// Trigger a connect → publish discovery + state → disconnect cycle.
// Safe to call from any task; non-blocking (the MQTT event loop does the work).
void mqtt_send_sensor_data(float temperature, float humidity);
