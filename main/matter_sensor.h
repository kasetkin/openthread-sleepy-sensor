#pragma once
//
// Matter sensor endpoint interface — the Matter replacement for mqtt_sender.h.
//
// This header is the clean boundary between the app's C++26 code and the
// Matter/CHIP (gnu++17) implementation in matter_sensor.cpp: it pulls in NO
// esp-matter/chip headers, only <optional>, so `main` can include it freely.
//
// Unlike MQTT (connect → publish → disconnect each cycle), Matter is a
// subscription/report model: the sensor task just pushes the latest values into
// the cluster attributes and the Matter stack + ICD report them to subscribers
// on their own schedule. No connect/disconnect, no wait-for-idle.

#include <optional>

// Create the Matter root node + Temperature (0x0402) and Relative Humidity
// (0x0405) sensor endpoints, configure the OpenThread platform, and start the
// Matter stack (BLE commissioning, Thread transport). Call once from app_main
// after NVS init.
void matter_sensor_init();

// Push the latest calibrated reading into the cluster MeasuredValue attributes.
// A disengaged optional leaves that cluster's value untouched. Safe to call every
// sensor cycle from the sensor task.
void matter_sensor_update(std::optional<float> temperature_c,
                          std::optional<float> humidity_pct);
