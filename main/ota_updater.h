#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>

#include "mqtt_client.h"
#include "network_link.h"

// MQTT OTA updater. The firmware image is staged on the broker as ONE retained MQTT message
// (tools/ota_push.py); esp-mqtt delivers it as sequential MQTT_EVENT_DATA segments which are
// streamed straight into the passive OTA partition — TCP receive-window backpressure is the
// flow control, so there is no chunk protocol and no live host during the download.
//
// Split of responsibilities with mqtt_sender.cpp (which owns the per-cycle client):
//   - mqtt_sender subscribes to the manifest/install topics each cycle, routes every
//     MQTT_EVENT_DATA to ota_on_mqtt_data(), and calls ota_run_session() after a confirmed
//     state publish when ota_update_due() says an update is pending.
//   - this module owns the manifest/install state, the download state machine, the
//     esp_ota_* handle, SHA-256 verification, status publishing, and the reboot.

// Everything below `<device_id>/`:
inline constexpr std::string_view OTA_SUFFIX_MANIFEST  = "ota/manifest";   // retained JSON {"version","size","sha256","force"}
inline constexpr std::string_view OTA_SUFFIX_IMAGE     = "ota/image";      // retained raw .bin (broker-streamed)
inline constexpr std::string_view OTA_SUFFIX_INSTALL   = "ota/install";    // retained install request (HA Update entity's command_topic)
inline constexpr std::string_view OTA_SUFFIX_INSTALLED = "ota/installed";  // retained running-version string, published once per boot
inline constexpr std::string_view OTA_SUFFIX_STATUS    = "ota/status";     // non-retained progress/success/error JSON

// Call once from mqtt_sender_init(). Builds the full topic strings from device_id and keeps
// `link` for the OTA-window (rx-on-when-idle) hooks; both must outlive every publish cycle.
void ota_updater_init(std::string_view device_id, const NetworkLink *link);

// Full topic accessors (NUL-terminated, stable storage after ota_updater_init()).
const char *ota_topic_manifest();
const char *ota_topic_image();
const char *ota_topic_install();
const char *ota_topic_installed();
const char *ota_topic_status();

// Route every MQTT_EVENT_DATA here (esp-mqtt event-handler context). topic/topic_len are
// empty on continuation segments of a message larger than the client RX buffer; offset and
// total are esp-mqtt's current_data_offset/total_data_len. Blocking inside this call (flash
// writes) is deliberate: it stalls esp-mqtt's socket reads and lets TCP throttle the broker.
void ota_on_mqtt_data(const char *topic, size_t topic_len,
                      const char *data, size_t data_len,
                      size_t offset, size_t total);

// True once a retained manifest with a version different from the running image AND a
// retained install request have both been seen this cycle (and the per-boot attempt budget
// is not exhausted). Cleared implicitly by rebooting into the new image.
bool ota_update_due();

// True while a download session is running. The sensor task uses this to (a) not count the
// long-running publish cycle against the reboot supervisor and (b) skip its error blinks.
bool ota_session_in_progress();

// Run the download session on an already-CONNECTED client (mqtt_pub task context). On
// success this reboots into the new image and never returns. Returns on failure/deferral
// after restoring the sleepy link mode; the caller just proceeds with normal cycle teardown.
// battery_percent (when known) gates the session: below the threshold the update is
// deferred unless the manifest carries "force":true.
void ota_run_session(esp_mqtt_client_handle_t client, std::optional<int> battery_percent);
