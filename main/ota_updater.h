#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>

#include "mqtt_client.h"
#include "network_link.h"

// MQTT OTA updater. The firmware image is staged on the broker as N retained CHUNK messages
// (`<id>/ota/image/<n>`, tools/ota_push.py), each small enough to fit the client's RX buffer.
// The device pulls them strictly in order: subscribe to chunk n -> broker delivers the
// retained message -> esp_ota_write() -> subscribe to chunk n+1. Chunking is NOT an
// optimisation — it is a correctness requirement: esp-mqtt cannot survive a multi-second
// data gap while receiving a message LARGER than its RX buffer (deliver_publish() returns
// mid-message on a read timeout with no resume state, desyncing the parser — observed on
// hardware as "invalid header=0xcc" connection aborts mid-download over Thread). A message
// that fits the buffer is delivered atomically in one event, so gaps between chunks cost
// nothing and a gap inside a chunk at worst aborts the connection cleanly.
//
// Failed sessions RESUME: the flash handle/hash state survive across publish cycles (within
// one boot), so the next cycle continues from the first missing chunk instead of restarting.
//
// Split of responsibilities with mqtt_sender.cpp (which owns the per-cycle client):
//   - mqtt_sender subscribes to the manifest/install topics each cycle, routes every
//     MQTT_EVENT_DATA/ERROR/DISCONNECTED here, and calls ota_run_session() after a
//     confirmed state publish when ota_update_due() says an update is pending.
//   - this module owns the manifest/install state, the chunk-pull state machine, the
//     esp_ota_* handle, SHA-256 verification, status publishing, and the reboot.

// Everything below `<device_id>/`:
inline constexpr std::string_view OTA_SUFFIX_MANIFEST  = "ota/manifest";   // retained JSON {"version","size","sha256","chunk_size","force"}
inline constexpr std::string_view OTA_SUFFIX_IMAGE_PREFIX = "ota/image/";  // retained raw chunk at "<prefix><n>", n decimal from 0
inline constexpr std::string_view OTA_SUFFIX_INSTALL   = "ota/install";    // retained install request (HA Update entity's command_topic)
inline constexpr std::string_view OTA_SUFFIX_INSTALLED = "ota/installed";  // retained running-version string, published once per boot
inline constexpr std::string_view OTA_SUFFIX_STATUS    = "ota/status";     // non-retained progress/success/error JSON

// Largest chunk payload the device accepts; the manifest's chunk_size must not exceed it.
// tools/ota_push.py defaults to exactly this value. Kept moderate: a chunk transfers in
// ~1-2 s over Thread, so a radio stall rarely lands inside one, and the RX buffer (below)
// stays a small, per-cycle heap cost.
inline constexpr size_t OTA_MAX_CHUNK_SIZE = 8192;

// esp-mqtt RX buffer that guarantees a max-size chunk message (topic + headers + payload)
// is delivered as ONE event — the property the whole protocol rests on (see file comment).
inline constexpr size_t OTA_MQTT_RX_BUFFER_SIZE = OTA_MAX_CHUNK_SIZE + 512;

// Call once from mqtt_sender_init(). Builds the full topic strings from device_id and keeps
// `link` for the OTA-window (rx-on-when-idle) hooks; both must outlive every publish cycle.
void ota_updater_init(std::string_view device_id, const NetworkLink *link);

// Full topic accessors (NUL-terminated, stable storage after ota_updater_init()).
const char *ota_topic_manifest();
const char *ota_topic_install();
const char *ota_topic_installed();
const char *ota_topic_status();

// Route every MQTT_EVENT_DATA here (esp-mqtt event-handler context); demuxes by topic
// (manifest / install / image chunk). Blocking inside this call (flash writes) is fine:
// chunks are self-contained messages, so stalling esp-mqtt's reads only delays the next one.
void ota_on_mqtt_data(const char *topic, size_t topic_len,
                      const char *data, size_t data_len,
                      size_t offset, size_t total);

// Route MQTT_EVENT_ERROR / MQTT_EVENT_DISCONNECTED here (esp-mqtt event-handler context).
// Aborts an active session immediately with a "connection lost" verdict; the partial
// download is kept for resume. No-op when no session is running.
void ota_on_mqtt_error();

// True once a retained manifest with a version different from the running image AND a
// retained install request have both been seen this cycle (and the consecutive-no-progress
// attempt budget is not exhausted). Cleared implicitly by rebooting into the new image.
bool ota_update_due();

// True while a download session is running. The sensor task uses this to (a) not count the
// long-running publish cycle against the reboot supervisor and (b) skip its error blinks.
bool ota_session_in_progress();

// Run the chunk-pull session on an already-CONNECTED client (mqtt_pub task context). On
// success this reboots into the new image and never returns. Returns on failure/deferral
// after restoring the sleepy link mode; a partial download is kept and resumed on the next
// call (same staged image only). battery_percent (when known) gates the session: below the
// threshold the update is deferred unless the manifest carries "force":true.
void ota_run_session(esp_mqtt_client_handle_t client, std::optional<int> battery_percent);
