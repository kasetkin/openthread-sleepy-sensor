#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>

#include "mqtt_client.h"
#include "lp_sensor_core.h"

// HA-tunable device parameters over MQTT, applied without a reflash. Split of responsibilities
// with mqtt_sender.cpp (which owns the per-cycle client), mirroring ota_updater.h/.cpp's shape:
//   - mqtt_sender subscribes to the wildcard below each cycle, routes every MQTT_EVENT_DATA
//     here, and calls runtime_config_apply_pending() after the cycle's own publish work.
//   - this module owns the pending-change state, the NVS persistence, the live application
//     (LP shared-memory config block for the 7 numeric fields, enableExtAntenna() for the 8th),
//     and the boot-time NVS-override resolver main.cpp uses.
//
// Every topic is retained (the device is not a long-lived MQTT client -- see mqtt_sender.cpp --
// so a briefly-connected wake could miss a non-retained command), and every payload is a bare
// scalar, not JSON: this lets HA's MQTT `number`/`switch` entities use state_topic ==
// command_topic (their documented pattern for reading current value back from the same retained
// topic they command), and keeps this module consistent with the project's hand-parsed-over-
// structured convention. The device republishes its own retained echo of the applied (post-
// clamp) value after accepting a change, so HA's display always reflects what was actually
// applied.

// Everything below `<device_id>/`.
inline constexpr std::string_view CFG_SUFFIX_TEMP_OFFSET        = "cfg/temp_offset";
inline constexpr std::string_view CFG_SUFFIX_TEMP_MIN_CHANGE    = "cfg/temp_min_change";
inline constexpr std::string_view CFG_SUFFIX_RH_OFFSET          = "cfg/rh_offset";
inline constexpr std::string_view CFG_SUFFIX_RH_MIN_CHANGE      = "cfg/rh_min_change";
inline constexpr std::string_view CFG_SUFFIX_MAX_SKIP_CYCLES    = "cfg/max_skip_cycles";
inline constexpr std::string_view CFG_SUFFIX_HEATER_PERIOD_MIN  = "cfg/heater_period_minutes";
inline constexpr std::string_view CFG_SUFFIX_HEATER_HIGH_RH_MIN = "cfg/heater_high_rh_trigger_minutes";
inline constexpr std::string_view CFG_SUFFIX_EXT_ANTENNA        = "cfg/ext_antenna";
// One SUBSCRIBE for all 8 topics, not 8 -- minimizes SUBSCRIBE-packet overhead in the brief
// per-cycle awake window (see run_publish_cycle()'s existing manifest/install subscribes).
inline constexpr std::string_view CFG_SUFFIX_WILDCARD           = "cfg/#";

inline constexpr std::string_view EXT_ANTENNA_PAYLOAD_ON  = "ON";
inline constexpr std::string_view EXT_ANTENNA_PAYLOAD_OFF = "OFF";

// Validation ranges -- the ONE source both HA's `number` discovery min/max/step
// (mqtt_sender.cpp) and this module's own clamp-on-apply read, so they can't drift apart.
inline constexpr float TEMP_OFFSET_MIN = -10.0f;
inline constexpr float TEMP_OFFSET_MAX = 10.0f;
inline constexpr float TEMP_OFFSET_STEP = 0.1f;
inline constexpr float TEMP_MIN_CHANGE_MIN = 0.05f;
inline constexpr float TEMP_MIN_CHANGE_MAX = 5.0f;
inline constexpr float TEMP_MIN_CHANGE_STEP = 0.05f;
inline constexpr float RH_OFFSET_MIN = -20.0f;
inline constexpr float RH_OFFSET_MAX = 20.0f;
inline constexpr float RH_OFFSET_STEP = 0.5f;
inline constexpr float RH_MIN_CHANGE_MIN = 0.5f;
inline constexpr float RH_MIN_CHANGE_MAX = 20.0f;
inline constexpr float RH_MIN_CHANGE_STEP = 0.5f;
inline constexpr uint32_t MAX_SKIP_CYCLES_MIN = 0;
inline constexpr uint32_t MAX_SKIP_CYCLES_MAX = 1000;
inline constexpr uint32_t MAX_SKIP_CYCLES_STEP = 1;
// Up to 1 week; 0 = disabled.
inline constexpr uint32_t HEATER_PERIOD_MIN_MINUTES = 0;
inline constexpr uint32_t HEATER_PERIOD_MAX_MINUTES = 10080;
inline constexpr uint32_t HEATER_PERIOD_STEP_MINUTES = 1;
// Up to 24h; 0 = disabled.
inline constexpr uint32_t HEATER_HIGH_RH_MIN_MINUTES = 0;
inline constexpr uint32_t HEATER_HIGH_RH_MAX_MINUTES = 1440;
inline constexpr uint32_t HEATER_HIGH_RH_STEP_MINUTES = 1;

// Call once from main.cpp, right after lp_sensor_core_start() succeeds. `boot_config` seeds
// this module's shadow of the 7 numeric fields (lp_sensor_core_apply_config() always writes
// the full struct, so a partial live change still needs the other fields' current values);
// `poll_interval_sec` is the boot's fixed LP poll interval, needed to convert a live
// heater-minutes change to LP cycles the same way main.cpp's minutes_to_lp_cycles() does at
// boot (lp_poll_interval_sec itself is NOT retunable live -- out of scope for this feature).
// `heater_period_minutes`/`heater_high_rh_trigger_minutes`/`ext_antenna_on` are the same
// boot-resolved values `boot_config`/enableExtAntenna() were already built from, in their
// HA-facing units (minutes, not LP cycles; boot_config only carries the cycle-converted form) --
// needed so runtime_config_current_values() has a real value to report from the very first boot.
void runtime_config_init(std::string_view device_id, uint32_t poll_interval_sec,
                          const lp_sensor_core_config_t &boot_config,
                          uint32_t heater_period_minutes, uint32_t heater_high_rh_trigger_minutes,
                          bool ext_antenna_on);

// The 8 HA-tunable parameters' current resolved value, in HA-facing units (heater fields in
// minutes, not LP cycles) -- whichever is freshest of the boot default/NVS override or the
// latest MQTT change accepted since. Used by mqtt_sender.cpp to publish each cfg/* topic's
// retained state alongside its discovery config (see publish_number_discovery()/
// publish_switch_discovery()), so HA never shows a number/switch entity as "Unknown" simply
// because it has never been commanded.
struct RuntimeConfigValues
{
    float temp_offset_c;
    float temp_min_change_c;
    float rh_offset_pct;
    float rh_min_change_pct;
    uint32_t max_skip_cycles;
    uint32_t heater_period_minutes;
    uint32_t heater_high_rh_trigger_minutes;
    bool ext_antenna_on;
};
RuntimeConfigValues runtime_config_current_values();

// Stable NUL-terminated topic accessors, mirroring ota_topic_*().
const char *runtime_config_topic_wildcard();
const char *runtime_config_topic_temp_offset();
const char *runtime_config_topic_temp_min_change();
const char *runtime_config_topic_rh_offset();
const char *runtime_config_topic_rh_min_change();
const char *runtime_config_topic_max_skip_cycles();
const char *runtime_config_topic_heater_period_minutes();
const char *runtime_config_topic_heater_high_rh_trigger_minutes();
const char *runtime_config_topic_ext_antenna();

// Route every MQTT_EVENT_DATA here (esp-mqtt event-handler context), alongside the sibling
// ota_on_mqtt_data() call. Parses the (small, single-event, never chunked) payload,
// range-clamps it, and stores it in a mutex-guarded pending struct for later application.
// ONLY sets state -- never touches the client, matching mqtt_event_handler()'s invariant.
void runtime_config_on_mqtt_data(const char *topic, size_t topic_len,
                                  const char *data, size_t data_len);

// Task-context only: call from run_publish_cycle() (mqtt_pub task) on a CONNECTED client,
// after the cycle's own state/discovery publish work. Applies every field changed since the
// last call: merges into the shadow LP config and writes it live via
// lp_sensor_core_apply_config() (for the 7 calibration/threshold/heater-schedule fields) or
// calls enableExtAntenna() directly (for ext_antenna), persists each changed field to NVS,
// and publishes a retained echo of the applied (post-clamp) value on its own cfg/* topic.
// Returns the number of fields applied (0 if nothing was pending).
int runtime_config_apply_pending(esp_mqtt_client_handle_t client);

// Boot-time NVS-override resolution -- wraps the existing parse_as_float_or()/
// parse_as_uint32_or() result (or, for ext_antenna, the compiled default -- no YAML tier
// exists for it). Returns it unchanged if no override was ever stored under `nvs_key` in the
// "rtcfg" namespace; the stored override otherwise. Call only after initNvsFlash().
float    runtime_config_nvs_override(float yaml_or_default, const char *nvs_key);
uint32_t runtime_config_nvs_override(uint32_t yaml_or_default, const char *nvs_key);
bool     runtime_config_nvs_override(bool yaml_or_default, const char *nvs_key);
