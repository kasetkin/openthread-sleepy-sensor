#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>

#include "mqtt_client.h"
#include "lp_sensor_core.h"
#include "network_link.h"

// HA-tunable device parameters over MQTT, applied without a reflash. Split of responsibilities
// with mqtt_sender.cpp (which owns the per-cycle client), mirroring ota_updater.h/.cpp's shape:
//   - mqtt_sender subscribes to the wildcard below each cycle, routes every MQTT_EVENT_DATA
//     here, and calls runtime_config_apply_pending() after the cycle's own publish work.
//   - this module owns the pending-change state, the NVS persistence, the live application
//     (LP shared-memory config block for the 8 numeric fields, enableExtAntenna() for the 9th),
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
inline constexpr std::string_view CFG_SUFFIX_MAX_PUBLISH_GAP_SEC = "cfg/max_publish_gap_sec";
inline constexpr std::string_view CFG_SUFFIX_HEATER_PERIOD_MIN  = "cfg/heater_period_minutes";
inline constexpr std::string_view CFG_SUFFIX_HEATER_HIGH_RH_MIN = "cfg/heater_high_rh_trigger_minutes";
inline constexpr std::string_view CFG_SUFFIX_EXT_ANTENNA        = "cfg/ext_antenna";
inline constexpr std::string_view CFG_SUFFIX_TX_POWER_DBM       = "cfg/tx_power_dbm";
inline constexpr std::string_view CFG_SUFFIX_SENSOR_SAMPLES     = "cfg/sensor_samples";
inline constexpr std::string_view CFG_SUFFIX_RTC_CAL_MODE       = "cfg/rtc_cal_mode";
inline constexpr std::string_view CFG_SUFFIX_RTC_TRIM_MODE      = "cfg/rtc_trim_mode";
// One SUBSCRIBE for all 12 topics, not one per topic -- minimizes SUBSCRIBE-packet overhead in
// the brief per-cycle awake window (see run_publish_cycle()'s existing manifest/install subscribes).
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
// A fixed, lp_poll_interval_sec-independent ceiling (unlike the old cycle-count cap, whose
// real-world meaning silently scaled with whatever poll interval was configured): 6h is a
// generous but sane worst-case staleness bound for a sensor whose whole point is periodic
// reporting. Step is 1s, not something coarser -- the HA entity is mode:"box" (free-text, not a
// slider), so a coarser step buys no UI benefit and only risks rejecting a value the device would
// otherwise accept and gracefully floor to the nearest whole poll.
inline constexpr uint32_t MAX_PUBLISH_GAP_SEC_MIN = 0;
inline constexpr uint32_t MAX_PUBLISH_GAP_SEC_MAX = 21600;
inline constexpr uint32_t MAX_PUBLISH_GAP_SEC_STEP = 1;
// Up to 1 week; 0 = disabled.
inline constexpr uint32_t HEATER_PERIOD_MIN_MINUTES = 0;
inline constexpr uint32_t HEATER_PERIOD_MAX_MINUTES = 10080;
inline constexpr uint32_t HEATER_PERIOD_STEP_MINUTES = 1;
// Up to 24h; 0 = disabled.
inline constexpr uint32_t HEATER_HIGH_RH_MIN_MINUTES = 0;
inline constexpr uint32_t HEATER_HIGH_RH_MAX_MINUTES = 1440;
inline constexpr uint32_t HEATER_HIGH_RH_STEP_MINUTES = 1;
// dBm is always integral (the PHY table itself quantizes to whole dBm), so this stays a signed
// integer end to end rather than float -- see runtime_config.cpp's design note.
inline constexpr int32_t TX_POWER_DBM_MIN = -15;
inline constexpr int32_t TX_POWER_DBM_MAX = 20;
inline constexpr int32_t TX_POWER_DBM_STEP = 1;
// The radio's PHY-table ceiling AND the always-safe fallback: current pre-feature behavior,
// applied before the first successful attach, during every OTA window, and whenever no
// known-good value has ever been confirmed.
inline constexpr int8_t TX_POWER_TABLE_MAX_DBM = 20;
// How many raw SHT4x reads the LP core averages into one reported value per poll (see
// shared_layout.h's sensor_samples comment). MAX=16 is a generous UI ceiling, not a
// recommendation -- each extra sample costs one more ~100ms inter-sample delay
// (lp_core/main.cpp's kInterSampleDelayUs) worth of LP-core active time per poll.
inline constexpr uint32_t SENSOR_SAMPLES_MIN = 1;
inline constexpr uint32_t SENSOR_SAMPLES_MAX = 16;
inline constexpr uint32_t SENSOR_SAMPLES_STEP = 1;
// The light-sleep clock fix's modes (rtc_clock_fix.h), live from the next sleep. Calibration:
// 0 = ESP-IDF's own at sleep entry, 1 = the latest cold one, 2 = the mean of the last 32 cold
// ones. Trim: 0 = off, 1 = the latest NTP interval's, 2 = the mean of the last 8 intervals'.
inline constexpr uint32_t RTC_CAL_MODE_MIN = 0;
inline constexpr uint32_t RTC_CAL_MODE_MAX = 2;
inline constexpr uint32_t RTC_CAL_MODE_STEP = 1;
inline constexpr uint32_t RTC_TRIM_MODE_MIN = 0;
inline constexpr uint32_t RTC_TRIM_MODE_MAX = 2;
inline constexpr uint32_t RTC_TRIM_MODE_STEP = 1;

// Call once from main.cpp, right after lp_sensor_core_start() succeeds. `boot_config` seeds
// this module's shadow of the 8 numeric fields (lp_sensor_core_apply_config() always writes
// the full struct, so a partial live change still needs the other fields' current values);
// `poll_interval_sec` is the boot's fixed LP poll interval, needed to convert a live
// heater-minutes change to LP cycles the same way main.cpp's minutes_to_lp_cycles() does at
// boot (lp_poll_interval_sec itself is NOT retunable live -- out of scope for this feature).
// `heater_period_minutes`/`heater_high_rh_trigger_minutes`/`max_publish_gap_sec`/`ext_antenna_on`
// are the same boot-resolved values `boot_config`/enableExtAntenna() were already built from, in
// their HA-facing units (minutes/seconds, not LP cycles; boot_config only carries the
// cycle-converted form) -- needed so runtime_config_current_values() has a real value to report
// from the very first boot. `tx_power_known_good_dbm` is the boot-resolved TX power ceiling
// (NVS override -> device_config.yaml -> table max); `link` lets this module call
// NetworkLink::setTxPowerDbm() itself for the post-attach apply / live-change / OTA-pin-restore
// paths -- see the TX power section below.
void runtime_config_init(std::string_view device_id, uint32_t poll_interval_sec,
                          const lp_sensor_core_config_t &boot_config,
                          uint32_t max_publish_gap_sec,
                          uint32_t heater_period_minutes, uint32_t heater_high_rh_trigger_minutes,
                          bool ext_antenna_on, int32_t tx_power_known_good_dbm,
                          const NetworkLink *link);

// The 12 HA-tunable parameters' current resolved value, in HA-facing units (heater fields in
// minutes, max_publish_gap in seconds -- neither in LP cycles) -- whichever is freshest of the
// boot default/NVS override or the latest MQTT change accepted since. Used by mqtt_sender.cpp to
// publish each cfg/* topic's retained state alongside its discovery config (see
// publish_number_discovery()/publish_switch_discovery()), so HA never shows a number/switch
// entity as "Unknown" simply because it has never been commanded. tx_power_dbm here can show an
// outstanding, still-unconfirmed trial -- see runtime_config_tx_power_active_dbm() below for
// "what's actually in effect right now".
struct RuntimeConfigValues
{
    float temp_offset_c;
    float temp_min_change_c;
    float rh_offset_pct;
    float rh_min_change_pct;
    uint32_t max_publish_gap_sec;
    uint32_t heater_period_minutes;
    uint32_t heater_high_rh_trigger_minutes;
    bool ext_antenna_on;
    int32_t tx_power_dbm;
    uint32_t sensor_samples;
    uint32_t rtc_cal_mode;
    uint32_t rtc_trim_mode;
};
RuntimeConfigValues runtime_config_current_values();

// Stable NUL-terminated topic accessors, mirroring ota_topic_*().
const char *runtime_config_topic_wildcard();
const char *runtime_config_topic_temp_offset();
const char *runtime_config_topic_temp_min_change();
const char *runtime_config_topic_rh_offset();
const char *runtime_config_topic_rh_min_change();
const char *runtime_config_topic_max_publish_gap_sec();
const char *runtime_config_topic_heater_period_minutes();
const char *runtime_config_topic_heater_high_rh_trigger_minutes();
const char *runtime_config_topic_ext_antenna();
const char *runtime_config_topic_tx_power_dbm();
const char *runtime_config_topic_sensor_samples();
const char *runtime_config_topic_rtc_cal_mode();
const char *runtime_config_topic_rtc_trim_mode();

// ── TX power (Phase B) ──────────────────────────────────────────────────────────────────────
// A tx_power_dbm value low enough to break the uplink would strand the device (its only
// recovery path, OTA over MQTT over Thread, needs exactly the link a bad value just broke), so
// unlike the other 8 cfg/* fields this one goes through a pending/known-good/revert state
// machine before a change is trusted. See runtime_config.cpp for the full design.

// "TX power (active)" diagnostic value -- separate from RuntimeConfigValues::tx_power_dbm
// (which can show an unconfirmed trial): whatever dBm the ordinary policy last actually applied.
int8_t runtime_config_tx_power_active_dbm();

// Call once, idempotently, the first time (this boot) NetworkLink::waitForReady() succeeds. Safe
// to call on every subsequent successful attach too -- only the first call this boot does
// anything. Before this fires the radio stays at its native table-max power regardless of any
// persisted state. A trial left outstanding by a PREVIOUS boot is treated as failed, not resumed.
void runtime_config_tx_power_note_first_attach();

// Call once per sensor-task wake to report this cycle's outcome for TX-power confirm/revert
// purposes. Needs TWO tap points in sensorstask.cpp -- both a Thread attach failure and an
// MQTT publish failure are evidence a bad TX power broke the link; see sensorstask.cpp's call
// sites. No-op if no trial is outstanding.
void runtime_config_tx_power_note_cycle_result(bool ok);

// OTA-window bracket -- call adjacent to NetworkLink::onOtaWindowBegin()/onOtaWindowEnd() in
// ota_updater.cpp, not folded into those seams (TX-power pin/restore is transport-agnostic
// bookkeeping that belongs here, not duplicated per-transport).
void runtime_config_tx_power_pin_max_for_ota();
void runtime_config_tx_power_unpin_after_ota();

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
int32_t  runtime_config_nvs_override(int32_t yaml_or_default, const char *nvs_key);
