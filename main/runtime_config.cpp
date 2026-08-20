#include "runtime_config.h"

#include <algorithm>
#include <charconv>
#include <cstring>
#include <format>
#include <string>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs.h"

#include "common_utils.h"

static const char *TAG = "runtime-config";
static constexpr const char *NVS_NAMESPACE = "rtcfg";

// ── state ─────────────────────────────────────────────────────────────────────
static std::string s_topic_wildcard;
static std::string s_topic_temp_offset;
static std::string s_topic_temp_min_change;
static std::string s_topic_rh_offset;
static std::string s_topic_rh_min_change;
static std::string s_topic_max_publish_gap_sec;
static std::string s_topic_heater_period_minutes;
static std::string s_topic_heater_high_rh_trigger_minutes;
static std::string s_topic_ext_antenna;
static std::string s_topic_sensor_samples;

static uint32_t s_poll_interval_sec = 20;
// Always-current shadow of the 7 calibration/threshold/heater-schedule fields, seeded from
// the boot config and merged in-place as changes are accepted -- lp_sensor_core_apply_config()
// always writes the full struct, so a partial change (e.g. only temp_offset) still needs the
// other fields' current values, not stale defaults.
static lp_sensor_core_config_t s_shadow{};

// The 5 of the 10 HA-tunable parameters NOT already trackable from s_shadow in HA-facing units:
// the heater fields and max_publish_gap_sec live in s_shadow as LP cycles (not the minutes/
// seconds HA displays), antenna selection isn't part of s_shadow at all, and TX power has its
// own dedicated state machine below (s_tx_power_*) rather than living in s_shadow at all.
// (sensor_samples is NOT one of these 5 -- it has no unit conversion, so it's read straight off
// s_shadow.sensor_samples like temp_offset_c etc.) Seeded at init,
// updated in apply_pending() alongside the existing shadow/NVS/echo updates for these fields --
// backs runtime_config_current_values().
static uint32_t s_max_publish_gap_sec = 0;
static uint32_t s_heater_period_minutes = 0;
static uint32_t s_heater_high_rh_trigger_minutes = 0;
static bool s_ext_antenna_on = false;

// ── TX power (Phase B) ───────────────────────────────────────────────────────────────────────
// Runtime state is genuinely dBm-range (int8_t), unlike the wider int32_t used at the
// MQTT-parse/HA-facing boundary (PendingCfg, RuntimeConfigValues) -- narrowed once, right after
// the clamp, and never again. s_tx_power_has_pending true means a trial is outstanding and not
// yet confirmed; s_tx_power_known_good is what a revert falls back to.
static const NetworkLink *s_link = nullptr;
static std::string s_topic_tx_power_dbm;
static int8_t s_tx_power_known_good = TX_POWER_TABLE_MAX_DBM;
static int8_t s_tx_power_pending = TX_POWER_TABLE_MAX_DBM;
static bool s_tx_power_has_pending = false;
static uint32_t s_tx_power_unconfirmed_cycles = 0;
// What the radio is ACTUALLY set to right now -- stays at table max until the first successful
// attach (see runtime_config_tx_power_note_first_attach()), independent of any persisted state.
static int8_t s_tx_power_active_dbm = TX_POWER_TABLE_MAX_DBM;
static bool s_tx_power_first_attach_done = false;
// Deliberately shorter than UNCONFIRMED_OTA_REBOOT_AFTER_CYCLES (5, sensorstask.h): a bad TX
// power is higher-severity than an unconfirmed OTA image -- no reboot/rollback fixes it, only
// physical access does -- so the grace window should be the tighter of the two.
static constexpr uint32_t TX_POWER_REVERT_AFTER_CYCLES = 3;

// Every value HA has sent since the last runtime_config_apply_pending() call, guarded by
// s_mutex. Mirrors ota_updater.cpp's Manifest/s_mutex pattern: the event-handler-context
// writer takes the mutex only for a fast, non-blocking struct copy.
struct PendingCfg
{
    bool temp_offset_set = false;
    bool temp_min_change_set = false;
    bool rh_offset_set = false;
    bool rh_min_change_set = false;
    bool max_publish_gap_sec_set = false;
    bool heater_period_set = false;
    bool heater_high_rh_set = false;
    bool ext_antenna_set = false;
    bool tx_power_dbm_set = false;
    bool sensor_samples_set = false;

    float temp_offset_c = 0.0f;
    float temp_min_change_c = 0.0f;
    float rh_offset_pct = 0.0f;
    float rh_min_change_pct = 0.0f;
    uint32_t max_publish_gap_sec = 0;
    uint32_t heater_period_minutes = 0;
    uint32_t heater_high_rh_trigger_minutes = 0;
    bool ext_antenna_on = false;
    int32_t tx_power_dbm = 0;
    uint32_t sensor_samples = 0;
};
static PendingCfg s_pending;
static SemaphoreHandle_t s_mutex = nullptr;

// ── helpers ───────────────────────────────────────────────────────────────────
static bool topic_is(const char *topic, size_t topic_len, const std::string &full)
{
    return topic_len == full.size() && std::memcmp(topic, full.data(), topic_len) == 0;
}

static bool parse_float(const char *data, size_t len, float &out)
{
    const auto [ptr, ec] = std::from_chars(data, data + len, out);
    return ec == std::errc{} && ptr == data + len;
}

static bool parse_uint32(const char *data, size_t len, uint32_t &out)
{
    const auto [ptr, ec] = std::from_chars(data, data + len, out);
    return ec == std::errc{} && ptr == data + len;
}

static bool parse_int32(const char *data, size_t len, int32_t &out)
{
    const auto [ptr, ec] = std::from_chars(data, data + len, out);
    return ec == std::errc{} && ptr == data + len;
}

// device_config.yaml expresses the heater schedule in wall-clock minutes; the LP program
// counts poll cycles. Mirrors main.cpp's minutes_to_lp_cycles() -- kept as a local copy
// rather than a shared header, matching this codebase's light-duplication-over-cross-module-
// coupling convention for one 3-line helper. Keep the two in sync if either changes.
static uint32_t minutes_to_lp_cycles(uint32_t minutes, uint32_t poll_sec)
{
    if (minutes == 0)
        return 0;
    return (minutes * 60u + poll_sec - 1) / poll_sec;
}

// HA expresses the guaranteed max publish gap in wall-clock seconds; the LP program counts
// skipped polls. Floors (unlike minutes_to_lp_cycles above) so the actual guarantee
// (result+1)*poll_sec never exceeds what was requested -- ceiling here would let the real gap
// overshoot the promise by almost a full poll period, which is the wrong direction for a
// staleness bound (unlike the heater schedule, where overshooting is the safe direction). A
// request at or below one poll period is the finest granularity achievable (LP only samples
// every poll_sec) and maps to 0 -- publish every cycle, same as this field's old "0 = fail-safe"
// meaning. Mirrors main.cpp's copy of this same helper -- keep the two in sync if either changes.
static uint32_t publish_gap_sec_to_skip_cycles(uint32_t gap_sec, uint32_t poll_sec)
{
    if (gap_sec <= poll_sec)
        return 0;
    return gap_sec / poll_sec - 1;
}

// ── public API ────────────────────────────────────────────────────────────────

void runtime_config_init(std::string_view device_id, uint32_t poll_interval_sec,
                          const lp_sensor_core_config_t &boot_config,
                          uint32_t max_publish_gap_sec,
                          uint32_t heater_period_minutes, uint32_t heater_high_rh_trigger_minutes,
                          bool ext_antenna_on, int32_t tx_power_known_good_dbm,
                          const NetworkLink *link)
{
    s_poll_interval_sec = poll_interval_sec;
    s_shadow = boot_config;
    s_max_publish_gap_sec = max_publish_gap_sec;
    s_heater_period_minutes = heater_period_minutes;
    s_heater_high_rh_trigger_minutes = heater_high_rh_trigger_minutes;
    s_ext_antenna_on = ext_antenna_on;

    s_link = link;
    s_tx_power_known_good = static_cast<int8_t>(
        std::clamp(tx_power_known_good_dbm, TX_POWER_DBM_MIN, TX_POWER_DBM_MAX));
    // Radio genuinely stays at table max until the first successful attach -- see
    // runtime_config_tx_power_note_first_attach() -- regardless of whatever known-good/pending
    // state gets loaded from NVS below.
    s_tx_power_active_dbm = TX_POWER_TABLE_MAX_DBM;

    const auto full = [&](std::string_view suffix) {
        return std::format("{}/{}", device_id, suffix);
    };
    s_topic_wildcard                       = full(CFG_SUFFIX_WILDCARD);
    s_topic_temp_offset                    = full(CFG_SUFFIX_TEMP_OFFSET);
    s_topic_temp_min_change                = full(CFG_SUFFIX_TEMP_MIN_CHANGE);
    s_topic_rh_offset                      = full(CFG_SUFFIX_RH_OFFSET);
    s_topic_rh_min_change                  = full(CFG_SUFFIX_RH_MIN_CHANGE);
    s_topic_max_publish_gap_sec             = full(CFG_SUFFIX_MAX_PUBLISH_GAP_SEC);
    s_topic_heater_period_minutes          = full(CFG_SUFFIX_HEATER_PERIOD_MIN);
    s_topic_heater_high_rh_trigger_minutes = full(CFG_SUFFIX_HEATER_HIGH_RH_MIN);
    s_topic_ext_antenna                    = full(CFG_SUFFIX_EXT_ANTENNA);
    s_topic_tx_power_dbm                   = full(CFG_SUFFIX_TX_POWER_DBM);
    s_topic_sensor_samples                 = full(CFG_SUFFIX_SENSOR_SAMPLES);

    // A trial left outstanding by a previous boot -- runtime_config_tx_power_note_first_attach()
    // decides what to do with it (never resumed, always treated as failed).
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) == ESP_OK) {
        uint8_t pendFlag = 0;
        if (nvs_get_u8(nvs, "txp_pend_flag", &pendFlag) == ESP_OK && pendFlag) {
            int8_t pending = TX_POWER_TABLE_MAX_DBM;
            if (nvs_get_i8(nvs, "txp_pending", &pending) == ESP_OK) {
                s_tx_power_pending = pending;
                s_tx_power_has_pending = true;
            }
        }
        nvs_close(nvs);
    }

    if (!s_mutex)
        s_mutex = xSemaphoreCreateMutex();
}

const char *runtime_config_topic_wildcard()                       { return s_topic_wildcard.c_str(); }
const char *runtime_config_topic_temp_offset()                    { return s_topic_temp_offset.c_str(); }
const char *runtime_config_topic_temp_min_change()                { return s_topic_temp_min_change.c_str(); }
const char *runtime_config_topic_rh_offset()                      { return s_topic_rh_offset.c_str(); }
const char *runtime_config_topic_rh_min_change()                  { return s_topic_rh_min_change.c_str(); }
const char *runtime_config_topic_max_publish_gap_sec()            { return s_topic_max_publish_gap_sec.c_str(); }
const char *runtime_config_topic_heater_period_minutes()          { return s_topic_heater_period_minutes.c_str(); }
const char *runtime_config_topic_heater_high_rh_trigger_minutes() { return s_topic_heater_high_rh_trigger_minutes.c_str(); }
const char *runtime_config_topic_ext_antenna()                    { return s_topic_ext_antenna.c_str(); }
const char *runtime_config_topic_tx_power_dbm()                   { return s_topic_tx_power_dbm.c_str(); }
const char *runtime_config_topic_sensor_samples()                 { return s_topic_sensor_samples.c_str(); }

void runtime_config_on_mqtt_data(const char *topic, size_t topic_len,
                                  const char *data, size_t data_len)
{
    if (topic_len == 0)
        return;

    if (topic_is(topic, topic_len, s_topic_temp_offset)) {
        float v;
        if (!parse_float(data, data_len, v))
            return;
        v = std::clamp(v, TEMP_OFFSET_MIN, TEMP_OFFSET_MAX);
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        s_pending.temp_offset_c = v;
        s_pending.temp_offset_set = true;
        xSemaphoreGive(s_mutex);
    } else if (topic_is(topic, topic_len, s_topic_temp_min_change)) {
        float v;
        if (!parse_float(data, data_len, v))
            return;
        v = std::clamp(v, TEMP_MIN_CHANGE_MIN, TEMP_MIN_CHANGE_MAX);
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        s_pending.temp_min_change_c = v;
        s_pending.temp_min_change_set = true;
        xSemaphoreGive(s_mutex);
    } else if (topic_is(topic, topic_len, s_topic_rh_offset)) {
        float v;
        if (!parse_float(data, data_len, v))
            return;
        v = std::clamp(v, RH_OFFSET_MIN, RH_OFFSET_MAX);
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        s_pending.rh_offset_pct = v;
        s_pending.rh_offset_set = true;
        xSemaphoreGive(s_mutex);
    } else if (topic_is(topic, topic_len, s_topic_rh_min_change)) {
        float v;
        if (!parse_float(data, data_len, v))
            return;
        v = std::clamp(v, RH_MIN_CHANGE_MIN, RH_MIN_CHANGE_MAX);
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        s_pending.rh_min_change_pct = v;
        s_pending.rh_min_change_set = true;
        xSemaphoreGive(s_mutex);
    } else if (topic_is(topic, topic_len, s_topic_max_publish_gap_sec)) {
        uint32_t v;
        if (!parse_uint32(data, data_len, v))
            return;
        v = std::clamp(v, MAX_PUBLISH_GAP_SEC_MIN, MAX_PUBLISH_GAP_SEC_MAX);
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        s_pending.max_publish_gap_sec = v;
        s_pending.max_publish_gap_sec_set = true;
        xSemaphoreGive(s_mutex);
    } else if (topic_is(topic, topic_len, s_topic_heater_period_minutes)) {
        uint32_t v;
        if (!parse_uint32(data, data_len, v))
            return;
        v = std::clamp(v, HEATER_PERIOD_MIN_MINUTES, HEATER_PERIOD_MAX_MINUTES);
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        s_pending.heater_period_minutes = v;
        s_pending.heater_period_set = true;
        xSemaphoreGive(s_mutex);
    } else if (topic_is(topic, topic_len, s_topic_heater_high_rh_trigger_minutes)) {
        uint32_t v;
        if (!parse_uint32(data, data_len, v))
            return;
        v = std::clamp(v, HEATER_HIGH_RH_MIN_MINUTES, HEATER_HIGH_RH_MAX_MINUTES);
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        s_pending.heater_high_rh_trigger_minutes = v;
        s_pending.heater_high_rh_set = true;
        xSemaphoreGive(s_mutex);
    } else if (topic_is(topic, topic_len, s_topic_ext_antenna)) {
        const std::string_view payload(data, data_len);
        bool value;
        if (payload == EXT_ANTENNA_PAYLOAD_ON) {
            value = true;
        } else if (payload == EXT_ANTENNA_PAYLOAD_OFF) {
            value = false;
        } else {
            ESP_LOGW(TAG, "cfg/ext_antenna: unrecognized payload '%.*s' — ignored",
                     static_cast<int>(data_len), data);
            return;
        }
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        s_pending.ext_antenna_on = value;
        s_pending.ext_antenna_set = true;
        xSemaphoreGive(s_mutex);
    } else if (topic_is(topic, topic_len, s_topic_tx_power_dbm)) {
        int32_t v;
        if (!parse_int32(data, data_len, v))
            return;
        v = std::clamp(v, TX_POWER_DBM_MIN, TX_POWER_DBM_MAX);
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        s_pending.tx_power_dbm = v;
        s_pending.tx_power_dbm_set = true;
        xSemaphoreGive(s_mutex);
    } else if (topic_is(topic, topic_len, s_topic_sensor_samples)) {
        uint32_t v;
        if (!parse_uint32(data, data_len, v))
            return;
        v = std::clamp(v, SENSOR_SAMPLES_MIN, SENSOR_SAMPLES_MAX);
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        s_pending.sensor_samples = v;
        s_pending.sensor_samples_set = true;
        xSemaphoreGive(s_mutex);
    }
}

int runtime_config_apply_pending(esp_mqtt_client_handle_t client)
{
    PendingCfg snap;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    snap = s_pending;
    s_pending = PendingCfg{};
    xSemaphoreGive(s_mutex);

    int applied = 0;
    bool lpChanged = false;

    if (snap.temp_offset_set)      { s_shadow.temp_offset_c = snap.temp_offset_c; lpChanged = true; ++applied; }
    if (snap.temp_min_change_set)  { s_shadow.temp_min_change_c = snap.temp_min_change_c; lpChanged = true; ++applied; }
    if (snap.rh_offset_set)        { s_shadow.rh_offset_pct = snap.rh_offset_pct; lpChanged = true; ++applied; }
    if (snap.rh_min_change_set)    { s_shadow.rh_min_change_pct = snap.rh_min_change_pct; lpChanged = true; ++applied; }
    if (snap.sensor_samples_set)   { s_shadow.sensor_samples = snap.sensor_samples; lpChanged = true; ++applied; }
    if (snap.max_publish_gap_sec_set) {
        s_shadow.max_skip_cycles = publish_gap_sec_to_skip_cycles(snap.max_publish_gap_sec, s_poll_interval_sec);
        s_max_publish_gap_sec = snap.max_publish_gap_sec;
        lpChanged = true;
        ++applied;
    }
    if (snap.heater_period_set) {
        s_shadow.heater_period_cycles = minutes_to_lp_cycles(snap.heater_period_minutes, s_poll_interval_sec);
        s_heater_period_minutes = snap.heater_period_minutes;
        lpChanged = true;
        ++applied;
    }
    if (snap.heater_high_rh_set) {
        s_shadow.high_rh_trigger_cycles = minutes_to_lp_cycles(snap.heater_high_rh_trigger_minutes, s_poll_interval_sec);
        s_heater_high_rh_trigger_minutes = snap.heater_high_rh_trigger_minutes;
        lpChanged = true;
        ++applied;
    }
    if (lpChanged)
        lp_sensor_core_apply_config(&s_shadow);

    if (snap.ext_antenna_set) {
        enableExtAntenna(snap.ext_antenna_on);
        s_ext_antenna_on = snap.ext_antenna_on;
        ++applied;
    }

    // A fresh trial always resets the unconfirmed-cycle counter -- see
    // runtime_config_tx_power_note_cycle_result() for how it's driven from here on.
    if (snap.tx_power_dbm_set) {
        const int8_t dbm = static_cast<int8_t>(snap.tx_power_dbm);
        s_tx_power_pending = dbm;
        s_tx_power_has_pending = true;
        s_tx_power_unconfirmed_cycles = 0;
        if (s_link && s_link->setTxPowerDbm && s_link->setTxPowerDbm(dbm) == ESP_OK)
            s_tx_power_active_dbm = dbm;
        ++applied;
    }

    if (applied == 0)
        return 0;

    // Persist whichever fields changed. Floats have no native NVS type -- raw 4-byte blob,
    // simpler than a string round-trip. Heater fields and max_publish_gap_sec are stored in
    // their HA-facing units (minutes/seconds), not pre-converted LP cycles, so a later
    // lp_poll_interval_sec change (a reflash, since it's boot-only) re-derives the right cycle
    // count instead of replaying a now-stale one.
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
        if (snap.temp_offset_set)
            nvs_set_blob(nvs, "temp_offset", &snap.temp_offset_c, sizeof(float));
        if (snap.temp_min_change_set)
            nvs_set_blob(nvs, "temp_min_change", &snap.temp_min_change_c, sizeof(float));
        if (snap.rh_offset_set)
            nvs_set_blob(nvs, "rh_offset", &snap.rh_offset_pct, sizeof(float));
        if (snap.rh_min_change_set)
            nvs_set_blob(nvs, "rh_min_change", &snap.rh_min_change_pct, sizeof(float));
        if (snap.max_publish_gap_sec_set)
            nvs_set_u32(nvs, "max_pub_gap_s", snap.max_publish_gap_sec);
        if (snap.heater_period_set)
            nvs_set_u32(nvs, "htr_period_min", snap.heater_period_minutes);
        if (snap.heater_high_rh_set)
            nvs_set_u32(nvs, "htr_hi_rh_trig", snap.heater_high_rh_trigger_minutes);
        if (snap.ext_antenna_set)
            nvs_set_u8(nvs, "ext_antenna", snap.ext_antenna_on ? 1 : 0);
        if (snap.tx_power_dbm_set) {
            nvs_set_i8(nvs, "txp_pending", static_cast<int8_t>(snap.tx_power_dbm));
            nvs_set_u8(nvs, "txp_pend_flag", 1);
        }
        if (snap.sensor_samples_set)
            nvs_set_u32(nvs, "sensor_samples", snap.sensor_samples);
        if (nvs_commit(nvs) != ESP_OK)
            ESP_LOGE(TAG, "nvs_commit failed — change applied live but may not survive a reboot");
        nvs_close(nvs);
    } else {
        ESP_LOGE(TAG, "nvs_open (%s, RW) failed — change applied live but not persisted", NVS_NAMESPACE);
    }

    // Echo each changed value back on its own retained topic -- fire-and-forget, no ACK wait:
    // the value is already durably applied above, so delivery only affects how promptly HA's
    // display catches up, not correctness.
    if (client) {
        std::string val;
        const auto echo = [&](const char *topic) {
            esp_mqtt_client_publish(client, topic, val.c_str(), static_cast<int>(val.size()), 1, 1);
            val.clear();
        };
        if (snap.temp_offset_set)     { appendNum(val, snap.temp_offset_c); echo(s_topic_temp_offset.c_str()); }
        if (snap.temp_min_change_set) { appendNum(val, snap.temp_min_change_c); echo(s_topic_temp_min_change.c_str()); }
        if (snap.rh_offset_set)       { appendNum(val, snap.rh_offset_pct); echo(s_topic_rh_offset.c_str()); }
        if (snap.rh_min_change_set)   { appendNum(val, snap.rh_min_change_pct); echo(s_topic_rh_min_change.c_str()); }
        if (snap.max_publish_gap_sec_set) { appendNum(val, snap.max_publish_gap_sec); echo(s_topic_max_publish_gap_sec.c_str()); }
        if (snap.heater_period_set)   { appendNum(val, snap.heater_period_minutes); echo(s_topic_heater_period_minutes.c_str()); }
        if (snap.heater_high_rh_set)  { appendNum(val, snap.heater_high_rh_trigger_minutes); echo(s_topic_heater_high_rh_trigger_minutes.c_str()); }
        if (snap.ext_antenna_set)     { val = snap.ext_antenna_on ? "ON" : "OFF"; echo(s_topic_ext_antenna.c_str()); }
        if (snap.tx_power_dbm_set)    { appendNum(val, snap.tx_power_dbm); echo(s_topic_tx_power_dbm.c_str()); }
        if (snap.sensor_samples_set)  { appendNum(val, snap.sensor_samples); echo(s_topic_sensor_samples.c_str()); }
    }

    ESP_LOGI(TAG, "applied %d runtime config change(s)", applied);
    return applied;
}

float runtime_config_nvs_override(float yaml_or_default, const char *nvs_key)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK)
        return yaml_or_default;
    float value = 0.0f;
    size_t len = sizeof(value);
    const esp_err_t err = nvs_get_blob(nvs, nvs_key, &value, &len);
    nvs_close(nvs);
    if (err != ESP_OK || len != sizeof(value))
        return yaml_or_default;
    ESP_LOGI(TAG, "'%s': NVS override %.3f (yaml/default was %.3f)",
             nvs_key, static_cast<double>(value), static_cast<double>(yaml_or_default));
    return value;
}

uint32_t runtime_config_nvs_override(uint32_t yaml_or_default, const char *nvs_key)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK)
        return yaml_or_default;
    uint32_t value = 0;
    const esp_err_t err = nvs_get_u32(nvs, nvs_key, &value);
    nvs_close(nvs);
    if (err != ESP_OK)
        return yaml_or_default;
    ESP_LOGI(TAG, "'%s': NVS override %lu (yaml/default was %lu)",
             nvs_key, static_cast<unsigned long>(value), static_cast<unsigned long>(yaml_or_default));
    return value;
}

bool runtime_config_nvs_override(bool yaml_or_default, const char *nvs_key)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK)
        return yaml_or_default;
    uint8_t value = 0;
    const esp_err_t err = nvs_get_u8(nvs, nvs_key, &value);
    nvs_close(nvs);
    if (err != ESP_OK)
        return yaml_or_default;
    ESP_LOGI(TAG, "'%s': NVS override %d (yaml/default was %d)",
             nvs_key, static_cast<int>(value), static_cast<int>(yaml_or_default));
    return value != 0;
}

int32_t runtime_config_nvs_override(int32_t yaml_or_default, const char *nvs_key)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK)
        return yaml_or_default;
    int32_t value = 0;
    const esp_err_t err = nvs_get_i32(nvs, nvs_key, &value);
    nvs_close(nvs);
    if (err != ESP_OK)
        return yaml_or_default;
    ESP_LOGI(TAG, "'%s': NVS override %ld (yaml/default was %ld)",
             nvs_key, static_cast<long>(value), static_cast<long>(yaml_or_default));
    return value;
}

RuntimeConfigValues runtime_config_current_values()
{
    return RuntimeConfigValues{
        .temp_offset_c = s_shadow.temp_offset_c,
        .temp_min_change_c = s_shadow.temp_min_change_c,
        .rh_offset_pct = s_shadow.rh_offset_pct,
        .rh_min_change_pct = s_shadow.rh_min_change_pct,
        .max_publish_gap_sec = s_max_publish_gap_sec,
        .heater_period_minutes = s_heater_period_minutes,
        .heater_high_rh_trigger_minutes = s_heater_high_rh_trigger_minutes,
        .ext_antenna_on = s_ext_antenna_on,
        .tx_power_dbm = s_tx_power_has_pending ? int32_t(s_tx_power_pending) : int32_t(s_tx_power_known_good),
        .sensor_samples = s_shadow.sensor_samples,
    };
}

int8_t runtime_config_tx_power_active_dbm()
{
    return s_tx_power_active_dbm;
}

void runtime_config_tx_power_note_first_attach()
{
    if (s_tx_power_first_attach_done)
        return;
    s_tx_power_first_attach_done = true;

    if (s_tx_power_has_pending) {
        // A trial left outstanding by a PREVIOUS boot -- a boot happening at all isn't evidence
        // the value was safe (it could itself be a symptom of a bad TX power, or unrelated), so
        // it's discarded rather than resumed with a fresh grace window.
        ESP_LOGW(TAG, "TX power: discarding unconfirmed trial (%d dBm) left over from a previous "
                      "boot, falling back to known-good (%d dBm)",
                 s_tx_power_pending, s_tx_power_known_good);
        s_tx_power_has_pending = false;
        s_tx_power_unconfirmed_cycles = 0;
        nvs_handle_t nvs;
        if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
            nvs_set_u8(nvs, "txp_pend_flag", 0);
            nvs_commit(nvs);
            nvs_close(nvs);
        }
    }

    const int8_t dbm = s_tx_power_has_pending ? s_tx_power_pending : s_tx_power_known_good;
    if (s_link && s_link->setTxPowerDbm && s_link->setTxPowerDbm(dbm) == ESP_OK)
        s_tx_power_active_dbm = dbm;
}

void runtime_config_tx_power_note_cycle_result(bool ok)
{
    if (!s_tx_power_has_pending)
        return;

    if (ok) {
        ESP_LOGI(TAG, "TX power: trial (%d dBm) confirmed, promoted to known-good",
                 s_tx_power_pending);
        s_tx_power_known_good = s_tx_power_pending;
        s_tx_power_has_pending = false;
        s_tx_power_unconfirmed_cycles = 0;
        nvs_handle_t nvs;
        if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
            nvs_set_i8(nvs, "txp_known_good", s_tx_power_known_good);
            nvs_set_u8(nvs, "txp_pend_flag", 0);
            nvs_commit(nvs);
            nvs_close(nvs);
        }
        return;
    }

    if (++s_tx_power_unconfirmed_cycles < TX_POWER_REVERT_AFTER_CYCLES)
        return;

    ESP_LOGE(TAG, "TX power: trial (%d dBm) unconfirmed for %lu cycles — reverting to known-good "
                  "(%d dBm)",
             s_tx_power_pending, static_cast<unsigned long>(s_tx_power_unconfirmed_cycles),
             s_tx_power_known_good);
    s_tx_power_has_pending = false;
    s_tx_power_unconfirmed_cycles = 0;
    if (s_link && s_link->setTxPowerDbm && s_link->setTxPowerDbm(s_tx_power_known_good) == ESP_OK)
        s_tx_power_active_dbm = s_tx_power_known_good;
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_set_u8(nvs, "txp_pend_flag", 0);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
}

void runtime_config_tx_power_pin_max_for_ota()
{
    if (s_link && s_link->setTxPowerDbm)
        s_link->setTxPowerDbm(TX_POWER_TABLE_MAX_DBM);
}

void runtime_config_tx_power_unpin_after_ota()
{
    if (s_link && s_link->setTxPowerDbm)
        s_link->setTxPowerDbm(s_tx_power_active_dbm);
}
