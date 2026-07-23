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
static std::string s_topic_max_skip_cycles;
static std::string s_topic_heater_period_minutes;
static std::string s_topic_heater_high_rh_trigger_minutes;
static std::string s_topic_ext_antenna;

static uint32_t s_poll_interval_sec = 20;
// Always-current shadow of the 7 calibration/threshold/heater-schedule fields, seeded from
// the boot config and merged in-place as changes are accepted -- lp_sensor_core_apply_config()
// always writes the full struct, so a partial change (e.g. only temp_offset) still needs the
// other fields' current values, not stale defaults.
static lp_sensor_core_config_t s_shadow{};

// Every value HA has sent since the last runtime_config_apply_pending() call, guarded by
// s_mutex. Mirrors ota_updater.cpp's Manifest/s_mutex pattern: the event-handler-context
// writer takes the mutex only for a fast, non-blocking struct copy.
struct PendingCfg
{
    bool temp_offset_set = false;
    bool temp_min_change_set = false;
    bool rh_offset_set = false;
    bool rh_min_change_set = false;
    bool max_skip_cycles_set = false;
    bool heater_period_set = false;
    bool heater_high_rh_set = false;
    bool ext_antenna_set = false;

    float temp_offset_c = 0.0f;
    float temp_min_change_c = 0.0f;
    float rh_offset_pct = 0.0f;
    float rh_min_change_pct = 0.0f;
    uint32_t max_skip_cycles = 0;
    uint32_t heater_period_minutes = 0;
    uint32_t heater_high_rh_trigger_minutes = 0;
    bool ext_antenna_on = false;
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

// ── public API ────────────────────────────────────────────────────────────────

void runtime_config_init(std::string_view device_id, uint32_t poll_interval_sec,
                          const lp_sensor_core_config_t &boot_config)
{
    s_poll_interval_sec = poll_interval_sec;
    s_shadow = boot_config;

    const auto full = [&](std::string_view suffix) {
        return std::format("{}/{}", device_id, suffix);
    };
    s_topic_wildcard                       = full(CFG_SUFFIX_WILDCARD);
    s_topic_temp_offset                    = full(CFG_SUFFIX_TEMP_OFFSET);
    s_topic_temp_min_change                = full(CFG_SUFFIX_TEMP_MIN_CHANGE);
    s_topic_rh_offset                      = full(CFG_SUFFIX_RH_OFFSET);
    s_topic_rh_min_change                  = full(CFG_SUFFIX_RH_MIN_CHANGE);
    s_topic_max_skip_cycles                = full(CFG_SUFFIX_MAX_SKIP_CYCLES);
    s_topic_heater_period_minutes          = full(CFG_SUFFIX_HEATER_PERIOD_MIN);
    s_topic_heater_high_rh_trigger_minutes = full(CFG_SUFFIX_HEATER_HIGH_RH_MIN);
    s_topic_ext_antenna                    = full(CFG_SUFFIX_EXT_ANTENNA);

    if (!s_mutex)
        s_mutex = xSemaphoreCreateMutex();
}

const char *runtime_config_topic_wildcard()                       { return s_topic_wildcard.c_str(); }
const char *runtime_config_topic_temp_offset()                    { return s_topic_temp_offset.c_str(); }
const char *runtime_config_topic_temp_min_change()                { return s_topic_temp_min_change.c_str(); }
const char *runtime_config_topic_rh_offset()                      { return s_topic_rh_offset.c_str(); }
const char *runtime_config_topic_rh_min_change()                  { return s_topic_rh_min_change.c_str(); }
const char *runtime_config_topic_max_skip_cycles()                { return s_topic_max_skip_cycles.c_str(); }
const char *runtime_config_topic_heater_period_minutes()          { return s_topic_heater_period_minutes.c_str(); }
const char *runtime_config_topic_heater_high_rh_trigger_minutes() { return s_topic_heater_high_rh_trigger_minutes.c_str(); }
const char *runtime_config_topic_ext_antenna()                    { return s_topic_ext_antenna.c_str(); }

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
    } else if (topic_is(topic, topic_len, s_topic_max_skip_cycles)) {
        uint32_t v;
        if (!parse_uint32(data, data_len, v))
            return;
        v = std::clamp(v, MAX_SKIP_CYCLES_MIN, MAX_SKIP_CYCLES_MAX);
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        s_pending.max_skip_cycles = v;
        s_pending.max_skip_cycles_set = true;
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
    if (snap.max_skip_cycles_set)  { s_shadow.max_skip_cycles = snap.max_skip_cycles; lpChanged = true; ++applied; }
    if (snap.heater_period_set) {
        s_shadow.heater_period_cycles = minutes_to_lp_cycles(snap.heater_period_minutes, s_poll_interval_sec);
        lpChanged = true;
        ++applied;
    }
    if (snap.heater_high_rh_set) {
        s_shadow.high_rh_trigger_cycles = minutes_to_lp_cycles(snap.heater_high_rh_trigger_minutes, s_poll_interval_sec);
        lpChanged = true;
        ++applied;
    }
    if (lpChanged)
        lp_sensor_core_apply_config(&s_shadow);

    if (snap.ext_antenna_set) {
        enableExtAntenna(snap.ext_antenna_on);
        ++applied;
    }

    if (applied == 0)
        return 0;

    // Persist whichever fields changed. Floats have no native NVS type -- raw 4-byte blob,
    // simpler than a string round-trip. Heater fields are stored in MINUTES (the HA-facing
    // unit), not pre-converted LP cycles, so they stay independent of lp_poll_interval_sec.
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
        if (snap.max_skip_cycles_set)
            nvs_set_u32(nvs, "max_skip_cycles", snap.max_skip_cycles);
        if (snap.heater_period_set)
            nvs_set_u32(nvs, "htr_period_min", snap.heater_period_minutes);
        if (snap.heater_high_rh_set)
            nvs_set_u32(nvs, "htr_hi_rh_trig", snap.heater_high_rh_trigger_minutes);
        if (snap.ext_antenna_set)
            nvs_set_u8(nvs, "ext_antenna", snap.ext_antenna_on ? 1 : 0);
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
        if (snap.max_skip_cycles_set) { appendNum(val, snap.max_skip_cycles); echo(s_topic_max_skip_cycles.c_str()); }
        if (snap.heater_period_set)   { appendNum(val, snap.heater_period_minutes); echo(s_topic_heater_period_minutes.c_str()); }
        if (snap.heater_high_rh_set)  { appendNum(val, snap.heater_high_rh_trigger_minutes); echo(s_topic_heater_high_rh_trigger_minutes.c_str()); }
        if (snap.ext_antenna_set)     { val = snap.ext_antenna_on ? "ON" : "OFF"; echo(s_topic_ext_antenna.c_str()); }
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
