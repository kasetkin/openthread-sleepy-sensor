#include <memory>
#include <string>
#include <string_view>
#include <charconv>
#include <format>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_system.h"

#include "main.h"
#include "common_utils.h"
#include "secrets.h"
#include "device_config.h"
#include "sensorstask.h"
#include "errortask.h"
#include "mqtt_sender.h"
#include "runtime_config.h"
#include "hp_awake_stats.h"

#include "esp_event.h"
#include "esp_netif.h"
#include "esp_vfs_eventfd.h"
#include "esp_private/esp_clk.h"
#include "nvs_flash.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "network_link.h"

#include "lp_sensor_core.h"

static const char *TAG = "main-body";
constexpr uint32_t DEFAULT_TASK_STACK_SIZE = 16384;

static std::shared_ptr<SensorsTask> sensorTask;
static std::shared_ptr<ErrorTask> errorTask;
static NetworkLink s_link;

void startErrorTask(ErrorTask::ErrorCode code)
{
    errorTask = std::make_shared<ErrorTask>(code);
    xTaskCreate([](void *) static
    {
        errorTask->execute();
        vTaskDelete(nullptr);
    }, "error_task", DEFAULT_TASK_STACK_SIZE, nullptr, 6, nullptr);
}

// Build a stable, unique MQTT device id from the human-readable name plus the
// chip's factory MAC suffix, so two boards sharing a secrets.yaml still differ.
// The name is sanitised to MQTT/HA-safe characters ([A-Za-z0-9_-]); everything
// else becomes '_'.
static std::string addOTMacSuffix(std::string_view usernamePrefix)
{
    std::string id;
    id.reserve(usernamePrefix.size() + 1 + 2 * MQTT_MAC_ADDRESS_BYTES);
    for (char c : usernamePrefix) {
        const bool ok = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                        (c >= '0' && c <= '9') || c == '-' || c == '_';
        id.push_back(ok ? c : '_');
    }

    uint8_t mac[8] = {};
    // Ties this array's size (and the format string's 8 placeholders below) to
    // MQTT_MAX_DEVICE_ID_LEN's derivation in mqtt_sender.h -- if one changes without the other,
    // this fails to compile instead of silently mismatching every buffer sized from that bound.
    static_assert(sizeof(mac) == MQTT_MAC_ADDRESS_BYTES);
    esp_read_mac(mac, ESP_MAC_IEEE802154);
    id += std::format("-{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}",
                      mac[0], mac[1], mac[2], mac[3],
                      mac[4], mac[5], mac[6], mac[7]);
    return id;
}

// parse_as_float()/parse_as_uint32() (secrets.h) return nullopt for a missing or malformed
// device_config.yaml key rather than silently defaulting to 0 -- these two wrappers apply an
// explicit, loud fallback at the one place (main.cpp) that owns device_config.yaml policy.
// Defaults are chosen to fail *safe*, not fail *silent-and-low-power*: e.g. lp_poll_interval_sec
// defaulting to 0 would turn the LP timer into a busy-loop, and max_publish_gap_sec defaulting
// to 0 would (correctly, if noisily) publish every cycle rather than silently drop changed
// readings.
static float parse_as_float_or(std::string_view content, std::string_view key, float def)
{
    if (const auto v = parse_as_float(content, key))
        return *v;
    ESP_LOGW("main", "device_config.yaml missing/invalid '%.*s', falling back to %.3f",
             static_cast<int>(key.size()), key.data(), static_cast<double>(def));
    return def;
}

static uint32_t parse_as_uint32_or(std::string_view content, std::string_view key, uint32_t def)
{
    if (const auto v = parse_as_uint32(content, key))
        return *v;
    ESP_LOGW("main", "device_config.yaml missing/invalid '%.*s', falling back to %lu",
             static_cast<int>(key.size()), key.data(), static_cast<unsigned long>(def));
    return def;
}

static int32_t parse_as_int32_or(std::string_view content, std::string_view key, int32_t def)
{
    if (const auto v = parse_as_int32(content, key))
        return *v;
    ESP_LOGW("main", "device_config.yaml missing/invalid '%.*s', falling back to %ld",
             static_cast<int>(key.size()), key.data(), static_cast<long>(def));
    return def;
}

// device_config.yaml expresses the heater schedule in wall-clock minutes; the LP program counts
// poll cycles (see lp_sensor_core_config_t). Ceiling division so any non-zero schedule is at
// least one cycle; 0 passes through as the "disabled" sentinel.
static uint32_t minutes_to_lp_cycles(uint32_t minutes, uint32_t poll_sec)
{
    if (minutes == 0)
        return 0;
    return (minutes * 60u + poll_sec - 1) / poll_sec;
}

// device_config.yaml expresses the guaranteed max publish gap in wall-clock seconds; the LP
// program counts skipped polls. Floors (unlike minutes_to_lp_cycles above) so the actual
// guarantee (result+1)*poll_sec never exceeds what was requested -- ceiling here would let the
// real gap overshoot the promise by almost a full poll period, the wrong direction for a
// staleness bound. A request at or below one poll period is the finest granularity achievable
// and maps to 0 (publish every cycle). Mirrors runtime_config.cpp's copy of this same helper --
// keep the two in sync if either changes.
static uint32_t publish_gap_sec_to_skip_cycles(uint32_t gap_sec, uint32_t poll_sec)
{
    if (gap_sec <= poll_sec)
        return 0;
    return gap_sec / poll_sec - 1;
}

static bool parse_as_bool_or(std::string_view content, std::string_view key, bool def)
{
    if (const auto v = parse_as_bool(content, key))
        return *v;
    ESP_LOGW("main", "device_config.yaml missing/invalid '%.*s', falling back to %s",
             static_cast<int>(key.size()), key.data(), def ? "true" : "false");
    return def;
}

// Reads, increments and persists the lifetime boot counter (published as HA's "Boot count"
// diagnostic). Returns the new value (1 on the first-ever boot), or 0 with a log on NVS
// failure — a storage problem then shows up in HA as a counter stuck at 0, not silently.
static uint32_t incrementBootCount()
{
    nvs_handle_t nvs;
    if (nvs_open("app", NVS_READWRITE, &nvs) != ESP_OK) {
        ESP_LOGE("main", "boot counter: nvs_open failed");
        return 0;
    }
    uint32_t count = 0;
    nvs_get_u32(nvs, "boot_count", &count);  // key missing on first boot: count stays 0
    ++count;
    if (nvs_set_u32(nvs, "boot_count", count) != ESP_OK || nvs_commit(nvs) != ESP_OK)
        ESP_LOGE("main", "boot counter: persist failed");
    nvs_close(nvs);
    return count;
}

// Maps esp_reset_reason() to the short string published as HA's "Reset reason" diagnostic.
// Every literal must fit MQTT_MAX_RESET_REASON_LEN (mqtt_sender.h) — the state-JSON buffer
// is sized against it. Both RTC markers are consumed unconditionally (they must be cleared
// even on a non-SW reset) and refine ESP_RST_SW: a successful OTA reboot, the LP-core-stall
// supervisor's reboot, and the bad-OTA safety net's reboot are all "software reset" to IDF,
// but only the latter two are failure signals worth telling apart.
static const char *resetReasonString()
{
    const bool lpStallReboot = consumeLpStallRebootMarker();
    const bool otaUnconfirmedReboot = consumeOtaUnconfirmedRebootMarker();
    switch (esp_reset_reason()) {
    case ESP_RST_POWERON:   return "power_on";
    case ESP_RST_SW:
        if (lpStallReboot)       return "lp_stall_reboot";
        if (otaUnconfirmedReboot) return "ota_unconfirmed";
        return "sw_reset";
    case ESP_RST_PANIC:     return "panic";
    case ESP_RST_BROWNOUT:  return "brownout";
    case ESP_RST_INT_WDT:   return "int_wdt";
    case ESP_RST_TASK_WDT:  return "task_wdt";
    case ESP_RST_WDT:       return "wdt";
    case ESP_RST_EXT:       return "external";
    case ESP_RST_DEEPSLEEP: return "deep_sleep";
    default:                return "unknown";
    }
}

extern "C" void app_main(void)
{
    esp_err_t ret;

    ret = initNvsFlash();
    if (ret != ESP_OK) {
        ESP_LOGE("main", "Cannot init NVS flash. Exit.");
        return;
    }

    const esp_vfs_eventfd_config_t eventfd_config = {
        .max_fds = 3,
    };

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_vfs_eventfd_register(&eventfd_config));

    enableRf(true);
    // No device_config.yaml tier for this one -- exclusively HA/NVS-driven (see
    // runtime_config.cpp); compiled default false (ceramic) matches the pre-feature behavior
    // when no override has ever been stored. Kept as a named local (not inlined into
    // enableExtAntenna()) so it can also seed runtime_config_init()'s current-value tracking
    // below, alongside enableExtAntenna()'s own boot-time GPIO write.
    const bool ext_antenna_on = runtime_config_nvs_override(false, "ext_antenna");
    enableExtAntenna(ext_antenna_on);

    // Must run before the network link starts (esp_openthread_start()/esp_wifi_start()):
    // OpenThread's own radio-state PM lock (esp_openthread_sleep_init(), see esp_openthread
    // component) only gates sleep through this automatic path.
    ESP_ERROR_CHECK(enableAutomaticLightSleep());
    // Sleep time before this point is untracked -- same ordering constraint as the call above.
    ESP_ERROR_CHECK(hp_awake_stats_init());

    // ── parse secrets ─────────────────────────────────────────────────────────
    const std::string_view yaml = secrets_yaml();
    ESP_LOGI("main", "secrets.yaml embedded size: %d bytes", static_cast<int>(yaml.size()));
    ESP_LOGI("main", "secrets.yaml first 40 chars: %.40s", yaml.data());
    const std::string ot_tlv    = yaml_get_string(yaml, "ot_tlv");
    std::string deviceNamePrefix = yaml_get_string(yaml, "device_name");
    if (deviceNamePrefix.empty()) {
        deviceNamePrefix = "esp32-OT-sensor";
        ESP_LOGW("main", "secrets.yaml has no 'device_name', falling back to '%s'", deviceNamePrefix.c_str());
    }
    // Bounds mqtt_name_and_id below to MQTT_MAX_DEVICE_ID_LEN chars -- mqtt_sender.cpp's
    // fixed-size topic/payload buffers are derived from that same bound (see
    // MQTT_MAX_DEVICE_NAME_LEN/MQTT_MAX_DEVICE_ID_LEN's doc comments in mqtt_sender.h).
    if (deviceNamePrefix.size() > MQTT_MAX_DEVICE_NAME_LEN) {
        ESP_LOGW("main", "secrets.yaml 'device_name' (%s) exceeds %u chars, truncating",
                 deviceNamePrefix.c_str(), static_cast<unsigned>(MQTT_MAX_DEVICE_NAME_LEN));
        deviceNamePrefix.resize(MQTT_MAX_DEVICE_NAME_LEN);
    }

    const std::string mqtt_name_and_id = addOTMacSuffix(deviceNamePrefix);
    const std::string mqtt_broker_address = yaml_get_string(yaml, "mqtt_broker_address");
    const std::string mqtt_port_str = yaml_get_string(yaml, "mqtt_port");
    const std::string mqtt_user = yaml_get_string(yaml, "mqtt_username");
    const std::string mqtt_pass = yaml_get_string(yaml, "mqtt_password");

    const std::string mqtt_tls_str = yaml_get_string(yaml, "mqtt_tls");
    bool mqtt_tls = false;
    if (!mqtt_tls_str.empty() && mqtt_tls_str != "true" && mqtt_tls_str != "false") {
        ESP_LOGW("main", "secrets.yaml has invalid 'mqtt_tls' (%s), falling back to 'false'",
                 mqtt_tls_str.c_str());
    } else {
        mqtt_tls = (mqtt_tls_str == "true");
    }
    const std::string mqtt_tls_ca_cert = yaml_get_string(yaml, "mqtt_tls_ca_cert");

    std::string transport_str = yaml_get_string(yaml, "transport");
    if (transport_str.empty())
        transport_str = "thread";
    if (transport_str != "thread" && transport_str != "wifi") {
        ESP_LOGW("main", "secrets.yaml has invalid 'transport' (%s), falling back to 'thread'",
                 transport_str.c_str());
        transport_str = "thread";
    }
    const TransportKind transport_kind = (transport_str == "wifi") ? TransportKind::Wifi : TransportKind::Thread;

    const std::string wifi_ssid = yaml_get_string(yaml, "wifi_ssid");
    const std::string wifi_password = yaml_get_string(yaml, "wifi_password");
    std::string wifi_address_family = yaml_get_string(yaml, "wifi_address_family");
    if (wifi_address_family.empty())
        wifi_address_family = "ipv4";
    if (wifi_address_family != "ipv4" && wifi_address_family != "ipv6") {
        ESP_LOGW("main", "secrets.yaml has invalid 'wifi_address_family' (%s), falling back to 'ipv4'",
                 wifi_address_family.c_str());
        wifi_address_family = "ipv4";
    }

    if (transport_kind == TransportKind::Thread && ot_tlv.empty()) {
        ESP_LOGE("main", "secrets.yaml missing 'ot_tlv' (required for transport \"thread\")");
        return;
    }
    if (transport_kind == TransportKind::Wifi && wifi_ssid.empty()) {
        ESP_LOGE("main", "secrets.yaml missing 'wifi_ssid' (required for transport \"wifi\")");
        return;
    }
    // wifi_address_family must agree with mqtt_broker_address's own family — Wi-Fi only brings
    // up the one family it's told to, so a mismatch would connect to nothing every cycle.
    // (Only checkable because mqtt_broker_address is always a literal today; revisit once
    // DNS-name broker support exists.)
    if (transport_kind == TransportKind::Wifi) {
        const bool broker_is_v6 = looksLikeIpv6(mqtt_broker_address);
        const bool family_is_v6 = (wifi_address_family == "ipv6");
        if (broker_is_v6 != family_is_v6) {
            ESP_LOGE("main", "wifi_address_family (%s) doesn't match mqtt_broker_address's family (%s) — fix secrets.yaml",
                     wifi_address_family.c_str(), broker_is_v6 ? "ipv6" : "ipv4");
            return;
        }
    }

    uint16_t mqtt_port = 1883;
    if (!mqtt_port_str.empty())
        std::from_chars(mqtt_port_str.data(),
                        mqtt_port_str.data() + mqtt_port_str.size(), mqtt_port);

    // ── network link ─────────────────────────────────────────────────────────
    s_link = makeNetworkLink(transport_kind, NetworkLinkConfig{
        .ot_tlv_hex = ot_tlv,
        .wifi_ssid = wifi_ssid,
        .wifi_password = wifi_password,
        .wifi_address_family = wifi_address_family,
    });

    // ── initialise MQTT sender ─────────────────────────────────────────────────
    // The LP cadence pair is parsed here, ahead of the sensor/LP-core settings below that
    // also use it, because MqttConfig's expire_after_sec is derived from the same two
    // values (see its doc comment). An lp_poll_interval_sec of 0 would arm the LP timer
    // with no delay (busy-loop) -- the non-zero fallback default guards that too.
    const uint32_t lp_poll_interval_sec = parse_as_uint32_or(device_config_yaml(), "lp_poll_interval_sec",
                                                             SensorsTaskSettings{}.lpPollIntervalSec);
    // 0 (not device_config.yaml's own "300" default) is the compiled fail-safe fallback here,
    // matching this file's fail-*safe*-not-fail-silent-and-low-power policy (see the comment
    // above parse_as_uint32_or()): a missing/malformed key publishes every cycle rather than
    // silently assuming some "normal" cadence.
    const uint32_t max_publish_gap_sec = runtime_config_nvs_override(
        parse_as_uint32_or(device_config_yaml(), "max_publish_gap_sec", 0u),
        "max_pub_gap_s");
    const uint32_t max_skip_cycles = publish_gap_sec_to_skip_cycles(max_publish_gap_sec, lp_poll_interval_sec);
    const uint32_t boot_count = incrementBootCount();
    std::string reset_reason = resetReasonString();
    if (reset_reason.size() > MQTT_MAX_RESET_REASON_LEN)
        reset_reason.resize(MQTT_MAX_RESET_REASON_LEN);
    ESP_LOGI("main", "boot #%lu, reset reason: %s",
             static_cast<unsigned long>(boot_count), reset_reason.c_str());

    ESP_LOGI("main", "MQTT device_id: %s", mqtt_name_and_id.c_str());
    mqtt_sender_init(MqttConfig{
        .broker_address = mqtt_broker_address,
        .port      = mqtt_port,
        .username  = mqtt_user,
        .password  = mqtt_pass,
        .device_id = mqtt_name_and_id,
        .device_name = mqtt_name_and_id,
        .use_tls = mqtt_tls,
        .tls_ca_cert_b64 = mqtt_tls_ca_cert,
        .expire_after_sec = 2 * SensorsTask::safeguardWakeSec(lp_poll_interval_sec, max_skip_cycles),
        .boot_count = boot_count,
        .reset_reason = reset_reason,
    }, &s_link);

    // ── sensors ───────────────────────────────────────────────────────────────
    // The LP core (components/lp_sensor_core) owns the sensor entirely: I2C, calibration,
    // the skip-threshold publish decision, and heater maintenance. HP never touches the
    // sensor bus -- see the migration plan. The sensor task sets no cadence of its own; it
    // derives its safeguard wake timeout from the same LP cadence pair
    // (SensorsTask::safeguardWakeSec).
    const SensorsTaskSettings sSettings {
        .lpPollIntervalSec = lp_poll_interval_sec,
        .maxSkipCycles = max_skip_cycles,
        .readVoltageViaAdc = parse_as_bool_or(device_config_yaml(), "read_battery_via_adc",
                                                  SensorsTaskSettings{}.readVoltageViaAdc),
        .batteryDividerRVbatOhm = parse_as_float_or(device_config_yaml(), "battery_divider_r_vbat_ohm",
                                                  static_cast<float>(SensorsTaskSettings{}.batteryDividerRVbatOhm)),
        .batteryDividerRGndOhm = parse_as_float_or(device_config_yaml(), "battery_divider_r_gnd_ohm",
                                                  static_cast<float>(SensorsTaskSettings{}.batteryDividerRGndOhm)),
    };

    ESP_LOGI("main", "sensor settings: lp_poll_interval_sec=%lu max_publish_gap_sec=%lu (%lu LP cycles) "
                     "safeguard_wake_sec=%lu read_battery_via_adc=%d battery_divider=%.0f/%.0f Ohm",
             static_cast<unsigned long>(sSettings.lpPollIntervalSec),
             static_cast<unsigned long>(max_publish_gap_sec),
             static_cast<unsigned long>(sSettings.maxSkipCycles),
             static_cast<unsigned long>(SensorsTask::safeguardWakeSec(sSettings.lpPollIntervalSec,
                                                                      sSettings.maxSkipCycles)),
             static_cast<int>(sSettings.readVoltageViaAdc),
             sSettings.batteryDividerRVbatOhm, sSettings.batteryDividerRGndOhm);

    sensorTask = std::make_shared<SensorsTask>(sSettings);

    sensorTask->configureReadyEvent([](const SensorsValues &values) static
    {
        mqtt_send_sensor_data(values.envTemperature, values.envHumidity,
                              values.batteryPercent, values.batteryVoltageMilliV,
                              values.heaterProblem, values.heaterRunCount,
                              values.adcTimeUs);
    });

    // Gate each sensor cycle on the network link being ready (OT: Thread CHILD role) so the
    // task never light-sleeps before attachment (which would stall OpenThread's MLE attach).
    // Wrapped (rather than passing s_link.waitForReady directly) so TX power's "always transmit
    // at table max until the very first successful attach" rule has exactly one place to live --
    // see runtime_config_tx_power_note_first_attach()'s doc comment.
    sensorTask->configureAttachGate([](uint32_t timeoutMs) {
        const bool ready = s_link.waitForReady(timeoutMs);
        if (ready)
            runtime_config_tx_power_note_first_attach();
        return ready;
    });

    // On a failed publish the sensor task asks the link to refresh (OT: re-scan Thread
    // network data for a changed NAT64 prefix; Wi-Fi: kick a reconnect), so it recovers
    // without waiting for a reboot.
    sensorTask->configureRefreshNat64(s_link.refresh);

    // ── bring up the network link ──────────────────────────────────────────────
    ESP_ERROR_CHECK(s_link.start());

    // Attachment is awaited per-cycle inside the sensor task via the gate configured above,
    // so it also covers later re-attachment.

    // ── LP core sensor ownership ─────────────────────────────────────────────
    // Heater fields resolved as named locals (not inlined into minutes_to_lp_cycles() below)
    // so their HA-facing minutes value -- not just the LP-cycle-converted form lpConfig carries
    // -- is also available to seed runtime_config_init()'s current-value tracking.
    const uint32_t heater_period_minutes = runtime_config_nvs_override(
        parse_as_uint32_or(device_config_yaml(), "heater_period_minutes", 1440), "htr_period_min");
    const uint32_t heater_high_rh_trigger_minutes = runtime_config_nvs_override(
        parse_as_uint32_or(device_config_yaml(), "heater_high_rh_trigger_minutes", 60), "htr_hi_rh_trig");
    const int32_t tx_power_known_good_dbm = runtime_config_nvs_override(
        parse_as_int32_or(device_config_yaml(), "tx_power_dbm",
                          static_cast<int32_t>(TX_POWER_TABLE_MAX_DBM)),
        "txp_known_good");

    const lp_sensor_core_config_t lpConfig {
        .temp_offset_c = runtime_config_nvs_override(
            parse_as_float_or(device_config_yaml(), "temp_offset", 0.0f), "temp_offset"),
        .temp_min_change_c = runtime_config_nvs_override(
            parse_as_float_or(device_config_yaml(), "temp_min_change", 0.2f), "temp_min_change"),
        .rh_offset_pct = runtime_config_nvs_override(
            parse_as_float_or(device_config_yaml(), "rh_offset", 0.0f), "rh_offset"),
        .rh_min_change_pct = runtime_config_nvs_override(
            parse_as_float_or(device_config_yaml(), "rh_min_change", 2.0f), "rh_min_change"),
        // parsed above, ahead of mqtt_sender_init() -- expire_after_sec derives from it
        .max_skip_cycles = max_skip_cycles,
        .heater_period_cycles = minutes_to_lp_cycles(heater_period_minutes, lp_poll_interval_sec),
        .high_rh_trigger_cycles = minutes_to_lp_cycles(heater_high_rh_trigger_minutes, lp_poll_interval_sec),
    };
    ESP_LOGI(TAG, "LP sensor core: poll interval %lu s, temp_offset=%.2f temp_min_change=%.2f "
                  "rh_offset=%.2f rh_min_change=%.2f max_publish_gap_sec=%lu (%lu cycles) "
                  "heater_period=%lu cycles high_rh_trigger=%lu cycles",
             static_cast<unsigned long>(lp_poll_interval_sec),
             static_cast<double>(lpConfig.temp_offset_c), static_cast<double>(lpConfig.temp_min_change_c),
             static_cast<double>(lpConfig.rh_offset_pct), static_cast<double>(lpConfig.rh_min_change_pct),
             static_cast<unsigned long>(max_publish_gap_sec),
             static_cast<unsigned long>(lpConfig.max_skip_cycles),
             static_cast<unsigned long>(lpConfig.heater_period_cycles),
             static_cast<unsigned long>(lpConfig.high_rh_trigger_cycles));

    if (const esp_err_t lpInitErr = lp_sensor_core_init(&lpConfig); lpInitErr != ESP_OK) {
        ESP_LOGE(TAG, "lp_sensor_core_init failed: %d", lpInitErr);
        startErrorTask(ErrorTask::ErrorCode::ecSensorsFail);
        return;
    }
    if (const esp_err_t lpStartErr = lp_sensor_core_start(lp_poll_interval_sec * 1'000'000u); lpStartErr != ESP_OK) {
        ESP_LOGE(TAG, "lp_sensor_core_start failed: %d", lpStartErr);
        startErrorTask(ErrorTask::ErrorCode::ecSensorsFail);
        return;
    }

    // Seeds runtime_config's shadow of the 8 live-tunable fields with the boot config just
    // applied above (already NVS-override-resolved), and builds the <device_id>/cfg/* topic
    // strings HA drives via MQTT number/switch entities.
    runtime_config_init(mqtt_name_and_id, lp_poll_interval_sec, lpConfig, max_publish_gap_sec,
                         heater_period_minutes, heater_high_rh_trigger_minutes, ext_antenna_on,
                         tx_power_known_good_dbm, &s_link);

    xTaskCreate([](void *) static
    {
        sensorTask->executeTask();
        vTaskDelete(nullptr);
    }, "sensors_task", DEFAULT_TASK_STACK_SIZE, nullptr, 6, nullptr);

    startErrorTask(ErrorTask::ErrorCode::ecOK);

    // sensors_task is now the sole driver of the read→publish→wait cadence (it blocks via
    // lp_sensor_core_wait_for_wake() between cycles). Light sleep itself is fully automatic
    // (see enableAutomaticLightSleep() above) — nothing has to call esp_light_sleep_start().
    // app_main has nothing left to do; returning is fine — the FreeRTOS scheduler keeps the
    // other tasks running.
}
