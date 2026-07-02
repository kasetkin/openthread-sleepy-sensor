#include <memory>
#include <string>
#include <string_view>
#include <charconv>
#include <format>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_mac.h"

#include "main.h"
#include "common_utils.h"
#include "secrets.h"
#include "calibration.h"
#include "sensorstask.h"
#include "errortask.h"
#include "mqtt_sender.h"

#include "esp_event.h"
#include "esp_netif.h"
#include "esp_vfs_eventfd.h"
#include "esp_private/esp_clk.h"
#include "nvs_flash.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "network_link.h"

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
    id.reserve(usernamePrefix.size() + 7);
    for (char c : usernamePrefix) {
        const bool ok = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                        (c >= '0' && c <= '9') || c == '-' || c == '_';
        id.push_back(ok ? c : '_');
    }

    uint8_t mac[8] = {};
    esp_read_mac(mac, ESP_MAC_IEEE802154);
    id += std::format("-{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}",
                      mac[0], mac[1], mac[2], mac[3],
                      mac[4], mac[5], mac[6], mac[7]);
    return id;
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
    enableExtAntenna(false);

    // ── parse secrets ─────────────────────────────────────────────────────────
    const std::string_view yaml = secrets_yaml();
    ESP_LOGI("main", "secrets.yaml embedded size: %d bytes", static_cast<int>(yaml.size()));
    ESP_LOGI("main", "secrets.yaml first 40 chars: %.40s", yaml.data());
    const std::string ot_tlv    = yaml_get_string(yaml, "ot_tlv");
    std::string deviceNamePrefix = yaml_get_string(yaml, "device_name");
    if (deviceNamePrefix.empty()) {
        deviceNamePrefix = "esp32-OT-MQTT-sensor";
        ESP_LOGW("main", "secrets.yaml has no 'device_name', falling back to '%s'", deviceNamePrefix.c_str());
    }

    const std::string mqtt_name_and_id = addOTMacSuffix(deviceNamePrefix);
    const std::string mqtt_broker_address = yaml_get_string(yaml, "mqtt_broker_address");
    const std::string mqtt_port_str = yaml_get_string(yaml, "mqtt_port");
    const std::string mqtt_user = yaml_get_string(yaml, "mqtt_username");
    const std::string mqtt_pass = yaml_get_string(yaml, "mqtt_password");

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
    ESP_LOGI("main", "MQTT device_id: %s", mqtt_name_and_id.c_str());
    mqtt_sender_init(MqttConfig{
        .broker_address = mqtt_broker_address,
        .port      = mqtt_port,
        .username  = mqtt_user,
        .password  = mqtt_pass,
        .device_id = mqtt_name_and_id,
        .device_name = mqtt_name_and_id,
    }, &s_link);

    // ── sensors ───────────────────────────────────────────────────────────────
    const SensorsTaskSettings sSettings {
        .rh_offset = parse_as_float(calibration_txt(), "rh_offset"),
        .rh_min_change = parse_as_float(calibration_txt(), "rh_min_change"),
        .temp_offset = parse_as_float(calibration_txt(), "temp_offset"),
        .temp_min_change = parse_as_float(calibration_txt(), "temp_min_change"),
        .max_skip_cycles = parse_as_uint32(calibration_txt(), "max_skip_cycles"),
        .cycle_duration_sec = parse_as_uint32(calibration_txt(), "cycle_duration_sec")
    };

    ESP_LOGI("main", "sensor settings: TODO");

    ESP_LOGI(TAG, "create sensors task");
    sensorTask = std::make_shared<SensorsTask>(sSettings);

    ret = sensorTask->init();
    if (ret != ESP_OK) {
        startErrorTask(ErrorTask::ErrorCode::ecSensorsFail);
        return;
    }

    sensorTask->configureReadyEvent([](const SensorsValues &values) static
    {
        mqtt_send_sensor_data(values.envTemperature, values.envHumidity);
    });

    // Gate each sensor cycle on the network link being ready (OT: Thread CHILD role) so the
    // task never light-sleeps before attachment (which would stall OpenThread's MLE attach).
    sensorTask->configureAttachGate(s_link.waitForReady);

    // On a failed publish the sensor task asks the link to refresh (OT: re-scan Thread
    // network data for a changed NAT64 prefix; Wi-Fi: kick a reconnect), so it recovers
    // without waiting for a reboot.
    sensorTask->configureRefreshNat64(s_link.refresh);

    // ── bring up the network link ──────────────────────────────────────────────
    ESP_ERROR_CHECK(s_link.start());

    // Attachment is awaited per-cycle inside the sensor task via the gate configured above,
    // so it also covers later re-attachment.

    xTaskCreate([](void *) static
    {
        sensorTask->executeTask();
        vTaskDelete(nullptr);
    }, "sensors_task", DEFAULT_TASK_STACK_SIZE, nullptr, 6, nullptr);

    startErrorTask(ErrorTask::ErrorCode::ecOK);

    // sensors_task is now the sole driver of the read→publish→wait→light-sleep cadence
    // (it arms the wakeup timer and calls correctLightSleep itself). app_main has nothing
    // left to do; returning is fine — the FreeRTOS scheduler keeps the other tasks running.
}
