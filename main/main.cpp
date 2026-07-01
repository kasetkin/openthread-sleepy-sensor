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
#include "esp_openthread.h"
#include "esp_openthread_lock.h"
#include "esp_openthread_netif_glue.h"
#include "esp_vfs_eventfd.h"
#include "esp_private/esp_clk.h"
#include "nvs_flash.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include "openthread/dataset.h"
#include "openthread/ip6.h"
#include "openthread/link.h"
#include "openthread/netdata.h"
#include "openthread/thread.h"

static constexpr uint32_t CONFIG_OPENTHREAD_NETWORK_POLLPERIOD_TIME_MS = 70000;

static const char *TAG = "main-body";
constexpr uint32_t DEFAULT_TASK_STACK_SIZE = 16384;

static std::shared_ptr<SensorsTask> sensorTask;
static std::shared_ptr<ErrorTask> errorTask;

// Set once the device attaches to the Thread mesh (role CHILD/ROUTER/LEADER), cleared on
// detach. The sensor task blocks on this before each read→publish→sleep cycle so it never
// light-sleeps — which would stall OpenThread's MLE attachment — before the OTBR has answered.
static constexpr EventBits_t BIT_ATTACHED = BIT0;
static EventGroupHandle_t s_ot_attached_eg = nullptr;

// Blocks up to timeout_ms for the device to be attached; returns true if attached.
// Returns immediately when already attached.
static bool wait_for_ot_attached(uint32_t timeout_ms)
{
    if (!s_ot_attached_eg)
    	return false;
    	
    const EventBits_t bits = xEventGroupWaitBits(
        s_ot_attached_eg, BIT_ATTACHED, pdFALSE, pdTRUE, pdMS_TO_TICKS(timeout_ms));
        
    return (bits & BIT_ATTACHED) != 0;
}

void startErrorTask(ErrorTask::ErrorCode code)
{
    errorTask = std::make_shared<ErrorTask>(code);
    xTaskCreate([](void *) static
    {
        errorTask->execute();
        vTaskDelete(nullptr);
    }, "error_task", DEFAULT_TASK_STACK_SIZE, nullptr, 6, nullptr);
}

static void configure_ot_network(const std::string &ot_tlv_hex)
{
    otOperationalDatasetTlvs dataset_tlvs;
    if (!parse_dataset_tlvs(ot_tlv_hex, dataset_tlvs)) {
        ESP_LOGE(TAG, "Failed to parse OT TLV, cannot join network");
        return;
    }
    ESP_LOGI("OT-config", "parsed TLV, size %u", static_cast<unsigned>(dataset_tlvs.mLength));

    esp_openthread_lock_acquire(portMAX_DELAY);

    // Force-set dataset from secrets.yaml every boot, overriding any stale NVS cache.
    // esp_openthread_auto_start() skips this if NVS already has a dataset, which can
    // leave the device on a stale mesh-local prefix from a previous network.
    if (otDatasetSetActiveTlvs(esp_openthread_get_instance(), &dataset_tlvs) != OT_ERROR_NONE)
        ESP_LOGE(TAG, "Failed to set OT dataset TLVs");

    otLinkModeConfig link_mode = {};
    link_mode.mRxOnWhenIdle = false;  // sleepy end device
    link_mode.mDeviceType   = false;  // MTD
    link_mode.mNetworkData  = false;  // minimal network data

    if (otLinkSetPollPeriod(esp_openthread_get_instance(),
                            CONFIG_OPENTHREAD_NETWORK_POLLPERIOD_TIME_MS) != OT_ERROR_NONE)
        ESP_LOGE(TAG, "Failed to set OT poll period");

    if (otThreadSetLinkMode(esp_openthread_get_instance(), link_mode) != OT_ERROR_NONE)
        ESP_LOGE(TAG, "Failed to set OT link mode");

    esp_openthread_lock_release();

    ESP_ERROR_CHECK(esp_openthread_auto_start(&dataset_tlvs));
    ESP_LOGI("OT-config", "END");
}

static void log_thread_network_info()
{
    otInstance *ot = esp_openthread_get_instance();
    char buf[OT_IP6_ADDRESS_STRING_SIZE];

    ESP_LOGI(TAG, "=== Thread IPv6 addresses ===");
    for (const otNetifAddress *a = otIp6GetUnicastAddresses(ot); a; a = a->mNext) {
        otIp6AddressToString(&a->mAddress, buf, sizeof(buf));
        ESP_LOGI(TAG, "  %s/%u (preferred=%d)", buf, a->mPrefixLength, a->mPreferred);
    }

    ESP_LOGI(TAG, "=== Thread external routes ===");
    otNetworkDataIterator it = OT_NETWORK_DATA_ITERATOR_INIT;
    otExternalRouteConfig route;
    bool any = false;
    while (otNetDataGetNextRoute(ot, &it, &route) == OT_ERROR_NONE) {
        any = true;
        otIp6AddressToString(&route.mPrefix.mPrefix, buf, sizeof(buf));
        ESP_LOGI(TAG, "  %s/%u  nat64=%d stable=%d",
                 buf, route.mPrefix.mLength, route.mNat64, route.mStable);
        if (route.mNat64 && route.mPrefix.mLength == 96)
            mqtt_sender_set_nat64_prefix(route.mPrefix.mPrefix.mFields.m8);
    }
    if (!any) ESP_LOGW(TAG, "  (none — NAT64 route not yet in network data)");
}

// Re-scan current Thread network data for a NAT64 /96 route and update the sender's prefix.
// Called from the sensor task (which holds no OpenThread lock) after a failed publish, so it
// acquires the lock itself — unlike log_thread_network_info(), which runs inside the already-locked
// state-changed callback. Lets a merely-stale NAT64 prefix recover without waiting for a reboot.
static void refresh_nat64_prefix()
{
    otInstance *ot = esp_openthread_get_instance();
    esp_openthread_lock_acquire(portMAX_DELAY);
    otNetworkDataIterator it = OT_NETWORK_DATA_ITERATOR_INIT;
    otExternalRouteConfig route;
    bool found = false;
    while (otNetDataGetNextRoute(ot, &it, &route) == OT_ERROR_NONE) {
        if (route.mNat64 && route.mPrefix.mLength == 96) {
            mqtt_sender_set_nat64_prefix(route.mPrefix.mPrefix.mFields.m8);
            found = true;
        }
    }
    esp_openthread_lock_release();
    if (!found)
        ESP_LOGW(TAG, "refresh_nat64_prefix: no NAT64 route in current network data");
}

void process_state_change(otChangedFlags flags, void* context)
{
    otDeviceRole role = otThreadGetDeviceRole(esp_openthread_get_instance());
    switch (role) {
        case OT_DEVICE_ROLE_DISABLED:
            ESP_LOGI(TAG, "OT role: DISABLED");
            if (s_ot_attached_eg)
            	xEventGroupClearBits(s_ot_attached_eg, BIT_ATTACHED);
            break;
        case OT_DEVICE_ROLE_DETACHED:
            ESP_LOGI(TAG, "OT role: DETACHED");
            if (s_ot_attached_eg)
            	xEventGroupClearBits(s_ot_attached_eg, BIT_ATTACHED);
            break;
        case OT_DEVICE_ROLE_CHILD:
            ESP_LOGI(TAG, "OT role: CHILD — joined network as sleepy end device");
            log_thread_network_info();
            if (s_ot_attached_eg)
            	xEventGroupSetBits(s_ot_attached_eg, BIT_ATTACHED);
            break;
        case OT_DEVICE_ROLE_ROUTER:
            ESP_LOGI(TAG, "OT role: ROUTER");
            if (s_ot_attached_eg)
            	xEventGroupSetBits(s_ot_attached_eg, BIT_ATTACHED);
            break;
        case OT_DEVICE_ROLE_LEADER:
            ESP_LOGI(TAG, "OT role: LEADER");
            if (s_ot_attached_eg)
            	xEventGroupSetBits(s_ot_attached_eg, BIT_ATTACHED);
            break;
    }
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
    const std::string mqtt_broker_ipv4   = yaml_get_string(yaml, "mqtt_broker_ipv4");
    const std::string mqtt_port_str = yaml_get_string(yaml, "mqtt_port");
    const std::string mqtt_user = yaml_get_string(yaml, "mqtt_username");
    const std::string mqtt_pass = yaml_get_string(yaml, "mqtt_password");

    if (ot_tlv.empty()) {
        ESP_LOGE("main", "secrets.yaml missing 'ot_tlv'");
        return;
    }

    uint16_t mqtt_port = 1883;
    if (!mqtt_port_str.empty())
        std::from_chars(mqtt_port_str.data(),
                        mqtt_port_str.data() + mqtt_port_str.size(), mqtt_port);

    // ── initialise MQTT sender ─────────────────────────────────────────────────
    ESP_LOGI("main", "MQTT device_id: %s", mqtt_name_and_id.c_str());
    mqtt_sender_init(MqttConfig{
        .ha_ipv4   = mqtt_broker_ipv4,
        .port      = mqtt_port,
        .username  = mqtt_user,
        .password  = mqtt_pass,
        .device_id = mqtt_name_and_id,
        .device_name = mqtt_name_and_id,
    });

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
        esp_openthread_lock_acquire(portMAX_DELAY);
        const otDeviceRole role = otThreadGetDeviceRole(esp_openthread_get_instance());
        esp_openthread_lock_release();

        if (role != OT_DEVICE_ROLE_CHILD) return;

        mqtt_send_sensor_data(values.envTemperature, values.envHumidity);
    });

    // Gate each sensor cycle on Thread attachment so the task never light-sleeps before the
    // device has reached CHILD (which would stall OpenThread's MLE attachment / re-attach).
    sensorTask->configureAttachGate([](uint32_t ms) static
    {
    	return wait_for_ot_attached(ms);
    });

    // On a failed publish the sensor task asks us to re-read network data, so a changed NAT64
    // prefix is picked up without needing the reboot supervisor to kick in.
    sensorTask->configureRefreshNat64([]() static
    {
        refresh_nat64_prefix();
    });

    // ── OpenThread ────────────────────────────────────────────────────────────
    // Created before the state-changed callback is registered so no early CHILD transition is missed.
    s_ot_attached_eg = xEventGroupCreate();
    esp_openthread_radio_config_t radio_config {};
    radio_config.radio_mode = RADIO_MODE_NATIVE;
    esp_openthread_host_connection_config_t host_config {};
    host_config.host_connection_mode = HOST_CONNECTION_MODE_NONE;

    static esp_openthread_config_t config = {
        .netif_config = ESP_NETIF_DEFAULT_OPENTHREAD(),
        .platform_config = {
            .radio_config = radio_config,
            .host_config = host_config,
            .port_config = {
                .storage_partition_name = "nvs",
                .netif_queue_size = 10,
                .task_queue_size = 10,
            }
        }
    };
    ESP_ERROR_CHECK(esp_openthread_start(&config));

    esp_netif_set_default_netif(esp_openthread_get_netif());

    otSetStateChangedCallback(esp_openthread_get_instance(),
                              process_state_change,
                              esp_openthread_get_instance());

    configure_ot_network(ot_tlv);

    // Attachment is awaited per-cycle inside the sensor task via the gate configured above
    // (process_state_change() sets BIT_ATTACHED on CHILD), so it also covers later re-attachment.

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
