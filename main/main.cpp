#include <memory>
#include <string>
#include "esp_err.h"
#include "esp_log.h"

#include "main.h"
#include "common_utils.h"
#include "secrets.h"
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

void startErrorTask(ErrorTask::ErrorCode code)
{
    errorTask = std::make_shared<ErrorTask>(code);
    xTaskCreate([](void *) static
    {
        errorTask->execute();
        vTaskDelete(nullptr);
    }, "error_task", DEFAULT_TASK_STACK_SIZE, nullptr, 6, nullptr);
}

static uint8_t hex_nibble(char c)
{
    if (c >= '0' && c <= '9') return static_cast<uint8_t>(c - '0');
    if (c >= 'a' && c <= 'f') return static_cast<uint8_t>(c - 'a' + 10);
    if (c >= 'A' && c <= 'F') return static_cast<uint8_t>(c - 'A' + 10);
    return 0;
}

static bool parse_dataset_tlvs(const std::string &hex, otOperationalDatasetTlvs &out)
{
    size_t hex_len = hex.size();

    if (hex_len == 0 || hex_len % 2 != 0 || hex_len / 2 > OT_OPERATIONAL_DATASET_MAX_LENGTH) {
        ESP_LOGE(TAG, "Invalid OT TLV hex string (len=%d)", static_cast<int>(hex_len));
        return false;
    }

    memset(&out, 0, sizeof(out));
    for (size_t i = 0; i < hex_len; i += 2)
        out.mTlvs[out.mLength++] = static_cast<uint8_t>(
            (hex_nibble(hex[i]) << 4) | hex_nibble(hex[i + 1]));

    ESP_LOGI(TAG, "Parsed %d bytes from OT TLV dataset", out.mLength);
    return true;
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

void process_state_change(otChangedFlags flags, void* context)
{
    otDeviceRole role = otThreadGetDeviceRole(esp_openthread_get_instance());
    switch (role) {
        case OT_DEVICE_ROLE_DISABLED:
            ESP_LOGI(TAG, "OT role: DISABLED");
            break;
        case OT_DEVICE_ROLE_DETACHED:
            ESP_LOGI(TAG, "OT role: DETACHED");
            break;
        case OT_DEVICE_ROLE_CHILD:
            ESP_LOGI(TAG, "OT role: CHILD — joined network as sleepy end device");
            log_thread_network_info();
            break;
        case OT_DEVICE_ROLE_ROUTER:
            ESP_LOGI(TAG, "OT role: ROUTER");
            break;
        case OT_DEVICE_ROLE_LEADER:
            ESP_LOGI(TAG, "OT role: LEADER");
            break;
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
    enableExtAntenna(false);
    if (const esp_err_t timerErr = registerWakeupTimer(static_cast<uint32_t>(100'000)); timerErr != ESP_OK)
        ESP_LOGE(TAG, "registerWakeupTimer failed: %d", timerErr);

    // ── parse secrets ─────────────────────────────────────────────────────────
    const std::string_view yaml = secrets_yaml();
    ESP_LOGI("main", "secrets.yaml embedded size: %d bytes", static_cast<int>(yaml.size()));
    ESP_LOGI("main", "secrets.yaml first 40 chars: %.40s", yaml.data());
    const std::string ot_tlv    = yaml_get_string(yaml, "ot_tlv");
    const std::string ha_ipv4   = yaml_get_string(yaml, "ha_ipv4");
    const std::string mqtt_port_str = yaml_get_string(yaml, "mqtt_port");
    const std::string mqtt_user = yaml_get_string(yaml, "mqtt_username");
    const std::string mqtt_pass = yaml_get_string(yaml, "mqtt_password");

    if (ot_tlv.empty()) {
        ESP_LOGE("main", "secrets.yaml missing 'ot_tlv'");
        return;
    }

    const uint16_t mqtt_port = mqtt_port_str.empty()
        ? 1883
        : static_cast<uint16_t>(std::stoul(mqtt_port_str));

    // ── initialise MQTT sender ─────────────────────────────────────────────────
    static constexpr std::string_view DEVICE_ID = "esp32c6_sensor_1";
    mqtt_sender_init(MqttConfig{
        .ha_ipv4   = ha_ipv4,
        .port      = mqtt_port,
        .username  = mqtt_user,
        .password  = mqtt_pass,
        .device_id = DEVICE_ID,
    });

    // ── sensors ───────────────────────────────────────────────────────────────
    ESP_LOGI(TAG, "create sensors task");
    sensorTask = std::make_shared<SensorsTask>();

    ret = sensorTask->init();
    if (ret != ESP_OK) {
        startErrorTask(ErrorTask::ErrorCode::ecSensorsFail);
        return;
    }

    sensorTask->configureReadyEvent([](const SensorsValues &values) static
    {
        if (!values.envTemperature || !values.envHumidity) return;

        esp_openthread_lock_acquire(portMAX_DELAY);
        const otDeviceRole role = otThreadGetDeviceRole(esp_openthread_get_instance());
        esp_openthread_lock_release();

        if (role != OT_DEVICE_ROLE_CHILD) return;

        mqtt_send_sensor_data(*values.envTemperature, *values.envHumidity);
    });

    xTaskCreate([](void *) static
    {
        sensorTask->executeTask();
        vTaskDelete(nullptr);
    }, "sensors_task", DEFAULT_TASK_STACK_SIZE, nullptr, 6, nullptr);

    // ── OpenThread ────────────────────────────────────────────────────────────
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

    startErrorTask(ErrorTask::ErrorCode::ecOK);
}
