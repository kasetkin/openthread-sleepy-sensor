//
// Matter sensor endpoints — Temperature Measurement (0x0402) + Relative Humidity
// Measurement (0x0405) for a Thread sleepy device. Built at gnu++17 (esp-matter
// default) in its own component, isolated from the app's C++26 code.
//
// Reuse note: the SHT3x read + calibration + heater logic in sensorstask.cpp is
// unchanged; the only swap is the per-cycle sink — matter_sensor_update() replaces
// the old mqtt_send_sensor_data().

#include "matter_sensor.h"

#include <esp_log.h>
#include <esp_matter.h>
#include <esp_matter_endpoint.h>

#include <esp_openthread.h>

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#include <platform/ESP32/OpenthreadLauncher.h>
#endif

using namespace esp_matter;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters;
using namespace chip::DeviceLayer;

static const char *TAG = "matter-sensor";

static uint16_t s_temp_endpoint_id = 0;
static uint16_t s_humidity_endpoint_id = 0;

// Matter MeasuredValue encodings:
//   TemperatureMeasurement.MeasuredValue      : int16,  centi-°C (22.50 -> 2250)
//   RelativeHumidityMeasurement.MeasuredValue : uint16, centi-%  (45.20 -> 4520)
static int16_t  to_centi_i16(float v) { return static_cast<int16_t>(v * 100.0f + (v < 0 ? -0.5f : 0.5f)); }
static uint16_t to_centi_u16(float v) { return static_cast<uint16_t>(v * 100.0f + 0.5f); }

// Sensor endpoints are read-only: accept any write without action.
static esp_err_t on_attribute_update(attribute::callback_type_t type, uint16_t endpoint_id,
                                     uint32_t cluster_id, uint32_t attribute_id,
                                     esp_matter_attr_val_t *val, void *priv_data)
{
    return ESP_OK;
}

static esp_err_t on_identification(identification::callback_type_t type, uint16_t endpoint_id,
                                   uint8_t effect_id, uint8_t effect_variant, void *priv_data)
{
    return ESP_OK;
}

// Matter stack events (commissioning progress, Thread attach, fabric changes, …).
static void on_matter_event(const ChipDeviceEvent *event, intptr_t arg)
{
    switch (event->Type) {
    case DeviceEventType::kCommissioningComplete:
        ESP_LOGI(TAG, "Matter commissioning complete — running over Thread");
        break;
    case DeviceEventType::kThreadConnectivityChange:
        ESP_LOGI(TAG, "Thread connectivity changed");
        break;
    case DeviceEventType::kInterfaceIpAddressChanged:
        ESP_LOGI(TAG, "Interface IP address changed");
        break;
    default:
        break;
    }
}

void matter_sensor_init()
{
    node::config_t node_config;
    node_t *node = node::create(&node_config, on_attribute_update, on_identification);
    if (!node) {
        ESP_LOGE(TAG, "Failed to create Matter root node");
        return;
    }

    temperature_sensor::config_t temp_config;
    const uint16_t nodesCount = get_count(node);
    ESP_LOGI(TAG, "node count %u", nodesCount);
    
    endpoint_t *temp_ep = temperature_sensor::create(node, &temp_config, ENDPOINT_FLAG_NONE, nullptr);
    if (!temp_ep) {
        ESP_LOGE(TAG, "Failed to create Temperature sensor endpoint");
        return;
    }

    humidity_sensor::config_t humidity_config;
    endpoint_t *humidity_ep = humidity_sensor::create(node, &humidity_config, ENDPOINT_FLAG_NONE, nullptr);
    if (!humidity_ep) {
        ESP_LOGE(TAG, "Failed to create Temperature sensor endpoint");
        return;
    }

    s_temp_endpoint_id     = endpoint::get_id(temp_ep);
    s_humidity_endpoint_id = endpoint::get_id(humidity_ep);
    ESP_LOGI(TAG, "Endpoints — temperature=%u humidity=%u", s_temp_endpoint_id, s_humidity_endpoint_id);

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
    // Required before esp_matter::start() on Thread: give the OT stack its platform config.
    // static esp_openthread_platform_config_t ot_config = {
    //     .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
    //     .host_config  = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
    //     .port_config  = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    // };
    esp_openthread_radio_config_t radio_config {};
    radio_config.radio_mode = RADIO_MODE_NATIVE;
    esp_openthread_host_connection_config_t host_config {};
    host_config.host_connection_mode = HOST_CONNECTION_MODE_NONE;

    static esp_openthread_platform_config_t ot_platform_config = {
        .radio_config = radio_config,
        .host_config = host_config,
        .port_config = {
            .storage_partition_name = "nvs",
            .netif_queue_size = 10,
            .task_queue_size = 10,
        }
    };

    set_openthread_platform_config(&ot_platform_config);
#endif

    // Starts CHIP: BLE commissioning advert, Thread, ICD server. esp-matter prints
    // the onboarding QR / manual pairing code to the console at startup.
    esp_err_t err = esp_matter::start(on_matter_event);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "esp_matter::start failed: %d", err);
}

void matter_sensor_update(std::optional<float> temperature_c, std::optional<float> humidity_pct)
{
    if (temperature_c.has_value() && s_temp_endpoint_id) {
        esp_matter_attr_val_t val = esp_matter_nullable_int16(to_centi_i16(*temperature_c));
        attribute::update(s_temp_endpoint_id, TemperatureMeasurement::Id,
                          TemperatureMeasurement::Attributes::MeasuredValue::Id, &val);
    }
    if (humidity_pct.has_value() && s_humidity_endpoint_id) {
        esp_matter_attr_val_t val = esp_matter_nullable_uint16(to_centi_u16(*humidity_pct));
        attribute::update(s_humidity_endpoint_id, RelativeHumidityMeasurement::Id,
                          RelativeHumidityMeasurement::Attributes::MeasuredValue::Id, &val);
    }
}
