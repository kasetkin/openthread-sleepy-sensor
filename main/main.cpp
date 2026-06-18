#include <memory>
#include <string>
#include <string_view>
#include <charconv>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_flash.h"
#include "nvs_flash.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "main.h"
#include "common_utils.h"
#include "secrets.h"
#include "calibration.h"
#include "sensorstask.h"
#include "errortask.h"
#include "matter_sensor.h"

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

// Parse a float offset from calibration.txt; returns 0.0f if the key is missing/unparseable.
// from_chars (not stof) because C++ exceptions are disabled in this build.
static float parse_calib_offset(std::string_view content, std::string_view key)
{
    const std::string s = yaml_get_string(content, key);
    float value = 0.0f;
    if (!s.empty())
        std::from_chars(s.data(), s.data() + s.size(), value);  // leaves value=0 on failure
    return value;
}

extern "C" void app_main(void)
{
    esp_err_t ret;

    uint32_t flash_size = 0;
    if (esp_flash_get_size(NULL, &flash_size) == ESP_OK) {
        ESP_LOGI("main", "Detected flash size: %u bytes (%u MB)",
                 static_cast<unsigned>(flash_size),
                 static_cast<unsigned>(flash_size / (1024 * 1024)));
    } else {
        ESP_LOGE("main", "esp_flash_get_size failed");
    }

    ret = initNvsFlash();
    if (ret != ESP_OK) {
        ESP_LOGE("main", "Cannot init NVS flash. Exit.");
        return;
    }

    // Seeed C6 RF front-end: enable the radio path, select the on-board antenna.
    enableRf(true);
    enableExtAntenna(false);

    // ── sensors ───────────────────────────────────────────────────────────────
    ESP_LOGI(TAG, "create sensors task");
    sensorTask = std::make_shared<SensorsTask>();
    if (sensorTask->init() != ESP_OK) {
        startErrorTask(ErrorTask::ErrorCode::ecSensorsFail);
        return;
    }

    // per-device calibration (calibration.txt, embedded at build time)
    const float rh_offset   = parse_calib_offset(calibration_txt(), "rh_offset");
    const float temp_offset = parse_calib_offset(calibration_txt(), "temp_offset");
    ESP_LOGI("main", "calibration offsets: RH %+.2f %%RH, T %+.2f C", rh_offset, temp_offset);
    sensorTask->configureCalibration(rh_offset, temp_offset);

    // Each cycle's calibrated reading is pushed into the Matter Temperature/Humidity
    // attributes; the Matter subscription engine + ICD report it to the controller.
    // No per-cycle connection to open/drain — so no attach gate and no wait-for-idle
    // (both of which the MQTT design needed).
    sensorTask->configureReadyEvent([](const SensorsValues &values) static
    {
        matter_sensor_update(values.envTemperature, values.envHumidity);
    });

    // ── Matter ────────────────────────────────────────────────────────────────
    // Root node + Temperature (0x0402) + Humidity (0x0405) endpoints, OpenThread
    // platform config, then start CHIP (BLE commissioning + Thread + ICD). esp-matter
    // prints the onboarding QR / manual pairing code to the console at startup.
    matter_sensor_init();

    // sensors_task is the driver of the read→update→sleep cadence.
    xTaskCreate([](void *) static
    {
        sensorTask->executeTask();
        vTaskDelete(nullptr);
    }, "sensors_task", DEFAULT_TASK_STACK_SIZE, nullptr, 6, nullptr);

    startErrorTask(ErrorTask::ErrorCode::ecOK);

    // app_main returns; the FreeRTOS scheduler keeps sensors_task and the Matter/OT
    // tasks running. Light sleep is automatic (CONFIG_PM_ENABLE + tickless idle),
    // coordinated with the Matter ICD — not forced from the sensor task.
}
