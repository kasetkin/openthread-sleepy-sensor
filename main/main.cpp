#include <memory>
#include <ranges>
#include <string>
#include "esp_err.h"
#include "esp_log.h"

#include <sys/stat.h>

#include "main.h"
#include "common_utils.h"
#include "sensorstask.h"
#include "errortask.h"

static const char *TAG = "main-body";
constexpr uint32_t DEFAULT_TASK_STACK_SIZE = 16384;
constexpr bool ADC_READING_ENABLE = false;

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

extern "C" void app_main(void)
{
    esp_err_t ret;

    ret = initNvsFlash();
    if (ret != ESP_OK) {
        ESP_LOGE("main", "Can not inint NVS flash. Exit.");
        return;
    }

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_vfs_eventfd_register(&eventfd_config));

    enableRf(true);
    enableExtAntenna(false);
    if (const esp_err_t timerErr = registerWakeupTimer(static_cast<uint32_t>(100'000)); timerErr != ESP_OK)
        ESP_LOGE(TAG, "registerWakeupTimer failed: %d", timerErr);

    ESP_LOGI(TAG, "create sensors task");
    sensorTask = std::make_shared<SensorsTask>();

    ESP_LOGI(TAG, "configure sensors");
    ret = sensorTask->init(ADC_READING_ENABLE);
    if (ret != ESP_OK) {
        startErrorTask(ErrorTask::ErrorCode::ecSensorsFail);
        return;
    }

    ESP_LOGI(TAG, "configure Sensors::ReadyEvent (pass battery, temp, humidity to BLE task)");
    sensorTask->configureReadyEvent([](const SensorsValues &values) static
    {
        ESP_LOGI(TAG, "Logger: ReadyEnevt");
        const std::string message = values.toTelemetryString();
        // if (bleTask) {
        //     ESP_LOGI(TAG, "Logger: ReadyEnevt: %s", values.toLogString().c_str());
        //     bleTask->setSensorsValues(values);
        // }
        // if (loggerTask)
        //     loggerTask->setSensorsLog(message);
    });

    ESP_LOGI(TAG, "read sensors, start task");
    xTaskCreate([](void *) static
    {
        sensorTask->executeTask();
        vTaskDelete(nullptr);
    }, "sensors_task", DEFAULT_TASK_STACK_SIZE, nullptr, 6, nullptr);

    startErrorTask(ErrorTask::ErrorCode::ecOK);
}
