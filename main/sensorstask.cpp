#include "sensorstask.h"

#include <algorithm>
#include <charconv>
#include <ranges>
#include <cmath>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_sleep.h>

#include "common_utils.h"
#include "mqtt_sender.h"

std::string SensorsValues::toTelemetryRoundedString(const float value)
{
    // buf[24]: fixed,3 for sensor ranges (±150 °C, 0–1200 hPa) never exceeds 10 chars.
    char buf[24];
    auto [ptr, ec] = std::to_chars(buf, buf + sizeof(buf), value, std::chars_format::fixed, 3);
    if (ec != std::errc{})
        return "ERR";
    std::string_view sv(buf, ptr);
    if (!sv.contains('.'))
        return std::string(sv);
    const auto isTrailingZero = [](char c) static {
        return c == '0';
    };
    const auto trailing = sv | std::views::reverse | std::views::take_while(isTrailingZero);
    sv.remove_suffix(std::ranges::distance(trailing));
    if (sv.ends_with('.'))
        sv.remove_suffix(1);
    return std::string(sv);
}

std::string SensorsValues::toTelemetryString() const
{
    std::string message;
    if (envTemperature) {
        message += std::string_view("TEMP;");
        message += toTelemetryRoundedString(envTemperature.value());
        message += std::string_view(";");
    }
    if (envHumidity) {
        message += std::string_view("HUMID;");
        message += toTelemetryRoundedString(envHumidity.value());
        message += std::string_view(";");
    }
    if (barometricPressure) {
        message += std::string_view("PRESS;");
        message += toTelemetryRoundedString(barometricPressure.value());
        message += std::string_view(";");
    }
    return message;
}

std::string SensorsValues::toLogString() const
{
    auto appendOpt = [](std::string& out, const auto& opt) static {
        if (opt.has_value()) appendNum(out, opt.value());
        else out += "NO_VALUE";
    };
    std::string result;
    result.reserve(100);
    result += "envTemperature: ";
    appendOpt(result, envTemperature);
    result += ", envHumidity: ";
    appendOpt(result, envHumidity);
    result += ", barometricPressure: ";
    appendOpt(result, barometricPressure);
    return result;
}

void SensorsTask::configureReadyEvent(SensorsReadyEvent readyEvent)
{
    m_readyEvent = std::move(readyEvent);
}

void SensorsTask::configureAttachGate(AttachGate attachGate)
{
    m_attachGate = std::move(attachGate);
}

esp_err_t SensorsTask::initI2C() 
{
    static const char * TAG = "sensors-init-i2c";
    if (const esp_err_t initErr = i2cdev_init(); initErr != ESP_OK) {
        ESP_LOGE(TAG, "can not init I2C: %d err", initErr);
        return ESP_FAIL;
    }

    m_sht3dev = sht3x_t{};
    if (const esp_err_t descriptorInitErr = sht3x_init_desc(
            &m_sht3dev, SHT3X_ADDR, SHT3X_I2C_PORT, I2C_MASTER_SDA, I2C_MASTER_SCL);
        descriptorInitErr != ESP_OK) {
        ESP_LOGE(TAG, "can not init I2C descriptor structure: %d err", descriptorInitErr);
        i2cdev_done();
        return ESP_FAIL;
    }

    if (const esp_err_t sensorInitErr = sht3x_init(&m_sht3dev); sensorInitErr != ESP_OK) {
        ESP_LOGE(TAG, "can not init SHT3X sensor via I2C: %d err", sensorInitErr);
        sht3x_free_desc(&m_sht3dev);
        i2cdev_done();
        return ESP_FAIL;
    }

    m_i2cInitialized = true;
    return ESP_OK;
}

esp_err_t SensorsTask::init()
{
    static const char * TAG = "sensors-init";
    ESP_LOGI(TAG, "init all sensors: start");

    deinitI2C();
    if (const esp_err_t i2cErr = initI2C(); i2cErr != ESP_OK) {
        deinitI2C();
        return i2cErr;
    }

    return ESP_OK;
}

SensorsTask::~SensorsTask()
{
    deinitI2C();
}

void SensorsTask::deinitI2C()
{
    if (!m_i2cInitialized)
        return;
    sht3x_free_desc(&m_sht3dev);
    i2cdev_done();
    m_i2cInitialized = false;
}

esp_err_t SensorsTask::readEnvironment(float &temperature, float &humidity)
{
    return sht3x_measure(&m_sht3dev, &temperature, &humidity);
}

void SensorsTask::executeTask()
{
    static const char * TAG = "sensors-task";

    // Cap on how long to stay awake for a publish before sleeping anyway.
    static constexpr uint32_t PUBLISH_TIMEOUT_MS = 15 * 1000;
    static constexpr uint32_t LED_BLINK_MS       = 50;

    // This task is the sole driver of the sleep cadence: arm the wakeup timer once, then
    // read → publish → wait-for-idle → light-sleep on every iteration.
    if (const esp_err_t timerRes = registerWakeupTimer(static_cast<uint64_t>(SENSORS_PERIOD_MS) * 1000);
        timerRes != ESP_OK)
        ESP_LOGE(TAG, "can not register wakeup timer: %d — sleep interval undefined", timerRes);

    while (true) {

        // Don't read/publish until attached as CHILD. This is a blocking wait (not light sleep) so
        // OpenThread can finish MLE attachment / re-attach after a lost parent; light-sleeping while
        // detached would freeze the radio and stall attachment. If the network is absent the gate
        // times out and we fall through to one sleep period and retry next wake.
        if (m_attachGate && !m_attachGate(ATTACH_TIMEOUT_MS)) {
            ESP_LOGW(TAG, "OT not attached within %u ms, retrying next cycle", ATTACH_TIMEOUT_MS);
        } else {
            SensorsValues v{};

            float tempVal, humidVal;
            if (const esp_err_t readError = sht3x_measure(&m_sht3dev, &tempVal, &humidVal);
                readError == ESP_OK) {
                v.envTemperature = tempVal;
                v.envHumidity = humidVal;
                ESP_LOGI(TAG, "SHT3x Sensor: %.2f °C, %.2f %%", tempVal, humidVal);
            } else {
                ESP_LOGE(TAG, "sensor read error: %d", readError);
            }

            if (m_readyEvent)
                m_readyEvent(v);  // triggers an async MQTT publish when attached as CHILD

            // Only wait/blink if a publish was actually started (skipped when not yet attached).
            // Waiting before sleeping stops light sleep from freezing the MQTT task/radio mid-flight:
            // a completed publish gets a brief LED heartbeat; a timeout means MQTT stalled, so we
            // skip the blink and sleep anyway rather than stay awake burning battery.
            if (mqtt_is_busy()) {
                if (mqtt_wait_for_idle(PUBLISH_TIMEOUT_MS))
                    blinkUserLED(LED_BLINK_MS);
                else
                    ESP_LOGW(TAG, "publish did not finish within %u ms, sleeping anyway", PUBLISH_TIMEOUT_MS);
            }
        }

        ESP_LOGI(TAG, "sensor values ready, go to sleep");
        if (isUsbConsoleConnected())
            vTaskDelay(pdMS_TO_TICKS(SENSORS_PERIOD_MS));
        else
            correctLightSleep();  // light-sleep for SENSORS_PERIOD_MS until the next cycle
    }
}