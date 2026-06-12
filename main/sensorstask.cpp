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
    while (true) {

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
            m_readyEvent(v);
            
        vTaskDelay(pdMS_TO_TICKS(SENSORS_PERIOD_MS));
    }
}