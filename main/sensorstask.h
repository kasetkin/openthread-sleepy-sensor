#pragma once

#include <cmath>
#include <numeric>
#include <optional>
#include <expected>
#include <memory>
#include <string>
#include <functional>
#include <esp_err.h>
#include <driver/gpio.h>
#include "i2cdev.h"
#include "sht3x.h"

struct SensorsValues
{
public:
    std::optional<float> envTemperature;
    std::optional<float> envHumidity;
    std::optional<float> barometricPressure;

    std::string toTelemetryString() const;
    std::string toLogString() const;
    /// up to 3 decimal digits; trailing zeros and dot stripped
    static std::string toTelemetryRoundedString(const float value);
};

class SensorsTask
{
public:
    ~SensorsTask();
    SensorsTask() = default;
    SensorsTask(const SensorsTask &) = delete("SensorsTask owns I2C device handles — copying aliases hardware resources");
    SensorsTask &operator=(const SensorsTask &) = delete("SensorsTask owns I2C device handles — copying aliases hardware resources");

    [[nodiscard("sensors unavailable if init failure ignored")]]
    esp_err_t init();

    void executeTask();

    [[nodiscard("output params undefined on failure")]]
    esp_err_t readEnvironment(float &temperature, float &humidity);

    using SensorsReadyEvent = std::function<void(const SensorsValues &values)>;
    void configureReadyEvent(SensorsReadyEvent readyEvent);

private:
    static constexpr uint32_t SENSORS_PERIOD_MS = 10 * 1000;

    /// ENVIRONMENT sensor, SHT31 via I2C bus
    static constexpr uint8_t SHT3X_ADDR = SHT3X_I2C_ADDR_GND; // 0x44
    static constexpr gpio_num_t I2C_MASTER_SDA = GPIO_NUM_22;
    static constexpr gpio_num_t I2C_MASTER_SCL = GPIO_NUM_23;
    static constexpr i2c_port_t SHT3X_I2C_PORT = I2C_NUM_0;
    
    bool m_i2cInitialized = false;
    SensorsReadyEvent m_readyEvent;
    sht3x_t m_sht3dev;

    [[nodiscard("I2C unavailable if init failure ignored")]]
    esp_err_t initI2C();
    void deinitI2C();
};