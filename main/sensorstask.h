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
    SensorsTask(const SensorsTask &) = delete; //("SensorsTask owns I2C device handles — copying aliases hardware resources");
    SensorsTask &operator=(const SensorsTask &) = delete; //("SensorsTask owns I2C device handles — copying aliases hardware resources");

    [[nodiscard("sensors unavailable if init failure ignored")]]
    esp_err_t init();

    void executeTask();

    [[nodiscard("output params undefined on failure")]]
    esp_err_t readEnvironment(float &temperature, float &humidity);

    using SensorsReadyEvent = std::function<void(const SensorsValues &values)>;
    void configureReadyEvent(SensorsReadyEvent readyEvent);

    /// Blocks up to the given ms for Thread attachment; returns true once attached.
    /// Injected by main so the task gates each cycle without depending on OpenThread directly.
    using AttachGate = std::function<bool(uint32_t timeoutMs)>;
    void configureAttachGate(AttachGate attachGate);

    /// Per-device calibration offsets added to each raw reading before publishing.
    /// Sourced from calibration.txt and injected by main (default 0 = no correction).
    void configureCalibration(float rhOffset, float tempOffset);

private:
    /// single source of truth for the wake→read→publish→sleep cadence
    static constexpr uint32_t SENSORS_PERIOD_MS = 60 * 1000;

    /// per-cycle awake budget to (re)attach before sleeping anyway; > typical attach time and SENSORS_PERIOD_MS
    static constexpr uint32_t ATTACH_TIMEOUT_MS = 30 * 1000;

    /// --- heater maintenance / plausibility self-test (see Sensirion docs in docs/) ---
    /// periodic cadence (~24 h at SENSORS_PERIOD_MS): routine plausibility check + creep mitigation
    static constexpr uint32_t HEATER_PERIODIC_CYCLES = 24 * 60;
    /// condition trigger: run heater after sustained high humidity (evaluated on calibrated RH)
    static constexpr float    HIGH_RH_THRESHOLD       = 90.0f;   // %RH
    static constexpr uint32_t HIGH_RH_TRIGGER_CYCLES  = 60;      // ~1 h above threshold
    static constexpr uint32_t HEATER_DURATION_MS      = 20 * 1000; // heating window (rise is a few °C)
    static constexpr float    HEATER_MIN_DELTA_T_C    = 1.5f;    // pass if heated T − baseline T ≥
    static constexpr float    HEATER_MIN_RH_DROP      = 2.0f;    // pass if baseline RH − heated RH ≥
    static constexpr float    COOLDOWN_EPSILON_C      = 0.3f;    // resume once T within this of baseline
    static constexpr uint32_t COOLDOWN_MAX_MS         = 90 * 1000; // cap on cooldown wait
    static constexpr uint32_t COOLDOWN_POLL_MS        = 2 * 1000;  // re-read interval while cooling
    /// LED indicator while heating: 5 blinks/second, 10 ms each
    static constexpr uint32_t HEATER_LED_ON_MS        = 10;
    static constexpr uint32_t HEATER_LED_BLINKS_PER_SEC = 5;

    /// ENVIRONMENT sensor, SHT31 via I2C bus
    static constexpr uint8_t SHT3X_ADDR = SHT3X_I2C_ADDR_GND; // 0x44
    static constexpr gpio_num_t I2C_MASTER_SDA = GPIO_NUM_22;
    static constexpr gpio_num_t I2C_MASTER_SCL = GPIO_NUM_23;
    static constexpr i2c_port_t SHT3X_I2C_PORT = I2C_NUM_0;
    
    bool m_i2cInitialized = false;
    SensorsReadyEvent m_readyEvent;
    AttachGate m_attachGate;
    sht3x_t m_sht3dev;

    /// maintenance scheduling counters (persist across light sleep — RAM is retained)
    uint32_t m_cyclesSinceHeater = 0;
    uint32_t m_highRhCycles = 0;
    /// calibration offsets injected via configureCalibration()
    float m_rhOffset = 0.0f;
    float m_tempOffset = 0.0f;

    [[nodiscard("I2C unavailable if init failure ignored")]]
    esp_err_t initI2C();
    void deinitI2C();

    /// Heater plausibility self-test + creep/dew maintenance: baseline → heat (with LED
    /// pattern) → verify T↑/RH↓ → cooldown to baseline. Publishing is suppressed by the
    /// caller during this. Outputs the raw post-cooldown reading; caller applies calibration.
    esp_err_t runHeaterMaintenance(float &cleanTemp, float &cleanHum);

    /// LED shown while the heater is on: 5 blinks/s for durationMs (its delays are the heat wait).
    void heaterLedPattern(uint32_t durationMs);
};