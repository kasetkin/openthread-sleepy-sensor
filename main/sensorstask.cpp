#include "sensorstask.h"

#include <algorithm>
#include <charconv>
#include <ranges>
#include <cmath>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_sleep.h>
#include <esp_system.h>
#include <esp_timer.h>

#include "common_utils.h"
#include "mqtt_sender.h"

SensorsTask::SensorsTask(SensorsTaskSettings settings):
    m_settings{settings}
{

}

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

void SensorsTask::configureRefreshNat64(RefreshNat64 refreshNat64)
{
    m_refreshNat64 = std::move(refreshNat64);
}

std::expected<SensorBackend, esp_err_t> SensorsTask::probeSht4x()
{
    static const char * TAG = "sensors-init-i2c";

    m_sht4dev = sht4x_t{};
    // SHT4x has a fixed I2C address (0x44) — no address argument, unlike SHT3X.
    if (const esp_err_t descErr =
            sht4x_init_desc(&m_sht4dev, SHT3X_I2C_PORT, I2C_MASTER_SDA, I2C_MASTER_SCL);
        descErr != ESP_OK) {
        ESP_LOGW(TAG, "can not init SHT4x I2C descriptor: %d err", descErr);
        return std::unexpected(descErr);
    }

    // sht4x_init() reads the serial with a CRC-checked transfer, so an SHT3X (or clone) on the
    // shared 0x44 address fails here and we fall through to the SHT3X probe.
    if (const esp_err_t sensorErr = sht4x_init(&m_sht4dev); sensorErr != ESP_OK) {
        ESP_LOGW(TAG, "SHT4x not detected via I2C: %d err — trying SHT3X", sensorErr);
        sht4x_free_desc(&m_sht4dev);
        return std::unexpected(sensorErr);
    }

    ESP_LOGI(TAG, "SHT4x detected, serial number: 0x%08lX",
             static_cast<unsigned long>(m_sht4dev.serial));

    m_sht4dev.heater = SHT4X_HEATER_OFF;
    m_sht4dev.repeatability = SHT4X_HIGH; // 10ms for one measurement

    return SensorBackend::Sht4x;
}

std::expected<SensorBackend, esp_err_t> SensorsTask::probeSht3x()
{
    static const char * TAG = "sensors-init-i2c";

    m_sht3dev = sht3x_t{};
    if (const esp_err_t descErr = sht3x_init_desc(
            &m_sht3dev, SHT3X_ADDR, SHT3X_I2C_PORT, I2C_MASTER_SDA, I2C_MASTER_SCL);
        descErr != ESP_OK) {
        ESP_LOGW(TAG, "can not init SHT3X I2C descriptor: %d err", descErr);
        return std::unexpected(descErr);
    }

    if (const esp_err_t sensorErr = sht3x_init(&m_sht3dev); sensorErr != ESP_OK) {
        ESP_LOGW(TAG, "SHT3X not detected via I2C: %d err", sensorErr);
        sht3x_free_desc(&m_sht3dev);
        return std::unexpected(sensorErr);
    }

    // Authenticity check: genuine Sensirion parts return a CRC-valid serial; clones usually
    // NACK or fail the CRC. Non-fatal — the sensor still works either way.
    if (uint32_t serial = 0; sht3x_read_serial(&m_sht3dev, &serial) == ESP_OK)
        ESP_LOGI(TAG, "SHT3x serial number: 0x%08lX", static_cast<unsigned long>(serial));
    else
        ESP_LOGW(TAG, "could not read SHT3x serial — sensor may be a counterfeit/clone");

    return SensorBackend::Sht3x;
}

std::expected<SensorBackend, esp_err_t> SensorsTask::probeInternalTemp()
{
    static const char * TAG = "sensors-init-i2c";

    // No external sensor responded — the I2C bus is unused from here on.
    i2cdev_done();
    m_i2cInitialized = false;

    temperature_sensor_config_t cfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
    if (const esp_err_t installErr = temperature_sensor_install(&cfg, &m_tsensHandle);
        installErr != ESP_OK) {
        ESP_LOGE(TAG, "can not install internal temperature sensor: %d err", installErr);
        m_tsensHandle = nullptr;
        return std::unexpected(installErr);
    }

    if (const esp_err_t enableErr = temperature_sensor_enable(m_tsensHandle);
        enableErr != ESP_OK) {
        ESP_LOGE(TAG, "can not enable internal temperature sensor: %d err", enableErr);
        temperature_sensor_uninstall(m_tsensHandle);
        m_tsensHandle = nullptr;
        return std::unexpected(enableErr);
    }

    ESP_LOGW(TAG, "no external sensor; using internal CPU temperature (no humidity)");
    return SensorBackend::InternalTemp;
}

esp_err_t SensorsTask::initI2C()
{
    static const char * TAG = "sensors-init-i2c";
    if (const esp_err_t initErr = i2cdev_init(); initErr != ESP_OK) {
        ESP_LOGE(TAG, "can not init I2C: %d err", initErr);
        return ESP_FAIL;
    }
    m_i2cInitialized = true;

    // Try the modern SHT4x first, fall back to SHT3X, and finally to the internal CPU
    // temperature sensor. or_else only runs on failure, so the first sensor to respond wins.
    const std::expected<SensorBackend, esp_err_t> selected =
        probeSht4x()
            .or_else([this](esp_err_t) { return probeSht3x(); })
            .or_else([this](esp_err_t) { return probeInternalTemp(); });

    if (!selected) {
        deinitI2C();
        return ESP_FAIL;
    }

    m_backend = *selected;
    const std::string_view name = backendName(m_backend);
    ESP_LOGI(TAG, "active sensor backend: %.*s", static_cast<int>(name.size()), name.data());
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
    switch (m_backend) {
    case SensorBackend::Sht4x:
        sht4x_free_desc(&m_sht4dev);
        break;
    case SensorBackend::Sht3x:
        sht3x_free_desc(&m_sht3dev);
        break;
    case SensorBackend::InternalTemp:
        if (m_tsensHandle) {
            temperature_sensor_disable(m_tsensHandle);
            temperature_sensor_uninstall(m_tsensHandle);
            m_tsensHandle = nullptr;
        }
        break;
    case SensorBackend::None:
        break;
    }

    // The internal-temp backend already released the bus in probeInternalTemp().
    if (m_i2cInitialized) {
        i2cdev_done();
        m_i2cInitialized = false;
    }
    m_backend = SensorBackend::None;
}

std::expected<EnvReading, esp_err_t> SensorsTask::readEnvironment()
{
    float t = 0.0f, h = 0.0f;
    switch (m_backend) {
    case SensorBackend::Sht4x:
        if (const esp_err_t e = sht4x_measure(&m_sht4dev, &t, &h); e != ESP_OK)
            return std::unexpected(e);
        return EnvReading{ .temperature = t, .humidity = h };

    case SensorBackend::Sht3x:
        if (const esp_err_t e = sht3x_measure(&m_sht3dev, &t, &h); e != ESP_OK)
            return std::unexpected(e);
        return EnvReading{ .temperature = t, .humidity = h };

    case SensorBackend::InternalTemp:
        // CPU die temperature only — no humidity channel.
        if (const esp_err_t e = temperature_sensor_get_celsius(m_tsensHandle, &t); e != ESP_OK)
            return std::unexpected(e);
        return EnvReading{ .temperature = t, .humidity = std::nullopt };

    case SensorBackend::None:
        break;
    }
    return std::unexpected(ESP_ERR_INVALID_STATE);
}

void SensorsTask::heaterLedPattern(uint32_t durationMs)
{
    // 5 blinks/second: each blink = LED on HEATER_LED_ON_MS, off for the rest of the slot.
    const uint32_t slotMs = 1000 / HEATER_LED_BLINKS_PER_SEC;                       // 200 ms
    const uint32_t offMs  = slotMs > HEATER_LED_ON_MS ? slotMs - HEATER_LED_ON_MS : 0; // 190 ms
    const uint32_t blinks = durationMs / slotMs;
    for (uint32_t i = 0; i < blinks; i++) {
        blinkUserLED(HEATER_LED_ON_MS);          // on, then off (blocks HEATER_LED_ON_MS)
        vTaskDelay(pdMS_TO_TICKS(offMs));
    }
}

esp_err_t SensorsTask::runHeaterMaintenanceSht3X(float &cleanTemp, float &cleanHum)
{
    static const char * TAG = "sensors-heater-sht3x";

    // 1) baseline — needed for the plausibility deltas and the cooldown target
    float t0, rh0;
    if (const esp_err_t e = sht3x_measure(&m_sht3dev, &t0, &rh0); e != ESP_OK) {
        ESP_LOGE(TAG, "baseline read failed: %d — skipping heater maintenance", e);
        return e;
    }
    ESP_LOGI(TAG, "baseline %.2f C / %.2f %%RH — heating %u ms", t0, rh0, HEATER_DURATION_MS);

    // 2) heater on; 3) LED pattern spans the heating window (its delays ARE the heat wait)
    if (const esp_err_t e = sht3x_set_heater(&m_sht3dev, true); e != ESP_OK)
        ESP_LOGW(TAG, "heater ON command failed: %d", e);
    heaterLedPattern(HEATER_DURATION_MS);

    // 4) heated reading → plausibility check (genuine sensor: T rises, RH drops)
    float t1 = t0, rh1 = rh0;
    if (const esp_err_t e = sht3x_measure(&m_sht3dev, &t1, &rh1); e == ESP_OK) {
        const float dT = t1 - t0;
        const float dRh = rh0 - rh1;
        ESP_LOGI(TAG, "heated %.2f C / %.2f %%RH (dT=%.2f C, dRH=%.2f %%)", t1, rh1, dT, dRh);
        if (dT >= HEATER_MIN_DELTA_T_SHT3X_C && dRh >= HEATER_MIN_RH_DROP)
            ESP_LOGI(TAG, "heater plausibility check PASSED");
        else
            ESP_LOGW(TAG, "heater plausibility check FAILED (need dT>=%.1f, dRH>=%.1f) — "
                          "sensor may be dead or a counterfeit/clone",
                     HEATER_MIN_DELTA_T_SHT3X_C, HEATER_MIN_RH_DROP);
    } else {
        ESP_LOGW(TAG, "heated read failed: %d", e);
    }

    // 5) heater off
    if (const esp_err_t e = sht3x_set_heater(&m_sht3dev, false); e != ESP_OK)
        ESP_LOGW(TAG, "heater OFF command failed: %d", e);

    // 6) cooldown — RH is biased low until the die returns to baseline (at 90%RH, 1 C ~ 5 %RH)
    const int64_t cooldownStart = esp_timer_get_time();
    float t = t1, rh = rh1;
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(COOLDOWN_POLL_MS));
        if (float tc, rhc; sht3x_measure(&m_sht3dev, &tc, &rhc) == ESP_OK) {
            t = tc; rh = rhc;
            if (t <= t0 + COOLDOWN_EPSILON_C) {
                ESP_LOGI(TAG, "cooled to %.2f C after %lld ms", t,
                         (esp_timer_get_time() - cooldownStart) / 1000);
                break;
            }
        }
        if ((esp_timer_get_time() - cooldownStart) / 1000 >= COOLDOWN_MAX_MS) {
            ESP_LOGW(TAG, "cooldown timed out at %.2f C (baseline %.2f C)", t, t0);
            break;
        }
    }

    // 7) final clean reading (fall back to last cooldown sample if it fails)
    if (const esp_err_t e = sht3x_measure(&m_sht3dev, &cleanTemp, &cleanHum); e != ESP_OK) {
        ESP_LOGW(TAG, "post-cooldown read failed: %d — using last cooldown sample", e);
        cleanTemp = t;
        cleanHum = rh;
    }
    ESP_LOGI(TAG, "post-maintenance clean reading: %.2f C / %.2f %%RH", cleanTemp, cleanHum);
    return ESP_OK;
}

esp_err_t SensorsTask::runHeaterMaintenanceSht4X(sht4x_heater_t heaterMode, float minDeltaT,
                                                 float &cleanTemp, float &cleanHum)
{
    static const char * TAG = "sensors-heater-sht4x";

    // 1) baseline — needed for the plausibility deltas and the cooldown target
    m_sht4dev.heater = SHT4X_HEATER_OFF;
    float t0, rh0;
    if (const esp_err_t e = sht4x_measure(&m_sht4dev, &t0, &rh0); e != ESP_OK) {
        ESP_LOGE(TAG, "baseline read failed: %d — skipping heater maintenance", e);
        return e;
    }
    ESP_LOGI(TAG, "baseline %.2f C / %.2f %%RH — heating up to %u ms (delta-T target %.1f C)",
             t0, rh0, HEATER_DURATION_MS, minDeltaT);

    // 2+3) The SHT4x heater has no continuous mode: each measurement command fires one heater
    //      pulse, then returns a reading. The caller picks the power/duration via `heaterMode`
    //      (gentle MEDIUM_SHORT for the self-test, aggressive HIGH_LONG for creep mitigation).
    //      Pulse until the die has heated enough (delta-T target — usually a single pulse) or
    //      the window elapses (a dead/missing sensor never heats → plausibility fails below).
    m_sht4dev.heater = heaterMode;
    float t1 = t0, rh1 = rh0;
    const int64_t heatStart = esp_timer_get_time();
    int iterationCounter = 0;
    while ((esp_timer_get_time() - heatStart) / 1000 < HEATER_DURATION_MS) {
        iterationCounter += 1;
        if (float tp, rhp; sht4x_measure(&m_sht4dev, &tp, &rhp) == ESP_OK) {
            t1 = tp;
            rh1 = rhp;
            if (t1 - t0 >= minDeltaT)  // heated enough — stop pulsing
                break;
        }
        blinkUserLED(HEATER_LED_ON_MS);  // brief per-pulse indicator (the pulse dominates timing)
    }

    // 4) heater off — return to normal (~10 ms) measurements
    m_sht4dev.heater = SHT4X_HEATER_OFF;

    // Plausibility self-test on the TEMPERATURE RISE only. Per Sensirion's creep-mitigation
    // app note (docs/SHT4x_in_humidity_env.txt) the RH reading taken during/right after a heater
    // pulse is "corrupted by the additional heat in the sensor" — not a valid signal, and it can
    // even rise on a short pulse — so we must NOT gate on it. A genuine, powered sensor always
    // heats; a dead/missing one never reaches the delta-T target. The trustworthy creep-free RH
    // is the post-cooldown reading taken below, once the die has re-equilibrated to ambient.
    const float dT = t1 - t0;
    const float dRh = rh0 - rh1;
    ESP_LOGI(TAG, "heated %.2f C / %.2f %%RH (dT=%.2f C, dRH=%.2f %% — heated RH not trusted) from %d iterations",
             t1, rh1, dT, dRh, iterationCounter);
    if (dT >= minDeltaT)
        ESP_LOGI(TAG, "heater plausibility check PASSED");
    else
        ESP_LOGW(TAG, "heater plausibility check FAILED (need dT>=%.1f C) after %d iterations — "
                      "sensor may be dead or a counterfeit/clone",
                 minDeltaT, iterationCounter);

    // 5) cooldown — RH is biased low until the die returns to baseline (at 90%RH, 1 C ~ 5 %RH)
    const int64_t cooldownStart = esp_timer_get_time();
    float t = t1, rh = rh1;
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(COOLDOWN_POLL_MS));
        if (float tc, rhc; sht4x_measure(&m_sht4dev, &tc, &rhc) == ESP_OK) {
            t = tc; rh = rhc;
            if (t <= t0 + COOLDOWN_EPSILON_C) {
                ESP_LOGI(TAG, "cooled to %.2f C after %lld ms", t,
                         (esp_timer_get_time() - cooldownStart) / 1000);
                break;
            }
        }
        if ((esp_timer_get_time() - cooldownStart) / 1000 >= COOLDOWN_MAX_MS) {
            ESP_LOGW(TAG, "cooldown timed out at %.2f C (baseline %.2f C)", t, t0);
            break;
        }
    }

    // 6) final clean reading (fall back to last cooldown sample if it fails)
    if (const esp_err_t e = sht4x_measure(&m_sht4dev, &cleanTemp, &cleanHum); e != ESP_OK) {
        ESP_LOGW(TAG, "post-cooldown read failed: %d — using last cooldown sample", e);
        cleanTemp = t;
        cleanHum = rh;
    }
    ESP_LOGI(TAG, "post-maintenance clean reading: %.2f C / %.2f %%RH", cleanTemp, cleanHum);
    return ESP_OK;
}

void SensorsTask::executeTask()
{
    static const char * TAG = "sensors-task";

    /// \todo use constexpr or values from calibration.txt for min/max values
    const auto calibrateTemperature = [this](float t) {
        return std::clamp(t + m_settings.temp_min_change, -273.15f, 3000.0f);
    };

    /// \todo use constexpr for min/max values
    const auto calibrateHum = [this](float rh) {
        return std::clamp(rh + m_settings.rh_offset, 0.0f, 100.0f);
    };

    // Cap on how long to stay awake for a publish before sleeping anyway.
    static constexpr uint32_t PUBLISH_TIMEOUT_MS = 15 * 1000;
    static constexpr uint32_t LED_BLINK_MS       = 50;

    // This task is the sole driver of the sleep cadence: arm the wakeup timer once, then
    // read → publish → wait-for-idle → light-sleep on every iteration.
    if (const esp_err_t timerRes = registerWakeupTimer(static_cast<uint64_t>(m_settings.cycle_duration_sec) * 1000 * 1000);
        timerRes != ESP_OK)
        ESP_LOGE(TAG, "can not register wakeup timer: %d — sleep interval undefined", timerRes);

    while (true) {
        // Whether data actually reached the broker this cycle. Stays false unless a publish was
        // started AND mqtt_last_publish_succeeded() confirms a connected, ACKed state message.
        bool publishedOk = false;
        bool skippedSameValuesCycle = false;
        // Don't read/publish until attached as CHILD. This is a blocking wait (not light sleep) so
        // OpenThread can finish MLE attachment / re-attach after a lost parent; light-sleeping while
        // detached would freeze the radio and stall attachment. If the network is absent the gate
        // times out and we fall through to one sleep period and retry next wake.
        if (m_attachGate && !m_attachGate(ATTACH_TIMEOUT_MS)) {
            ESP_LOGW(TAG, "OT not attached within %u ms, retrying next cycle", ATTACH_TIMEOUT_MS);
        } else {
            SensorsValues v{};

            if (auto reading = readEnvironment(); reading) {
                std::optional<float> rawTemp = reading->temperature;
                std::optional<float> rawHum = reading->humidity;

                if (m_backend == SensorBackend::Sht3x || m_backend == SensorBackend::Sht4x) {
                    m_cyclesSinceHeater++;
                    // The high-RH trigger evaluates the calibrated RH of THIS (pre-heat) reading so
                    // a biased sensor can't self-trigger endlessly.
                    m_highRhCycles = (rawHum.transform(calibrateHum).value_or(0.0f) > HIGH_RH_THRESHOLD)
                                         ? m_highRhCycles + 1 : 0;
                    const bool periodicDue = m_cyclesSinceHeater >= HEATER_PERIODIC_CYCLES;
                    const bool humidityDue = m_highRhCycles >= HIGH_RH_TRIGGER_CYCLES;

                    if (periodicDue || humidityDue) {
                        ESP_LOGI(TAG, "heater maintenance due (periodic=%d, humidity=%d)",
                                 periodicDue, humidityDue);
                        // Sustained high RH → real creep mitigation (aggressive heat); the periodic
                        // trigger only needs a gentle alive/genuine self-test. A clean post-cooldown
                        // value replaces this cycle's reading — we calibrate and publish AFTER
                        // heating, never the (heat-corrupted) heated value.
                        const bool creep = humidityDue;
                        float ct, ch;
                        const esp_err_t maint = (m_backend == SensorBackend::Sht4x)
                            ? runHeaterMaintenanceSht4X(
                                  creep ? HEATER_SHT4X_CREEP_MODE    : HEATER_SHT4X_SELFTEST_MODE,
                                  creep ? HEATER_SHT4X_CREEP_DELTA_T : HEATER_SHT4X_SELFTEST_DELTA_T,
                                  ct, ch)
                            : runHeaterMaintenanceSht3X(ct, ch);
                        if (maint == ESP_OK) {
                            rawTemp = ct;
                            rawHum  = ch;
                        }
                        m_cyclesSinceHeater = 0;
                        m_highRhCycles = 0;
                    }
                }

                // Calibrate AFTER maintenance so the published value is the clean post-cooldown
                // reading (for the creep trigger that is the point: fire heater, then measure).
                const std::optional<float> calTemp = rawTemp.transform(calibrateTemperature);
                const std::optional<float> calHum = rawHum.transform(calibrateHum);

                v.envTemperature = calTemp;
                // Humidity is published for both real sensors; the internal-temp fallback has none.
                if (calHum)
                    v.envHumidity = *calHum;

                const std::string_view name = backendName(m_backend);
                ESP_LOGI(TAG, "%.*s raw %.2f C / %.2f %%RH  ->  calibrated %.2f C / %.2f %%RH",
                         static_cast<int>(name.size()), name.data(),
                         rawTemp.value_or(NAN), rawHum.value_or(NAN),
                         calTemp.value_or(NAN), calHum.value_or(NAN));
            } else {
                ESP_LOGE(TAG, "sensor read error: %d", reading.error());
            }

            bool sameAsPreviousDelivered = true;
            const float tempChange = std::abs(v.envTemperature.value_or(1000000.0) - m_previousDeliveredValue.envTemperature.value_or(0.0));
            if (tempChange >= m_settings.temp_min_change)
                sameAsPreviousDelivered = false;

            const float rhChange = std::abs(v.envHumidity.value_or(1000000.0) - m_previousDeliveredValue.envHumidity.value_or(0.0));
            if (rhChange >= m_settings.rh_min_change)
                sameAsPreviousDelivered = false;

            skippedSameValuesCycle = sameAsPreviousDelivered && (m_sameValueSkippedCycles <= m_settings.max_skip_cycles);

            if (!skippedSameValuesCycle) {
                if (m_readyEvent)
                    m_readyEvent(v);  // triggers an async MQTT publish when attached as CHILD

                // Only wait if a publish was actually started (skipped when not yet attached). Waiting
                // before sleeping stops light sleep from freezing the MQTT task/radio mid-flight.
                // mqtt_wait_for_idle() only reports the task ended, not that it succeeded — that ending
                // is identical for a failed connect — so the real outcome comes from
                // mqtt_last_publish_succeeded(). A timeout means MQTT stalled; sleep anyway rather than
                // stay awake burning battery.
                if (mqtt_is_busy()) {
                    if (mqtt_wait_for_idle(PUBLISH_TIMEOUT_MS)) {
                        publishedOk = mqtt_last_publish_succeeded();
                        m_previousDeliveredValue = v;
                        m_sameValueSkippedCycles = 0;
                    } else {
                        ESP_LOGW(TAG, "publish did not finish within %u ms, sleeping anyway", PUBLISH_TIMEOUT_MS);
                    }
                }
            } else {
                /// skip this cycle because it has nearly the same values as already delivered ones
                m_sameValueSkippedCycles += 1;
            }
        }

        // Honest local indicator: 1 blink = data reached the broker, 5 blinks = it did not.
        if (publishedOk) {
            blinkUserLED(LED_BLINK_MS);
        } else {
            if (skippedSameValuesCycle) {
                ESP_LOGI(TAG, "data was not published because it has same values");
                blinkUserLED(LED_BLINK_MS, 2);
            } else {
                ESP_LOGW(TAG, "data was not published to the broker this cycle");
                blinkUserLED(LED_BLINK_MS, 5);
            }
        }

        // Recovery supervisor: count consecutive failed cycles and reboot once they pass the
        // threshold. There is no other path back from a persistent reachability loss — a reboot
        // re-attaches to Thread and re-learns the NAT64 route. Before that, give a softer nudge:
        // re-read network data so a merely-stale NAT64 prefix is fixed without a reboot.
        if (publishedOk || skippedSameValuesCycle) {
            m_consecutiveFailures = 0;
        } else {
            if (m_refreshNat64)
                m_refreshNat64();
                
            if (++m_consecutiveFailures >= REBOOT_AFTER_FAILS) {
                ESP_LOGE(TAG, "%lu consecutive cycles without a successful publish — rebooting to recover",
                         static_cast<unsigned long>(m_consecutiveFailures));
                esp_restart();
            }
        }

        ESP_LOGI(TAG, "sensor values ready, go to sleep");
        vTaskDelay(pdMS_TO_TICKS(static_cast<uint32_t>(m_settings.cycle_duration_sec) * 1000));
        // correctLightSleep();  // light-sleep for SENSORS_PERIOD_MS until the next cycle
    }
}