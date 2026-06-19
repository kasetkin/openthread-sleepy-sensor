#include "sensorstask.h"

#include <algorithm>
#include <cmath>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_timer.h>

#include "common_utils.h"

void SensorsTask::configureReadyEvent(SensorsReadyEvent readyEvent)
{
    m_readyEvent = std::move(readyEvent);
}

void SensorsTask::configureCalibration(float rhOffset, float tempOffset)
{
    m_rhOffset = rhOffset;
    m_tempOffset = tempOffset;
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

    // Authenticity check: genuine Sensirion parts return a CRC-valid serial; clones usually
    // NACK or fail the CRC. Non-fatal — the sensor still works either way.
    if (uint32_t serial = 0; sht3x_read_serial(&m_sht3dev, &serial) == ESP_OK)
        ESP_LOGI(TAG, "SHT3x serial number: 0x%08lX", static_cast<unsigned long>(serial));
    else
        ESP_LOGW(TAG, "could not read SHT3x serial — sensor may be a counterfeit/clone");

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

esp_err_t SensorsTask::runHeaterMaintenance(float &cleanTemp, float &cleanHum)
{
    static const char * TAG = "sensors-heater";

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
        if (dT >= HEATER_MIN_DELTA_T_C && dRh >= HEATER_MIN_RH_DROP)
            ESP_LOGI(TAG, "heater plausibility check PASSED");
        else
            ESP_LOGW(TAG, "heater plausibility check FAILED (need dT>=%.1f, dRH>=%.1f) — "
                          "sensor may be dead or a counterfeit/clone",
                     HEATER_MIN_DELTA_T_C, HEATER_MIN_RH_DROP);
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

void SensorsTask::executeTask()
{
    static const char * TAG = "sensors-task";

    static constexpr uint32_t LED_BLINK_MS = 50;

    // This task drives the read→report→sleep cadence. Matter stores the latest attribute
    // values and its subscription engine reports them whenever the device is connected, so
    // there is no attach gate: a reading taken before Thread is up is simply held in the
    // cluster and reported once a controller is reachable.
    while (true) {

        SensorsValues v{};

        float rawTemp, rawHum;
        if (const esp_err_t readError = sht3x_measure(&m_sht3dev, &rawTemp, &rawHum);
            readError == ESP_OK) {
            // calibrated values drive both the reported value and the high-humidity trigger
            float calTemp = rawTemp + m_tempOffset;
            float calHum  = std::clamp(rawHum + m_rhOffset, 0.0f, 100.0f);

            // Schedule heater maintenance: periodic (~24 h) or after sustained high
            // humidity. The humidity trigger uses calibrated RH so a biased sensor can't
            // self-trigger endlessly.
            m_cyclesSinceHeater++;
            m_highRhCycles = (calHum > HIGH_RH_THRESHOLD) ? m_highRhCycles + 1 : 0;
            const bool periodicDue = m_cyclesSinceHeater >= HEATER_PERIODIC_CYCLES;
            const bool humidityDue = m_highRhCycles >= HIGH_RH_TRIGGER_CYCLES;

            if (periodicDue || humidityDue) {
                ESP_LOGI(TAG, "heater maintenance due (periodic=%d, humidity=%d)",
                         periodicDue, humidityDue);
                // Reporting is suppressed during maintenance (heated readings are discarded);
                // a clean post-cooldown value replaces this cycle's reading.
                if (float ct, ch; runHeaterMaintenance(ct, ch) == ESP_OK) {
                    rawTemp = ct;
                    rawHum  = ch;
                    calTemp = rawTemp + m_tempOffset;
                    calHum  = std::clamp(rawHum + m_rhOffset, 0.0f, 100.0f);
                }
                m_cyclesSinceHeater = 0;
                m_highRhCycles = 0;
            }

            v.envTemperature = calTemp;
            v.envHumidity    = calHum;

            ESP_LOGI(TAG, "SHT3x raw %.2f C / %.2f %%RH  ->  calibrated %.2f C / %.2f %%RH",
                     rawTemp, rawHum, calTemp, calHum);
        } else {
            ESP_LOGE(TAG, "sensor read error: %d", readError);
        }

        if (m_readyEvent) {
            ESP_LOGI(TAG, "handing reading to Matter (T=%s, RH=%s)",
                     v.envTemperature.has_value() ? "set" : "none",
                     v.envHumidity.has_value() ? "set" : "none");
            m_readyEvent(v);  // pushes the reading into the Matter attributes
        }

        // Matter takes the attribute update synchronously; the subscription engine reports it
        // in the background. Brief heartbeat blink on a fresh reading.
        if (v.envTemperature.has_value() || v.envHumidity.has_value())
            blinkUserLED(LED_BLINK_MS);

        ESP_LOGI(TAG, "sensor values reported, sleeping for %u ms", SENSORS_PERIOD_MS);
        // Plain delay: with CONFIG_PM_ENABLE + tickless idle the system auto-light-sleeps
        // when idle (don't force esp_light_sleep here — it would freeze the Matter/OpenThread
        // tasks mid-poll).
        vTaskDelay(pdMS_TO_TICKS(SENSORS_PERIOD_MS));
    }
}