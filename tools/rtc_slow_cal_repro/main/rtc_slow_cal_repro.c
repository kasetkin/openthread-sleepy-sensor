/*
 * Reproducer: with automatic light sleep and the internal RC as RTC_SLOW, ESP-IDF calibrates
 * RTC_SLOW at sleep entry, while the die is still warm from the active phase. The RC speeds up
 * as the die cools during the sleep, so the sleep is converted with a period that is too long and
 * esp_timer / the RTC time run fast.
 *
 * The app wakes every REPRO_WAKE_PERIOD_MS, busy-loops REPRO_BUSY_MS to warm the die, and every
 * REPRO_REPORT_EVERY wakes prints a line with esp_timer_get_time():
 *
 *     CLK n=<wakes> t_us=<esp_timer> cal=<RTC_SLOW period, Q19 us>
 *     die_cold=<C, right after the wake> die_warm=<C, at the end of the busy time>
 *
 * clock_drift.py timestamps each line on arrival and fits esp_timer against the host clock.
 */

#include <stdbool.h>
#include <inttypes.h>
#include <math.h>                           // NAN

#include "driver/temperature_sensor.h"      // read die temperature
#include "esp_log.h"                        // debug output
#include "esp_idf_version.h"                // for esp_get_idf_version() debug output
#include "esp_pm.h"                         // enable light sleep
#include "esp_private/esp_clk.h"            // for esp_clk_slowclk_cal_get() debug output
#include "esp_timer.h"                      
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"                      // get config option for mode_name() debug output


static const char *mode_name(void)
{
#if CONFIG_PM_LIGHTSLEEP_RTC_OSC_CAL_AFTER_WAKEUP
    return "fix-idf";
#else
    return "stock";
#endif
}

// Readings averaged per die temperature value, and the gap between them: the sensor's output
// only moves once per conversion.
#define DIE_TEMP_SAMPLES        16
#define DIE_TEMP_SPACING_US     50

// Installs the sensor for each reading and uninstalls it after, so it is never left powered (and
// holding its power domain) during light sleep. NAN if the sensor fails.
static float die_temp_read(void)
{
    const temperature_sensor_config_t config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
    temperature_sensor_handle_t tsens = NULL;
    if (temperature_sensor_install(&config, &tsens) != ESP_OK)
        return NAN;

    float result = NAN;
    if (temperature_sensor_enable(tsens) == ESP_OK) {
        float sum = 0;
        int count = 0;
        for (int i = 0; i < DIE_TEMP_SAMPLES; i++) {
            float celsius = 0;
            if (temperature_sensor_get_celsius(tsens, &celsius) == ESP_OK) {
                sum += celsius;
                count++;
            }
            esp_rom_delay_us(DIE_TEMP_SPACING_US);
        }
        if (count > 0) {
            result = sum / count;
        }
        temperature_sensor_disable(tsens);
    }
    temperature_sensor_uninstall(tsens);
    return result;
}

static void busy_for_ms(uint32_t ms)
{
    const int64_t end = esp_timer_get_time() + (int64_t)ms * 1000;
    volatile int32_t trash = 0;
    while (esp_timer_get_time() < end)
        trash = (trash + 239) % 1001;
}

void app_main(void)
{
    const char *LOGID = "temperature_sensor";
    const esp_pm_config_t pm_config = {
        .max_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
        .min_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
        .light_sleep_enable = true,
    };
    ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
    // The driver logs its measuring range on every install, and this installs on every reading.
    esp_log_level_set(LOGID, ESP_LOG_WARN);

    ESP_LOGI(LOGID, "REPRO idf=%s wake_ms=%d busy_ms=%d report_every=%d",
           esp_get_idf_version(), CONFIG_REPRO_WAKE_PERIOD_MS, CONFIG_REPRO_BUSY_MS, CONFIG_REPRO_REPORT_EVERY
          );

    for (uint32_t n = 1;; n++) {
        vTaskDelay(pdMS_TO_TICKS(CONFIG_REPRO_WAKE_PERIOD_MS));
        const bool report = (n % CONFIG_REPRO_REPORT_EVERY == 0);
        const float die_cold = report ? die_temp_read() : NAN;
        busy_for_ms(CONFIG_REPRO_BUSY_MS);
        if (report) {
            const float die_warm = die_temp_read();
            ESP_LOGI(LOGID, "CLK n=%" PRIu32 " t_us=%" PRId64 " mode=%s cal=%" PRIu32 " die_cold=%.2f die_warm=%.2f\n",
                   n, esp_timer_get_time(), mode_name(), esp_clk_slowclk_cal_get(), die_cold, die_warm);
        }
    }
}
