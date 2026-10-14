/*
 * Reproducer: with automatic light sleep and the internal RC as RTC_SLOW, ESP-IDF calibrates
 * RTC_SLOW at sleep entry, while the die is still warm from the active phase. The RC speeds up
 * as the die cools during the sleep, so the sleep is converted with a period that is too long and
 * esp_timer / the RTC time run fast.
 *
 * The app wakes every REPRO_WAKE_PERIOD_MS, busy-loops REPRO_BUSY_MS to warm the die, and every
 * REPRO_REPORT_EVERY wakes prints a line with esp_timer_get_time():
 *
 *     CLK n=<wakes> t_us=<esp_timer> mode=<stock|fix-emul|fix-idf> cal=<RTC_SLOW period, Q19 us>
 *         die_cold=<C, right after the wake> die_warm=<C, at the end of the busy time>
 *
 * clock_drift.py timestamps each line on arrival and fits esp_timer against the host clock.
 */

#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "driver/temperature_sensor.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_err.h"
#include "esp_idf_version.h"
#include "esp_pm.h"
#include "esp_private/esp_clk.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "soc/rtc.h"

#if CONFIG_REPRO_EMULATE_CAL_AFTER_WAKEUP
// RTC_CLK_SRC_CAL_CYCLES in esp_hw_support/sleep_modes.c: only the light-sleep path calibrates
// RTC_SLOW with this length.
#define SLEEP_PATH_CAL_CYCLES   10
// A shorter sleep doesn't let the die cool from what came before it.
#define COLD_MIN_SLEEP_US       (50 * 1000)

static volatile bool s_fix_on = false;
// Taken right after the last long-enough sleep; 0 until the first one.
static volatile uint32_t s_cold_period = 0;

uint32_t __real_rtc_clk_cal(soc_clk_freq_calculation_src_t cal_clk_sel, uint32_t slowclk_cycles);

// The sleep path calls rtc_clk_cal() from IRAM with interrupts off.
IRAM_ATTR uint32_t __wrap_rtc_clk_cal(soc_clk_freq_calculation_src_t cal_clk_sel, uint32_t slowclk_cycles)
{
    if (s_fix_on && s_cold_period != 0 && cal_clk_sel == CLK_CAL_RTC_SLOW && slowclk_cycles == SLEEP_PATH_CAL_CYCLES) {
        return s_cold_period;
    }
    return __real_rtc_clk_cal(cal_clk_sel, slowclk_cycles);
}

// Runs from the IDLE task right after each light sleep, before any other task warms the die.
static esp_err_t on_light_sleep_exit(int64_t sleep_time_us, void *arg)
{
    if (s_fix_on && sleep_time_us >= COLD_MIN_SLEEP_US) {
        s_cold_period = __real_rtc_clk_cal(CLK_CAL_RTC_SLOW, SLEEP_PATH_CAL_CYCLES);
    }
    return ESP_OK;
}

static void set_fix(bool on)
{
    // A reading from before an earlier stretch with the fix off may be long stale.
    s_cold_period = 0;
    s_fix_on = on;
}
#endif

static const char *mode_name(void)
{
#if CONFIG_PM_LIGHTSLEEP_RTC_OSC_CAL_AFTER_WAKEUP
    return "fix-idf";
#elif CONFIG_REPRO_EMULATE_CAL_AFTER_WAKEUP
    return s_fix_on ? "fix-emul" : "stock";
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
    if (temperature_sensor_install(&config, &tsens) != ESP_OK) {
        return NAN;
    }
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
    while (esp_timer_get_time() < end) {
    }
}

void app_main(void)
{
    const esp_pm_config_t pm_config = {
        .max_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
        .min_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
        .light_sleep_enable = true,
    };
    ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
    // The driver logs its measuring range on every install, and this installs on every reading.
    esp_log_level_set("temperature_sensor", ESP_LOG_WARN);

#if CONFIG_REPRO_EMULATE_CAL_AFTER_WAKEUP
    esp_pm_sleep_cbs_register_config_t cbs_conf = {
        .exit_cb = on_light_sleep_exit,
    };
    ESP_ERROR_CHECK(esp_pm_light_sleep_register_cbs(&cbs_conf));
#if CONFIG_REPRO_START_WITH_FIX
    set_fix(true);
#endif
    const int64_t switch_period_us = (int64_t)CONFIG_REPRO_MODE_SWITCH_PERIOD_S * 1000000;
    int64_t next_switch_us = esp_timer_get_time() + switch_period_us;
#endif

    printf("REPRO idf=%s wake_ms=%d busy_ms=%d report_every=%d switch_s=%d\n",
           esp_get_idf_version(), CONFIG_REPRO_WAKE_PERIOD_MS, CONFIG_REPRO_BUSY_MS, CONFIG_REPRO_REPORT_EVERY,
#if CONFIG_REPRO_EMULATE_CAL_AFTER_WAKEUP
           CONFIG_REPRO_MODE_SWITCH_PERIOD_S
#else
           0
#endif
          );

    for (uint32_t n = 1;; n++) {
        vTaskDelay(pdMS_TO_TICKS(CONFIG_REPRO_WAKE_PERIOD_MS));
        const bool report = (n % CONFIG_REPRO_REPORT_EVERY == 0);
        const float die_cold = report ? die_temp_read() : NAN;
        busy_for_ms(CONFIG_REPRO_BUSY_MS);
        if (report) {
            const float die_warm = die_temp_read();
            printf("CLK n=%" PRIu32 " t_us=%" PRId64 " mode=%s cal=%" PRIu32 " die_cold=%.2f die_warm=%.2f\n",
                   n, esp_timer_get_time(), mode_name(), esp_clk_slowclk_cal_get(), die_cold, die_warm);
        }
#if CONFIG_REPRO_EMULATE_CAL_AFTER_WAKEUP
        if (switch_period_us != 0 && esp_timer_get_time() >= next_switch_us) {
            set_fix(!s_fix_on);
            next_switch_us += switch_period_us;
        }
#endif
    }
}
