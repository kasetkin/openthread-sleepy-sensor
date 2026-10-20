/*
 * Reproducer: with automatic light sleep and the internal RC as RTC_SLOW, ESP-IDF calibrates
 * RTC_SLOW at sleep entry, while the die is still warm from the active phase. The RC speeds up
 * as the die cools during the sleep, so the sleep is converted with a period that is too long and
 * esp_timer / the RTC time run fast.
 *
 * The app sleeps REPRO_WAKE_PERIOD_MS, then stays active for exactly REPRO_BUSY_MS counted from
 * the wakeup: every REPRO_REPORT_EVERY wakes it first reads the die temperature and prints a
 * line, then busy-loops until the end of that window to warm the die. The logging is part of the
 * window, so it never changes the active time.
 *
 * Boot banner, once:
 *     REPRO idf=<version> mode=<stock|fix-idf-<MIN_SLP_MS>> wake_ms=.. busy_ms=.. report_every=..
 * Report line, kept short (about 5 ms at 115200 baud):
 *     C n=.. t=.. c=.. d=.. r=.. s=.. k=.. o=..
 *       n  wakes since boot
 *       t  esp_timer_get_time(), us
 *       c  RTC_SLOW period used for the last light sleep, Q19 us (esp_clk_slowclk_cal_get())
 *       d  die temperature right after the wake, 0.1 C (left out if the sensor fails)
 *       r  requested sleep, ms
 *       s  time spent in light sleep since the previous line, us
 *       k  light sleeps since the previous line
 *       o  windows so far where the work ran past REPRO_BUSY_MS (left out while 0)
 *
 * clock_drift.py timestamps each line on arrival and fits esp_timer against the host clock.
 */

#include <stdbool.h>
#include <stdio.h>
#include <inttypes.h>
#include <math.h>                           // NAN, lroundf

#include "driver/temperature_sensor.h"      // read die temperature
#include "esp_attr.h"                       // IRAM_ATTR for the light sleep exit callback
#include "esp_log.h"                        // silence the temperature sensor driver
#include "esp_idf_version.h"                // for esp_get_idf_version() debug output
#include "esp_pm.h"                         // enable light sleep, light sleep exit callback
#include "esp_private/esp_clk.h"            // for esp_clk_slowclk_cal_get() debug output
#include "esp_sleep.h"                      // console UART handling in light sleep
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"                      // get config options for the mode name


#define STRINGIFY_(x) #x
#define STRINGIFY(x) STRINGIFY_(x)

#if CONFIG_PM_LIGHTSLEEP_RTC_OSC_CAL_AFTER_WAKEUP && defined(CONFIG_PM_LIGHTSLEEP_RTC_OSC_CAL_AFTER_WAKEUP_MIN_SLP_MS)
#define MODE_NAME "fix-idf-" STRINGIFY(CONFIG_PM_LIGHTSLEEP_RTC_OSC_CAL_AFTER_WAKEUP_MIN_SLP_MS)
#elif CONFIG_PM_LIGHTSLEEP_RTC_OSC_CAL_AFTER_WAKEUP
#define MODE_NAME "fix-idf"                 // an ESP-IDF revision without the threshold option
#else
#define MODE_NAME "stock"
#endif

// Readings averaged per die temperature value, and the gap between them: the sensor's output
// only moves once per conversion.
#define DIE_TEMP_SAMPLES        16
#define DIE_TEMP_SPACING_US     50

// Light sleeps since the previous report line and the time spent in them. Written by the light
// sleep exit callback, which runs in the idle task while the report task is blocked, so a write
// never overlaps a read.
static volatile int64_t s_slept_us = 0;
static volatile uint32_t s_sleeps = 0;

static IRAM_ATTR esp_err_t on_light_sleep_exit(int64_t slept_us, void *arg)
{
    // The PM also calls this when it skips a sleep that would be too short, with 0.
    if (slept_us > 0) {
        s_slept_us += slept_us;
        s_sleeps++;
    }
    return ESP_OK;
}

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

static void report(uint32_t n, uint32_t requested_ms, uint32_t overruns)
{
    // Snapshot first: anything below may block, and the idle task may then sleep again.
    const int64_t slept_us = s_slept_us;
    const uint32_t sleeps = s_sleeps;
    s_slept_us = 0;
    s_sleeps = 0;

    const float die = die_temp_read();
    char die_field[16] = "";
    if (!isnan(die)) {
        snprintf(die_field, sizeof(die_field), " d=%d", (int)lroundf(die * 10));
    }
    char overrun_field[16] = "";
    if (overruns > 0) {
        snprintf(overrun_field, sizeof(overrun_field), " o=%" PRIu32, overruns);
    }
    printf("C n=%" PRIu32 " t=%" PRId64 " c=%" PRIu32 "%s r=%" PRIu32 " s=%" PRId64 " k=%" PRIu32 "%s\n",
           n, esp_timer_get_time(), esp_clk_slowclk_cal_get(), die_field, requested_ms, slept_us, sleeps,
           overrun_field);
    fflush(stdout);
}

// Busy-loops the CPU until the deadline to warm the die, the way a radio burst or a sensor read
// would. False if the deadline had already passed.
static bool busy_until(int64_t deadline_us)
{
    if (esp_timer_get_time() > deadline_us)
        return false;
    volatile int32_t trash = 0;
    while (esp_timer_get_time() < deadline_us)
        trash = (trash + 239) % 1001;
    return true;
}

void app_main(void)
{
    const esp_pm_config_t pm_config = {
        .max_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
        .min_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
        .light_sleep_enable = true,
    };
    ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
    esp_pm_sleep_cbs_register_config_t sleep_cbs = {
        .exit_cb = on_light_sleep_exit,
    };
    ESP_ERROR_CHECK(esp_pm_light_sleep_register_cbs(&sleep_cbs));
    // By default light sleep suspends the console UART and sends the rest of its FIFO only after
    // the next wake, which would stamp a line one (random) sleep late on the host. The report is
    // printed at the start of the active window, so the FIFO is normally empty by then anyway.
    ESP_ERROR_CHECK(esp_sleep_set_console_uart_handling_mode(ESP_SLEEP_ALWAYS_FLUSH_UART));
    // The driver logs its measuring range on every install, and this installs on every reading.
    esp_log_level_set("temperature_sensor", ESP_LOG_WARN);

    printf("REPRO idf=%s mode=%s wake_ms=%d busy_ms=%d report_every=%d\n",
           esp_get_idf_version(), MODE_NAME, CONFIG_REPRO_WAKE_PERIOD_MS, CONFIG_REPRO_BUSY_MS,
           CONFIG_REPRO_REPORT_EVERY);

    uint32_t overruns = 0;
    for (uint32_t n = 1;; n++) {
        vTaskDelay(pdMS_TO_TICKS(CONFIG_REPRO_WAKE_PERIOD_MS));
        const int64_t wake_us = esp_timer_get_time();
        if (n % CONFIG_REPRO_REPORT_EVERY == 0) {
            report(n, CONFIG_REPRO_WAKE_PERIOD_MS, overruns);
        }
        if (!busy_until(wake_us + (int64_t)CONFIG_REPRO_BUSY_MS * 1000)) {
            overruns++;
        }
    }
}
