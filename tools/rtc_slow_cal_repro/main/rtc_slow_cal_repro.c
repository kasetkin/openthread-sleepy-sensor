/*
 * Reproducer: with automatic light sleep and the internal RC as RTC_SLOW, ESP-IDF calibrates
 * RTC_SLOW at sleep entry, while the die is still warm from the active phase. The RC speeds up
 * as the die cools during the sleep, so the sleep is converted with a period that is too long and
 * esp_timer / the RTC time run fast.
 *
 * The app sleeps a random time, log-uniform in [REPRO_SLEEP_MIN_MS, REPRO_SLEEP_MAX_MS] from a fixed
 * seed (or always REPRO_WAKE_PERIOD_MS), then stays active for a window counted from the wakeup,
 * its length taken in turn from k_busy_ms below: every REPRO_REPORT_EVERY wakes it first reads the
 * die temperature and prints a line, then busy-loops until the end of that window to warm the die.
 * The logging is part of the window, so it never changes the active time.
 *
 * Besides ESP-IDF's own calibration, the app takes two RTC_SLOW readings of its own around every
 * light sleep, the same way ESP-IDF would: one just before the sleep (the stock entry reading) and
 * one right after it (the post-wake reading). Both are logged, so the clock error of any
 * calibration policy can be computed offline from one run (clock_drift.py --replay).
 *
 * Boot banner, once:
 *     REPRO idf=<version> mode=<stock|fix-idf-<MIN_SLP_MS>> sleep_ms=<min>..<max> seed=<hex>
 *           busy_ms=<a,b,..> report_every=..           (sleep_ms=<period>, no seed, if fixed)
 * Report line, kept short (about 7.6 ms at 115200 baud, within the shortest window):
 *     C n=.. t=.. c=.. d=.. r=.. s=.. k=.. e=.. w=.. b=.. o=..
 *       n  wakes since boot
 *       t  esp_timer_get_time(), us
 *       c  RTC_SLOW period used for the last light sleep, Q19 us (esp_clk_slowclk_cal_get())
 *       d  die temperature right after the wake, 0.1 C (left out if the sensor fails)
 *       r  requested duration of the sleep before this wake, ms
 *       s  time spent in light sleep since the previous line, us
 *       k  light sleeps since the previous line
 *       e  the app's RTC_SLOW reading just before the last light sleep, Q19 us
 *       w  the app's RTC_SLOW reading right after the last light sleep, Q19 us
 *       b  length of the active window that starts with this wake, ms
 *       o  windows so far where the work ran past their length (left out while 0)
 *
 * clock_drift.py timestamps each line on arrival and fits esp_timer against the host clock.
 */

#include <stdbool.h>
#include <stdio.h>
#include <string.h>                         // strlen for the banner
#include <inttypes.h>
#include <math.h>                           // NAN, lroundf, lround, pow

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
#include "soc/rtc.h"                        // rtc_clk_cal() for the app's own readings


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

// Active window after each wake, ms, used in turn. {200} is the MIN_SLP_MS sweep of 2026-10-20.
static const uint16_t k_busy_ms[] = {10, 10, 200, 200};
#define BUSY_PATTERN_LEN        (sizeof(k_busy_ms) / sizeof(k_busy_ms[0]))

// RC cycles per RTC_SLOW reading: the same as ESP-IDF's RTC_CLK_SRC_CAL_CYCLES in sleep_modes.c, so
// the app's readings have the same noise as the ones ESP-IDF times the sleeps with.
#define CAL_CYCLES              10

#if CONFIG_REPRO_SLEEP_LOG_UNIFORM
// xorshift32: the same sequence on every boot and with every ESP-IDF, unlike esp_random()
static uint32_t s_rng_state = CONFIG_REPRO_SEED;

static uint32_t next_random(void)
{
    s_rng_state ^= s_rng_state << 13;
    s_rng_state ^= s_rng_state >> 17;
    s_rng_state ^= s_rng_state << 5;
    return s_rng_state;
}
#endif

// Requested duration of the next sleep, ms.
static uint32_t next_sleep_ms(void)
{
#if CONFIG_REPRO_SLEEP_LOG_UNIFORM
    const double u = next_random() / 4294967296.0;     // [0, 1)
    const double ratio = (double)CONFIG_REPRO_SLEEP_MAX_MS / CONFIG_REPRO_SLEEP_MIN_MS;
    return (uint32_t)lround(CONFIG_REPRO_SLEEP_MIN_MS * pow(ratio, u));
#else
    return CONFIG_REPRO_WAKE_PERIOD_MS;
#endif
}

// Light sleeps since the previous report line, the time spent in them, and the app's readings
// around the last one. Written by the light sleep callbacks, which run in the idle task while the
// report task is blocked, so a write never overlaps a read.
static volatile int64_t s_slept_us = 0;
static volatile uint32_t s_sleeps = 0;
static volatile uint32_t s_entry_cal = 0;
static volatile uint32_t s_wake_cal = 0;
static uint32_t s_entry_cal_pending = 0;

// Runs just before esp_light_sleep_start(), so right before ESP-IDF's own entry reading.
static IRAM_ATTR esp_err_t on_light_sleep_enter(int64_t sleep_time_us, void *arg)
{
    s_entry_cal_pending = rtc_clk_cal(CLK_CAL_RTC_SLOW, CAL_CYCLES);
    return ESP_OK;
}

// Runs right after esp_light_sleep_start() returns, so right after ESP-IDF's post-wake reading.
static IRAM_ATTR esp_err_t on_light_sleep_exit(int64_t slept_us, void *arg)
{
    // The PM also calls both callbacks when it skips a sleep that would be too short, with 0 here.
    if (slept_us > 0) {
        s_slept_us += slept_us;
        s_sleeps++;
        s_entry_cal = s_entry_cal_pending;
        s_wake_cal = rtc_clk_cal(CLK_CAL_RTC_SLOW, CAL_CYCLES);
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

static void report(uint32_t n, uint32_t requested_ms, uint32_t busy_ms, uint32_t overruns)
{
    // Snapshot first: anything below may block, and the idle task may then sleep again.
    const int64_t slept_us = s_slept_us;
    const uint32_t sleeps = s_sleeps;
    const uint32_t entry_cal = s_entry_cal;
    const uint32_t wake_cal = s_wake_cal;
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
    printf("C n=%" PRIu32 " t=%" PRId64 " c=%" PRIu32 "%s r=%" PRIu32 " s=%" PRId64 " k=%" PRIu32
           " e=%" PRIu32 " w=%" PRIu32 " b=%" PRIu32 "%s\n",
           n, esp_timer_get_time(), esp_clk_slowclk_cal_get(), die_field, requested_ms, slept_us, sleeps,
           entry_cal, wake_cal, busy_ms, overrun_field);
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
        .enter_cb = on_light_sleep_enter,
        .exit_cb = on_light_sleep_exit,
    };
    ESP_ERROR_CHECK(esp_pm_light_sleep_register_cbs(&sleep_cbs));
    // By default light sleep suspends the console UART and sends the rest of its FIFO only after
    // the next wake, which would stamp a line one (random) sleep late on the host. The report is
    // printed at the start of the active window, so the FIFO is normally empty by then anyway.
    ESP_ERROR_CHECK(esp_sleep_set_console_uart_handling_mode(ESP_SLEEP_ALWAYS_FLUSH_UART));
    // The driver logs its measuring range on every install, and this installs on every reading.
    esp_log_level_set("temperature_sensor", ESP_LOG_WARN);

    char busy_list[64] = "";
    for (size_t i = 0; i < BUSY_PATTERN_LEN; i++) {
        const size_t used = strlen(busy_list);
        snprintf(busy_list + used, sizeof(busy_list) - used, "%s%u", i ? "," : "", k_busy_ms[i]);
    }
#if CONFIG_REPRO_SLEEP_LOG_UNIFORM
    printf("REPRO idf=%s mode=%s sleep_ms=%d..%d seed=0x%x busy_ms=%s report_every=%d\n",
           esp_get_idf_version(), MODE_NAME, CONFIG_REPRO_SLEEP_MIN_MS, CONFIG_REPRO_SLEEP_MAX_MS,
           CONFIG_REPRO_SEED, busy_list, CONFIG_REPRO_REPORT_EVERY);
#else
    printf("REPRO idf=%s mode=%s sleep_ms=%d busy_ms=%s report_every=%d\n",
           esp_get_idf_version(), MODE_NAME, CONFIG_REPRO_WAKE_PERIOD_MS, busy_list,
           CONFIG_REPRO_REPORT_EVERY);
#endif

    uint32_t overruns = 0;
    for (uint32_t n = 1;; n++) {
        const uint32_t sleep_ms = next_sleep_ms();
        vTaskDelay(pdMS_TO_TICKS(sleep_ms));
        const int64_t wake_us = esp_timer_get_time();
        const uint32_t busy_ms = k_busy_ms[(n - 1) % BUSY_PATTERN_LEN];
        if (n % CONFIG_REPRO_REPORT_EVERY == 0) {
            report(n, sleep_ms, busy_ms, overruns);
        }
        if (!busy_until(wake_us + (int64_t)busy_ms * 1000)) {
            overruns++;
        }
    }
}
