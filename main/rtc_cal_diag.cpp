#include "rtc_cal_diag.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "esp_attr.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "esp_private/esp_clk.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/rtc.h"

static const char *TAG = "rtc-cal";

// RTC_CLK_SRC_CAL_CYCLES in esp_hw_support/sleep_modes.c: what the light-sleep path passes to
// rtc_clk_cal() before every sleep. Not configurable.
static constexpr uint32_t SLEEP_PATH_CAL_CYCLES = 10;
static constexpr uint32_t REFERENCE_CAL_CYCLES = 4096;

static constexpr uint32_t FIRST_RUN_DELAY_MS = 60 * 1000;
static constexpr uint32_t RUN_PERIOD_MS = 30 * 60 * 1000;

extern "C" uint32_t __real_rtc_clk_cal(soc_clk_freq_calculation_src_t cal_clk_sel,
                                       uint32_t slow_clk_cycles);
extern "C" uint32_t __wrap_rtc_clk_cal(soc_clk_freq_calculation_src_t cal_clk_sel,
                                       uint32_t slow_clk_cycles);

// Written by the wrap below, which the sleep path calls with interrupts already off; read by the
// diagnostic task. The critical section keeps the pair consistent for the reader.
static portMUX_TYPE s_sleep_cal_mux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_sleep_cal_count = 0;
static uint64_t s_sleep_cal_sum = 0;

static esp_pm_lock_handle_t s_no_sleep_lock = nullptr;

// rtc_clk_cal() lives in IRAM (rtc_time: noflash_text) and is called from the sleep path, so the
// wrap has to be in IRAM too. Pass-through: it only records the sleep path's own 10-cycle
// results. The diagnostic's measurements call __real_rtc_clk_cal() directly and aren't counted.
IRAM_ATTR uint32_t __wrap_rtc_clk_cal(soc_clk_freq_calculation_src_t cal_clk_sel, uint32_t slow_clk_cycles)
{
    const uint32_t period = __real_rtc_clk_cal(cal_clk_sel, slow_clk_cycles);
    if (cal_clk_sel == CLK_CAL_RTC_SLOW && slow_clk_cycles == SLEEP_PATH_CAL_CYCLES && period != 0) {
        portENTER_CRITICAL_SAFE(&s_sleep_cal_mux);
        s_sleep_cal_count++;
        s_sleep_cal_sum += period;
        portEXIT_CRITICAL_SAFE(&s_sleep_cal_mux);
    }
    return period;
}

struct CalStats {
    uint32_t cycles;
    uint32_t count;
    uint32_t failed;
    double mean;
    double sd;
};

// Mean and standard deviation of `reps` calibrations over `cycles` RTC_SLOW cycles, in
// rtc_clk_cal()'s Q19 microseconds-per-cycle. Summed as offsets from the first value, so the
// variance doesn't drown in cancellation (values ~3.85e6, spread ~1e3).
static CalStats measure(uint32_t cycles, uint32_t reps)
{
    CalStats stats = {};
    stats.cycles = cycles;
    int64_t base = 0;
    int64_t sum = 0;
    int64_t sum_sq = 0;

    for (uint32_t i = 0; i < reps; i++) {
        const uint32_t period = __real_rtc_clk_cal(CLK_CAL_RTC_SLOW, cycles);
        if (period == 0) {
            stats.failed++;
            continue;
        }
        if (stats.count == 0)
            base = period;
        const int64_t offset = static_cast<int64_t>(period) - base;
        sum += offset;
        sum_sq += offset * offset;
        stats.count++;
    }

    if (stats.count > 0) {
        const double n = stats.count;
        const double mean_offset = sum / n;
        stats.mean = base + mean_offset;
        stats.sd = std::sqrt(std::max(0.0, sum_sq / n - mean_offset * mean_offset));
    }
    return stats;
}

static double ppm_vs(double value, double reference)
{
    return (value / reference - 1.0) * 1e6;
}

static void log_bias(const CalStats &stats, double reference)
{
    const double sd_ppm = stats.sd / reference * 1e6;
    const double se_ppm = stats.count > 0 ? sd_ppm / std::sqrt(static_cast<double>(stats.count)) : 0.0;
    ESP_LOGW(TAG, "N=%-4lu x%-3lu: %+7.0f +-%3.0f ppm vs N=%lu (sd %4.0f ppm, %lu failed)",
             static_cast<unsigned long>(stats.cycles), static_cast<unsigned long>(stats.count),
             ppm_vs(stats.mean, reference), se_ppm,
             static_cast<unsigned long>(REFERENCE_CAL_CYCLES), sd_ppm,
             static_cast<unsigned long>(stats.failed));
}

static void run_once()
{
    esp_pm_lock_acquire(s_no_sleep_lock);
    const CalStats ref_before = measure(REFERENCE_CAL_CYCLES, 4);
    const CalStats n10 = measure(SLEEP_PATH_CAL_CYCLES, 256);
    const CalStats n32 = measure(32, 128);
    const CalStats n100 = measure(100, 64);
    const CalStats n1024 = measure(1024, 16);
    const CalStats ref_after = measure(REFERENCE_CAL_CYCLES, 4);
    const uint32_t in_use = esp_clk_slowclk_cal_get();
    esp_pm_lock_release(s_no_sleep_lock);

    portENTER_CRITICAL(&s_sleep_cal_mux);
    const uint32_t sleep_count = s_sleep_cal_count;
    const uint64_t sleep_sum = s_sleep_cal_sum;
    s_sleep_cal_count = 0;
    s_sleep_cal_sum = 0;
    portEXIT_CRITICAL(&s_sleep_cal_mux);

    if (ref_before.count == 0 || ref_after.count == 0) {
        ESP_LOGW(TAG, "reference calibration failed (%lu/%lu), skipping this run",
                 static_cast<unsigned long>(ref_before.failed), static_cast<unsigned long>(ref_after.failed));
        return;
    }
    const double reference = (ref_before.mean + ref_after.mean) / 2.0;
    const double khz = 1e6 * (1 << RTC_CLK_CAL_FRACT) / reference / 1000.0;

    ESP_LOGW(TAG, "RTC_SLOW %.3f kHz by N=%lu (changed %+.0f ppm during the run)", khz,
             static_cast<unsigned long>(REFERENCE_CAL_CYCLES), ppm_vs(ref_after.mean, ref_before.mean));
    log_bias(n10, reference);
    log_bias(n32, reference);
    log_bias(n100, reference);
    log_bias(n1024, reference);
    if (sleep_count > 0)
        ESP_LOGW(TAG, "sleep path N=%lu: %lu calibrations since last run, mean %+.0f ppm vs N=%lu "
                      "| in use now %+.0f ppm",
                 static_cast<unsigned long>(SLEEP_PATH_CAL_CYCLES), static_cast<unsigned long>(sleep_count),
                 ppm_vs(static_cast<double>(sleep_sum) / sleep_count, reference),
                 static_cast<unsigned long>(REFERENCE_CAL_CYCLES), ppm_vs(in_use, reference));
}

static void rtc_cal_diag_task(void *)
{
    vTaskDelay(pdMS_TO_TICKS(FIRST_RUN_DELAY_MS));
    while (true) {
        run_once();
        vTaskDelay(pdMS_TO_TICKS(RUN_PERIOD_MS));
    }
}

esp_err_t rtc_cal_diag_start()
{
    esp_err_t err = esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "rtc_cal_diag", &s_no_sleep_lock);
    if (err != ESP_OK)
        return err;
    // Lowest priority above idle: the measurement busy-waits (~0.5 s per run) and must never hold
    // up OpenThread or the sensor/MQTT tasks; being preempted only stretches it.
    if (xTaskCreate(rtc_cal_diag_task, "rtc_cal_diag", 4096, nullptr, tskIDLE_PRIORITY + 1, nullptr) != pdPASS)
        return ESP_ERR_NO_MEM;
    return ESP_OK;
}
