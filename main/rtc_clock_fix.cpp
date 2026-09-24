#include "rtc_clock_fix.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <optional>

#include "esp_attr.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "esp_private/esp_clk.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/rtc.h"

#include "die_temp.h"

static const char *TAG = "rtc-fix";

// RTC_CLK_SRC_CAL_CYCLES in esp_hw_support/sleep_modes.c: the light-sleep path's rtc_clk_cal()
// call is the only one with this length, so it's the one the wrap takes over.
static constexpr uint32_t SLEEP_PATH_CAL_CYCLES = 10;
// Same length as ESP-IDF's, so the cost stays the same (~80 us with interrupts off); only the
// moment moves.
static constexpr uint32_t COLD_CAL_CYCLES = 10;
// Only a sleep this long counts as having let the die cool from whatever came before it.
static constexpr int64_t COLD_MIN_SLEEP_US = 50 * 1000;
// A cold value this far from the mean is dropped as a glitch: ~13 C of RC drift between wakes.
static constexpr uint64_t COLD_MAX_JUMP_PPM = 20000;
// The die can't move that far between two wakes, so this many in a row means it's the mean that
// is stale (a long stretch awake, say): the ring starts over from the next one instead.
static constexpr uint32_t COLD_MAX_REJECT_RUN = 8;

static constexpr uint32_t DIE_TEMP_SAMPLES = 16;

extern "C" uint32_t __real_rtc_clk_cal(soc_clk_freq_calculation_src_t cal_clk_sel,
                                       uint32_t slow_clk_cycles);
extern "C" uint32_t __wrap_rtc_clk_cal(soc_clk_freq_calculation_src_t cal_clk_sel,
                                       uint32_t slow_clk_cycles);

// Written by the exit callback and by rtc_clock_fix_set_samples(), read by the wrap: one core, and
// all three with interrupts off (the setter through s_mux), so never at the same time. The status
// task reads them under s_mux too.
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_samples = RTC_CAL_SAMPLES_DEFAULT;
static uint32_t s_ring[RTC_CAL_SAMPLES_MAX];
// Accepted since the ring last (re)started; the newest is at (s_count - 1) % RTC_CAL_SAMPLES_MAX.
static uint32_t s_count = 0;
static uint32_t s_last = 0;
// Of the newest min(s_count, s_samples).
static uint32_t s_mean = 0;
static uint32_t s_reject_run = 0;
// Since boot.
static uint32_t s_taken = 0;
static uint32_t s_rejected = 0;
static uint32_t s_restarts = 0;
// Sleeps timed with the mean and with ESP-IDF's own calibration, since the last status line.
static uint32_t s_sleeps_cold = 0;
static uint32_t s_sleeps_measured = 0;

static std::atomic<uint32_t> s_period_sec{RTC_CAL_PERIOD_SEC_DEFAULT};
static TaskHandle_t s_status_task = nullptr;

static double period_khz(uint32_t period)
{
    return period != 0 ? 1e3 * (1 << RTC_CLK_CAL_FRACT) / period : 0.0;
}

// rtc_clk_cal() lives in IRAM and the sleep path calls it with interrupts off, so the wrap has to
// be IRAM too, as does everything it touches. Every other caller passes straight through.
IRAM_ATTR uint32_t __wrap_rtc_clk_cal(soc_clk_freq_calculation_src_t cal_clk_sel, uint32_t slow_clk_cycles)
{
    if (cal_clk_sel != CLK_CAL_RTC_SLOW || slow_clk_cycles != SLEEP_PATH_CAL_CYCLES)
        return __real_rtc_clk_cal(cal_clk_sel, slow_clk_cycles);

    // ESP-IDF's own until the ring holds as many as it averages -- the first few sleeps of a boot.
    if (s_samples != 0 && s_count >= s_samples) {
        s_sleeps_cold++;
        return s_mean;
    }
    s_sleeps_measured++;
    return __real_rtc_clk_cal(cal_clk_sel, slow_clk_cycles);
}

// Under s_mux. Summed afresh each time rather than kept as a running sum, so a new s_samples
// needs nothing special -- at most 32 adds per wake.
static void update_mean()
{
    const uint32_t n = std::min(s_count, s_samples);
    uint64_t sum = 0;
    for (uint32_t i = 1; i <= n; i++)
        sum += s_ring[(s_count - i) % RTC_CAL_SAMPLES_MAX];
    s_mean = n != 0 ? static_cast<uint32_t>((sum + n / 2) / n) : 0;
}

// Under s_mux.
static void accept_cold(uint32_t period)
{
    if (s_count >= s_samples) {
        const uint32_t diff = period > s_mean ? period - s_mean : s_mean - period;
        if (uint64_t{diff} * 1000000 > uint64_t{s_mean} * COLD_MAX_JUMP_PPM) {
            if (++s_reject_run < COLD_MAX_REJECT_RUN) {
                s_rejected++;
                return;
            }
            s_count = 0;
            s_restarts++;
        }
    }
    s_reject_run = 0;
    s_ring[s_count % RTC_CAL_SAMPLES_MAX] = period;
    s_count++;
    s_taken++;
    s_last = period;
    update_mean();
}

// Runs from the IDLE task after every light-sleep attempt, inside the PM critical section and
// before any other task (or OpenThread's radio) runs -- no blocking. Registered first, so the
// calibration is the first thing after the wake.
static esp_err_t on_light_sleep_exit(int64_t sleep_time_us, void *)
{
    if (s_samples == 0 || sleep_time_us < COLD_MIN_SLEEP_US)
        return ESP_OK;
    const uint32_t cold = __real_rtc_clk_cal(CLK_CAL_RTC_SLOW, COLD_CAL_CYCLES);
    if (cold == 0)
        return ESP_OK;
    portENTER_CRITICAL_SAFE(&s_mux);
    accept_cold(cold);
    portEXIT_CRITICAL_SAFE(&s_mux);
    return ESP_OK;
}

void rtc_clock_fix_set_samples(uint32_t samples)
{
    samples = rtc_cal_clamp_samples(samples);
    portENTER_CRITICAL(&s_mux);
    const uint32_t was = s_samples;
    s_samples = samples;
    // Nothing was taken while it was off, so whatever the ring still holds may be hours old.
    if (was == 0)
        s_count = 0;
    s_reject_run = 0;
    update_mean();
    portEXIT_CRITICAL(&s_mux);
    if (samples == was)
        return;
    if (samples == 0)
        ESP_LOGW(TAG, "sleep path calibration now ESP-IDF's own");
    else
        ESP_LOGW(TAG, "sleep path calibration now the mean of the last %lu cold ones",
                 static_cast<unsigned long>(samples));
}

uint32_t rtc_clock_fix_samples()
{
    portENTER_CRITICAL(&s_mux);
    const uint32_t samples = s_samples;
    portEXIT_CRITICAL(&s_mux);
    return samples;
}

void rtc_clock_fix_set_period_sec(uint32_t period_sec)
{
    period_sec = rtc_cal_clamp_period_sec(period_sec);
    if (s_period_sec.exchange(period_sec) == period_sec)
        return;
    if (period_sec == 0)
        ESP_LOGW(TAG, "status off");
    else
        ESP_LOGW(TAG, "status every %lu s", static_cast<unsigned long>(period_sec));
    if (s_status_task != nullptr)
        xTaskNotifyGive(s_status_task);
}

uint32_t rtc_clock_fix_period_sec()
{
    return s_period_sec.load();
}

static void log_status()
{
    const std::optional<DieTemp> die = die_temp_read(DIE_TEMP_SAMPLES);

    portENTER_CRITICAL(&s_mux);
    const uint32_t samples = s_samples;
    const uint32_t last = s_last;
    const uint32_t mean = s_mean;
    const uint32_t taken = s_taken;
    const uint32_t rejected = s_rejected;
    const uint32_t restarts = s_restarts;
    const uint32_t sleeps_cold = s_sleeps_cold;
    const uint32_t sleeps_measured = s_sleeps_measured;
    s_sleeps_cold = 0;
    s_sleeps_measured = 0;
    portEXIT_CRITICAL(&s_mux);
    // What the last sleep path calibration left behind for everything else timed off RTC_SLOW.
    const uint32_t stored = esp_clk_slowclk_cal_get();

    if (samples == 0) {
        ESP_LOGW(TAG, "status: die %.1f C (raw %.1f) | cold calibration off | %lu sleeps measured | stored %.3f kHz",
                 die ? die->celsius : NAN, die ? die->raw : NAN, static_cast<unsigned long>(sleeps_measured),
                 period_khz(stored));
        return;
    }
    ESP_LOGW(TAG, "status: die %.1f C (raw %.1f) | cold last %.3f kHz, mean of %lu %.3f kHz (%+.0f ppm), "
                  "%lu taken, %lu rejected, %lu restarts | %lu sleeps cold, %lu measured | stored %.3f kHz",
             die ? die->celsius : NAN, die ? die->raw : NAN, period_khz(last), static_cast<unsigned long>(samples),
             period_khz(mean), mean != 0 ? (static_cast<double>(last) / mean - 1.0) * 1e6 : 0.0,
             static_cast<unsigned long>(taken), static_cast<unsigned long>(rejected),
             static_cast<unsigned long>(restarts), static_cast<unsigned long>(sleeps_cold),
             static_cast<unsigned long>(sleeps_measured), period_khz(stored));
}

// Every s_period_sec: the die temperature, what the cold calibrations say and what timed the
// sleeps. Woken by its own timer, so the die reading is taken right after a wake too -- and that
// wake went through the exit callback's calibration like any other.
static void status_task(void *)
{
    while (true) {
        const uint32_t period_sec = s_period_sec.load();
        // Off: wait for rtc_clock_fix_set_period_sec() to say otherwise.
        if (period_sec == 0) {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            continue;
        }
        // In seconds' worth of ticks: pdMS_TO_TICKS() multiplies in 32 bits, and an hour in ms
        // times the 1 kHz tick rate is too close to overflowing it. A period change cuts the wait
        // short and starts a new one.
        if (ulTaskNotifyTake(pdTRUE, period_sec * pdMS_TO_TICKS(1000)) != 0)
            continue;
        log_status();
    }
}

esp_err_t rtc_clock_fix_start(uint32_t samples, uint32_t period_sec)
{
    s_samples = rtc_cal_clamp_samples(samples);
    s_period_sec.store(rtc_cal_clamp_period_sec(period_sec));

    esp_pm_sleep_cbs_register_config_t cbs_conf = {
        .enter_cb = nullptr,
        .exit_cb = on_light_sleep_exit,
        .enter_cb_user_arg = nullptr,
        .exit_cb_user_arg = nullptr,
        .enter_cb_prior = 0,
        .exit_cb_prior = 0,
    };
    if (const esp_err_t err = esp_pm_light_sleep_register_cbs(&cbs_conf); err != ESP_OK)
        return err;
    // Lowest priority above idle: its only work is a couple of ms of die-temperature reading.
    if (xTaskCreate(status_task, "rtc_fix", 4096, nullptr, tskIDLE_PRIORITY + 1, &s_status_task) != pdPASS)
        return ESP_ERR_NO_MEM;

    if (s_samples == 0)
        ESP_LOGI(TAG, "sleep path calibration ESP-IDF's own, status every %lu s",
                 static_cast<unsigned long>(s_period_sec.load()));
    else
        ESP_LOGI(TAG, "sleep path calibration the mean of the last %lu cold ones, status every %lu s",
                 static_cast<unsigned long>(s_samples), static_cast<unsigned long>(s_period_sec.load()));
    return ESP_OK;
}
