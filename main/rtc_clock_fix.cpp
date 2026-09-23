#include "rtc_clock_fix.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <optional>

#include "esp_attr.h"
#include "esp_log.h"
#include "esp_pm.h"
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
// Averaging beyond this buys nothing the clock can feel and only adds lag behind a drifting die
// (see rtc_clock_fix.h). Until the ring is full the sleep path keeps ESP-IDF's own calibration,
// which on the 2026-09-22 capture was 13 sleeps out of 144150, all in the first seconds of a boot.
static constexpr uint32_t COLD_RING_SIZE = 8;
// A cold value this far from the mean is dropped as a glitch: ~10 C of RC drift between wakes.
static constexpr uint64_t COLD_MAX_JUMP_PPM = 20000;

static constexpr uint32_t STATUS_PERIOD_MS = 5 * 60 * 1000;
static constexpr uint32_t DIE_TEMP_SAMPLES = 16;

extern "C" uint32_t __real_rtc_clk_cal(soc_clk_freq_calculation_src_t cal_clk_sel,
                                       uint32_t slow_clk_cycles);
extern "C" uint32_t __wrap_rtc_clk_cal(soc_clk_freq_calculation_src_t cal_clk_sel,
                                       uint32_t slow_clk_cycles);

// Read by the wrap in the sleep path, written from the MQTT task.
static std::atomic<uint32_t> s_cal_mode{static_cast<uint32_t>(RtcCalMode::ColdMean)};

static RtcSleepCalObserver s_observer = nullptr;
static RtcColdCalObserver s_cold_observer = nullptr;

// Written by the exit callback and read by the wrap -- same core, both with interrupts off, so
// never at the same time -- and read by the status task under s_mux.
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_cold_ring[COLD_RING_SIZE];
// Accepted since boot.
static uint32_t s_cold_count = 0;
// Of the ring's current contents.
static uint64_t s_cold_sum = 0;
static uint32_t s_cold_last = 0;
static uint32_t s_cold_mean = 0;
static uint32_t s_cold_rejected = 0;
// Sleeps timed with a cold value and with ESP-IDF's own calibration, since the last status line.
static uint32_t s_sleeps_cold = 0;
static uint32_t s_sleeps_measured = 0;

static const char *cal_mode_name(uint32_t mode)
{
    switch (static_cast<RtcCalMode>(mode)) {
        case RtcCalMode::EspIdf: return "esp-idf";
        case RtcCalMode::ColdLast: return "cold-last";
        case RtcCalMode::ColdMean: return "cold-mean";
    }
    return "?";
}

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

    const auto mode = static_cast<RtcCalMode>(s_cal_mode.load(std::memory_order_relaxed));
    const RtcSleepCalObserver observer = s_observer;
    uint32_t measured = 0;
    uint32_t period = 0;
    const bool cold = mode != RtcCalMode::EspIdf && s_cold_count >= COLD_RING_SIZE;
    if (cold) {
        period = mode == RtcCalMode::ColdLast ? s_cold_last : s_cold_mean;
        // Nothing needs it, so it is only worth its ~80 us while the diagnostic is watching.
        if (observer != nullptr)
            measured = __real_rtc_clk_cal(cal_clk_sel, slow_clk_cycles);
        s_sleeps_cold++;
    } else {
        measured = __real_rtc_clk_cal(cal_clk_sel, slow_clk_cycles);
        period = measured;
        s_sleeps_measured++;
    }
    if (observer != nullptr && period != 0)
        observer(period, measured, cold);
    return period;
}

// Under s_mux.
static void accept_cold(uint32_t period)
{
    if (s_cold_count >= COLD_RING_SIZE) {
        const uint32_t diff = period > s_cold_mean ? period - s_cold_mean : s_cold_mean - period;
        if (uint64_t{diff} * 1000000 > uint64_t{s_cold_mean} * COLD_MAX_JUMP_PPM) {
            s_cold_rejected++;
            return;
        }
    }
    const uint32_t slot = s_cold_count % COLD_RING_SIZE;
    if (s_cold_count >= COLD_RING_SIZE)
        s_cold_sum -= s_cold_ring[slot];
    s_cold_ring[slot] = period;
    s_cold_sum += period;
    s_cold_count++;
    s_cold_last = period;
    const uint32_t n = std::min(s_cold_count, COLD_RING_SIZE);
    s_cold_mean = static_cast<uint32_t>((s_cold_sum + n / 2) / n);
}

// Runs from the IDLE task after every light-sleep attempt, inside the PM critical section and
// before any other task (or OpenThread's radio) runs -- no blocking. Registered first, so the
// calibration is the first thing after the wake.
static esp_err_t on_light_sleep_exit(int64_t sleep_time_us, void *)
{
    if (sleep_time_us <= 0)
        return ESP_OK;
    const RtcColdCalObserver observer = s_cold_observer;
    const bool take = sleep_time_us >= COLD_MIN_SLEEP_US;
    if (!take && observer == nullptr)
        return ESP_OK;
    const uint32_t cold = __real_rtc_clk_cal(CLK_CAL_RTC_SLOW, COLD_CAL_CYCLES);
    if (cold == 0)
        return ESP_OK;
    if (take) {
        portENTER_CRITICAL_SAFE(&s_mux);
        accept_cold(cold);
        portEXIT_CRITICAL_SAFE(&s_mux);
    }
    if (observer != nullptr)
        observer(sleep_time_us, cold, take);
    return ESP_OK;
}

void rtc_clock_fix_set_cal_mode(uint32_t mode)
{
    mode = std::min<uint32_t>(mode, static_cast<uint32_t>(RtcCalMode::ColdMean));
    if (s_cal_mode.exchange(mode) != mode)
        ESP_LOGW(TAG, "sleep path calibration now %s", cal_mode_name(mode));
}

uint32_t rtc_clock_fix_cal_mode()
{
    return s_cal_mode.load();
}

void rtc_clock_fix_set_sleep_cal_observer(RtcSleepCalObserver observer)
{
    s_observer = observer;
}

void rtc_clock_fix_set_cold_cal_observer(RtcColdCalObserver observer)
{
    s_cold_observer = observer;
}

// Every STATUS_PERIOD_MS: the die temperature, what the cold calibrations say and what timed the
// sleeps. Woken by its own timer, so the die reading is taken right after a wake too.
static void status_task(void *)
{
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(STATUS_PERIOD_MS));
        const std::optional<DieTemp> die = die_temp_read(DIE_TEMP_SAMPLES);

        portENTER_CRITICAL(&s_mux);
        const uint32_t cold_count = s_cold_count;
        const uint32_t cold_last = s_cold_last;
        const uint32_t cold_mean = s_cold_mean;
        const uint32_t cold_rejected = s_cold_rejected;
        const uint32_t sleeps_cold = s_sleeps_cold;
        const uint32_t sleeps_measured = s_sleeps_measured;
        s_sleeps_cold = 0;
        s_sleeps_measured = 0;
        portEXIT_CRITICAL(&s_mux);

        ESP_LOGW(TAG, "status: die %.1f C (raw %.1f) | cold last %.3f kHz, mean %.3f kHz (%+.0f ppm), "
                      "%lu taken, %lu rejected | %lu sleeps cold, %lu measured | calibration %s",
                 die ? die->celsius : NAN, die ? die->raw : NAN, period_khz(cold_last), period_khz(cold_mean),
                 cold_mean != 0 ? (static_cast<double>(cold_last) / cold_mean - 1.0) * 1e6 : 0.0,
                 static_cast<unsigned long>(cold_count), static_cast<unsigned long>(cold_rejected),
                 static_cast<unsigned long>(sleeps_cold), static_cast<unsigned long>(sleeps_measured),
                 cal_mode_name(s_cal_mode.load()));
    }
}

esp_err_t rtc_clock_fix_start(uint32_t cal_mode)
{
    s_cal_mode.store(std::min<uint32_t>(cal_mode, static_cast<uint32_t>(RtcCalMode::ColdMean)));

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
    if (xTaskCreate(status_task, "rtc_fix", 4096, nullptr, tskIDLE_PRIORITY + 1, nullptr) != pdPASS)
        return ESP_ERR_NO_MEM;

    ESP_LOGI(TAG, "sleep path calibration %s", cal_mode_name(s_cal_mode.load()));
    return ESP_OK;
}
