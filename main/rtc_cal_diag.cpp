#include "rtc_cal_diag.h"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "esp_attr.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "esp_private/esp_clk.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/rtc.h"

static const char *TAG = "rtc-cal";

// RTC_CLK_SRC_CAL_CYCLES in esp_hw_support/sleep_modes.c: what the light-sleep path passes to
// rtc_clk_cal() before every sleep. Not configurable.
static constexpr uint32_t SLEEP_PATH_CAL_CYCLES = 10;
// Reference and timeline step: ~7 ms per calibration, where one XTAL count is ~3.5 ppm.
static constexpr uint32_t REFERENCE_CAL_CYCLES = 1024;

static constexpr uint32_t TIMELINE_MAX_SAMPLES = 160;
static constexpr int64_t TIMELINE_US = 1000 * 1000;
// The timeline's tail that "settled" means: its mean is what every ppm figure is relative to.
static constexpr uint32_t TIMELINE_SETTLED_SAMPLES = 16;
static constexpr uint32_t RAW_PER_LINE = 32;

static constexpr uint32_t BIAS_REPS = 48;
static constexpr uint32_t BIAS_CYCLES[] = {SLEEP_PATH_CAL_CYCLES, 32, 100};
static constexpr size_t BIAS_N = sizeof(BIAS_CYCLES) / sizeof(BIAS_CYCLES[0]);

// Power of two, so the ring index in the IRAM wrap is a mask. ~3 min of CSL-idle sleeps.
static constexpr uint32_t SLEEP_RING_SIZE = 512;

static constexpr uint32_t FIRST_RUN_DELAY_MS = 5 * 60 * 1000;
static constexpr uint32_t RUN_PERIOD_MS = 15 * 60 * 1000;

extern "C" uint32_t __real_rtc_clk_cal(soc_clk_freq_calculation_src_t cal_clk_sel,
                                       uint32_t slow_clk_cycles);
extern "C" uint32_t __wrap_rtc_clk_cal(soc_clk_freq_calculation_src_t cal_clk_sel,
                                       uint32_t slow_clk_cycles);

// One light sleep as the sleep path timed it: the wrap fills the calibration and how long the
// core had been awake before it, the exit callback then adds the sleep it was used for.
struct SleepCal {
    uint32_t period;
    uint32_t awake_us;
    uint32_t slept_us;
};

// Written by the wrap and the exit callback, both of which run with interrupts already off;
// read by the diagnostic task. The critical section keeps an entry consistent for the reader.
static portMUX_TYPE s_sleep_cal_mux = portMUX_INITIALIZER_UNLOCKED;
static SleepCal s_sleep_ring[SLEEP_RING_SIZE];
// Entries ever written; the newest one is at head - 1.
static uint32_t s_sleep_ring_head = 0;
// The newest entry's sleep hasn't been reported yet.
static bool s_sleep_pending = false;
static int64_t s_last_wake_us = 0;
static uint32_t s_sleep_cal_count = 0;

struct TimelineSample {
    uint32_t period;
    uint32_t t_us;
};

static TimelineSample s_timeline[TIMELINE_MAX_SAMPLES];

static esp_pm_lock_handle_t s_no_sleep_lock = nullptr;

static IRAM_ATTR uint32_t clamp_us(int64_t us)
{
    return us < 0 ? 0 : us > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(us);
}

// rtc_clk_cal() lives in IRAM (rtc_time: noflash_text) and is called from the sleep path, so the
// wrap has to be in IRAM too, as does everything it calls (esp_timer_get_time() is, with
// CONFIG_ESP_TIMER_IN_IRAM). Pass-through: it only records the sleep path's own 10-cycle results.
// The diagnostic's measurements call __real_rtc_clk_cal() directly and aren't recorded.
IRAM_ATTR uint32_t __wrap_rtc_clk_cal(soc_clk_freq_calculation_src_t cal_clk_sel, uint32_t slow_clk_cycles)
{
    const uint32_t period = __real_rtc_clk_cal(cal_clk_sel, slow_clk_cycles);
    if (cal_clk_sel == CLK_CAL_RTC_SLOW && slow_clk_cycles == SLEEP_PATH_CAL_CYCLES && period != 0) {
        const int64_t now = esp_timer_get_time();
        portENTER_CRITICAL_SAFE(&s_sleep_cal_mux);
        SleepCal &entry = s_sleep_ring[s_sleep_ring_head % SLEEP_RING_SIZE];
        entry.period = period;
        entry.awake_us = s_last_wake_us > 0 ? clamp_us(now - s_last_wake_us) : UINT32_MAX;
        entry.slept_us = 0;
        s_sleep_ring_head++;
        s_sleep_pending = true;
        s_sleep_cal_count++;
        portEXIT_CRITICAL_SAFE(&s_sleep_cal_mux);
    }
    return period;
}

// Runs from the IDLE task after every automatic light-sleep attempt, inside the PM critical
// section -- no blocking. sleep_time_us is what esp_timer says was slept, i.e. already scaled by
// the calibration the sleep path just made.
static esp_err_t on_light_sleep_exit(int64_t sleep_time_us, void *)
{
    if (sleep_time_us <= 0)
        return ESP_OK;
    const int64_t now = esp_timer_get_time();
    portENTER_CRITICAL_SAFE(&s_sleep_cal_mux);
    s_last_wake_us = now;
    if (s_sleep_pending) {
        s_sleep_ring[(s_sleep_ring_head - 1) % SLEEP_RING_SIZE].slept_us = clamp_us(sleep_time_us);
        s_sleep_pending = false;
    }
    portEXIT_CRITICAL_SAFE(&s_sleep_cal_mux);
    return ESP_OK;
}

static double ppm_vs(double value, double reference)
{
    return (value / reference - 1.0) * 1e6;
}

struct PpmStats {
    uint32_t count;
    uint32_t failed;
    double sum;
    double sum_sq;

    void add(double ppm)
    {
        count++;
        sum += ppm;
        sum_sq += ppm * ppm;
    }
    double mean() const { return count > 0 ? sum / count : 0.0; }
    double sd() const { return count > 0 ? std::sqrt(std::max(0.0, sum_sq / count - mean() * mean())) : 0.0; }
    double se() const { return count > 0 ? sd() / std::sqrt(static_cast<double>(count)) : 0.0; }
};

// Appends to a log line being built in `buf`, never past its end.
static void append(char *buf, size_t size, size_t *len, const char *fmt, ...) __attribute__((format(printf, 4, 5)));
static void append(char *buf, size_t size, size_t *len, const char *fmt, ...)
{
    if (*len >= size)
        return;
    va_list args;
    va_start(args, fmt);
    const int written = vsnprintf(buf + *len, size - *len, fmt, args);
    va_end(args);
    if (written > 0)
        *len = std::min(size, *len + static_cast<size_t>(written));
}

// How RTC_SLOW settles after a wake: back-to-back reference calibrations for a second, in
// log-spaced bins of samples (1, 1, 2, 4, 8, ...), then every sample.
static void log_timeline(uint32_t count, double settled, double awake_before_ms)
{
    char line[320];
    size_t len = 0;
    append(line, sizeof(line), &len, "after wake (ms since wake: ppm vs settled):");
    uint32_t begin = 0;
    while (begin < count) {
        const uint32_t end = std::min(count, begin < 2 ? begin + 1 : 2 * begin);
        PpmStats bin = {};
        double t_sum = 0;
        for (uint32_t i = begin; i < end; i++) {
            bin.add(ppm_vs(s_timeline[i].period, settled));
            t_sum += s_timeline[i].t_us;
        }
        append(line, sizeof(line), &len, " %.0f:%+.0f", awake_before_ms + t_sum / bin.count / 1000.0, bin.mean());
        begin = end;
    }
    ESP_LOGW(TAG, "%s", line);

    for (begin = 0; begin < count; begin += RAW_PER_LINE) {
        const uint32_t end = std::min(count, begin + RAW_PER_LINE);
        len = 0;
        append(line, sizeof(line), &len, "raw, %lu-%lu ms into the run:", static_cast<unsigned long>(s_timeline[begin].t_us / 1000),
               static_cast<unsigned long>(s_timeline[end - 1].t_us / 1000));
        for (uint32_t i = begin; i < end; i++)
            append(line, sizeof(line), &len, " %+.0f", ppm_vs(s_timeline[i].period, settled));
        ESP_LOGW(TAG, "%s", line);
    }
}

static void log_bias(const PpmStats (&bias)[BIAS_N])
{
    char line[256];
    size_t len = 0;
    append(line, sizeof(line), &len, "sandwiched between N=%lu:", static_cast<unsigned long>(REFERENCE_CAL_CYCLES));
    for (size_t i = 0; i < BIAS_N; i++)
        append(line, sizeof(line), &len, "%s N=%lu %+.0f +-%.0f ppm (sd %.0f, n %lu, %lu failed)", i > 0 ? " |" : "",
               static_cast<unsigned long>(BIAS_CYCLES[i]), bias[i].mean(), bias[i].se(), bias[i].sd(),
               static_cast<unsigned long>(bias[i].count), static_cast<unsigned long>(bias[i].failed));
    ESP_LOGW(TAG, "%s", line);
}

static double median_ppm(std::vector<double> &values)
{
    if (values.empty())
        return 0.0;
    const auto mid = values.begin() + values.size() / 2;
    std::nth_element(values.begin(), mid, values.end());
    return *mid;
}

// The sleep path's own calibrations since the ring last wrapped: overall, and split by how long
// the core had been awake before calibrating, since the timeline shows RTC_SLOW moving while
// awake. "sleep-weighted" weighs each by the sleep it timed, so it is what esp_timer actually used.
static void log_sleep_path(const std::vector<SleepCal> &sleeps, uint32_t cal_count, double settled, uint32_t in_use)
{
    struct AwakeBin {
        const char *name;
        uint32_t below_us;
    };
    static constexpr AwakeBin BINS[] = {
        {"<1ms", 1000}, {"1-5ms", 5000}, {"5-20ms", 20000}, {"20-100ms", 100000}, {"0.1-1s", 1000000}, {">1s", UINT32_MAX},
    };
    static constexpr size_t BIN_N = sizeof(BINS) / sizeof(BINS[0]);

    std::vector<double> all;
    std::vector<double> binned[BIN_N];
    double weighted_sum = 0;
    double slept_sum = 0;
    double bin_slept[BIN_N] = {};
    all.reserve(sleeps.size());
    for (const SleepCal &sleep : sleeps) {
        const double ppm = ppm_vs(sleep.period, settled);
        all.push_back(ppm);
        weighted_sum += ppm * sleep.slept_us;
        slept_sum += sleep.slept_us;
        size_t bin = 0;
        while (bin + 1 < BIN_N && sleep.awake_us >= BINS[bin].below_us)
            bin++;
        binned[bin].push_back(ppm);
        bin_slept[bin] += sleep.slept_us;
    }

    ESP_LOGW(TAG, "sleep path N=%lu: %lu calibrations since last run, last %u: median %+.0f, sleep-weighted %+.0f "
                  "ppm vs settled (%.1f s slept) | in use now %+.0f",
             static_cast<unsigned long>(SLEEP_PATH_CAL_CYCLES), static_cast<unsigned long>(cal_count), static_cast<unsigned>(sleeps.size()),
             median_ppm(all), slept_sum > 0 ? weighted_sum / slept_sum : 0.0, slept_sum / 1e6,
             ppm_vs(in_use, settled));

    char line[320];
    size_t len = 0;
    append(line, sizeof(line), &len, "sleep path by awake time before the calibration (n, median ppm, share of sleep):");
    for (size_t i = 0; i < BIN_N; i++) {
        if (binned[i].empty())
            continue;
        append(line, sizeof(line), &len, " %s %u %+.0f %.0f%%", BINS[i].name, static_cast<unsigned>(binned[i].size()), median_ppm(binned[i]),
               slept_sum > 0 ? 100.0 * bin_slept[i] / slept_sum : 0.0);
    }
    ESP_LOGW(TAG, "%s", line);
}

static void run_once()
{
    esp_pm_lock_acquire(s_no_sleep_lock);
    const int64_t start_us = esp_timer_get_time();
    const uint64_t rtc_ticks = rtc_time_get();
    portENTER_CRITICAL(&s_sleep_cal_mux);
    const int64_t last_wake_us = s_last_wake_us;
    portEXIT_CRITICAL(&s_sleep_cal_mux);

    uint32_t timeline_count = 0;
    uint32_t timeline_failed = 0;
    while (timeline_count < TIMELINE_MAX_SAMPLES) {
        const int64_t now = esp_timer_get_time();
        if (now - start_us >= TIMELINE_US)
            break;
        const uint32_t period = __real_rtc_clk_cal(CLK_CAL_RTC_SLOW, REFERENCE_CAL_CYCLES);
        if (period == 0) {
            timeline_failed++;
            continue;
        }
        s_timeline[timeline_count].period = period;
        s_timeline[timeline_count].t_us = static_cast<uint32_t>(now - start_us);
        timeline_count++;
    }

    // Each short calibration against the mean of the reference just before and just after it,
    // ~15 ms apart, so RTC_SLOW moving during the run cancels out.
    PpmStats bias[BIAS_N] = {};
    uint32_t ref_before = __real_rtc_clk_cal(CLK_CAL_RTC_SLOW, REFERENCE_CAL_CYCLES);
    for (uint32_t rep = 0; rep < BIAS_REPS; rep++) {
        for (size_t i = 0; i < BIAS_N; i++) {
            const uint32_t period = __real_rtc_clk_cal(CLK_CAL_RTC_SLOW, BIAS_CYCLES[i]);
            const uint32_t ref_after = __real_rtc_clk_cal(CLK_CAL_RTC_SLOW, REFERENCE_CAL_CYCLES);
            if (period != 0 && ref_before != 0 && ref_after != 0)
                bias[i].add(ppm_vs(period, (static_cast<double>(ref_before) + ref_after) / 2.0));
            else
                bias[i].failed++;
            ref_before = ref_after;
        }
    }
    const uint32_t in_use = esp_clk_slowclk_cal_get();
    const int64_t end_us = esp_timer_get_time();
    esp_pm_lock_release(s_no_sleep_lock);

    // No light sleep happened during the run, so the ring holds the sleeps just before it.
    std::vector<SleepCal> sleeps;
    sleeps.reserve(SLEEP_RING_SIZE);
    portENTER_CRITICAL(&s_sleep_cal_mux);
    const uint32_t head = s_sleep_ring_head;
    const uint32_t cal_count = s_sleep_cal_count;
    s_sleep_cal_count = 0;
    for (uint32_t i = head - std::min(head, SLEEP_RING_SIZE); i < head; i++)
        sleeps.push_back(s_sleep_ring[i % SLEEP_RING_SIZE]);
    portEXIT_CRITICAL(&s_sleep_cal_mux);

    if (timeline_count < TIMELINE_SETTLED_SAMPLES) {
        ESP_LOGW(TAG, "only %lu timeline calibrations (%lu failed), skipping this run",
                 static_cast<unsigned long>(timeline_count), static_cast<unsigned long>(timeline_failed));
        return;
    }
    double settled = 0;
    for (uint32_t i = timeline_count - TIMELINE_SETTLED_SAMPLES; i < timeline_count; i++)
        settled += s_timeline[i].period;
    settled /= TIMELINE_SETTLED_SAMPLES;
    const double awake_before_ms = last_wake_us > 0 ? (start_us - last_wake_us) / 1000.0 : -1.0;

    // esp_timer and the RTC tick counter read together: between two runs, their ratio is the tick
    // length esp_timer effectively used, and against real time (the border router's log) the tick
    // length RTC_SLOW really had -- mostly while asleep.
    ESP_LOGW(TAG, "run: esp_timer=%lld us rtc_ticks=%llu | awake %.1f ms before it, took %lld ms | "
                  "RTC_SLOW %.3f kHz settled (N=%lu, %lu failed)",
             start_us, rtc_ticks, awake_before_ms, (end_us - start_us) / 1000,
             1e6 * (1 << RTC_CLK_CAL_FRACT) / settled / 1000.0, static_cast<unsigned long>(REFERENCE_CAL_CYCLES),
             static_cast<unsigned long>(timeline_failed));
    log_timeline(timeline_count, settled, awake_before_ms);
    log_bias(bias);
    log_sleep_path(sleeps, cal_count, settled, in_use);
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
    esp_pm_sleep_cbs_register_config_t cbs_conf = {
        .enter_cb = nullptr,
        .exit_cb = on_light_sleep_exit,
        .enter_cb_user_arg = nullptr,
        .exit_cb_user_arg = nullptr,
        .enter_cb_prior = 0,
        .exit_cb_prior = 0,
    };
    err = esp_pm_light_sleep_register_cbs(&cbs_conf);
    if (err != ESP_OK)
        return err;
    // Lowest priority above idle: the measurement busy-waits (~2 s per run) and must never hold
    // up OpenThread or the sensor/MQTT tasks; being preempted only stretches it.
    if (xTaskCreate(rtc_cal_diag_task, "rtc_cal_diag", 6144, nullptr, tskIDLE_PRIORITY + 1, nullptr) != pdPASS)
        return ESP_ERR_NO_MEM;
    return ESP_OK;
}
