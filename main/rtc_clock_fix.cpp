#include "rtc_clock_fix.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <mutex>
#include <optional>

#include "esp_attr.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "soc/rtc.h"

#include "die_temp.h"

static const char *TAG = "rtc-fix";

// RTC_CLK_SRC_CAL_CYCLES in esp_hw_support/sleep_modes.c: the light-sleep path's rtc_clk_cal()
// call is the only one with this length, so it's the one the wrap takes over.
static constexpr uint32_t SLEEP_PATH_CAL_CYCLES = 10;

// ── A: cold calibration ──────────────────────────────────────────────────────────────────────
// Same length as ESP-IDF's, so the cost stays the same (~80 us with interrupts off); only the
// moment moves.
static constexpr uint32_t COLD_CAL_CYCLES = 10;
// Only a sleep this long counts as having let the die cool from whatever came before it.
static constexpr int64_t COLD_MIN_SLEEP_US = 50 * 1000;
static constexpr uint32_t COLD_RING_SIZE = 32;
// Until this many cold values exist the sleep path keeps ESP-IDF's own calibration.
static constexpr uint32_t COLD_MIN_SAMPLES = 8;
// A cold value this far from the mean is dropped as a glitch: ~10 C of RC drift between wakes.
static constexpr uint64_t COLD_MAX_JUMP_PPM = 20000;

// ── C: NTP-learned trim ──────────────────────────────────────────────────────────────────────
static constexpr int64_t NTP_FIRST_SAMPLE_AFTER_US = 5LL * 60 * 1000 * 1000;
static constexpr int64_t NTP_PERIOD_US = 30LL * 60 * 1000 * 1000;
static constexpr int64_t NTP_RETRY_US = 10LL * 60 * 1000 * 1000;
// A shorter interval (e.g. right after a failed sample's retry) resolves the rate too coarsely:
// the best exchange's one-way asymmetry is still tens of ms.
static constexpr int64_t NTP_MIN_INTERVAL_US = 20LL * 60 * 1000 * 1000;
// The exchange with the shortest round trip is kept: its reply waited least for a poll or a CSL
// window at the parent, so it's the least asymmetric.
static constexpr uint32_t NTP_EXCHANGES = 3;
static constexpr uint32_t NTP_REPLY_TIMEOUT_MS = 1500;
static constexpr uint32_t NTP_SPACING_MS = 1000;
static constexpr uint32_t NTP_BURST_BUDGET_MS = NTP_EXCHANGES * NTP_REPLY_TIMEOUT_MS + 500;
static constexpr int64_t NTP_MAX_RTT_US = NTP_REPLY_TIMEOUT_MS * 1000;
static constexpr uint16_t NTP_PORT = 123;
static constexpr size_t NTP_PACKET_LEN = 48;
static constexpr uint32_t TRIM_HISTORY_SIZE = 8;
// An interval claiming more than this is a bad sample, not something to steer the clock by.
static constexpr double NTP_MAX_ERROR_PPM = 5000;
static constexpr double TRIM_LIMIT_PPM = 3000;

static constexpr uint32_t STATUS_PERIOD_MS = 5 * 60 * 1000;
static constexpr uint32_t DIE_TEMP_SAMPLES = 16;

extern "C" uint32_t __real_rtc_clk_cal(soc_clk_freq_calculation_src_t cal_clk_sel,
                                       uint32_t slow_clk_cycles);
extern "C" uint32_t __wrap_rtc_clk_cal(soc_clk_freq_calculation_src_t cal_clk_sel,
                                       uint32_t slow_clk_cycles);

// Both modes are read by the wrap in the sleep path and written from the MQTT task.
static std::atomic<uint32_t> s_cal_mode{static_cast<uint32_t>(RtcCalMode::ColdMean)};
static std::atomic<uint32_t> s_trim_mode{static_cast<uint32_t>(RtcTrimMode::NtpMean)};
// The trim the sleep path applies, in ppm and as the Q30 factor the wrap multiplies by
// (2^30 = none): the wrap runs from IRAM with interrupts off, so no division there.
static std::atomic<int32_t> s_trim_applied_ppm{0};
static std::atomic<uint32_t> s_trim_factor_q30{1u << 30};

static RtcSleepCalObserver s_observer = nullptr;

// Cold values and the NTP interval's sleep accounting: written by the exit callback and read by
// the wrap -- same core, both with interrupts off, so never at the same time -- and read by
// tasks under s_mux.
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
// The trim the wrap applied for the sleep in progress, for the exit callback's accounting.
static int32_t s_sleep_trim_ppm = 0;
static uint64_t s_interval_slept_us = 0;
static int64_t s_interval_trim_x_slept = 0;
// Cleared by a calibration-mode change: that interval mixed two calibrations.
static bool s_interval_valid = true;

// Task-side state of the NTP trim: the MQTT task (samples, mode changes) and the status task.
static std::mutex s_task_mutex;
static std::string s_ntp_server;
static int64_t s_ntp_due_us = NTP_FIRST_SAMPLE_AFTER_US;
struct NtpSample {
    int64_t device_us;
    int64_t offset_us;
};
static std::optional<NtpSample> s_ntp_prev;
static int32_t s_trim_history[TRIM_HISTORY_SIZE];
static uint32_t s_trim_count = 0;
static int32_t s_trim_last_ppm = 0;
static int32_t s_trim_mean_ppm = 0;
// Die temperatures the status task read since the last NTP sample.
static double s_temp_sum = 0;
static uint32_t s_temp_count = 0;

static const char *cal_mode_name(uint32_t mode)
{
    switch (static_cast<RtcCalMode>(mode)) {
        case RtcCalMode::EspIdf: return "esp-idf";
        case RtcCalMode::ColdLast: return "cold-last";
        case RtcCalMode::ColdMean: return "cold-mean";
    }
    return "?";
}

static const char *trim_mode_name(uint32_t mode)
{
    switch (static_cast<RtcTrimMode>(mode)) {
        case RtcTrimMode::Off: return "off";
        case RtcTrimMode::NtpLast: return "ntp-last";
        case RtcTrimMode::NtpMean: return "ntp-mean";
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
    uint32_t measured = 0;
    uint32_t base = 0;
    if (mode != RtcCalMode::EspIdf && s_cold_count >= COLD_MIN_SAMPLES) {
        base = mode == RtcCalMode::ColdLast ? s_cold_last : s_cold_mean;
        s_sleeps_cold++;
    } else {
        measured = __real_rtc_clk_cal(cal_clk_sel, slow_clk_cycles);
        base = measured;
        s_sleeps_measured++;
    }
    if (base == 0)
        return 0;

    const uint64_t factor_q30 = s_trim_factor_q30.load(std::memory_order_relaxed);
    const auto period = static_cast<uint32_t>((uint64_t{base} * factor_q30) >> 30);
    s_sleep_trim_ppm = s_trim_applied_ppm.load(std::memory_order_relaxed);
    if (s_observer != nullptr)
        s_observer(period, measured);
    return period;
}

// Under s_mux.
static void accept_cold(uint32_t period)
{
    if (s_cold_count >= COLD_MIN_SAMPLES) {
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
    const uint32_t cold = sleep_time_us >= COLD_MIN_SLEEP_US
        ? __real_rtc_clk_cal(CLK_CAL_RTC_SLOW, COLD_CAL_CYCLES)
        : 0;
    portENTER_CRITICAL_SAFE(&s_mux);
    s_interval_slept_us += static_cast<uint64_t>(sleep_time_us);
    s_interval_trim_x_slept += sleep_time_us * s_sleep_trim_ppm;
    if (cold != 0)
        accept_cold(cold);
    portEXIT_CRITICAL_SAFE(&s_mux);
    return ESP_OK;
}

// Under s_task_mutex.
static void update_applied_trim_locked()
{
    int32_t ppm = 0;
    if (s_trim_count > 0) {
        switch (static_cast<RtcTrimMode>(s_trim_mode.load())) {
            case RtcTrimMode::NtpLast: ppm = s_trim_last_ppm; break;
            case RtcTrimMode::NtpMean: ppm = s_trim_mean_ppm; break;
            case RtcTrimMode::Off: break;
        }
    }
    s_trim_applied_ppm.store(ppm);
    s_trim_factor_q30.store(static_cast<uint32_t>(std::llround((1.0 + ppm * 1e-6) * (1u << 30))));
}

// ── NTP ─────────────────────────────────────────────────────────────────────────────────────

static void put_be64(uint8_t *p, uint64_t value)
{
    for (int i = 7; i >= 0; i--) {
        p[i] = static_cast<uint8_t>(value);
        value >>= 8;
    }
}

static uint64_t get_be64(const uint8_t *p)
{
    uint64_t value = 0;
    for (int i = 0; i < 8; i++)
        value = value << 8 | p[i];
    return value;
}

// An NTP timestamp (seconds since 1900, 32.32 fixed point) in microseconds.
static int64_t ntp_timestamp_us(const uint8_t *p)
{
    const uint64_t raw = get_be64(p);
    return static_cast<int64_t>((raw >> 32) * 1000000 + (((raw & 0xffffffffu) * 1000000) >> 32));
}

struct NtpExchange {
    // Device time halfway through the exchange; offset_us is real time minus device time there.
    int64_t device_us;
    int64_t offset_us;
    int64_t rtt_us;
};

static std::optional<NtpExchange> ntp_exchange(int sock)
{
    uint8_t request[NTP_PACKET_LEN] = {};
    request[0] = 0x23;  // leap 0, version 4, mode 3 (client)
    const int64_t t1 = esp_timer_get_time();
    // Our own send time as the transmit timestamp: the server echoes it as the originate
    // timestamp, which tells this request's reply from a late one to an earlier request.
    put_be64(request + 40, static_cast<uint64_t>(t1));
    if (send(sock, request, sizeof(request), 0) != static_cast<int>(sizeof(request)))
        return std::nullopt;

    while (true) {
        const int64_t left_us = int64_t{NTP_REPLY_TIMEOUT_MS} * 1000 - (esp_timer_get_time() - t1);
        if (left_us <= 0)
            return std::nullopt;
        timeval timeout = {};
        timeout.tv_sec = static_cast<time_t>(left_us / 1000000);
        timeout.tv_usec = static_cast<suseconds_t>(left_us % 1000000);
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

        uint8_t reply[NTP_PACKET_LEN];
        const int len = recv(sock, reply, sizeof(reply), 0);
        const int64_t t4 = esp_timer_get_time();
        if (len < 0)
            return std::nullopt;
        if (len < static_cast<int>(sizeof(reply)) || get_be64(reply + 24) != static_cast<uint64_t>(t1))
            continue;
        const uint8_t mode = reply[0] & 0x07;
        const uint8_t stratum = reply[1];
        // Not a server reply, or a kiss-o'-death (stratum 0).
        if (mode != 4 || stratum == 0 || stratum > 15)
            return std::nullopt;
        const int64_t t2 = ntp_timestamp_us(reply + 32);
        const int64_t t3 = ntp_timestamp_us(reply + 40);
        return NtpExchange{(t1 + t4) / 2, ((t2 - t1) + (t3 - t4)) / 2, (t4 - t1) - (t3 - t2)};
    }
}

struct NtpBurst {
    NtpExchange best;
    uint32_t replies;
};

static std::optional<NtpBurst> ntp_burst(const NetworkLink &link, const std::string &server)
{
    if (!link.waitForBrokerReachable(server, 0)) {
        ESP_LOGW(TAG, "ntp: %s not reachable yet", server.c_str());
        return std::nullopt;
    }
    const std::string host = link.connectAddress(server);
    sockaddr_storage addr = {};
    socklen_t addr_len = 0;
    if (looksLikeIpv6(host)) {
        auto *a = reinterpret_cast<sockaddr_in6 *>(&addr);
        a->sin6_family = AF_INET6;
        a->sin6_port = htons(NTP_PORT);
        if (inet_pton(AF_INET6, host.c_str(), &a->sin6_addr) != 1)
            addr_len = 0;
        else
            addr_len = sizeof(*a);
    } else {
        auto *a = reinterpret_cast<sockaddr_in *>(&addr);
        a->sin_family = AF_INET;
        a->sin_port = htons(NTP_PORT);
        if (inet_pton(AF_INET, host.c_str(), &a->sin_addr) != 1)
            addr_len = 0;
        else
            addr_len = sizeof(*a);
    }
    if (addr_len == 0) {
        ESP_LOGE(TAG, "ntp: bad server address '%s'", host.c_str());
        return std::nullopt;
    }

    const int sock = socket(addr.ss_family, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "ntp: socket() failed: %d", errno);
        return std::nullopt;
    }
    std::optional<NtpBurst> result;
    if (connect(sock, reinterpret_cast<const sockaddr *>(&addr), addr_len) != 0) {
        ESP_LOGE(TAG, "ntp: connect() failed: %d", errno);
    } else {
        for (uint32_t i = 0; i < NTP_EXCHANGES; i++) {
            const int64_t sent_us = esp_timer_get_time();
            const std::optional<NtpExchange> exchange = ntp_exchange(sock);
            if (exchange && exchange->rtt_us >= 0 && exchange->rtt_us <= NTP_MAX_RTT_US) {
                if (!result)
                    result = NtpBurst{*exchange, 0};
                else if (exchange->rtt_us < result->best.rtt_us)
                    result->best = *exchange;
                result->replies++;
            }
            const int64_t spent_ms = (esp_timer_get_time() - sent_us) / 1000;
            if (i + 1 < NTP_EXCHANGES && spent_ms < NTP_SPACING_MS)
                vTaskDelay(pdMS_TO_TICKS(NTP_SPACING_MS - spent_ms));
        }
    }
    close(sock);
    return result;
}

// Under s_task_mutex. Turns the exchange into an interval estimate against the previous one.
static void record_ntp_sample_locked(const NtpBurst &burst)
{
    portENTER_CRITICAL(&s_mux);
    const uint64_t slept_us = s_interval_slept_us;
    const int64_t trim_x_slept = s_interval_trim_x_slept;
    const bool valid = s_interval_valid;
    s_interval_slept_us = 0;
    s_interval_trim_x_slept = 0;
    s_interval_valid = true;
    portEXIT_CRITICAL(&s_mux);
    const double die_c = s_temp_count > 0 ? s_temp_sum / s_temp_count : NAN;
    s_temp_sum = 0;
    s_temp_count = 0;

    const NtpSample sample{burst.best.device_us, burst.best.offset_us};
    const std::optional<NtpSample> prev = s_ntp_prev;
    s_ntp_prev = sample;
    ESP_LOGI(TAG, "ntp: offset %+.1f ms, best round trip %.0f ms of %lu replies",
             sample.offset_us / 1000.0, burst.best.rtt_us / 1000.0, static_cast<unsigned long>(burst.replies));
    if (!prev) {
        ESP_LOGI(TAG, "ntp: first sample, the trim needs the next one");
        return;
    }
    if (!valid) {
        ESP_LOGI(TAG, "ntp: calibration mode changed during the interval, not used");
        return;
    }
    const double device_us = static_cast<double>(sample.device_us - prev->device_us);
    if (device_us < NTP_MIN_INTERVAL_US) {
        ESP_LOGI(TAG, "ntp: interval only %.1f min, not used", device_us / 60e6);
        return;
    }

    // Device time runs ahead of real time only while asleep (awake time comes from the 40 MHz
    // crystal), so the sleep path's period is off by error / (share of time asleep), on top of
    // the trim it already applied.
    const double real_us = device_us + static_cast<double>(sample.offset_us - prev->offset_us);
    const double error_ppm = (device_us / real_us - 1.0) * 1e6;
    const double asleep = slept_us / device_us;
    const double applied_ppm = slept_us > 0 ? static_cast<double>(trim_x_slept) / slept_us : 0.0;
    const double needed_ppm = applied_ppm - error_ppm / asleep;
    if (std::fabs(error_ppm) > NTP_MAX_ERROR_PPM || asleep < 0.5 || std::fabs(needed_ppm) > TRIM_LIMIT_PPM) {
        ESP_LOGW(TAG, "ntp: %.1f min interval rejected: clock %+.0f ppm, %.1f%% asleep, would need trim %+.0f ppm",
                 device_us / 60e6, error_ppm, asleep * 100, needed_ppm);
        return;
    }

    s_trim_history[s_trim_count % TRIM_HISTORY_SIZE] = static_cast<int32_t>(std::lround(needed_ppm));
    s_trim_count++;
    s_trim_last_ppm = static_cast<int32_t>(std::lround(needed_ppm));
    const uint32_t n = std::min(s_trim_count, TRIM_HISTORY_SIZE);
    int64_t sum = 0;
    for (uint32_t i = 0; i < n; i++)
        sum += s_trim_history[i];
    s_trim_mean_ppm = static_cast<int32_t>(std::lround(static_cast<double>(sum) / n));
    update_applied_trim_locked();

    ESP_LOGW(TAG, "ntp: %.1f min: clock %+.1f ppm with trim %+.1f applied (%.1f%% asleep, die %.1f C) "
                  "-> needs trim %+.1f ppm | trim last %+ld, mean %+ld of %lu -> applying %+ld (%s)",
             device_us / 60e6, error_ppm, applied_ppm, asleep * 100, die_c, needed_ppm,
             static_cast<long>(s_trim_last_ppm), static_cast<long>(s_trim_mean_ppm),
             static_cast<unsigned long>(n), static_cast<long>(s_trim_applied_ppm.load()),
             trim_mode_name(s_trim_mode.load()));
}

void rtc_clock_fix_ntp_sample_if_due(const NetworkLink &link, uint32_t budget_ms)
{
    std::string server;
    {
        const std::lock_guard<std::mutex> lock(s_task_mutex);
        if (s_ntp_server.empty() || esp_timer_get_time() < s_ntp_due_us)
            return;
        server = s_ntp_server;
    }
    if (budget_ms < NTP_BURST_BUDGET_MS) {
        ESP_LOGI(TAG, "ntp: sample due, but only %lu ms left in this publish window",
                 static_cast<unsigned long>(budget_ms));
        return;
    }

    const std::optional<NtpBurst> burst = ntp_burst(link, server);
    const std::lock_guard<std::mutex> lock(s_task_mutex);
    if (!burst) {
        ESP_LOGW(TAG, "ntp: no usable reply from %s, retrying in %lld min", server.c_str(), NTP_RETRY_US / 60000000);
        s_ntp_due_us = esp_timer_get_time() + NTP_RETRY_US;
        return;
    }
    s_ntp_due_us = esp_timer_get_time() + NTP_PERIOD_US;
    record_ntp_sample_locked(*burst);
}

// ── modes and status ────────────────────────────────────────────────────────────────────────

void rtc_clock_fix_set_cal_mode(uint32_t mode)
{
    mode = std::min<uint32_t>(mode, static_cast<uint32_t>(RtcCalMode::ColdMean));
    if (s_cal_mode.exchange(mode) == mode)
        return;
    portENTER_CRITICAL(&s_mux);
    s_interval_valid = false;
    portEXIT_CRITICAL(&s_mux);
    const std::lock_guard<std::mutex> lock(s_task_mutex);
    s_trim_count = 0;
    s_trim_last_ppm = 0;
    s_trim_mean_ppm = 0;
    update_applied_trim_locked();
    ESP_LOGW(TAG, "sleep path calibration now %s; the trims learned so far are discarded", cal_mode_name(mode));
}

void rtc_clock_fix_set_trim_mode(uint32_t mode)
{
    mode = std::min<uint32_t>(mode, static_cast<uint32_t>(RtcTrimMode::NtpMean));
    if (s_trim_mode.exchange(mode) == mode)
        return;
    const std::lock_guard<std::mutex> lock(s_task_mutex);
    update_applied_trim_locked();
    ESP_LOGW(TAG, "sleep path trim now %s: applying %+ld ppm", trim_mode_name(mode),
             static_cast<long>(s_trim_applied_ppm.load()));
}

uint32_t rtc_clock_fix_cal_mode()
{
    return s_cal_mode.load();
}

uint32_t rtc_clock_fix_trim_mode()
{
    return s_trim_mode.load();
}

void rtc_clock_fix_set_sleep_cal_observer(RtcSleepCalObserver observer)
{
    s_observer = observer;
}

// Every STATUS_PERIOD_MS: the die temperature, what the cold calibrations say and what the sleep
// path is using. Woken by its own timer, so the die reading is taken right after a wake too.
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

        int32_t trim_last = 0;
        int32_t trim_mean = 0;
        uint32_t trim_n = 0;
        {
            const std::lock_guard<std::mutex> lock(s_task_mutex);
            if (die) {
                s_temp_sum += die->celsius;
                s_temp_count++;
            }
            trim_last = s_trim_last_ppm;
            trim_mean = s_trim_mean_ppm;
            trim_n = std::min(s_trim_count, TRIM_HISTORY_SIZE);
        }

        ESP_LOGW(TAG, "status: die %.1f C (raw %.1f) | cold last %.3f kHz, mean %.3f kHz (%+.0f ppm), "
                      "%lu taken, %lu rejected | %lu sleeps cold, %lu measured | calibration %s, trim %s "
                      "%+ld ppm (last %+ld, mean %+ld of %lu)",
                 die ? die->celsius : NAN, die ? die->raw : NAN, period_khz(cold_last), period_khz(cold_mean),
                 cold_mean != 0 ? (static_cast<double>(cold_last) / cold_mean - 1.0) * 1e6 : 0.0,
                 static_cast<unsigned long>(cold_count), static_cast<unsigned long>(cold_rejected),
                 static_cast<unsigned long>(sleeps_cold), static_cast<unsigned long>(sleeps_measured),
                 cal_mode_name(s_cal_mode.load()), trim_mode_name(s_trim_mode.load()),
                 static_cast<long>(s_trim_applied_ppm.load()), static_cast<long>(trim_last),
                 static_cast<long>(trim_mean), static_cast<unsigned long>(trim_n));
    }
}

esp_err_t rtc_clock_fix_start(const RtcClockFixConfig &config)
{
    s_cal_mode.store(std::min<uint32_t>(config.cal_mode, static_cast<uint32_t>(RtcCalMode::ColdMean)));
    s_trim_mode.store(std::min<uint32_t>(config.trim_mode, static_cast<uint32_t>(RtcTrimMode::NtpMean)));
    {
        const std::lock_guard<std::mutex> lock(s_task_mutex);
        s_ntp_server = config.ntp_server;
        update_applied_trim_locked();
    }

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

    ESP_LOGI(TAG, "sleep path calibration %s, trim %s, NTP server %s",
             cal_mode_name(s_cal_mode.load()), trim_mode_name(s_trim_mode.load()),
             config.ntp_server.empty() ? "(none, no trim)" : config.ntp_server.c_str());
    return ESP_OK;
}
