#include "sensorstask.h"

#include <charconv>
#include <ranges>
#include <cmath>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <esp_rom_sys.h>

#include "common_utils.h"
#include "mqtt_sender.h"
#include "lp_sensor_core.h"

SensorsTask::SensorsTask(SensorsTaskSettings settings):
    m_settings{settings}
{

}

std::string SensorsValues::toTelemetryRoundedString(const float value)
{
    // buf[24]: fixed,3 for sensor ranges (±150 °C, 0–1200 hPa) never exceeds 10 chars.
    char buf[24];
    auto [ptr, ec] = std::to_chars(buf, buf + sizeof(buf), value, std::chars_format::fixed, 3);
    if (ec != std::errc{})
        return "ERR";
    std::string_view sv(buf, ptr);
    if (!sv.contains('.'))
        return std::string(sv);
    const auto isTrailingZero = [](char c) static {
        return c == '0';
    };
    const auto trailing = sv | std::views::reverse | std::views::take_while(isTrailingZero);
    sv.remove_suffix(std::ranges::distance(trailing));
    if (sv.ends_with('.'))
        sv.remove_suffix(1);
    return std::string(sv);
}

std::string SensorsValues::toTelemetryString() const
{
    std::string message;
    if (envTemperature) {
        message += std::string_view("TEMP;");
        message += toTelemetryRoundedString(envTemperature.value());
        message += std::string_view(";");
    }
    if (envHumidity) {
        message += std::string_view("HUMID;");
        message += toTelemetryRoundedString(envHumidity.value());
        message += std::string_view(";");
    }
    if (barometricPressure) {
        message += std::string_view("PRESS;");
        message += toTelemetryRoundedString(barometricPressure.value());
        message += std::string_view(";");
    }
    return message;
}

std::string SensorsValues::toLogString() const
{
    auto appendOpt = [](std::string& out, const auto& opt) static {
        if (opt.has_value()) appendNum(out, opt.value());
        else out += "NO_VALUE";
    };
    std::string result;
    result.reserve(100);
    result += "envTemperature: ";
    appendOpt(result, envTemperature);
    result += ", envHumidity: ";
    appendOpt(result, envHumidity);
    result += ", barometricPressure: ";
    appendOpt(result, barometricPressure);
    return result;
}

void SensorsTask::configureReadyEvent(SensorsReadyEvent readyEvent)
{
    m_readyEvent = std::move(readyEvent);
}

void SensorsTask::configureAttachGate(AttachGate attachGate)
{
    m_attachGate = std::move(attachGate);
}

void SensorsTask::configureRefreshNat64(RefreshNat64 refreshNat64)
{
    m_refreshNat64 = std::move(refreshNat64);
}

void SensorsTask::executeTask()
{
    static const char * TAG = "sensors-task";

    // This task is the sole driver of the HP backstop cadence: arm the wakeup timer once,
    // then wake → read LP's state → publish (if flagged) → wait-for-idle → light-sleep on
    // every iteration. The LP core (components/lp_sensor_core) can also wake HP early via
    // ulp_lp_core_wakeup_main_processor() -- see esp_sleep_enable_ulp_wakeup() in
    // lp_sensor_core_init() and correctLightSleep()'s "reason: ulp" branch.
    if (const esp_err_t timerRes = registerWakeupTimer(static_cast<uint64_t>(m_settings.cycle_duration_sec) * 1000 * 1000);
        timerRes != ESP_OK)
        ESP_LOGE(TAG, "can not register wakeup timer: %d — sleep interval undefined", timerRes);

    while (true) {
        // TEMP DIAGNOSTIC (2026-07-10 instability investigation): marks when this iteration
        // became CPU-awake, so the "awake for %lld ms" line below (right before sleeping) and
        // correctLightSleep()'s own "slept for %lld ms" line together account for the full
        // cycle -- letting a cycle much shorter than cycle_duration_sec/lp_poll_interval_sec
        // be attributed to either "spent too long awake" or "didn't actually sleep as long as
        // requested", instead of just observing the total gap from the log timestamps.
        const int64_t cycleAwakeStart_us = esp_timer_get_time();

        // Whether data actually reached the broker this cycle. Stays false unless a publish was
        // started AND mqtt_last_publish_succeeded() confirms a connected, ACKed state message.
        bool publishedOk = false;
        bool shouldWake = false;
        // Don't read/publish until attached as CHILD. This is a blocking wait (not light sleep) so
        // OpenThread can finish MLE attachment / re-attach after a lost parent; light-sleeping while
        // detached would freeze the radio and stall attachment. If the network is absent the gate
        // times out and we fall through to one sleep period and retry next wake.
        const bool attached = !m_attachGate || m_attachGate(ATTACH_TIMEOUT_MS);

        if (!attached) {
            ESP_LOGW(TAG, "OT not attached within %u ms, retrying next cycle", ATTACH_TIMEOUT_MS);
        } else {
            lp_shared_state_t state{};
            lp_sensor_core_get_state(&state);

            // TEMP DIAGNOSTIC (2026-07-10 instability investigation): how many real LP timer
            // cycles elapsed since the last HP wake. LP's own ULP timer re-arms itself for
            // lp_poll_interval_sec every invocation (see components/lp_sensor_core), so this
            // should read exactly 1 on every LP-triggered wake and something larger on a
            // backstop-timer wake that lands mid-cycle -- it should NEVER read 0. A 0 here
            // means HP was re-woken without any new LP cycle happening at all, which would
            // point squarely at the backstop timer (or esp_light_sleep_start() itself) firing
            // far more often than the configured cycle_duration_sec, not at the LP core.
            const uint32_t heartbeatDelta = state.heartbeat_counter - m_lastSeenHeartbeat;
            ESP_LOGD(TAG, "HP wake: LP heartbeat=%lu (delta %lu since last wake), sensor_ok=%d, should_wake_hp=%d",
                     static_cast<unsigned long>(state.heartbeat_counter),
                     static_cast<unsigned long>(heartbeatDelta),
                     state.sensor_ok != 0, state.should_wake_hp != 0);
            m_lastSeenHeartbeat = state.heartbeat_counter;

            // Post-hoc heater diagnostic -- there's no live LED indicator any more (heater
            // maintenance runs entirely on LP; see the migration plan's accepted behavior
            // change), so this log line is the only trail, printed once per new run.
            if (state.last_heater_run_cycle != m_lastSeenHeaterRunCycle) {
                ESP_LOGI(TAG, "LP ran heater maintenance at cycle %lu: delta_t=%.2fC passed=%d",
                         static_cast<unsigned long>(state.last_heater_run_cycle),
                         static_cast<double>(state.last_heater_delta_t),
                         state.last_heater_passed != 0);
                m_lastSeenHeaterRunCycle = state.last_heater_run_cycle;
            }

            if (!state.sensor_ok)
                ESP_LOGW(TAG, "LP reports sensor not OK (consec_fail=%lu)",
                         static_cast<unsigned long>(state.consec_fail_count));

            shouldWake = state.should_wake_hp != 0;

            if (shouldWake) {
                SensorsValues v{};
                v.envTemperature = state.cal_temp_c;
                v.envHumidity = state.cal_hum_pct;

                ESP_LOGI(TAG, "publishing LP-flagged value: %.2fC / %.2f%%RH",
                         static_cast<double>(state.cal_temp_c), static_cast<double>(state.cal_hum_pct));

                if (m_readyEvent)
                    m_readyEvent(v);  // triggers an async MQTT publish when attached as CHILD

                // Only wait if a publish was actually started. Waiting before sleeping stops
                // light sleep from freezing the MQTT task/radio mid-flight. mqtt_wait_for_idle()
                // only reports the task ended, not that it succeeded — that ending is identical
                // for a failed connect — so the real outcome comes from
                // mqtt_last_publish_succeeded(). A timeout means MQTT stalled; sleep anyway
                // rather than stay awake burning battery.
                if (mqtt_is_busy()) {
                    if (mqtt_wait_for_idle(PUBLISH_TIMEOUT_MS)) {
                        publishedOk = mqtt_last_publish_succeeded();
                        // Only ack on CONFIRMED delivery -- an attempted-but-failed publish
                        // must leave LP's baseline untouched, so the still-undelivered value
                        // keeps being flagged next cycle instead of silently getting dropped.
                        if (publishedOk)
                            lp_sensor_core_ack_delivered(state.cal_temp_c, state.cal_hum_pct);
                    } else {
                        ESP_LOGW(TAG, "publish did not finish within %u ms, sleeping anyway", PUBLISH_TIMEOUT_MS);
                    }
                } else {
                    ESP_LOGW(TAG, "mqtt is not working? not sure");
                }
            } else {
                // Fires on the majority of wakes in steady state -- ESP_LOGD, not I (see
                // correctLightSleep()'s comment on why per-cycle logging costs real awake
                // time). Bump CONFIG_LOG_DEFAULT_LEVEL to see it again for debugging.
                ESP_LOGD(TAG, "LP has nothing new to report this wake");
            }
        }

        // "Delivered, or nothing new to deliver" both count as a healthy cycle -- mirrors the
        // original HP-only code's (publishedOk || skipSameValuesCycle) reboot-supervisor gate.
        // Not attaching at all is never healthy, even if LP would have had nothing new to say.
        const bool cycleOk = publishedOk || (attached && !shouldWake);

        // Honest local indicator: 1 blink = data reached the broker, 5 = should have
        // published but didn't. No LED at all for "LP had nothing new" -- that's the most
        // common steady-state case (especially with a lengthened backstop) and isn't worth
        // forced-awake LED time for an unattended deployed sensor; the serial log line above
        // still covers it for bench debugging.
        if (publishedOk) {
            blinkUserLED(LED_BLINK_MS);
        } else if (!cycleOk) {
            ESP_LOGW(TAG, "data was not published to the broker this cycle (but should be)");
            blinkUserLED(LED_BLINK_MS, 5);
        }

        // Recovery supervisor: count consecutive failed cycles and reboot once they pass the
        // threshold. There is no other path back from a persistent reachability loss — a reboot
        // re-attaches to Thread and re-learns the NAT64 route. Before that, give a softer nudge:
        // re-read network data so a merely-stale NAT64 prefix is fixed without a reboot.
        if (cycleOk) {
            m_consecutiveFailures = 0;
        } else {
            if (m_refreshNat64)
                m_refreshNat64();

            if (++m_consecutiveFailures >= REBOOT_AFTER_FAILS) {
                ESP_LOGE(TAG, "%lu consecutive cycles without a successful publish — rebooting to recover",
                         static_cast<unsigned long>(m_consecutiveFailures));
                esp_restart();
            }
        }

        // TEMP DIAGNOSTIC (2026-07-10 instability investigation): pairs with
        // correctLightSleep()'s own "slept for %lld ms" line to account for the full cycle
        // duration -- see the comment on cycleAwakeStart_us above.
        ESP_LOGD(TAG, "awake for %lld ms, going to sleep", (esp_timer_get_time() - cycleAwakeStart_us) / 1000);

        // Instability investigation, 2026-07-10, retest 3: a POST-wake guaranteed-active
        // busy-wait (esp_rom_delay_us in correctLightSleep(), right after esp_light_sleep_
        // start() returns) did not fix the MQTT/OpenThread instability -- see common_utils.
        // cpp's history on this. The one stable build's second diagnostic line sat in this
        // PRE-sleep position instead (right before this call, i.e. right before the CPU
        // halts for sleep, not after it wakes). Testing that position in isolation: same
        // 10ms magnitude, same guaranteed-active mechanism, moved here.
        esp_rom_delay_us(10000);
        correctLightSleep();  // light-sleep until the backstop timer or an LP-triggered ULP wake
    }
}
