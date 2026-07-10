#include "sensorstask.h"

#include <charconv>
#include <ranges>
#include <cmath>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_system.h>

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

    // This task is the sole driver of the HP backstop cadence: wake → read LP's state →
    // publish (if flagged) → wait-for-idle → block until the next LP-flagged wake or the
    // cycle_duration_sec backstop, on every iteration. Light sleep itself is fully automatic
    // (see enableAutomaticLightSleep(), main/common_utils.cpp) -- this task never calls any
    // sleep API; it just blocks, which is what lets the CPU idle-sleep underneath it. The LP
    // core (components/lp_sensor_core) can wake this wait early via
    // ulp_lp_core_wakeup_main_processor() -- see lp_sensor_core_wait_for_wake().
    while (true) {
        // Whether data actually reached the broker this cycle. Stays false unless a publish was
        // started AND mqtt_last_publish_succeeded() confirms a connected, ACKed state message.
        bool publishedOk = false;
        bool shouldWake = false;
        // Don't read/publish until attached as CHILD. A plain blocking wait -- under automatic
        // light sleep this is safe to block through: OpenThread's own PM lock
        // (esp_openthread_sleep.c) keeps the CPU awake whenever the radio is actively
        // scanning/attaching, and only lets it idle-sleep once the radio itself is idle. If
        // the network is absent the gate times out and we fall through to one cycle and retry
        // next wake.
        const bool attached = !m_attachGate || m_attachGate(ATTACH_TIMEOUT_MS);

        if (!attached) {
            ESP_LOGW(TAG, "OT not attached within %u ms, retrying next cycle", ATTACH_TIMEOUT_MS);
        } else {
            lp_shared_state_t state{};
            lp_sensor_core_get_state(&state);

            // How many real LP timer cycles elapsed since the last HP wake. LP's own ULP
            // timer re-arms itself for lp_poll_interval_sec every invocation (see
            // components/lp_sensor_core), so this reads exactly 1 on an LP-triggered wake and
            // something larger on a backstop-timeout wake that lands mid-cycle.
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
                // Fires on the majority of wakes in steady state -- ESP_LOGD, not I: a
                // synchronous UART/USB-JTAG write on every quiet cycle is real, avoidable
                // awake time. Bump CONFIG_LOG_DEFAULT_LEVEL to see it again for debugging.
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

        // Block until the LP core flags something early (see lp_sensor_core_wait_for_wake())
        // or the cycle_duration_sec backstop elapses. Whichever fires, loop back and re-read
        // LP's state. This is a plain blocking wait -- ESP-IDF's automatic tickless-idle light
        // sleep (enableAutomaticLightSleep()) transparently sleeps the CPU underneath it
        // whenever no esp_pm lock (e.g. OpenThread's own radio-state lock) says otherwise.
        lp_sensor_core_wait_for_wake(m_settings.cycle_duration_sec * 1000);
    }
}
