#include "sensorstask.h"

#include <charconv>
#include <ranges>
#include <cmath>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_ota_ops.h>
#include <esp_system.h>

#include "common_utils.h"
#include "mqtt_sender.h"
#include "ota_updater.h"
#include "lp_sensor_core.h"
#include "history_log.h"
#include "runtime_config.h"

// First broker-ACKed publish after an OTA reboot proves the new image out and cancels the
// bootloader's pending rollback (CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE). If this never runs,
// the UNCONFIRMED_OTA_REBOOT_AFTER_CYCLES safety net below restarts a still-PENDING_VERIFY
// image and the bootloader falls back to the previous slot — that supervisor doubles as the
// rollback watchdog.
static void markAppValidOnFirstConfirmedPublish()
{
    static bool s_checked = false;
    if (s_checked)
        return;
    s_checked = true;

    esp_ota_img_states_t state;
    if (esp_ota_get_state_partition(esp_ota_get_running_partition(), &state) == ESP_OK &&
        state == ESP_OTA_IMG_PENDING_VERIFY) {
        esp_ota_mark_app_valid_cancel_rollback();
        ESP_LOGI("sensors-task", "OTA image confirmed by successful publish — rollback cancelled");
    }
}

SensorsTask::SensorsTask(SensorsTaskSettings settings):
    m_settings{settings},
    m_voltageDividerCoefficient{(settings.batteryDividerRGndOhm + settings.batteryDividerRVbatOhm)
                                / settings.batteryDividerRGndOhm}
{

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

void SensorsTask::configureNoteCycleResult(NoteCycleResult noteCycleResult)
{
    m_noteCycleResult = std::move(noteCycleResult);
}

void SensorsTask::executeTask()
{
    static const char * TAG = "sensors-task";

    // The LP core (components/lp_sensor_core) owns the publish cadence entirely: it
    // reads/calibrates every lpPollIntervalSec, flags a changed value immediately and an
    // unchanged one at latest every (maxSkipCycles + 1) polls (its skip budget), and wakes
    // this task via ulp_lp_core_wakeup_main_processor() -- see
    // lp_sensor_core_wait_for_wake(). This task just services those wakes: read LP's state
    // → publish (if flagged) → wait-for-idle → block again. The wait's timeout is purely a
    // safeguard for the wake mechanism failing (see safeguardWakeSec()). Light sleep itself
    // is fully automatic (see enableAutomaticLightSleep(), main/common_utils.cpp) -- this
    // task never calls any sleep API; it just blocks, which is what lets the CPU
    // idle-sleep underneath it.

    // Why the wait at the bottom of the loop returned. LpWake covers both a real LP wake
    // and the first pass at boot (a slow boot must never be misread as an LP stall). The
    // two timeout kinds differ in what they imply: only a full safeguard window without a
    // single LP poll says the LP core stalled -- the short OTA-retry wait says nothing.
    enum class WakeReason { LpWake, OtaRetryTimeout, SafeguardTimeout };
    WakeReason wakeReason = WakeReason::LpWake;

    while (true) {
        // Whether data actually reached the broker this cycle. Stays false unless a publish was
        // started AND mqtt_last_publish_succeeded() confirms a connected, ACKed state message.
        bool publishedOk = false;
        // A wake that legitimately had nothing to publish -- attached and quiet counts as a
        // healthy cycle below; anything else without a confirmed publish counts as a failure.
        bool quietCycle = false;
        // Set only in the confirmed-LP-stall branch below -- the sole trigger for a reboot
        // (see LP_STALL_REBOOT_THRESHOLD). lpProgressedThisCycle is the disproof: any real LP
        // heartbeat advance this cycle, whether or not attached/publish succeeded. Both stay
        // false on a !attached cycle -- not being attached says nothing about LP's health, so
        // the stall counter is simply left alone until we're attached again to actually check.
        bool lpStalledThisCycle = false;
        bool lpProgressedThisCycle = false;
        // Don't read/publish until attached as CHILD. A plain blocking wait -- under automatic
        // light sleep this is safe to block through: OpenThread's own PM lock
        // (esp_openthread_sleep.c) keeps the CPU awake whenever the radio is actively
        // scanning/attaching, and only lets it idle-sleep once the radio itself is idle. If
        // the network is absent the gate times out and we fall through to one cycle and retry
        // next wake.
        const bool attached = !m_attachGate || m_attachGate(ATTACH_TIMEOUT_MS);

        if (!attached) {
            ESP_LOGW(TAG, "OT not attached within %u ms, retrying next cycle", ATTACH_TIMEOUT_MS);
            // A failed attach is the most direct symptom of a TX power too low to sustain the
            // uplink -- it happens before the publish logic below is ever reached, so it needs
            // its own tap into the confirm/revert counter (see runtime_config.cpp's design note).
            runtime_config_tx_power_note_cycle_result(false);
            if (m_noteCycleResult)
                m_noteCycleResult(false);
        } else {
            lp_shared_state_t state{};
            lp_sensor_core_get_state(&state);

            // How many real LP timer cycles elapsed since the last HP wake. LP's own ULP
            // timer re-arms itself for lp_poll_interval_sec every invocation (see
            // components/lp_sensor_core), so this reads exactly 1 on an LP-triggered wake,
            // 0 on a wake LP did not produce (a spurious latched wake, or a safeguard
            // timeout with LP stalled), and larger when several polls passed unflagged.
            const uint32_t heartbeatDelta = state.heartbeat_counter - m_lastSeenHeartbeat;
            ESP_LOGD(TAG, "HP wake: LP heartbeat=%lu (delta %lu since last wake), sensor_ok=%d, should_wake_hp=%d",
                     static_cast<unsigned long>(state.heartbeat_counter),
                     static_cast<unsigned long>(heartbeatDelta),
                     state.sensor_ok != 0, state.should_wake_hp != 0);
            m_lastSeenHeartbeat = state.heartbeat_counter;
            lpProgressedThisCycle = heartbeatDelta > 0;

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

            // A set flag is only actionable if LP has actually run since the last wake we
            // serviced: LP rewrites should_wake_hp once per poll, so with no LP progress a
            // set flag is the already-handled leftover of the previous wake. Without this
            // gate a latched ULP wake request (LP fires while HP is still awake publishing;
            // the request then pops the next light sleep immediately) re-published
            // identical values seconds apart -- seen in the ota-logs-5 hardware capture.
            const bool shouldWake = heartbeatDelta > 0 && state.should_wake_hp != 0;

            if (ota_session_in_progress()) {
                // An OTA download owns the radio and the publish path — skip ALL sensor work
                // this wake: no ADC power-domain (MODEM/TOP) churn mid-download, no doomed
                // publish attempt, no log spam. LP's baseline stays unacked, so a flagged
                // value simply publishes after the OTA ends (or after the reboot it leads to).
                ESP_LOGI(TAG, "OTA download in progress — skipping sensor work this wake");
                quietCycle = true;
            } else if (wakeReason == WakeReason::SafeguardTimeout && heartbeatDelta == 0) {
                // A whole safeguard window without a single LP poll: the LP timer/program
                // is stalled. (A heater run can't look like this -- heartbeat_counter is
                // bumped before the multi-minute heater block.) Deliberately NOT quiet, so
                // the reboot supervisor below eventually restarts the chip, which reloads
                // and restarts the LP binary -- the only recovery path for a dead LP core.
                ESP_LOGE(TAG, "no LP progress across a full safeguard window (heartbeat stuck at %lu) — LP core stalled?",
                         static_cast<unsigned long>(state.heartbeat_counter));
                lpStalledThisCycle = true;
            } else if (shouldWake || ota_update_due()) {
                if (shouldWake && wakeReason == WakeReason::SafeguardTimeout)
                    ESP_LOGW(TAG, "LP flagged data but its wake never arrived — recovered by the safeguard timeout");
                // Either LP flagged data, or a staged update is pending — the latter turns
                // any wake without flagged data into an "OTA-only" cycle (empty values, no
                // ADC read), and the wait below shortens to one LP poll while an update is
                // pending so such a cycle starts promptly: waiting for LP's change detector
                // cost up to ~3 min of dead time per interrupted OTA session in the v4
                // hardware test.
                const bool otaOnly = !shouldWake;
                SensorsValues v{};
                if (otaOnly) {
                    ESP_LOGI(TAG, "firmware update pending — starting an OTA-only publish cycle");
                } else {
                    v.envTemperature = state.cal_temp_c;
                    v.envHumidity = state.cal_hum_pct;

                    // Both absent until the LP core has ever completed a heater run --
                    // mirrors the battery fields' "no data yet" convention below.
                    if (state.last_heater_run_cycle != 0) {
                        v.heaterProblem = state.last_heater_passed == 0;
                        v.heaterRunCount = state.heater_run_count;
                    }

                    // Battery is a passenger on this already-decided publish -- it is read here,
                    // and only here, so it can never wake HP or trigger a send by itself. Create ->
                    // read -> delete strictly inside this awake window: with
                    // CONFIG_PM_POWER_DOWN_PERIPHERAL_IN_LIGHT_SLEEP, adc_oneshot_new_unit() pins
                    // the MODEM+TOP power domains ON across light sleep until
                    // adc_oneshot_del_unit(), so holding the unit while blocked in
                    // lp_sensor_core_wait_for_wake() would silently raise sleep current every
                    // cycle. Any failure degrades to publishing without the battery fields (HA
                    // then keeps the entities' previous values).
                    if (m_settings.readVoltageViaAdc) {
                        if (initAdc() == ESP_OK) {
                            if (const auto milliVolts = readBatteryVoltageMilliV()) {
                                v.batteryVoltageMilliV = *milliVolts;
                                v.batteryPercent = SensorsValues::convertVoltageToPercent(*milliVolts);
                                ESP_LOGI(TAG, "battery: %d mV (%.2f%%)", *milliVolts, *v.batteryPercent);
                            } else {
                                ESP_LOGW(TAG, "battery ADC read failed (err=0x%x), publishing without battery",
                                         milliVolts.error());
                            }
                            deinitAdc();
                        } else {
                            ESP_LOGW(TAG, "battery ADC init failed, publishing without battery");
                        }
                    }

                    ESP_LOGI(TAG, "publishing LP-flagged value: %.2fC / %.2f%%RH",
                             static_cast<double>(state.cal_temp_c), static_cast<double>(state.cal_hum_pct));
                }

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
                        runtime_config_tx_power_note_cycle_result(publishedOk);
                        if (m_noteCycleResult)
                            m_noteCycleResult(publishedOk);
                        // Only ack on CONFIRMED delivery -- an attempted-but-failed publish
                        // must leave LP's baseline untouched, so the still-undelivered value
                        // keeps being flagged next cycle instead of silently getting dropped.
                        // An OTA-only cycle carries no sensor values, so it must not ack
                        // either: that would shift LP's baseline and re-arm its skip budget
                        // for a snapshot the broker never saw, postponing the guaranteed
                        // (maxSkipCycles + 1)-poll forced publish.
                        if (publishedOk) {
                            if (!otaOnly)
                                lp_sensor_core_ack_delivered(state.cal_temp_c, state.cal_hum_pct);
                            markAppValidOnFirstConfirmedPublish();
                        } else if (!otaOnly) {
                            // Broker unreachable (or its ACK never arrived) for a cycle that had
                            // a real reading -- preserve it for replay once the broker's back
                            // (see history_log.h). LP's baseline stays unacked regardless (its
                            // ack call is skipped above), so the CURRENT value also keeps being
                            // retried live every cycle -- this only protects what would
                            // otherwise be silently overwritten by LP's next poll meanwhile.
                            history_log_append(state.cal_temp_c, state.cal_hum_pct);
                        }
                    } else if (ota_session_in_progress()) {
                        // An OTA download legitimately owns the publish task for minutes;
                        // this is not a stall (see the cycleOk exemption below).
                        ESP_LOGI(TAG, "OTA download in progress — leaving the publish window open");
                    } else {
                        ESP_LOGW(TAG, "publish did not finish within %u ms, sleeping anyway", PUBLISH_TIMEOUT_MS);
                        runtime_config_tx_power_note_cycle_result(false);
                        if (m_noteCycleResult)
                            m_noteCycleResult(false);
                    }
                } else {
                    ESP_LOGW(TAG, "mqtt is not working? not sure");
                }
            } else {
                // Nothing to publish: a spurious latched wake (heartbeatDelta == 0), or LP
                // alive but with nothing flagged. On a safeguard timeout the latter is the
                // steady state of a broken sensor (LP deliberately flags only the OK→fail
                // transition), and HA's expire_after (2 × the safeguard window) is the
                // honest "no data" signal there -- no reboot, it wouldn't fix I2C. The
                // common cases stay ESP_LOGD, not I: a synchronous UART/USB-JTAG write on
                // every quiet cycle is real, avoidable awake time.
                if (wakeReason == WakeReason::SafeguardTimeout)
                    ESP_LOGW(TAG, "safeguard timeout: LP alive (%lu polls since last wake) but nothing to publish",
                             static_cast<unsigned long>(heartbeatDelta));
                else
                    ESP_LOGD(TAG, "LP has nothing new to report this wake");
                quietCycle = true;
            }
        }

        // "Delivered, or legitimately nothing to deliver" both count as a healthy cycle --
        // mirrors the original HP-only code's (publishedOk || skipSameValuesCycle) gate. Not
        // attaching at all is never healthy, even if LP had nothing to say; neither is an LP
        // stall (quietCycle stays false there). An in-flight OTA download also counts as
        // healthy: it blocks the publish path for minutes by design, and letting the bad-OTA
        // safety net below fire mid-download would reboot (and with rollback enabled, roll
        // back) a perfectly good update in progress.
        const bool cycleOk = publishedOk || (attached && quietCycle) || ota_session_in_progress();

        // Honest local indicator: 1 blink = data reached the broker, 5 = should have
        // published but didn't. No LED at all for a quiet cycle -- not worth forced-awake
        // LED time for an unattended deployed sensor; the serial log lines above still
        // cover it for bench debugging.
        if (publishedOk) {
            blinkUserLED(LED_BLINK_MS);
        } else if (!cycleOk) {
            ESP_LOGW(TAG, "data was not published to the broker this cycle (but should be)");
            blinkUserLED(LED_BLINK_MS, 5);
        }

        // Recovery supervisor: reboot ONLY on a confirmed LP-core stall (lpStalledThisCycle) --
        // link-down and broker-unreachable cycles retry forever at the existing cadence instead
        // (see LP_STALL_REBOOT_THRESHOLD's doc comment for why: neither has a local fix a
        // reboot can provide, and the retry loop already notices recovery on its own). The one
        // local fault broker-unreachable CAN have -- a stale NAT64 prefix -- still gets a
        // softer nudge here, without ever counting toward a reboot.
        if (attached && !cycleOk && m_refreshNat64)
            m_refreshNat64();

        if (lpStalledThisCycle) {
            if (++m_lpStalledCycles >= LP_STALL_REBOOT_THRESHOLD) {
                ESP_LOGE(TAG, "%lu consecutive LP-core-stall wakes — rebooting to recover",
                         static_cast<unsigned long>(m_lpStalledCycles));
                markLpStallReboot();  // next boot reports "lp_stall_reboot", not "sw_reset"
                esp_restart();
            }
        } else if (lpProgressedThisCycle) {
            m_lpStalledCycles = 0;
        }

        // Bad-OTA safety net (see UNCONFIRMED_OTA_REBOOT_AFTER_CYCLES): unlike the LP-stall
        // trigger above, this fires on ANY unhealthy cycle -- but only while the running image
        // has never confirmed itself, so it can't reintroduce a reboot storm for an
        // already-trusted image sitting through an ordinary blackout.
        esp_ota_img_states_t otaState;
        const bool stillUnconfirmed =
            esp_ota_get_state_partition(esp_ota_get_running_partition(), &otaState) == ESP_OK &&
            otaState == ESP_OTA_IMG_PENDING_VERIFY;
        if (stillUnconfirmed && !cycleOk) {
            if (++m_cyclesUnconfirmedAndFailing >= UNCONFIRMED_OTA_REBOOT_AFTER_CYCLES) {
                ESP_LOGE(TAG, "%lu consecutive unhealthy cycles with the image still unconfirmed "
                         "— rebooting so the bootloader can roll back",
                         static_cast<unsigned long>(m_cyclesUnconfirmedAndFailing));
                markOtaUnconfirmedReboot();  // next boot reports "ota_unconfirmed", not "sw_reset"
                esp_restart();
            }
        } else {
            m_cyclesUnconfirmedAndFailing = 0;
        }

        // Block until the LP core flags something (see lp_sensor_core_wait_for_wake()) or
        // the safeguard timeout says its wake mechanism broke. With a staged OTA update
        // pending, wait only one LP poll instead: waiting for LP's change detector cost up
        // to a full skip budget of dead time per interrupted OTA session in the v4 hardware
        // test, while one poll still bounds the retry loop if the OTA-only cycle keeps
        // failing. This is a plain blocking wait -- ESP-IDF's automatic tickless-idle light
        // sleep (enableAutomaticLightSleep()) transparently sleeps the CPU underneath it
        // whenever no esp_pm lock (e.g. OpenThread's own radio-state lock) says otherwise.
        const bool otaRetryWait = ota_update_due() && !ota_session_in_progress();
        const uint32_t waitSec = otaRetryWait
            ? m_settings.lpPollIntervalSec
            : safeguardWakeSec(m_settings.lpPollIntervalSec, m_settings.maxSkipCycles);
        if (lp_sensor_core_wait_for_wake(waitSec * 1000))
            wakeReason = WakeReason::LpWake;
        else
            wakeReason = otaRetryWait ? WakeReason::OtaRetryTimeout : WakeReason::SafeguardTimeout;
    }
}

void SensorsTask::adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_delete_scheme_curve_fitting(handle);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_delete_scheme_line_fitting(handle);
#endif
}

bool SensorsTask::adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    static const char * TAG = "ADC-calibration";

    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGD(TAG, "calibration scheme version is Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGD(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

esp_err_t SensorsTask::initAdc()
{
    static const char * TAG = "ADC-init";

    // Must be the same in the channel config and the calibration init. The divider midpoint
    // peaks at the ~4.2 V charger rail / m_voltageDividerCoefficient ≈ 2.1 V, so 12 dB is the
    // only attenuation whose range covers it -- 6 dB tops out around 1.7 V on the C6 and would
    // clip whenever the battery sits above ~3.5 V.
    const adc_atten_t ADC_ATTENUATION = ADC_ATTEN_DB_12;

    // Channel follows VOLTAGE_PIN so the pin constant stays the single source of truth.
    adc_unit_t unit = ADC_UNIT_1;
    if (const esp_err_t err = adc_oneshot_io_to_channel(VOLTAGE_PIN, &unit, &m_adcChannel);
        err != ESP_OK || unit != ADC_UNIT_1) {
        ESP_LOGW(TAG, "GPIO%d is not an ADC1 pin (err=0x%x)", VOLTAGE_PIN, err);
        return err != ESP_OK ? err : ESP_FAIL;
    }

    // No ESP_ERROR_CHECK here: a battery-measurement failure must degrade to "publish without
    // battery fields" (the caller logs and moves on), never abort the whole sensor node.

    //-------------ADC1 Init---------------//
    const adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
        .clk_src = ADC_DIGI_CLK_SRC_XTAL,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };
    if (const esp_err_t err = adc_oneshot_new_unit(&init_config1, &adc1_handle); err != ESP_OK) {
        ESP_LOGW(TAG, "adc_oneshot_new_unit failed: 0x%x", err);
        adc1_handle = nullptr;
        return err;
    }

    //-------------ADC1 Config---------------//
    const adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTENUATION,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (const esp_err_t err = adc_oneshot_config_channel(adc1_handle, m_adcChannel, &config); err != ESP_OK) {
        ESP_LOGW(TAG, "adc_oneshot_config_channel failed: 0x%x", err);
        deinitAdc();
        return err;
    }

    //-------------ADC1 Calibration Init---------------//
    if (!adc_calibration_init(ADC_UNIT_1, m_adcChannel, ADC_ATTENUATION, &adc1_cali_chan0_handle)) {
        deinitAdc();
        return ESP_FAIL;
    }
    return ESP_OK;
}

void SensorsTask::deinitAdc()
{
    if (adc1_cali_chan0_handle) {
        adc_calibration_deinit(adc1_cali_chan0_handle);
        adc1_cali_chan0_handle = nullptr;
    }
    if (adc1_handle) {
        adc_oneshot_del_unit(adc1_handle);
        adc1_handle = nullptr;
    }
}

std::expected<int, esp_err_t> SensorsTask::readBatteryVoltageMilliV()
{
    static const char * TAG = "ADC-measure";

    int adc_raw = 0;
    int voltage = 0;
    int voltage_first = 0;
    int voltage_min = 0;
    int voltage_max = 0;
    int32_t voltage_sum = 0;
    for (size_t i = 0; i < ADC_READS_COUNT; ++i) {
        if (const esp_err_t adcReadError = adc_oneshot_read(adc1_handle, m_adcChannel, &adc_raw);
            adcReadError != ESP_OK) {
            ESP_LOGE(TAG, "ADC reading error %d", adcReadError);
            return std::unexpected(adcReadError);
        }

        if (const esp_err_t calibrationErr = adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_raw, &voltage);
            calibrationErr != ESP_OK) {
            ESP_LOGE(TAG, "ADC calibration error %d, raw value is %d", calibrationErr, adc_raw);
            return std::unexpected(calibrationErr);
        }
        if (i == 0)
            voltage_first = voltage_min = voltage_max = voltage;
        voltage_min = std::min(voltage_min, voltage);
        voltage_max = std::max(voltage_max, voltage);
        voltage_sum += voltage;
    }

    // A first-sample-low / rising-across-burst pattern here means the SAR sampling cap is being
    // starved by the divider's source impedance -- the diagnostic to watch when trying larger
    // (lower-drain) divider resistors; see device_config.yaml's battery_divider_r_*_ohm keys.
    ESP_LOGD(TAG, "burst spread: min=%d max=%d first=%d last=%d mV",
             voltage_min, voltage_max, voltage_first, voltage);

    const int pinVoltage = voltage_sum / static_cast<int32_t>(ADC_READS_COUNT);
    ESP_LOGD(TAG, "ADC pin voltage: %d mV", pinVoltage);

    return static_cast<int>(m_voltageDividerCoefficient * pinVoltage);
}