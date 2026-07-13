#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <functional>
#include <cstdint>
#include <expected>
#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <driver/gpio.h>

struct SensorsValues
{
public:
    std::optional<int> batteryVoltageMilliV;
    std::optional<int> batteryPercent;
    std::optional<float> envTemperature;
    std::optional<float> envHumidity;
    std::optional<float> barometricPressure;

    static constexpr double MAX_VOLTAGE = 4090.0; // mV — fully charged Li-ion (measured)
    static constexpr double MIN_VOLTAGE = 3200.0; // mV — empty (0 %)

    static int convertVoltageToPercent(int batteryVoltageMilliV);
};

struct SensorsTaskSettings
{
    /// HP backstop interval. The LP core (components/lp_sensor_core) now owns the real
    /// read/calibrate/threshold cadence (see calibration.txt's lp_poll_interval_sec) and
    /// wakes HP early via ulp_lp_core_wakeup_main_processor() whenever it has something
    /// worth publishing; this is just the ceiling on how stale published data can get if LP
    /// never flags a change.
    uint32_t cycleDurationSec = 60;
    /// should HP core read battery Voltage via ADC GPIO pin
    bool readVoltageViaAdc = false;
};

class SensorsTask
{
public:
    static constexpr uint32_t PUBLISH_TIMEOUT_MS = 15 * 1000;
    // Flash-detection research suggested ~15-20ms as a safe floor, but this LED is bright
    // enough that 5ms is still clearly visible on the actual hardware (confirmed by eye) --
    // going shorter than the research's nominal threshold is fine given real brightness margin.
    static constexpr uint32_t LED_BLINK_MS       = 5;


    SensorsTask(SensorsTaskSettings settings);
    SensorsTask(const SensorsTask &) = delete;
    SensorsTask &operator=(const SensorsTask &) = delete;

    void executeTask();

    using SensorsReadyEvent = std::function<void(const SensorsValues &values)>;
    void configureReadyEvent(SensorsReadyEvent readyEvent);

    /// Blocks up to the given ms for Thread attachment; returns true once attached.
    /// Injected by main so the task gates each cycle without depending on OpenThread directly.
    using AttachGate = std::function<bool(uint32_t timeoutMs)>;
    void configureAttachGate(AttachGate attachGate);

    /// Re-reads Thread network data so a changed NAT64 prefix is picked up without a reboot.
    /// Injected by main (it owns the OpenThread instance); called after a failed publish cycle.
    using RefreshNat64 = std::function<void()>;
    void configureRefreshNat64(RefreshNat64 refreshNat64);

private:
    /// per-cycle awake budget to (re)attach before sleeping anyway
    static constexpr uint32_t ATTACH_TIMEOUT_MS = 30 * 1000;

    /// Recovery: reboot after this many consecutive cycles without a successful publish. A transient
    /// reachability loss (stale NAT64 prefix, broker blip) otherwise persists forever; rebooting
    /// re-attaches and re-learns the NAT64 route. ~5 cycles ≈ 5 min of no data before recovering.
    static constexpr uint32_t REBOOT_AFTER_FAILS = 5;

    /// Voltage section
    static constexpr gpio_num_t VOLTAGE_PIN = GPIO_NUM_2;
    static constexpr double RESISTOR_GND_2_SENSOR = 5035;
    static constexpr double RESISTOR_SENSOR_2_VBAT = 5021;
    static constexpr double voltageDividerCoefficient = (RESISTOR_GND_2_SENSOR + RESISTOR_SENSOR_2_VBAT) / RESISTOR_GND_2_SENSOR;
    static constexpr size_t ADC_READS_COUNT = 10;

    const SensorsTaskSettings m_settings;

    SensorsReadyEvent m_readyEvent;
    AttachGate m_attachGate;
    RefreshNat64 m_refreshNat64;

    adc_oneshot_unit_handle_t adc1_handle = nullptr;
    adc_cali_handle_t adc1_cali_chan0_handle = nullptr;

    /// consecutive cycles with no successful publish; drives the reboot supervisor (see REBOOT_AFTER_FAILS)
    uint32_t m_consecutiveFailures = 0;

    /// last heater run LP reported (heartbeat_counter value at the time), so the post-hoc
    /// diagnostic log line only fires once per new run rather than every wake.
    uint32_t m_lastSeenHeaterRunCycle = 0;

    /// LP's heartbeat_counter as of the last HP wake, so each new wake can log how many real
    /// LP cycles elapsed since then.
    uint32_t m_lastSeenHeartbeat = 0;

    [[nodiscard("false means ADC is uncalibrated")]]
    static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
    static void adc_calibration_deinit(adc_cali_handle_t handle);
    [[nodiscard("ADC unavailable if init failure ignored")]]
    esp_err_t initAdc();
    void deinitAdc();
    std::expected<int, esp_err_t> readBatteryVoltageMilliV();
};
