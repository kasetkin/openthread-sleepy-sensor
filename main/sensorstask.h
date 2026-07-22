#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <functional>
#include <cstdint>
#include <expected>
#include <array>
#include <algorithm>
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

    /// 0 % / 100 % anchors for the CURRENT pack (LiitoKala NCR18650B), applied to the
    /// pack-independent BATTERY_CURVE below at conversion time; after a battery swap only these
    /// two numbers change, never the curve. Planned to become MQTT-runtime settings.
    /// MIN: 0 % must land while the XIAO's LDO is still near regulation -- at ~3.2 V battery the
    /// 3.3 V rail already sits at the C6's 3.0 V floor and a TX-burst sag risks brownout; the
    /// board has no over-discharge cutoff, and the knee below 3.4 V holds only ~2-3 % capacity.
    /// MAX: measured resting full charge (4.124 V settled, minus ~10 mV publish-window load sag).
    /// Deliberately NOT the 4.20 V charger CV -- that voltage exists only on the charger, and
    /// anchoring there would cap the gauge at ~92 % forever once the surface charge settles
    /// (Meshtastic's 4190 mV default accepts that tradeoff; we calibrate to the pack instead).
    static constexpr int MIN_VOLTAGE = 3300; // mV → 0 %
    static constexpr int MAX_VOLTAGE = 4120; // mV → 100 %

    static constexpr int convertVoltageToPercent(int batteryVoltageMilliV,
                                                 int minMv = MIN_VOLTAGE, int maxMv = MAX_VOLTAGE);

private:
    struct BatteryCurvePoint
    {
        int mv;
        int pct;
    };

    /// Etalon (reference) resting-voltage discharge curve of a textbook 4.20 V/cell 1S Li-ion --
    /// the commonly published OCV↔SoC table. Pack-INDEPENDENT: never edit this for a specific
    /// battery; per-pack calibration lives only in the MIN_VOLTAGE/MAX_VOLTAGE anchors, which
    /// re-normalize this curve's shape in convertVoltageToPercent().
    static constexpr std::array<BatteryCurvePoint, 21> BATTERY_CURVE{{
        {3270,   0}, {3610,   5}, {3690,  10}, {3710,  15}, {3730,  20}, {3750,  25},
        {3770,  30}, {3790,  35}, {3800,  40}, {3820,  45}, {3840,  50}, {3850,  55},
        {3870,  60}, {3910,  65}, {3950,  70}, {3980,  75}, {4020,  80}, {4080,  85},
        {4110,  90}, {4150,  95}, {4200, 100},
    }};
    // Interpolation below requires both columns increasing; the endpoint asserts pin the curve to
    // the standard 3.27-4.20 V reference span so a "helpful" pack-specific edit fails the build.
    static_assert(std::ranges::is_sorted(BATTERY_CURVE, {}, &BatteryCurvePoint::mv));
    static_assert(std::ranges::is_sorted(BATTERY_CURVE, {}, &BatteryCurvePoint::pct));
    static_assert(BATTERY_CURVE.front().mv == 3270 && BATTERY_CURVE.front().pct == 0);
    static_assert(BATTERY_CURVE.back().mv == 4200 && BATTERY_CURVE.back().pct == 100);

    static constexpr double etalonSoc(int mv);
};

/// SoC of the reference 4.20 V cell at this voltage, in [0.0, 100.0] -- fractional precision is
/// needed by the anchor normalization in convertVoltageToPercent().
constexpr double SensorsValues::etalonSoc(const int mv)
{
    if (mv <= BATTERY_CURVE.front().mv)
        return BATTERY_CURVE.front().pct;
    if (mv >= BATTERY_CURVE.back().mv)
        return BATTERY_CURVE.back().pct;

    // First curve point with mv >= input; the clamps above guarantee it and its predecessor exist.
    const auto hi = std::ranges::lower_bound(BATTERY_CURVE, mv, {}, &BatteryCurvePoint::mv);
    const auto lo = hi - 1;
    return lo->pct + static_cast<double>(mv - lo->mv) * (hi->pct - lo->pct) / (hi->mv - lo->mv);
}

constexpr int SensorsValues::convertVoltageToPercent(const int batteryVoltageMilliV,
                                                     const int minMv, const int maxMv)
{
    // Misconfiguration guards rather than asserts: min/max will eventually arrive from a runtime
    // MQTT setting, so a degenerate or inverted range must degrade safely, not crash.
    if (maxMv <= minMv)
        return 0;
    const double socMin = etalonSoc(minMv);
    const double socMax = etalonSoc(maxMv);
    if (socMax <= socMin)  // both anchors clamped onto the same etalon endpoint
        return 0;

    // The etalon curve contributes only the SHAPE; the anchors set the scale (minMv → 0 %,
    // maxMv → 100 %). Inputs outside [minMv, maxMv] land outside [0, 100] and are clamped.
    const double normalized = (etalonSoc(batteryVoltageMilliV) - socMin) / (socMax - socMin) * 100.0;
    return static_cast<int>(std::clamp(normalized, 0.0, 100.0) + 0.5);
}

// Compile-time unit tests: the function is pure and there is no on-target test runner, so every
// build verifies the clamping, interpolation and anchor normalization directly.
static_assert(SensorsValues::convertVoltageToPercent(3000) == 0);    // below min anchor -> clamped
static_assert(SensorsValues::convertVoltageToPercent(3300) == 0);    // exact min anchor
static_assert(SensorsValues::convertVoltageToPercent(3610) == 5);    // knee region
static_assert(SensorsValues::convertVoltageToPercent(3840) == 55);   // plateau
static_assert(SensorsValues::convertVoltageToPercent(4022) == 88);   // value seen in 2026-07-13 hardware log
static_assert(SensorsValues::convertVoltageToPercent(4110) == 99);
static_assert(SensorsValues::convertVoltageToPercent(4120) == 100);  // exact max anchor
static_assert(SensorsValues::convertVoltageToPercent(4200) == 100);  // above max anchor -> clamped
// Anchor parameters: defaults wired through, custom anchors honored, degenerate range guarded.
static_assert(SensorsValues::convertVoltageToPercent(3840)
              == SensorsValues::convertVoltageToPercent(3840, SensorsValues::MIN_VOLTAGE, SensorsValues::MAX_VOLTAGE));
static_assert(SensorsValues::convertVoltageToPercent(4090, 3300, 4090) == 100);
static_assert(SensorsValues::convertVoltageToPercent(3700, 3300, 3300) == 0);

struct SensorsTaskSettings
{
    /// LP sensor-read cadence (device_config.yaml's lp_poll_interval_sec). The LP core
    /// (components/lp_sensor_core) owns the whole read/calibrate/threshold loop and wakes HP
    /// via ulp_lp_core_wakeup_main_processor() whenever it has something worth publishing;
    /// HP sets no publish cadence of its own — see SensorsTask::safeguardWakeSec() for the
    /// only timeout it applies.
    uint32_t lpPollIntervalSec = 20;
    /// LP's skip budget (device_config.yaml's max_skip_cycles): an unchanged value may be
    /// skipped at most this many LP polls before LP flags it anyway, so a publish is
    /// guaranteed at latest every (this + 1) × lpPollIntervalSec after the last ACKed one.
    uint32_t maxSkipCycles = 0;
    /// should HP core read battery voltage via the ADC GPIO pin right before each publish
    bool readVoltageViaAdc = false;
    /// Measured voltage-divider resistors in Ohms (battery+ → ADC pin, and ADC pin → battery−).
    /// Defaults are the resistors soldered on this board; override via device_config.yaml
    /// (battery_divider_r_vbat_ohm / battery_divider_r_gnd_ohm) after rewiring — e.g. for the
    /// planned high-impedance (2×200 kΩ) drain-reduction experiment.
    double batteryDividerRVbatOhm = 5021.0;
    double batteryDividerRGndOhm  = 5035.0;
};

class SensorsTask
{
public:
    static constexpr uint32_t PUBLISH_TIMEOUT_MS = 15 * 1000;
    // Flash-detection research suggested ~15-20ms as a safe floor, but this LED is bright
    // enough that 5ms is still clearly visible on the actual hardware (confirmed by eye) --
    // going shorter than the research's nominal threshold is fine given real brightness margin.
    static constexpr uint32_t LED_BLINK_MS       = 5;

    /// HP's wait-for-LP-wake timeout. LP guarantees a wake at latest every
    /// (maxSkip + 1) polls (its skip budget forces a flag once exhausted), so still being
    /// asleep two polls past that means the ULP wake was lost or the LP core stalled — this
    /// is a safeguard against that mechanism failing, never a publish cadence. main.cpp
    /// derives the HA sensors' expire_after as 2 × this, so entities only go unavailable
    /// after a whole extra safeguard window has passed with nothing delivered either.
    static constexpr uint32_t safeguardWakeSec(uint32_t pollSec, uint32_t maxSkip)
    {
        return (maxSkip + 3) * pollSec;
    }

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
    /// Recovery: reboot after this many CONSECUTIVE confirmed LP-core-stall wakes (a full
    /// safeguard window with zero LP heartbeat progress -- see executeTask()). This is the
    /// ONLY thing that can trigger esp_restart() -- link-down and broker-unreachable cycles
    /// never reach this counter: neither has a local fix a reboot can provide (no network to
    /// join; or a remote process being down, which rebooting THIS chip can't touch), and the
    /// existing per-cycle retry already notices recovery on its own regardless of any reboot.
    /// The LP core is the one genuine local firmware wedge a restart addresses -- it reloads
    /// and restarts the LP binary (lp_sensor_core_init/start run in app_main()).
    static constexpr uint32_t LP_STALL_REBOOT_THRESHOLD = 2;

    /// Bad-OTA safety net: reboot after this many CONSECUTIVE unhealthy cycles (!cycleOk --
    /// not attached, broker unreachable, or an LP stall) while the running image is still
    /// unconfirmed (ESP_OTA_IMG_PENDING_VERIFY -- see markAppValidOnFirstConfirmedPublish(),
    /// sensorstask.cpp). Unlike LP_STALL_REBOOT_THRESHOLD above, this fires on ANY of those
    /// causes: a freshly-flashed image that can never confirm itself has no other way back,
    /// and this reboot doubles as the rollback trigger (the bootloader falls back to the
    /// previous slot automatically, since esp_ota_mark_app_valid_cancel_rollback() was never
    /// called). Once an image confirms itself (any successful publish) this becomes
    /// permanently irrelevant for the rest of that boot. Same value as the pre-redesign
    /// REBOOT_AFTER_FAILS, since this inherits exactly the safety-net role that constant used
    /// to serve for every image, not just an unconfirmed one.
    static constexpr uint32_t UNCONFIRMED_OTA_REBOOT_AFTER_CYCLES = 5;

    /// per-cycle awake budget to (re)attach before sleeping anyway
    static constexpr uint32_t ATTACH_TIMEOUT_MS = 30 * 1000;

    /// Voltage section
    static constexpr gpio_num_t VOLTAGE_PIN = GPIO_NUM_2;
    static constexpr size_t ADC_READS_COUNT = 10;

    const SensorsTaskSettings m_settings;
    /// battery_mV = pin_mV × this; derived once from the configured divider resistors
    const double m_voltageDividerCoefficient;

    SensorsReadyEvent m_readyEvent;
    AttachGate m_attachGate;
    RefreshNat64 m_refreshNat64;

    adc_oneshot_unit_handle_t adc1_handle = nullptr;
    adc_cali_handle_t adc1_cali_chan0_handle = nullptr;
    /// derived from VOLTAGE_PIN in initAdc() so the pin constant stays the single source of truth
    adc_channel_t m_adcChannel = ADC_CHANNEL_2;

    /// consecutive confirmed LP-core-stall wakes; drives the reboot supervisor (see
    /// LP_STALL_REBOOT_THRESHOLD). Reset on any real LP heartbeat progress, not just a
    /// successful publish -- a live LP core disproves the stall theory regardless of whether
    /// the broker happened to be reachable that cycle.
    uint32_t m_lpStalledCycles = 0;

    /// consecutive unhealthy cycles while the running image is still unconfirmed; drives the
    /// bad-OTA safety net (see UNCONFIRMED_OTA_REBOOT_AFTER_CYCLES).
    uint32_t m_cyclesUnconfirmedAndFailing = 0;

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
