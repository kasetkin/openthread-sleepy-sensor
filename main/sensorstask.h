#pragma once

#include <cmath>
#include <numeric>
#include <optional>
#include <expected>
#include <memory>
#include <string>
#include <string_view>
#include <functional>
#include <esp_err.h>
#include <driver/gpio.h>
#include <driver/temperature_sensor.h>
#include "i2cdev.h"
#include "sht3x.h"
#include "sht4x.h"

struct SensorsValues
{
public:
    std::optional<float> envTemperature;
    std::optional<float> envHumidity;
    std::optional<float> barometricPressure;

    std::string toTelemetryString() const;
    std::string toLogString() const;
    /// up to 3 decimal digits; trailing zeros and dot stripped
    static std::string toTelemetryRoundedString(const float value);
};

/// Which environment sensor backend was selected at init. Probed in order
/// SHT4x → SHT3X → internal CPU temperature; the first to respond wins.
enum class SensorBackend
{
    None,
    Sht4x,
    Sht3x,
    InternalTemp,
};

/// One environment reading. Humidity is optional because the internal CPU
/// temperature fallback has no humidity channel (stays std::nullopt there).
struct EnvReading
{
    std::optional<float> temperature;
    std::optional<float> humidity;
};

class SensorsTask
{
public:
    ~SensorsTask();
    SensorsTask() = default;
    SensorsTask(const SensorsTask &) = delete("SensorsTask owns I2C device handles — copying aliases hardware resources");
    SensorsTask &operator=(const SensorsTask &) = delete("SensorsTask owns I2C device handles — copying aliases hardware resources");

    [[nodiscard("sensors unavailable if init failure ignored")]]
    esp_err_t init();

    void executeTask();

    /// Single dispatch point for a reading from the active backend. Temperature is always
    /// present on success; humidity is std::nullopt for the internal-temp fallback.
    [[nodiscard("reading discarded on failure")]]
    std::expected<EnvReading, esp_err_t> readEnvironment();

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

    /// Per-device calibration offsets added to each raw reading before publishing.
    /// Sourced from calibration.txt and injected by main (default 0 = no correction).
    void configureCalibration(float rhOffset, float tempOffset);

private:
    /// single source of truth for the wake→read→publish→sleep cadence
    static constexpr uint32_t SENSORS_PERIOD_MS = 60 * 1000;

    /// per-cycle awake budget to (re)attach before sleeping anyway; > typical attach time and SENSORS_PERIOD_MS
    static constexpr uint32_t ATTACH_TIMEOUT_MS = 30 * 1000;

    /// Recovery: reboot after this many consecutive cycles without a successful publish. A transient
    /// reachability loss (stale NAT64 prefix, broker blip) otherwise persists forever; rebooting
    /// re-attaches and re-learns the NAT64 route. ~5 cycles ≈ 5 min of no data before recovering.
    static constexpr uint32_t REBOOT_AFTER_FAILS = 5;

    /// --- heater maintenance / plausibility self-test (see Sensirion docs in docs/) ---
    /// periodic cadence (~24 h at SENSORS_PERIOD_MS): routine plausibility check + creep mitigation
    static constexpr uint32_t HEATER_PERIODIC_CYCLES = 24 * 60;
    /// condition trigger: run heater after sustained high humidity (evaluated on calibrated RH)
    static constexpr float    HIGH_RH_THRESHOLD       = 90.0f;   // %RH
    static constexpr uint32_t HIGH_RH_TRIGGER_CYCLES  = 60;      // ~1 h above threshold
    static constexpr uint32_t HEATER_DURATION_MS      = 20 * 1000; // max heating window (cap; SHT4x stops early on delta-T)
    /// plausibility pass / SHT4x early-stop target: heated T − baseline T ≥ this.
    /// SHT3X uses a gentle continuous heater (a few °C). The SHT4x HIGH_LONG heater is far more
    /// aggressive — measured rise after ~3 pulses is ~51 °C — so its threshold doubles as the
    /// "heated enough, stop pulsing" target (see runHeaterMaintenanceSht4X).
    static constexpr float    HEATER_MIN_DELTA_T_SHT3X_C = 1.5f;
    /// SHT4x heater profiles, chosen by trigger reason (see runHeaterMaintenanceSht4X):
    ///  - periodic self-test: gentle MEDIUM_SHORT pulse (~+16 C/pulse) + small delta-T floor —
    ///    just prove the heater + sensor are alive at minimal energy.
    ///  - sustained-high-RH creep mitigation: aggressive HIGH_LONG pulse (~+50 C/pulse) + large
    ///    delta-T — drive moisture out of the polymer (≈ Sensirion's 200 mW/1 s creep dose).
    static constexpr sht4x_heater_t HEATER_SHT4X_SELFTEST_MODE    = SHT4X_HEATER_MEDIUM_SHORT;
    static constexpr float          HEATER_SHT4X_SELFTEST_DELTA_T = 5.0f;
    static constexpr sht4x_heater_t HEATER_SHT4X_CREEP_MODE       = SHT4X_HEATER_HIGH_LONG;
    static constexpr float          HEATER_SHT4X_CREEP_DELTA_T    = 40.0f;
    static constexpr float    HEATER_MIN_RH_DROP      = 2.0f;    // pass if baseline RH − heated RH ≥ (SHT3X only)
    static constexpr float    COOLDOWN_EPSILON_C      = 0.3f;    // resume once T within this of baseline
    static constexpr uint32_t COOLDOWN_MAX_MS         = 240 * 1000; // cap on cooldown wait (SHT4x die can reach ~80 C; ~21 s time-constant ⇒ needs >110 s to settle)
    static constexpr uint32_t COOLDOWN_POLL_MS        = 2 * 1000;  // re-read interval while cooling
    /// LED indicator while heating: 5 blinks/second, 10 ms each
    static constexpr uint32_t HEATER_LED_ON_MS        = 10;
    static constexpr uint32_t HEATER_LED_BLINKS_PER_SEC = 5;

    /// ENVIRONMENT sensor, SHT31 via I2C bus
    static constexpr uint8_t SHT3X_ADDR = SHT3X_I2C_ADDR_GND; // 0x44
    static constexpr gpio_num_t I2C_MASTER_SDA = GPIO_NUM_22;
    static constexpr gpio_num_t I2C_MASTER_SCL = GPIO_NUM_23;
    static constexpr i2c_port_t SHT3X_I2C_PORT = I2C_NUM_0;
    
    bool m_i2cInitialized = false;
    SensorsReadyEvent m_readyEvent;
    AttachGate m_attachGate;
    RefreshNat64 m_refreshNat64;
    sht3x_t m_sht3dev;
    sht4x_t m_sht4dev;
    temperature_sensor_handle_t m_tsensHandle = nullptr;
    SensorBackend m_backend = SensorBackend::None;

    /// consecutive cycles with no successful publish; drives the reboot supervisor (see REBOOT_AFTER_FAILS)
    uint32_t m_consecutiveFailures = 0;

    /// maintenance scheduling counters (persist across light sleep — RAM is retained)
    uint32_t m_cyclesSinceHeater = 0;
    uint32_t m_highRhCycles = 0;
    /// calibration offsets injected via configureCalibration()
    float m_rhOffset = 0.0f;
    float m_tempOffset = 0.0f;

    [[nodiscard("I2C unavailable if init failure ignored")]]
    esp_err_t initI2C();
    void deinitI2C();

    /// Sensor probes, tried in order by initI2C(). Each returns the backend it claimed or
    /// the failing esp_err_t, and cleans up its own resources on failure so the next probe
    /// starts from a clean bus. SHT4x and SHT3X share I2C address 0x44 — SHT4x is tried first
    /// and its CRC-checked serial read rejects a clone/SHT3X cleanly, falling through to SHT3X.
    std::expected<SensorBackend, esp_err_t> probeSht4x();
    std::expected<SensorBackend, esp_err_t> probeSht3x();
    std::expected<SensorBackend, esp_err_t> probeInternalTemp();

    /// Human-readable name for the active backend (logging only).
    static constexpr std::string_view backendName(SensorBackend backend)
    {
        switch (backend) {
        case SensorBackend::Sht4x:        return "SHT4x";
        case SensorBackend::Sht3x:        return "SHT3x";
        case SensorBackend::InternalTemp: return "internal CPU temperature";
        case SensorBackend::None:         return "none";
        }
        return "unknown";
    }

    /// Heater plausibility self-test + creep/dew maintenance: baseline → heat (with LED
    /// pattern) → verify T↑/RH↓ → cooldown to baseline. Publishing is suppressed by the
    /// caller during this. Outputs the raw post-cooldown reading; caller applies calibration.
    esp_err_t runHeaterMaintenanceSht3X(float &cleanTemp, float &cleanHum);

    /// SHT4x creep/dew maintenance + plausibility self-test. The SHT4x heater is pulse-based:
    /// fires `heaterMode` measurement-pulses until the rise reaches `minDeltaT` (a healthy sensor
    /// needs ~1 pulse) or the window elapses, then returns to SHT4X_HEATER_OFF and reads a clean
    /// post-cooldown value. The caller picks the profile by trigger reason — gentle self-test vs
    /// aggressive creep mitigation. Outputs the post-cooldown raw reading; caller calibrates.
    esp_err_t runHeaterMaintenanceSht4X(sht4x_heater_t heaterMode, float minDeltaT,
                                        float &cleanTemp, float &cleanHum);

    /// LED shown while the heater is on: 5 blinks/s for durationMs (its delays are the heat wait).
    void heaterLedPattern(uint32_t durationMs);
};