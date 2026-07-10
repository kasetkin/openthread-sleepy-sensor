#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <functional>
#include <cstdint>

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

struct SensorsTaskSettings
{
    /// HP backstop interval. The LP core (components/lp_sensor_core) now owns the real
    /// read/calibrate/threshold cadence (see calibration.txt's lp_poll_interval_sec) and
    /// wakes HP early via ulp_lp_core_wakeup_main_processor() whenever it has something
    /// worth publishing; this is just the ceiling on how stale published data can get if LP
    /// never flags a change.
    uint32_t cycle_duration_sec = 60;
};

class SensorsTask
{
public:
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

    const SensorsTaskSettings m_settings;

    SensorsReadyEvent m_readyEvent;
    AttachGate m_attachGate;
    RefreshNat64 m_refreshNat64;

    /// consecutive cycles with no successful publish; drives the reboot supervisor (see REBOOT_AFTER_FAILS)
    uint32_t m_consecutiveFailures = 0;

    /// last heater run LP reported (heartbeat_counter value at the time), so the post-hoc
    /// diagnostic log line only fires once per new run rather than every wake.
    uint32_t m_lastSeenHeaterRunCycle = 0;
};
