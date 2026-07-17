// Freestanding C++ (no FreeRTOS, no STL, no heap, no C++ static initializers -- see the
// "C++ for the LP program" note in the migration plan).
//
// Phase 3 content: on top of Phase 2's calibrate+threshold logic, port heater maintenance
// (scheduling + pulse loop + cooldown poll) -- a direct port of the due-check in
// sensorstask.cpp::executeTask() (sensorstask.cpp:491-521) and runHeaterMaintenanceSht4X()
// (sensorstask.cpp:360-442). SHT3x's heater path is NOT ported -- this project's LP path
// only ever targets the SHT4x (see the migration plan's MVP scope). Ownership of heater
// maintenance now fully belongs to the LP core: a single LP wake can block for minutes
// running the whole baseline->pulse->cooldown sequence via ulp_lp_core_delay_us() busy-waits
// (confirmed safe: lp_core_startup.c calls main() and simply waits for it to return, no
// watchdog feed requirement anywhere in the LP core's own execution model;
// ulp_lp_core_delay_us() is a plain 32-bit cycle-counter busy-wait with no meaningful
// overflow risk at these durations). There's no LED indicator any more (GPIO15 is
// unreachable from LP) -- the shared struct's heater_active/last_heater_* fields are the
// only trail, readable post-hoc (or, for heater_active, even mid-run -- see below).
//
// Bench-validated on hardware with the schedule temporarily shrunk to 20/6 cycles (both
// trigger paths fired, producing plausible delta-T rises matching sht4x.c's documented
// per-pulse figures -- see the migration plan/memory for the capture). The schedule is
// runtime config now (device_config.yaml's heater_period_minutes /
// heater_high_rh_trigger_minutes, 0 = off) -- rerun that kind of bench test by setting
// small minute values there instead of editing constants.
//
// Phase 4 content: consume HP's delivery ack (see shared_layout.h's hp_ack_seq comment) so
// prev_delivered_temp_c/hum_pct only advances once HP confirms a value actually reached the
// broker, closing the "assume delivered" gap the earlier phases deliberately left open
// while nothing was really being published yet.

#include <stdint.h>
#include <stddef.h>

#include "ulp_lp_core_utils.h"
#include "ulp_lp_core_i2c.h"
#include "shared_layout.h"

lp_shared_state_t g_shared;

// LP-internal only (static -- not exported to the HP-visible generated header): tracks
// which hp_ack_seq value has already been folded into prev_delivered_temp_c/hum_pct, so a
// given ack is consumed exactly once. Persists across LP wakes the same way g_shared does
// (RTC memory retained, zeroed once at ulp_lp_core_load_binary()).
static uint32_t g_lastConsumedAckSeq;

namespace {

constexpr uint16_t kSht4xAddr = 0x44;

// Plain (heater-off) high-repeatability measurement -- matches sht4x.c's CMD_MEAS_HIGH.
constexpr uint8_t kCmdMeasureHigh = 0xFD;
constexpr uint32_t kMeasureDelayUs = 10000; // 10ms

// Heater-pulse measurement commands + conversion delays -- the SHT4x heats and measures in
// one shot, matching sht4x.c's get_meas_cmd()/get_duration_ms() for the two heater modes
// this project uses (see sensorstask.h's HEATER_SHT4X_SELFTEST_MODE/CREEP_MODE).
constexpr uint8_t kCmdHeaterMediumShort = 0x24;   // ~110mW, 0.1s pulse (periodic self-test)
constexpr uint32_t kHeaterMediumShortDelayUs = 110000;
constexpr uint8_t kCmdHeaterHighLong = 0x39;      // ~200mW, 1s pulse (sustained-high-RH creep mitigation)
constexpr uint32_t kHeaterHighLongDelayUs = 1100000;

constexpr int32_t kI2cTimeoutCycles = 5000;        // matches the reference lp_i2c example

// --- heater tuning constants. The heater SCHEDULE (periodic self-test interval, sustained
// high-RH duration before creep mitigation) is runtime config now:
// g_shared.heater_period_cycles / .high_rh_trigger_cycles, written once by
// lp_sensor_core_init() from device_config.yaml's heater_*_minutes keys (0 = that mechanism
// disabled). Only the physics stays compile-time below: the RH threshold, pulse
// modes/durations, and cooldown behavior.
constexpr float kHighRhThreshold = 90.0f;         // %RH

constexpr uint32_t kHeaterDurationMs = 20000;     // max pulse-loop window (cap; SHT4x usually stops after 1 pulse)
constexpr float kSelftestDeltaT = 5.0f;
constexpr float kCreepDeltaT = 40.0f;

constexpr float kCooldownEpsilonC = 0.3f;         // resume once T within this of baseline
constexpr uint32_t kCooldownMaxMs = 240000;       // cap on cooldown wait
constexpr uint32_t kCooldownPollMs = 2000;        // re-read interval while cooling

// Same CRC-8 (poly 0x31, init 0xFF) as components/sht4x/sht4x.c's crc8() -- duplicated
// here rather than shared because the LP core's freestanding toolchain can't link against
// the HP-side sht4x component.
uint8_t crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++)
            crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x31) : static_cast<uint8_t>(crc << 1);
    }
    return crc;
}

float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// Avoids depending on libm's fabsf in this freestanding link.
float absf(float v)
{
    return v < 0.0f ? -v : v;
}

// One write-command -> conversion-wait -> read+CRC-check -> convert cycle. Used both for
// the plain periodic read (kCmdMeasureHigh) and for each heater pulse/baseline/cooldown
// poll (the heater commands trigger a pulse AND return a reading in one shot -- see the
// comment on kCmdHeaterMediumShort above).
bool measure(uint8_t cmd, uint32_t delayUs, float &tempC, float &humPct)
{
    uint8_t cmdByte = cmd;
    if (lp_core_i2c_master_write_to_device(LP_I2C_NUM_0, kSht4xAddr, &cmdByte, 1, kI2cTimeoutCycles) != ESP_OK)
        return false;

    ulp_lp_core_delay_us(delayUs);

    uint8_t raw[6];
    if (lp_core_i2c_master_read_from_device(LP_I2C_NUM_0, kSht4xAddr, raw, sizeof(raw), kI2cTimeoutCycles) != ESP_OK)
        return false;
    if (raw[2] != crc8(raw, 2) || raw[5] != crc8(raw + 3, 2))
        return false;

    const uint16_t rawTemp = static_cast<uint16_t>((raw[0] << 8) | raw[1]);
    const uint16_t rawHum  = static_cast<uint16_t>((raw[3] << 8) | raw[4]);
    // Same conversion as sht4x_compute_values() in components/sht4x/sht4x.c.
    tempC = rawTemp * 175.0f / 65535.0f - 45.0f;
    humPct = clampf(rawHum * 125.0f / 65535.0f - 6.0f, 0.0f, 100.0f);
    return true;
}

// Port of runHeaterMaintenanceSht4X (sensorstask.cpp:360-442). Baseline -> pulse until the
// delta-T target is reached or the window elapses -> cooldown poll until back near baseline
// or timeout -> final clean reading. The pulse/cooldown loops are bounded by iteration
// count rather than wall-clock (the LP core has no confirmed clock-read API), using each
// loop's fixed per-iteration duration -- the worst-case wall-clock bound is the same as the
// original. Returns false (leaving the out-params untouched) if even the baseline read
// fails, matching the original's "skip maintenance" behavior on that failure.
bool runHeaterMaintenance(uint8_t heaterCmd, uint32_t heaterDelayUs, float minDeltaT,
                           float &cleanTemp, float &cleanHum, float &outDeltaT, bool &outPassed)
{
    float t0, rh0;
    if (!measure(kCmdMeasureHigh, kMeasureDelayUs, t0, rh0))
        return false;

    // Pulse loop -- each iteration IS a heater-encoded measurement. Stops early once the
    // delta-T target is reached (a healthy sensor needs ~1 pulse); otherwise capped by
    // iteration count so total pulse time can't exceed ~kHeaterDurationMs regardless of mode.
    const uint32_t maxPulseIterations = kHeaterDurationMs * 1000 / heaterDelayUs;
    float t1 = t0, rh1 = rh0;
    for (uint32_t i = 0; i < maxPulseIterations; i++) {
        float tp, rhp;
        if (measure(heaterCmd, heaterDelayUs, tp, rhp)) {
            t1 = tp;
            rh1 = rhp;
            if (t1 - t0 >= minDeltaT)
                break;
        }
    }

    // Plausibility self-test on the TEMPERATURE RISE only -- the heated RH reading isn't
    // trustworthy (see the original function's comment): a genuine, powered sensor always
    // heats, a dead/missing one never reaches the delta-T target.
    outDeltaT = t1 - t0;
    outPassed = outDeltaT >= minDeltaT;

    // Cooldown poll -- RH stays biased low until the die returns to baseline.
    const uint32_t maxCooldownIterations = kCooldownMaxMs / kCooldownPollMs;
    float t = t1, rh = rh1;
    for (uint32_t i = 0; i < maxCooldownIterations; i++) {
        ulp_lp_core_delay_us(kCooldownPollMs * 1000);
        float tc, rhc;
        if (measure(kCmdMeasureHigh, kMeasureDelayUs, tc, rhc)) {
            t = tc;
            rh = rhc;
            if (t <= t0 + kCooldownEpsilonC)
                break;
        }
    }

    // Final clean reading (fall back to the last cooldown sample if it fails).
    if (!measure(kCmdMeasureHigh, kMeasureDelayUs, cleanTemp, cleanHum)) {
        cleanTemp = t;
        cleanHum = rh;
    }
    return true;
}

} // namespace

extern "C" int main()
{
    // Consume a new HP delivery ack, if any, before this cycle's own threshold decision --
    // see shared_layout.h's hp_ack_seq comment. Plain (non-volatile) read is safe here even
    // though HP writes hp_ack_seq asynchronously: this is a single read per invocation, not
    // a loop re-checking it, so there's no stale-cached-across-iterations risk to guard
    // against with volatile.
    if (g_shared.hp_ack_seq != g_lastConsumedAckSeq) {
        g_shared.prev_delivered_temp_c = g_shared.hp_acked_temp_c;
        g_shared.prev_delivered_hum_pct = g_shared.hp_acked_hum_pct;
        g_shared.skipped_cycles = 0;
        g_lastConsumedAckSeq = g_shared.hp_ack_seq;
    }

    const uint32_t wasOk = g_shared.sensor_ok;

    g_shared.result_seq++; // odd: writing

    float tempC, humPct;
    const bool ok = measure(kCmdMeasureHigh, kMeasureDelayUs, tempC, humPct);

    g_shared.heartbeat_counter++;
    g_shared.sensor_ok = ok ? 1 : 0;
    g_shared.consec_fail_count = ok ? 0 : (g_shared.consec_fail_count + 1);

    // valueDriven tracks whether THIS cycle's calibrated value cleared the skip-threshold
    // gate -- one of should_wake_hp's two triggers (see below).
    bool valueDriven = false;
    if (ok) {
        g_shared.raw_temp_c = tempC;
        g_shared.raw_hum_pct = humPct;

        // Heater scheduling -- direct port of the due-check in
        // sensorstask.cpp::executeTask() (sensorstask.cpp:491-499). The high-RH trigger
        // evaluates the calibrated RH of THIS (pre-heat) reading so a biased sensor can't
        // self-trigger endlessly.
        g_shared.cycles_since_heater++;
        const float precalHum = clampf(humPct + g_shared.rh_offset_pct, 0.0f, 100.0f);
        g_shared.high_rh_cycles = (precalHum > kHighRhThreshold) ? g_shared.high_rh_cycles + 1 : 0;
        const bool periodicDue = g_shared.heater_period_cycles > 0
                              && g_shared.cycles_since_heater >= g_shared.heater_period_cycles;
        const bool humidityDue = g_shared.high_rh_trigger_cycles > 0
                              && g_shared.high_rh_cycles >= g_shared.high_rh_trigger_cycles;

        if (periodicDue || humidityDue) {
            const bool creep = humidityDue;
            const uint8_t heaterCmd = creep ? kCmdHeaterHighLong : kCmdHeaterMediumShort;
            const uint32_t heaterDelayUs = creep ? kHeaterHighLongDelayUs : kHeaterMediumShortDelayUs;
            const float minDeltaT = creep ? kCreepDeltaT : kSelftestDeltaT;

            // Publish "heater running" as its own stable snapshot before the long blocking
            // sequence below, so a HP poll that lands mid-run sees a real "heater is
            // running" signal instead of retrying against a multi-minute-long odd window
            // (lp_sensor_core_get_state() only retries 10 times, not for minutes).
            g_shared.heater_active = 1;
            g_shared.result_seq++; // even: stable (intermediate publish)

            float cleanTemp, cleanHum, deltaT;
            bool passed;
            if (runHeaterMaintenance(heaterCmd, heaterDelayUs, minDeltaT, cleanTemp, cleanHum, deltaT, passed)) {
                g_shared.result_seq++; // odd: resume writing
                tempC = cleanTemp;
                humPct = cleanHum;
                g_shared.raw_temp_c = tempC;
                g_shared.raw_hum_pct = humPct;
                g_shared.last_heater_run_cycle = g_shared.heartbeat_counter;
                g_shared.last_heater_delta_t = deltaT;
                g_shared.last_heater_passed = passed ? 1 : 0;
            } else {
                g_shared.result_seq++; // odd: resume writing (baseline read failed -- nothing new to report)
            }
            g_shared.heater_active = 0;

            g_shared.cycles_since_heater = 0;
            g_shared.high_rh_cycles = 0;
        }

        // Calibration AFTER maintenance, so the published value is the clean post-cooldown
        // reading when maintenance ran -- direct port of calibrateTemperature/calibrateHum.
        const float calTemp = clampf(tempC + g_shared.temp_offset_c, -273.15f, 3000.0f);
        const float calHum  = clampf(humPct + g_shared.rh_offset_pct, 0.0f, 100.0f);
        g_shared.cal_temp_c = calTemp;
        g_shared.cal_hum_pct = calHum;

        // Skip-threshold decision -- direct port of the compare block in
        // sensorstask.cpp::executeTask() (sensorstask.cpp:543-558), comparing against the
        // last HP-ACKED baseline (not this cycle's own guess -- see the ack-consumption
        // block at the top of this function).
        const float tempChange = absf(calTemp - g_shared.prev_delivered_temp_c);
        const float rhChange = absf(calHum - g_shared.prev_delivered_hum_pct);
        const bool sameAsPrevDelivered =
            tempChange < g_shared.temp_min_change_c && rhChange < g_shared.rh_min_change_pct;
        const bool skipThisCycle = sameAsPrevDelivered && g_shared.skipped_cycles < g_shared.max_skip_cycles;

        valueDriven = !skipThisCycle;
        if (skipThisCycle) {
            g_shared.skipped_cycles++;
        }
        // else: don't touch prev_delivered_*/skipped_cycles here -- only HP's ack (consumed
        // at the top of this function, on some future cycle) advances them. Until acked,
        // this same "different enough" value keeps being flagged valueDriven=true every
        // cycle, which is the correct retry-until-delivered behavior.
    }

    // Wake HP if the value cleared the threshold/skip-budget gate, or the sensor just
    // flipped from OK to failing (report a fault promptly rather than waiting for the
    // backstop). Deliberately NOT "every cycle while broken" -- that would defeat the
    // backstop's purpose and spam HP wakes during an extended outage.
    g_shared.should_wake_hp = (valueDriven || (wasOk != 0 && !ok)) ? 1 : 0;

    g_shared.result_seq++; // even: stable

    if (g_shared.should_wake_hp)
        ulp_lp_core_wakeup_main_processor();

    return 0;
}
