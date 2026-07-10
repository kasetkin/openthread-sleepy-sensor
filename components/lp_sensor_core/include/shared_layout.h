#pragma once

#include <stdint.h>

// POD layout shared between the LP program (lp_core/main.cpp) and the HP-side glue
// (lp_sensor_core.c). The auto-generated ULP export header types every global as a
// uint32_t/uint32_t[] regardless of its real C type, so this struct is never included
// through that generated header directly -- both sides #include this file instead and
// the HP side reinterprets the generated symbol's address as a pointer to it.
//
// result_seq is a seqlock: the LP program bumps it to odd before writing the rest of the
// struct and back to even once done; the HP side (lp_sensor_core_get_state()) retries if
// it observes an odd value or a value that changed mid-read, so a read can never observe a
// torn/partial update. Only fields the LP program writes every cycle need this protection
// -- the config block below is written once by HP before ulp_lp_core_run() starts the LP
// timer, and the LP program never writes it back, so it isn't part of the seqlock.
typedef struct {
    // --- HP -> LP config, written once by lp_sensor_core_init() before the LP timer
    // starts. Read-only from the LP program's point of view thereafter. Mirrors
    // SensorsTaskSettings (main/sensorstask.h).
    float    temp_offset_c;
    float    temp_min_change_c;
    float    rh_offset_pct;
    float    rh_min_change_pct;
    uint32_t max_skip_cycles;

    // --- LP -> HP, written every LP wake; result_seq is a seqlock (see above)
    uint32_t heartbeat_counter;   // bumped every LP wake (Phase 0 sanity signal, still handy)
    uint32_t result_seq;          // seqlock -- see above
    uint32_t sensor_ok;           // 1 if the most recent read passed CRC, 0 otherwise
    uint32_t consec_fail_count;   // consecutive I2C/CRC failures; resets to 0 on success
    float    raw_temp_c;          // last successful reading only -- stale (not zeroed) on failure
    float    raw_hum_pct;
    float    cal_temp_c;          // raw + offset, clamped -- stale (not zeroed) on failure, like raw
    float    cal_hum_pct;
    uint32_t should_wake_hp;      // 1 if this cycle should trigger an early HP wake (see main.cpp)

    // LP's own skip-threshold bookkeeping -- port of sensorstask.cpp's
    // m_previousDeliveredValue / m_sameValueSkippedCycles. Only advanced once HP's ack
    // (below) confirms a value actually reached the broker -- see the ack fields' comment.
    float    prev_delivered_temp_c;
    float    prev_delivered_hum_pct;
    uint32_t skipped_cycles;

    // --- HP -> LP ack, written by lp_sensor_core_ack_delivered() after a CONFIRMED
    // successful MQTT publish of a should_wake_hp=1 cycle's value. hp_ack_seq is a simple
    // "there's a new ack" change-counter, not a full seqlock -- single writer (HP), single
    // reader (LP), and the writer commits the values before bumping the counter, so the
    // reader never sees a bumped counter paired with a stale value (same volatile-ordering
    // convention as result_seq above; this codebase's shared RTC memory has proven reliable
    // with that convention, no cache coherency to worry about). LP applies a new ack at the
    // top of its next cycle (see lp_core/main.cpp), advancing prev_delivered_temp_c/hum_pct
    // to exactly what HP confirms it published. Until acked, a genuinely-changed value keeps
    // being flagged valueDriven=true every cycle -- the intended retry-until-delivered
    // behavior, mirroring the original HP-only code's publishedOk-gated update, and bounded
    // in the meantime by max_skip_cycles same as before.
    uint32_t hp_ack_seq;
    float    hp_acked_temp_c;
    float    hp_acked_hum_pct;

    // Heater scheduling state (port of sensorstask.cpp's m_cyclesSinceHeater /
    // m_highRhCycles) and last-run diagnostics. There's no live LED indicator any more
    // (GPIO15 is unreachable from LP) -- these are the only trail for what the heater did,
    // read post-hoc whenever HP next wakes. heater_active is published as its own stable
    // (even result_seq) snapshot before the long blocking pulse/cooldown sequence starts,
    // so a HP poll that lands mid-run sees a real "heater is running" signal instead of
    // spinning through 10 retries against a multi-minute-long odd window.
    uint32_t cycles_since_heater;
    uint32_t high_rh_cycles;
    uint32_t heater_active;
    uint32_t last_heater_run_cycle; // heartbeat_counter value of the most recent heater run
    float    last_heater_delta_t;   // heated - baseline, most recent run (0 until first run)
    uint32_t last_heater_passed;    // 1 if the most recent run's delta-T cleared its target
} lp_shared_state_t;
