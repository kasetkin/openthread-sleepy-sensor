#pragma once

#include <stdint.h>

// POD layout shared between the LP program (lp_core/main.cpp) and the HP-side glue
// (lp_sensor_core.c). The auto-generated ULP export header types every global as a
// uint32_t/uint32_t[] regardless of its real C type, so this struct is never included
// through that generated header directly -- both sides #include this file instead and
// the HP side reinterprets the generated symbol's address as a pointer to it.
//
// result_seq is a seqlock: the LP program bumps it to odd before writing the rest of the
// struct and back to even once done; the HP side (lp_sensor_core_get_state()) retries if it
// observes an odd value or a value that changed mid-read, so a read can never observe a
// torn/partial update. This protects fields the LP program itself writes every cycle. The
// config block below is HP-owned in both directions (HP is the only writer; LP only ever
// reads it) so it needs no protection on THAT axis -- but see its own comment below for why
// it still isn't given a seqlock of its own, now that it has a SECOND, live write path.
typedef struct {
    // --- HP -> LP config. Two write paths, both single-writer/HP-owned, both safe WITHOUT a
    // seqlock:
    //   1. lp_sensor_core_init() -- once, before ulp_lp_core_run() starts the LP timer, so
    //      there is no concurrent LP-side reader yet at all.
    //   2. lp_sensor_core_apply_config() -- live, any time after the LP timer has started
    //      (HA-driven runtime tuning via main/runtime_config.cpp). This IS a genuine
    //      concurrent writer-while-reader situation, deliberately NOT given a seqlock like
    //      result_seq/hp_ack_seq below, for reasons specific to this block:
    //        - every field is a plain float/uint32_t, individually 4-byte-aligned within the
    //          struct -- a single field's store is atomic at the hardware level, so no
    //          individual field can ever be torn/partially-written.
    //        - the LP program (lp_core/main.cpp) reads this block exactly once, at the top of
    //          its cycle, into local reasoning for that cycle -- it never re-reads a field
    //          mid-cycle, so the SAME field can't change value out from under one decision.
    //        - the eight fields are never compared cross-field against each other (each is an
    //          independent threshold/offset/budget/sample-count), so the one real risk a plain
    //          multi-field write carries -- the LP core observing an old/new MIX across
    //          different fields, if a config change lands mid-cycle -- is harmless here: worst
    //          case, one LP cycle (~one poll interval) applies part of a change (e.g. the old
    //          temp_min_change_c alongside a just-updated rh_offset_pct); the very next cycle
    //          sees the fully-new set. That's a one-cycle-late partial application, never a
    //          corrupted value.
    //      A seqlock would remove even that one-cycle skew, at the cost of the LP program
    //      retrying/re-reading this block on every single wake, forever, to protect a rare
    //      HA-driven write against fields nothing else depends on being cross-consistent --
    //      not worth it here, unlike result_seq/hp_ack_seq below, which protect fields
    //      written EVERY cycle and/or values that truly must be read together.
    // Mirrors SensorsTaskSettings (main/sensorstask.h) / lp_sensor_core_config_t.
    // How many raw SHT4x reads main.cpp's measureAveraged() takes (spaced kInterSampleDelayUs
    // apart) and means into one reported value per cycle -- reduces sample-to-sample noise so
    // it alone can't cross temp_min_change_c/rh_min_change_pct and trigger a spurious HP wake.
    uint32_t sensor_samples;
    float    temp_offset_c;
    float    temp_min_change_c;
    float    rh_offset_pct;
    float    rh_min_change_pct;
    uint32_t max_skip_cycles;
    // Heater schedule, pre-converted from minutes (device_config.yaml or, if overridden live,
    // main/runtime_config.cpp) to LP poll cycles using the boot's fixed lp_poll_interval_sec
    // (see minutes_to_lp_cycles()). 0 = that mechanism disabled.
    uint32_t heater_period_cycles;   // polls between periodic heater self-tests
    uint32_t high_rh_trigger_cycles; // consecutive >90%RH polls before creep mitigation

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
    uint32_t heater_run_count;      // cumulative completed heater runs since boot (monotonic)
} lp_shared_state_t;
