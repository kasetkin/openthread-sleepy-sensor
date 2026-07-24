#pragma once

#include <esp_err.h>
#include <stdbool.h>
#include <stdint.h>

#include "shared_layout.h"

#ifdef __cplusplus
extern "C" {
#endif

// Calibration/threshold config, mirroring device_config.yaml's rh_offset/rh_min_change/
// temp_offset/temp_min_change keys directly -- written once into LP shared memory by
// lp_sensor_core_init(), before the LP program ever runs.
typedef struct {
    float    temp_offset_c;
    float    temp_min_change_c;
    float    rh_offset_pct;
    float    rh_min_change_pct;
    // Skip budget in LP poll cycles. device_config.yaml expresses this as max_publish_gap_sec
    // (wall-clock seconds); main.cpp converts using lp_poll_interval_sec (floor, not ceil --
    // see publish_gap_sec_to_skip_cycles()).
    uint32_t max_skip_cycles;
    // Heater schedule in LP poll cycles, 0 = that mechanism disabled. device_config.yaml
    // expresses these in wall-clock minutes (heater_period_minutes /
    // heater_high_rh_trigger_minutes); main.cpp converts using lp_poll_interval_sec.
    uint32_t heater_period_cycles;
    uint32_t high_rh_trigger_cycles;
} lp_sensor_core_config_t;

// Configures the LP_I2C peripheral (SHT4x is wired to GPIO6/GPIO7, the SoC-fixed LP_I2C
// pins), loads the LP core binary, writes `config` into LP shared memory, and arms
// ESP_SLEEP_WAKEUP_ULP as a wakeup source. Call once at HP boot, before
// lp_sensor_core_start(). Caller must check the return value.
esp_err_t lp_sensor_core_init(const lp_sensor_core_config_t *config);

// Start the LP core program running on its own periodic timer. Caller must check the
// return value.
esp_err_t lp_sensor_core_start(uint32_t poll_interval_us);

// Live (post-start) re-application of the config block (calibration offsets, change
// thresholds, skip budget, heater schedule) -- the HA-tunable subset written by
// main/runtime_config.cpp after an accepted MQTT change. Unlike lp_sensor_core_init()'s
// one-time pre-timer-start write, this can be called at any time after lp_sensor_core_start()
// while the LP core is running its own cycle concurrently. See shared_layout.h's doc comment
// on the HP->LP config block for why a plain multi-field write (no seqlock) is safe here.
// Safe to call from any HP-side task at any time; takes effect on the LP core's NEXT wake (it
// re-reads these fields fresh at the top of every cycle -- see lp_core/main.cpp).
void lp_sensor_core_apply_config(const lp_sensor_core_config_t *config);

// Torn-read-safe snapshot of the LP program's shared state (seqlock retry internally --
// see shared_layout.h). Safe to call from any HP-side task at any time.
void lp_sensor_core_get_state(lp_shared_state_t *out);

// Confirms that `temp_c`/`hum_pct` (a value LP previously flagged via should_wake_hp) has
// been successfully delivered (e.g. a confirmed MQTT publish). LP advances its own
// "previous delivered" baseline to match on its next cycle -- see shared_layout.h's comment
// on hp_ack_seq. Call only after genuine confirmation, not merely after an attempt: this is
// what lets a failed publish keep the value flagged for retry instead of being silently
// dropped. Safe to call from any HP-side task at any time.
void lp_sensor_core_ack_delivered(float temp_c, float hum_pct);

// Blocks the calling task (intended: SensorsTask, once per cycle) until either the LP core
// signals should_wake_hp via ulp_lp_core_wakeup_main_processor() (see lp_core/main.cpp), or
// timeout_ms elapses. Does NOT itself trigger any sleep -- the caller is expected to have
// nothing else to do while blocked, which is exactly what lets ESP-IDF's automatic tickless-
// idle light sleep engage underneath this wait (see enableAutomaticLightSleep(),
// main/common_utils.cpp). Returns true if woken early by LP, false on timeout. Requires
// lp_sensor_core_init() to have run first (it registers the light-sleep exit callback that
// detects the ULP wakeup cause and signals this wait).
bool lp_sensor_core_wait_for_wake(uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif
