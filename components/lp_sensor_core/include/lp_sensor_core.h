#pragma once

#include <esp_err.h>
#include <stdint.h>

#include "shared_layout.h"

#ifdef __cplusplus
extern "C" {
#endif

// Calibration/threshold config, mirroring calibration.txt's rh_offset/rh_min_change/
// temp_offset/temp_min_change/max_skip_cycles keys -- written once into LP shared memory
// by lp_sensor_core_init(), before the LP program ever runs.
typedef struct {
    float    temp_offset_c;
    float    temp_min_change_c;
    float    rh_offset_pct;
    float    rh_min_change_pct;
    uint32_t max_skip_cycles;
} lp_sensor_core_config_t;

// Configures the LP_I2C peripheral (SHT4x is wired to GPIO6/GPIO7, the SoC-fixed LP_I2C
// pins), loads the LP core binary, writes `config` into LP shared memory, and arms
// ESP_SLEEP_WAKEUP_ULP as a wakeup source. Call once at HP boot, before
// lp_sensor_core_start(). Caller must check the return value.
esp_err_t lp_sensor_core_init(const lp_sensor_core_config_t *config);

// Start the LP core program running on its own periodic timer. Caller must check the
// return value.
esp_err_t lp_sensor_core_start(uint32_t poll_interval_us);

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

#ifdef __cplusplus
}
#endif
