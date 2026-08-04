#include "hp_awake_stats.h"

#include <algorithm>

#include "esp_pm.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

// Accumulated by exit_cb (IDLE task context) between reads; snapshot+reset by
// hp_awake_stats_get_and_reset_us() (MQTT task context) -- portMUX-protected across that
// boundary the same way esp_pm's own internal callback list is.
static uint64_t s_accumulated_sleep_us = 0;
static int64_t s_period_start_us = 0;

// No blocking calls allowed here -- runs from IDLE task context after every automatic
// light-sleep attempt (CONFIG_PM_LIGHT_SLEEP_CALLBACKS), same constraint documented on
// lp_sensor_core.c's sibling registration. sleep_time_us is the actual duration just slept.
static esp_err_t on_light_sleep_exit(int64_t sleep_time_us, void *arg)
{
    (void)arg;
    if (sleep_time_us > 0) {
        portENTER_CRITICAL(&s_mux);
        s_accumulated_sleep_us += static_cast<uint64_t>(sleep_time_us);
        portEXIT_CRITICAL(&s_mux);
    }
    return ESP_OK;
}

esp_err_t hp_awake_stats_init()
{
    s_period_start_us = esp_timer_get_time();

    esp_pm_sleep_cbs_register_config_t cbs_conf = {
        .enter_cb = nullptr,
        .exit_cb = on_light_sleep_exit,
        .enter_cb_user_arg = nullptr,
        .exit_cb_user_arg = nullptr,
        .enter_cb_prior = 0,
        .exit_cb_prior = 0,
    };
    return esp_pm_light_sleep_register_cbs(&cbs_conf);
}

uint32_t hp_awake_stats_get_and_reset_us()
{
    const int64_t now = esp_timer_get_time();

    portENTER_CRITICAL(&s_mux);
    const uint64_t slept = s_accumulated_sleep_us;
    s_accumulated_sleep_us = 0;
    portEXIT_CRITICAL(&s_mux);

    const int64_t elapsed = now - s_period_start_us;
    s_period_start_us = now;

    // Clamped defensively: callback-invocation timing skew could in principle make `slept`
    // slightly exceed `elapsed` right at the read boundary, and this must never wrap negative
    // into a huge uint32_t.
    return static_cast<uint32_t>(std::max<int64_t>(0, elapsed - static_cast<int64_t>(slept)));
}
