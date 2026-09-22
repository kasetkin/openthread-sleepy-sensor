#include "die_temp.h"

#include <mutex>

#include "driver/temperature_sensor.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "hal/temperature_sensor_ll.h"

// Gap between readings: the sensor's output register only moves once per conversion.
static constexpr uint32_t SAMPLE_SPACING_US = 50;

// Install/uninstall must not interleave between the tasks that read it.
static std::mutex s_mutex;

std::optional<DieTemp> die_temp_read(uint32_t samples)
{
    const std::lock_guard<std::mutex> lock(s_mutex);

    // The driver logs its measuring range on every install, and this installs on every read.
    static std::once_flag s_quiet;
    std::call_once(s_quiet, [] { esp_log_level_set("temperature_sensor", ESP_LOG_WARN); });

    const temperature_sensor_config_t config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
    temperature_sensor_handle_t tsens = nullptr;
    if (temperature_sensor_install(&config, &tsens) != ESP_OK)
        return std::nullopt;

    std::optional<DieTemp> result;
    if (temperature_sensor_enable(tsens) == ESP_OK) {
        double celsius_sum = 0;
        double raw_sum = 0;
        uint32_t count = 0;
        for (uint32_t i = 0; i < samples; i++) {
            float celsius = 0;
            if (temperature_sensor_get_celsius(tsens, &celsius) == ESP_OK) {
                celsius_sum += celsius;
                raw_sum += temperature_sensor_ll_get_raw_value();
                count++;
            }
            esp_rom_delay_us(SAMPLE_SPACING_US);
        }
        if (count > 0)
            result = DieTemp{static_cast<float>(celsius_sum / count), static_cast<float>(raw_sum / count)};
        temperature_sensor_disable(tsens);
    }
    temperature_sensor_uninstall(tsens);
    return result;
}
