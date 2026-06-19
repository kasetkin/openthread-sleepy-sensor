#include "common_utils.h"

#include <chrono>
#include <esp_log.h>
#include <nvs_flash.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

void enableRf(const bool enableRf)
{
    static const char *RFTAG = "gpio-RF";
    const gpio_num_t RF_ON_GPIO = GPIO_NUM_3; 

    const uint32_t RF_ON_LEVEL = 0;
    const uint32_t RF_OFF_LEVEL = (RF_ON_LEVEL + 1) % 2;
    const uint32_t rf_flag = enableRf ? RF_ON_LEVEL : RF_OFF_LEVEL;
    gpio_reset_pin(RF_ON_GPIO);
    gpio_set_direction(RF_ON_GPIO, GPIO_MODE_OUTPUT);
    const esp_err_t rfOnErr = gpio_set_level(RF_ON_GPIO, rf_flag);
    if (rfOnErr != ESP_OK)
        ESP_LOGE(RFTAG, "gpio_set_level error for GPIO_NUM_3 and level %d, error %d", rf_flag, rfOnErr);
    else
        ESP_LOGI(RFTAG, "gpio_set_level OK for GPIO_NUM_3 and level %d", rf_flag);
}

void enableExtAntenna(const bool enableExtAnt)
{
    static const char *ANTTAG = "gpio-ANT";
    const gpio_num_t EXT_ANT_GPIO = GPIO_NUM_14;
    const uint32_t EXTERNAL_ANT_ON_LEVEL = 1;
    const uint32_t EXTERNAL_ANT_OFF_LEVEL = (EXTERNAL_ANT_ON_LEVEL + 1) % 2;
    const uint32_t ant_flag = enableExtAnt ? EXTERNAL_ANT_ON_LEVEL : EXTERNAL_ANT_OFF_LEVEL;
    gpio_reset_pin(EXT_ANT_GPIO);
    gpio_set_direction(EXT_ANT_GPIO, GPIO_MODE_OUTPUT);
    const esp_err_t antSelectErr = gpio_set_level(EXT_ANT_GPIO, ant_flag);
    if (antSelectErr != ESP_OK)
        ESP_LOGE(ANTTAG, "gpio_set_level error for GPIO_NUM_14 and level %d, error %d", ant_flag, antSelectErr);
    else
        ESP_LOGI(ANTTAG, "gpio_set_level OK for GPIO_NUM_14 and level %d", ant_flag);

}

void enableUserLED(const bool enableLED)
{
    static const char *LEDTAG = "gpio-ANT";
    const gpio_num_t EXT_LED_GPIO = GPIO_NUM_15;
    const uint32_t LED_ON_LEVEL = 0;
    const uint32_t LED_OFF_LEVEL = (LED_ON_LEVEL + 1) % 2;
    const uint32_t led_flag = enableLED ? LED_ON_LEVEL : LED_OFF_LEVEL;
    gpio_reset_pin(EXT_LED_GPIO);
    gpio_set_direction(EXT_LED_GPIO, GPIO_MODE_OUTPUT);
    const esp_err_t ledEnableErr = gpio_set_level(EXT_LED_GPIO, led_flag);
    if (ledEnableErr != ESP_OK)
        ESP_LOGE(LEDTAG, "gpio_set_level error for GPIO_NUM_15 and level %d, error %d", led_flag, ledEnableErr);
    else
        ESP_LOGD(LEDTAG, "gpio_set_level OK for GPIO_NUM_15 and level %d", led_flag);
}

void blinkUserLED(const uint32_t onMs)
{
    enableUserLED(true);
    vTaskDelay(pdMS_TO_TICKS(onMs));
    enableUserLED(false);
}

esp_err_t initNvsFlash()
{
    static const char *NFSFLASHTAG = "NVS-flash";

    // Initialize NVS - required for controller to store calibration data
    const esp_err_t ret1 = nvs_flash_init();
    if (ret1 == ESP_ERR_NVS_NO_FREE_PAGES || ret1 == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        const esp_err_t ret2 = nvs_flash_init();
        if (ret2 != ESP_OK) {
            ESP_LOGE(NFSFLASHTAG, "Failed to initialize NVS(2): %d", ret2);
            return ret2;
        }
    }
    if (ret1 != ESP_OK) {
        ESP_LOGE(NFSFLASHTAG, "Failed to initialize NVS(1): %d", ret1);
        return ret1;
    }

    return ESP_OK;
}

unsigned long millisFromStart()
{
    static auto start_time = std::chrono::system_clock::now();

    auto end_time = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    return static_cast<unsigned long>(duration.count());
}

uint64_t getValidTime()
{
    const auto nowTime = std::chrono::system_clock::now();
    const auto nowAsDuration = nowTime.time_since_epoch();
    const auto durationSec = std::chrono::duration_cast<std::chrono::seconds>(nowAsDuration);
    const uint64_t rtc_sec = static_cast<uint64_t>(durationSec.count());
    return rtc_sec;
}
