#include "common_utils.h"

#include <chrono>
#include <ranges>
#include <string_view>
#include <esp_log.h>
#include <esp_sleep.h>
#include <esp_check.h>
#include <esp_timer.h>
#include <nvs_flash.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

esp_err_t registerWakeupTimer(const uint64_t wakeupMicrosec)
{
    static const char *TIMERTAG = "timer_wakeup";

    ESP_RETURN_ON_ERROR(esp_sleep_enable_timer_wakeup(wakeupMicrosec), TIMERTAG, "Configure timer as wakeup source failed");
    ESP_LOGI(TIMERTAG, "timer wakeup source is ready");
    return ESP_OK;
}

void correctLightSleep()
{
    static const char *LSLEEPTAG = "light_sleep";

    /// without this not all systems (RF, BLE, etc.) are ready right after wakeup
    /// not sure why,
    //! \todo check documentation and remove if possible 
    static const uint32_t AFTER_WAKEUP_SLEEP = 10; //ms

    ESP_LOGI(LSLEEPTAG, "cycle before light sleep");

    /// with this code, everything doesn't wotk without connected logger
    // uart_wait_tx_idle_polling(static_cast<uart_port_t>(CONFIG_ESP_CONSOLE_UART_NUM));

    const int64_t t_before_us = esp_timer_get_time();
    esp_light_sleep_start();
    const int64_t t_after_us = esp_timer_get_time();
    std::string_view wakeup_reason;

    if (const uint32_t wakeupCauses = esp_sleep_get_wakeup_causes();
        wakeupCauses & BIT(ESP_SLEEP_WAKEUP_TIMER))
        wakeup_reason = "timer";
    else if (wakeupCauses & BIT(ESP_SLEEP_WAKEUP_GPIO))
        wakeup_reason = "pin";
    else if (wakeupCauses & BIT(ESP_SLEEP_WAKEUP_UART))
        wakeup_reason = "uart";
        // vTaskDelay(1);
    else
        wakeup_reason = "other";

    ESP_LOGI(LSLEEPTAG, "Returned from light sleep, reason: %s, t=%lld ms, slept for %lld ms",
            wakeup_reason.data(), t_after_us / 1000, (t_after_us - t_before_us) / 1000);

    ESP_LOGI(LSLEEPTAG, "cycle right after wakeup, sleep for %d msec", AFTER_WAKEUP_SLEEP);
    vTaskDelay(pdMS_TO_TICKS(AFTER_WAKEUP_SLEEP));
}

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

void blinkUserLED(const uint32_t onMs, size_t count)
{
    for (size_t i = 0; i < count; ++i) {
        enableUserLED(true);
        vTaskDelay(pdMS_TO_TICKS(onMs));
        enableUserLED(false);
    }
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

std::optional<std::array<uint8_t, 4>> parseIpv4(std::string_view s)
{
    std::array<uint8_t, 4> octets{};
    std::size_t n = 0;
    for (const auto field : s | std::views::split('.')) {       // for each '.'-separated field
        const std::string_view tok{field.begin(), field.end()};
        unsigned value;
        const auto [ptr, ec] = std::from_chars(tok.data(), tok.data() + tok.size(), value);
        if (n >= octets.size() || ec != std::errc{} ||
            ptr != tok.data() + tok.size() || value > 255)      // non-numeric / trailing / >255
            return std::nullopt;
        octets[n++] = static_cast<uint8_t>(value);
    }
    return n == 4 ? std::optional{octets} : std::nullopt;       // reject wrong field count
}
