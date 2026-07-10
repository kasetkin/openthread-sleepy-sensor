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
#include <driver/uart.h>
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

    // Fires every wake, so this and the line below are ESP_LOGD (not I): a synchronous
    // UART/USB-JTAG write on every single cycle is real, avoidable awake-time on an
    // otherwise-quiet path. Bump CONFIG_LOG_DEFAULT_LEVEL to see them again for debugging.
    ESP_LOGD(LSLEEPTAG, "cycle before light sleep");

    // Matches ESP-IDF's own official light_sleep example (examples/system/light_sleep/main/
    // light_sleep_example_main.c) verbatim, same placement: "To make sure the complete line
    // is printed before entering sleep mode, need to wait until UART TX FIFO is empty." Was
    // previously commented out here with a note that it "doesn't work without connected
    // logger" -- but this whole file was bulk copy-pasted from an unrelated prior project
    // (commit b68eefd), so that note's origin/validity for *this* codebase is unverified, and
    // uart_wait_tx_idle_polling() is a plain register poll (uart_hal_is_tx_idle()) with no
    // dependency on a listener being present, unlike the USB-Serial-JTAG equivalent
    // (usb_serial_jtag_wait_tx_done(), not used here -- it dereferences a driver object only
    // allocated by an explicit usb_serial_jtag_driver_install(), which this project never
    // calls, only the passive Kconfig secondary-console path, so calling it would likely
    // crash). This only addresses the primary UART0 console (raw TX/RX pins, e.g. this
    // project's external CP2102 adapter capture path) -- USB-Serial-JTAG's own light-sleep
    // re-enumeration behavior is a separate, already-documented limitation (host-side, needs
    // a cable replug or CONFIG_USJ_NO_AUTO_LS_ON_CONNECTION, not fixable via a TX-wait call).
    uart_wait_tx_idle_polling(static_cast<uart_port_t>(CONFIG_ESP_CONSOLE_UART_NUM));

    const int64_t t_before_us = esp_timer_get_time();
    esp_light_sleep_start();
    const int64_t t_after_us = esp_timer_get_time();
    std::string_view wakeup_reason;

    if (const uint32_t wakeupCauses = esp_sleep_get_wakeup_causes();
        wakeupCauses & BIT(ESP_SLEEP_WAKEUP_TIMER))
        wakeup_reason = "timer";
    else if (wakeupCauses & BIT(ESP_SLEEP_WAKEUP_ULP))
        wakeup_reason = "ulp";
    else if (wakeupCauses & BIT(ESP_SLEEP_WAKEUP_GPIO))
        wakeup_reason = "pin";
    else if (wakeupCauses & BIT(ESP_SLEEP_WAKEUP_UART))
        wakeup_reason = "uart";
        // vTaskDelay(1);
    else
        wakeup_reason = "other";

    ESP_LOGD(LSLEEPTAG, "Returned from light sleep, reason: %s, t=%lld ms, slept for %lld ms",
            wakeup_reason.data(), t_after_us / 1000, (t_after_us - t_before_us) / 1000);

    // Instability investigation, 2026-07-10: this delay was removed entirely (not zeroed --
    // vTaskDelay(0) still forces a reschedule, see git history) as a bench test for whether
    // its original comment ("RF, BLE, etc. not ready") was cargo-cult copy-paste (commit
    // b68eefd, an unrelated prior project -- this one has no BLE, and neither ESP-IDF's docs
    // nor esp_openthread_sleep.c's PM-lock path document any need for it). The removal
    // reproduced a real MQTT/OpenThread failure: publish cycles firing every ~1.5-8s instead
    // of the configured 15s backstop, OpenThread's message-buffer pool exhausting ("Failed to
    // copy to OpenThread message: NoBufs"), detach, and a forced reboot.
    //
    // Retest 1: vTaskDelay(pdMS_TO_TICKS(10)) at this exact spot -- same magnitude as the
    // historically-stable value. Did NOT fix it: same rapid sub-3s LP-driven publish burst,
    // same connect-timeout/NoBufs/reboot chain, reproduced twice in a row with near-identical
    // timing.
    //
    // Retest 2: esp_rom_delay_us(10000) at this exact spot (a ROM-level busy-wait, immune to
    // being silently absorbed by automatic tickless-idle sleep the way vTaskDelay() can be).
    // Also did NOT fix it -- the failure's timing became much MORE regular (a consistent
    // ~2.18s between publishes, vs. the previous runs' irregular 1.2-2.8s) but the core
    // problem -- publishing every ~2s instead of every 15s -- was unchanged, and it still
    // ended in the same connect-timeout/NoBufs/reboot chain.
    //
    // Both retests kept the extra delay in this POST-wake position (right after
    // esp_light_sleep_start() returns). The one build that WAS stable (extra ESP_LOGI, no
    // delay at all here) also had a second log line in a PRE-sleep position -- right before
    // SensorsTask::executeTask() calls this function, i.e. right before the CPU halts for
    // sleep, not after it wakes -- which neither retest has isolated on its own. Moved there
    // for the next test (see SensorsTask::executeTask() in sensorstask.cpp) rather than
    // stacked here, to keep it a single-variable comparison. Root cause still not confirmed.
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
    static const char *LEDTAG = "gpio-LED";
    static const gpio_num_t EXT_LED_GPIO = GPIO_NUM_15;
    static const uint32_t LED_ON_LEVEL = 0;
    static const uint32_t LED_OFF_LEVEL = (LED_ON_LEVEL + 1) % 2;

    // Configure the pin once on first use rather than on every toggle -- this is called
    // twice per blink (on, then off), and gpio_reset_pin()/gpio_set_direction() only need
    // to run once.
    static bool configured = false;
    if (!configured) {
        gpio_reset_pin(EXT_LED_GPIO);
        gpio_set_direction(EXT_LED_GPIO, GPIO_MODE_OUTPUT);
        configured = true;
    }

    const uint32_t led_flag = enableLED ? LED_ON_LEVEL : LED_OFF_LEVEL;
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
        // Off-gap between successive blinks (not after the last one) so a multi-blink
        // pattern stays visually countable as separate pulses once onMs is short.
        if (i + 1 < count)
            vTaskDelay(pdMS_TO_TICKS(onMs));
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
