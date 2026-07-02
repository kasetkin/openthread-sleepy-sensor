#include "wifi_link.h"

#include <algorithm>
#include <cstring>
#include <format>
#include <string>

#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>

static const char *TAG = "wifi-link";

static constexpr EventBits_t BIT_GOT_IPV4 = BIT0;
static constexpr EventBits_t BIT_GOT_IPV6 = BIT1;
static EventGroupHandle_t s_wifi_eg = nullptr;

static esp_netif_t *s_sta_netif = nullptr;

// Wi-Fi config captured at makeWifiLink() time and consumed by start().
static std::string s_wifi_ssid;
static std::string s_wifi_password;
static std::string s_wifi_address_family = "ipv4";  // "ipv4" or "ipv6" — which single family to bring up

static void wifi_event_handler(void * /*arg*/, esp_event_base_t event_base,
                                int32_t event_id, void * /*event_data*/)
{
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
        case WIFI_EVENT_STA_START:
            esp_wifi_connect();
            break;
        case WIFI_EVENT_STA_CONNECTED:
            // IPv4: leave the default DHCP client running; IP_EVENT_STA_GOT_IP sets the bit.
            // IPv6: no IPv4 at all in this mode, and SLAAC needs link-local first.
            if (s_wifi_address_family == "ipv6") {
                esp_netif_dhcpc_stop(s_sta_netif);
                esp_netif_create_ip6_linklocal(s_sta_netif);
            }
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            if (s_wifi_eg) {
                const EventBits_t bit = (s_wifi_address_family == "ipv6") ? BIT_GOT_IPV6 : BIT_GOT_IPV4;
                xEventGroupClearBits(s_wifi_eg, bit);
            }
            esp_wifi_connect();
            break;
        default:
            break;
        }
    } else if (event_base == IP_EVENT) {
        switch (event_id) {
        case IP_EVENT_STA_GOT_IP:
            if (s_wifi_eg)
                xEventGroupSetBits(s_wifi_eg, BIT_GOT_IPV4);
            break;
        case IP_EVENT_GOT_IP6:
            if (s_wifi_eg)
                xEventGroupSetBits(s_wifi_eg, BIT_GOT_IPV6);
            break;
        default:
            break;
        }
    }
}

// Bounded copy into a fixed-size wifi_config_t field (ssid[32]/password[64]); the field is
// already zero-initialised by the caller's `= {}`, so a short copy leaves the rest as NUL.
static void copyBounded(uint8_t *dst, size_t dstCapacity, const std::string &src)
{
    const size_t n = std::min(src.size(), dstCapacity - 1);
    std::memcpy(dst, src.data(), n);
}

static esp_err_t start()
{
    s_wifi_eg = xEventGroupCreate();

    s_sta_netif = esp_netif_create_default_wifi_sta();

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t err = esp_wifi_init(&init_cfg);
    if (err != ESP_OK)
        return err;

    // Credentials come from secrets.yaml every boot; no need to persist them to flash NVS.
    esp_wifi_set_storage(WIFI_STORAGE_RAM);

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                          &wifi_event_handler, nullptr, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, ESP_EVENT_ANY_ID,
                                                          &wifi_event_handler, nullptr, nullptr));

    wifi_config_t wifi_config = {};
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    copyBounded(wifi_config.sta.ssid, sizeof(wifi_config.sta.ssid), s_wifi_ssid);
    copyBounded(wifi_config.sta.password, sizeof(wifi_config.sta.password), s_wifi_password);

    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK)
        return err;
    err = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (err != ESP_OK)
        return err;
    err = esp_wifi_start();
    if (err != ESP_OK)
        return err;

    esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
    esp_netif_set_default_netif(s_sta_netif);

    ESP_LOGI(TAG, "Wi-Fi STA starting, address family: %s", s_wifi_address_family.c_str());
    return ESP_OK;
}

static bool waitForWifiReady(uint32_t timeout_ms)
{
    if (!s_wifi_eg)
        return false;

    const EventBits_t want = (s_wifi_address_family == "ipv6") ? BIT_GOT_IPV6 : BIT_GOT_IPV4;
    const EventBits_t bits = xEventGroupWaitBits(s_wifi_eg, want, pdFALSE, pdTRUE, pdMS_TO_TICKS(timeout_ms));
    return (bits & want) != 0;
}

static std::string brokerUri(std::string_view broker_address, uint16_t port, bool use_tls)
{
    const auto scheme = mqttScheme(use_tls);
    if (looksLikeIpv6(broker_address))
        return std::format("{}://[{}]:{}", scheme, broker_address, port);
    return std::format("{}://{}:{}", scheme, broker_address, port);
}

// waitForReady() already guarantees the one family Wi-Fi brought up is ready, and the
// boot-time consistency check (main.cpp) guarantees that family matches broker_address —
// so there is nothing left to wait for here, unlike Thread's NAT64-prefix wait.
static bool waitForBrokerReachable(std::string_view /*broker_address*/, uint32_t /*timeout_ms*/)
{
    return true;
}

static void noop() {}

static void refresh()
{
    esp_wifi_connect();
}

NetworkLink makeWifiLink(const NetworkLinkConfig &cfg)
{
    s_wifi_ssid = cfg.wifi_ssid;
    s_wifi_password = cfg.wifi_password;
    s_wifi_address_family = cfg.wifi_address_family;

    NetworkLink link;
    link.start = start;
    link.waitForReady = waitForWifiReady;
    link.brokerUri = brokerUri;
    link.waitForBrokerReachable = waitForBrokerReachable;
    link.onPublishWindowBegin = noop;
    link.onPublishWindowEnd = noop;
    link.refresh = refresh;
    return link;
}
