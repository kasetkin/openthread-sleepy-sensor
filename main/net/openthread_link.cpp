#include "openthread_link.h"

#include <algorithm>
#include <format>
#include <string>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_openthread.h"
#include "esp_openthread_lock.h"
#include "esp_openthread_netif_glue.h"

#include "common_utils.h"
#include "secrets.h"

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>

#include "openthread/dataset.h"
#include "openthread/ip6.h"
#include "openthread/link.h"
#include "openthread/netdata.h"
#include "openthread/thread.h"

static const char *TAG = "ot-link";

// Set once the device attaches to the Thread mesh (role CHILD/ROUTER/LEADER), cleared on
// detach. The sensor task blocks on this before each read→publish→sleep cycle so it never
// light-sleeps — which would stall OpenThread's MLE attachment — before the OTBR has answered.
static constexpr EventBits_t BIT_ATTACHED = BIT0;
static EventGroupHandle_t s_ot_attached_eg = nullptr;

// Set once a NAT64 prefix has been learned from Thread network data. Only consulted when
// the configured broker address is IPv4 — an IPv6 broker never needs NAT64.
static constexpr EventBits_t BIT_PREFIX_KNOWN = BIT0;
// Set once Thread network data has at least one external route (the Border Router is
// publishing off-mesh routing), independent of NAT64. Consulted for IPv6 brokers — Network
// Data can lag a few hundred ms behind the CHILD-role transition, so a connect attempted
// right at attach can race a route table that's still empty.
static constexpr EventBits_t BIT_ROUTE_KNOWN = BIT1;
static EventGroupHandle_t s_prefix_eg = nullptr;

// NAT64 /96 prefix used to reach an IPv4 broker. Default: IANA well-known 64:ff9b::/96.
// Overridden at runtime via setNat64Prefix() once Thread network data arrives.
static uint8_t s_nat64_prefix[12] =
{
    0x00, 0x64, 0xff, 0x9b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// Thread dataset (hex TLV), captured at makeThreadLink() time and consumed by start().
static std::string s_ot_tlv_hex;

// Poll periods: fast during the MQTT publish window so TCP ACKs arrive promptly; slow
// the rest of the time — also the initial/steady-state SED poll period — to maximise sleep.
static constexpr uint32_t POLL_FAST_MS = 500;
static constexpr uint32_t POLL_SLOW_MS = 70000;

static void set_poll_period(uint32_t ms)
{
    esp_openthread_lock_acquire(portMAX_DELAY);
    otLinkSetPollPeriod(esp_openthread_get_instance(), ms);
    esp_openthread_lock_release();
}

static void setNat64Prefix(const uint8_t *p12)
{
    std::copy_n(p12, 12, s_nat64_prefix);
    if (s_prefix_eg)
        xEventGroupSetBits(s_prefix_eg, BIT_PREFIX_KNOWN);
}

static std::string make_nat64_uri(std::string_view ipv4, uint16_t port, bool use_tls)
{
    const auto octets = parseIpv4(ipv4);
    if (!octets) {
        ESP_LOGE(TAG, "bad IPv4 '%.*s'", static_cast<int>(ipv4.size()), ipv4.data());
        return {};
    }

    // Build full 128-bit IPv6 from 96-bit NAT64 prefix + 32-bit IPv4, then let OpenThread
    // format it (canonical, ::-compressed) instead of hand-pairing bytes into hextets.
    otIp6Address addr = {};
    std::copy_n(s_nat64_prefix, 12, addr.mFields.m8);   // 96-bit NAT64 prefix
    std::ranges::copy(*octets, addr.mFields.m8 + 12);   // 32-bit IPv4 -> low bytes

    char host[OT_IP6_ADDRESS_STRING_SIZE];
    otIp6AddressToString(&addr, host, sizeof(host));
    return std::format("{}://[{}]:{}", mqttScheme(use_tls), host, port);
}

static void log_thread_network_info()
{
    otInstance *ot = esp_openthread_get_instance();
    char buf[OT_IP6_ADDRESS_STRING_SIZE];

    ESP_LOGI(TAG, "=== Thread IPv6 addresses ===");
    for (const otNetifAddress *a = otIp6GetUnicastAddresses(ot); a; a = a->mNext) {
        otIp6AddressToString(&a->mAddress, buf, sizeof(buf));
        ESP_LOGI(TAG, "  %s/%u (preferred=%d)", buf, a->mPrefixLength, a->mPreferred);
    }

    ESP_LOGI(TAG, "=== Thread external routes ===");
    otNetworkDataIterator it = OT_NETWORK_DATA_ITERATOR_INIT;
    otExternalRouteConfig route;
    bool any = false;
    while (otNetDataGetNextRoute(ot, &it, &route) == OT_ERROR_NONE) {
        any = true;
        otIp6AddressToString(&route.mPrefix.mPrefix, buf, sizeof(buf));
        ESP_LOGI(TAG, "  %s/%u  nat64=%d stable=%d",
                 buf, route.mPrefix.mLength, route.mNat64, route.mStable);
        if (route.mNat64 && route.mPrefix.mLength == 96)
            setNat64Prefix(route.mPrefix.mPrefix.mFields.m8);
    }
    if (any) {
        if (s_prefix_eg)
            xEventGroupSetBits(s_prefix_eg, BIT_ROUTE_KNOWN);
    } else {
        ESP_LOGW(TAG, "  (none — NAT64 route not yet in network data)");
    }
}

// Re-scan current Thread network data for a NAT64 /96 route and update the prefix. Called
// from the sensor task (which holds no OpenThread lock) after a failed publish, so it
// acquires the lock itself — unlike log_thread_network_info(), which runs inside the already-
// locked state-changed callback. Lets a merely-stale NAT64 prefix recover without a reboot.
static void refresh_nat64_prefix()
{
    otInstance *ot = esp_openthread_get_instance();
    esp_openthread_lock_acquire(portMAX_DELAY);
    otNetworkDataIterator it = OT_NETWORK_DATA_ITERATOR_INIT;
    otExternalRouteConfig route;
    bool found = false;
    bool any = false;
    while (otNetDataGetNextRoute(ot, &it, &route) == OT_ERROR_NONE) {
        any = true;
        if (route.mNat64 && route.mPrefix.mLength == 96) {
            setNat64Prefix(route.mPrefix.mPrefix.mFields.m8);
            found = true;
        }
    }
    if (any && s_prefix_eg)
        xEventGroupSetBits(s_prefix_eg, BIT_ROUTE_KNOWN);
    esp_openthread_lock_release();
    if (!found)
        ESP_LOGW(TAG, "refresh_nat64_prefix: no NAT64 route in current network data");
}

static void process_state_change(otChangedFlags flags, void *context)
{
    otDeviceRole role = otThreadGetDeviceRole(esp_openthread_get_instance());
    switch (role) {
        case OT_DEVICE_ROLE_DISABLED:
            ESP_LOGI(TAG, "OT role: DISABLED");
            if (s_ot_attached_eg)
                xEventGroupClearBits(s_ot_attached_eg, BIT_ATTACHED);
            break;
        case OT_DEVICE_ROLE_DETACHED:
            ESP_LOGI(TAG, "OT role: DETACHED");
            if (s_ot_attached_eg)
                xEventGroupClearBits(s_ot_attached_eg, BIT_ATTACHED);
            break;
        case OT_DEVICE_ROLE_CHILD:
            ESP_LOGI(TAG, "OT role: CHILD — joined network as sleepy end device");
            log_thread_network_info();
            if (s_ot_attached_eg)
                xEventGroupSetBits(s_ot_attached_eg, BIT_ATTACHED);
            break;
        case OT_DEVICE_ROLE_ROUTER:
            ESP_LOGI(TAG, "OT role: ROUTER");
            if (s_ot_attached_eg)
                xEventGroupSetBits(s_ot_attached_eg, BIT_ATTACHED);
            break;
        case OT_DEVICE_ROLE_LEADER:
            ESP_LOGI(TAG, "OT role: LEADER");
            if (s_ot_attached_eg)
                xEventGroupSetBits(s_ot_attached_eg, BIT_ATTACHED);
            break;
    }
}

static bool wait_for_ot_attached(uint32_t timeout_ms)
{
    if (!s_ot_attached_eg)
        return false;

    const EventBits_t bits = xEventGroupWaitBits(
        s_ot_attached_eg, BIT_ATTACHED, pdFALSE, pdTRUE, pdMS_TO_TICKS(timeout_ms));

    return (bits & BIT_ATTACHED) != 0;
}

static void configure_ot_network(const std::string &ot_tlv_hex)
{
    otOperationalDatasetTlvs dataset_tlvs;
    if (!parse_dataset_tlvs(ot_tlv_hex, dataset_tlvs)) {
        ESP_LOGE(TAG, "Failed to parse OT TLV, cannot join network");
        return;
    }
    ESP_LOGI(TAG, "parsed TLV, size %u", static_cast<unsigned>(dataset_tlvs.mLength));

    esp_openthread_lock_acquire(portMAX_DELAY);

    // Force-set dataset from secrets.yaml every boot, overriding any stale NVS cache.
    // esp_openthread_auto_start() skips this if NVS already has a dataset, which can
    // leave the device on a stale mesh-local prefix from a previous network.
    if (otDatasetSetActiveTlvs(esp_openthread_get_instance(), &dataset_tlvs) != OT_ERROR_NONE)
        ESP_LOGE(TAG, "Failed to set OT dataset TLVs");

    otLinkModeConfig link_mode = {};
    link_mode.mRxOnWhenIdle = false;  // sleepy end device
    link_mode.mDeviceType   = false;  // MTD
    link_mode.mNetworkData  = false;  // minimal network data

    if (otLinkSetPollPeriod(esp_openthread_get_instance(), POLL_SLOW_MS) != OT_ERROR_NONE)
        ESP_LOGE(TAG, "Failed to set OT poll period");

    if (otThreadSetLinkMode(esp_openthread_get_instance(), link_mode) != OT_ERROR_NONE)
        ESP_LOGE(TAG, "Failed to set OT link mode");

    esp_openthread_lock_release();

    ESP_ERROR_CHECK(esp_openthread_auto_start(&dataset_tlvs));
    ESP_LOGI(TAG, "OT config done");
}

static esp_err_t start()
{
    // Created before the state-changed callback is registered so no early CHILD transition
    // (or NAT64 prefix) is missed.
    s_ot_attached_eg = xEventGroupCreate();
    s_prefix_eg = xEventGroupCreate();  // starts cleared: prefix unknown until learned

    esp_openthread_radio_config_t radio_config{};
    radio_config.radio_mode = RADIO_MODE_NATIVE;
    esp_openthread_host_connection_config_t host_config{};
    host_config.host_connection_mode = HOST_CONNECTION_MODE_NONE;

    static esp_openthread_config_t config = {
        .netif_config = ESP_NETIF_DEFAULT_OPENTHREAD(),
        .platform_config = {
            .radio_config = radio_config,
            .host_config = host_config,
            .port_config = {
                .storage_partition_name = "nvs",
                .netif_queue_size = 10,
                .task_queue_size = 10,
            }
        }
    };

    const esp_err_t err = esp_openthread_start(&config);
    if (err != ESP_OK)
        return err;

    esp_netif_set_default_netif(esp_openthread_get_netif());

    otSetStateChangedCallback(esp_openthread_get_instance(),
                              process_state_change,
                              esp_openthread_get_instance());

    configure_ot_network(s_ot_tlv_hex);
    return ESP_OK;
}

static bool waitForBrokerReachable(std::string_view broker_address, uint32_t timeout_ms)
{
    // NAT64 is only needed to reach an IPv4 broker; an IPv6 broker instead needs the Border
    // Router's off-mesh route to have actually landed in Thread network data — that can lag
    // a few hundred ms behind the CHILD-role transition, so wait for it rather than assuming
    // it's already there.
    if (looksLikeIpv6(broker_address)) {
        if (!s_prefix_eg ||
            !(xEventGroupWaitBits(s_prefix_eg, BIT_ROUTE_KNOWN, pdFALSE, pdFALSE,
                                  pdMS_TO_TICKS(timeout_ms)) & BIT_ROUTE_KNOWN)) {
            ESP_LOGE(TAG, "off-mesh route not learned within %lu ms", (unsigned long)timeout_ms);
            return false;
        }
        return true;
    }

    if (!s_prefix_eg ||
        !(xEventGroupWaitBits(s_prefix_eg, BIT_PREFIX_KNOWN, pdFALSE, pdFALSE,
                              pdMS_TO_TICKS(timeout_ms)) & BIT_PREFIX_KNOWN)) {
        ESP_LOGE(TAG, "NAT64 prefix not learned within %lu ms", (unsigned long)timeout_ms);
        return false;
    }
    return true;
}

static std::string brokerUri(std::string_view broker_address, uint16_t port, bool use_tls)
{
    if (looksLikeIpv6(broker_address))
        return std::format("{}://[{}]:{}", mqttScheme(use_tls), broker_address, port);
    return make_nat64_uri(broker_address, port, use_tls);
}

NetworkLink makeThreadLink(const NetworkLinkConfig &cfg)
{
    s_ot_tlv_hex = cfg.ot_tlv_hex;

    NetworkLink link;
    link.start = start;
    link.waitForReady = wait_for_ot_attached;
    link.brokerUri = brokerUri;
    link.waitForBrokerReachable = waitForBrokerReachable;
    link.onPublishWindowBegin = []() { set_poll_period(POLL_FAST_MS); };
    link.onPublishWindowEnd = []() { set_poll_period(POLL_SLOW_MS); };
    link.refresh = refresh_nat64_prefix;
    return link;
}
