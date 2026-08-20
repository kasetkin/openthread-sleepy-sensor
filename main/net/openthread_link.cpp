#include "openthread_link.h"

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <format>
#include <optional>
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
#include "openthread/link_metrics.h"
#include "openthread/netdata.h"
#include "openthread/platform/radio.h"
#include "openthread/radio_stats.h"
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
// OTA gets its own, much faster period: an OTA image chunk (~8 KB ≈ 7 TCP segments of
// parent-buffered downlink) must flow with gaps well under esp-mqtt's ~1 s mid-message
// no-progress abort, and the nominal period bounds the first-frame latency of every burst
// (frame-pending chaining keeps subsequent polls back-to-back on its own).
static constexpr uint32_t POLL_FAST_MS = 500;
static constexpr uint32_t POLL_OTA_MS  = 50;
static constexpr uint32_t POLL_SLOW_MS = 70000;

// otLinkSetCslPeriod() stores the period as a uint16_t count of 160 us units internally
// (link_api.cpp: ClampToUint16(aPeriod / OT_US_PER_TEN_SYMBOLS)) -- 65535 * 160 us =
// 10,485,600 us is a hard ceiling that gets silently CLAMPED to, not rejected, if exceeded
// (hardware-confirmed: an earlier 70,000,000 us value, meant to match POLL_SLOW_MS's idle
// cadence exactly, returned OT_ERROR_NONE but never actually ran at 70 s). That cadence is
// unreachable via this API, so the closest achievable "as infrequent as possible" value is
// used instead -- CSL wakes more often (~10.5 s) than today's classic idle Data-Poll (70 s),
// so any power comparison must account for that difference, not assume parity.
static constexpr uint32_t CSL_PERIOD_US = 65535u * 160u;
// Three missed CSL windows' worth of silence before OT gives up on the parent and forces a
// re-attach -- independent of (and doesn't replace) the MLE child timeout, which still governs
// classic keepalive/detach. A first-pass value, not yet hardware-tuned.
static constexpr uint32_t CSL_TIMEOUT_SEC = 3 * CSL_PERIOD_US / 1000000;

static void set_poll_period(uint32_t ms)
{
    esp_openthread_lock_acquire(portMAX_DELAY);
    otLinkSetPollPeriod(esp_openthread_get_instance(), ms);
    esp_openthread_lock_release();
}

// Engaged exactly once per boot, on the first cycle that already proved a full connect-publish-
// broker-ACK round trip works (cycleOk) -- NOT at first attach. otLinkSetCslPeriod(nonzero) is
// not passive: a sleepy child is auto-marked CSL-capable at attach, so this call immediately
// reprograms the radio's real receive schedule via otPlatRadioEnableCsl(). Doing that before the
// boot's first connection attempt (the original design) hardware-bricked two OTA flashes --
// see the CSL integration plan/memory for the incident. Gating on cycleOk instead means the
// negotiation "landing window" falls in idle time after a already-proven-working cycle, not on
// top of the most fragile connection attempt of the boot.
static bool s_csl_engaged = false;

// Idle-case poll period, decided fresh every cycle from LIVE negotiation state (never
// persisted): otLinkSetPollPeriod(instance, 0) clears the user override, the only way
// DataPollSender::GetDefaultPollPeriod()'s CSL-aware branch takes over. Falling back to
// POLL_SLOW_MS whenever otLinkIsCslEnabled() is false means a parent that drops CSL (or hasn't
// negotiated it yet) never loses classic polling as a fallback -- self-healing across a parent
// change without needing a persisted revert.
static void set_idle_poll_period(bool cycleOk)
{
    esp_openthread_lock_acquire(portMAX_DELAY);
    otInstance *ot = esp_openthread_get_instance();

    if (!s_csl_engaged && cycleOk) {
        s_csl_engaged = true;
        const otError periodErr = otLinkSetCslPeriod(ot, CSL_PERIOD_US);
        const otError timeoutErr = otLinkSetCslTimeout(ot, CSL_TIMEOUT_SEC);
        if (periodErr != OT_ERROR_NONE || timeoutErr != OT_ERROR_NONE)
            ESP_LOGE(TAG, "CSL engage failed: period=%d timeout=%d",
                     static_cast<int>(periodErr), static_cast<int>(timeoutErr));
        else
            ESP_LOGI(TAG, "CSL engaged after first successful publish: period=%lu us timeout=%lu s",
                     (unsigned long)CSL_PERIOD_US, (unsigned long)CSL_TIMEOUT_SEC);
    }

    const bool cslActive = otLinkIsCslEnabled(ot);
    const otError err = otLinkSetPollPeriod(ot, cslActive ? 0 : POLL_SLOW_MS);
    esp_openthread_lock_release();

    if (err != OT_ERROR_NONE)
        ESP_LOGE(TAG, "Failed to set OT idle poll period (csl_active=%d)", cslActive);
}

// Snapshot of CSL negotiation with the current parent -- Thread-only concept, no Wi-Fi
// equivalent. Not folded into LinkStats: it's attach-scoped state, not a per-cycle telemetry
// delta, so a bare accessor (matching hp_awake_stats_get_and_reset_us()'s shape) fits better
// than that struct's "read once, differenced against last cycle" contract.
static std::string_view cslStatus()
{
    otInstance *ot = esp_openthread_get_instance();

    esp_openthread_lock_acquire(portMAX_DELAY);
    const otDeviceRole role = otThreadGetDeviceRole(ot);
    const bool enabled = otLinkIsCslEnabled(ot);
    const bool supported = otLinkIsCslSupported(ot);
    esp_openthread_lock_release();

    if (role == OT_DEVICE_ROLE_DISABLED || role == OT_DEVICE_ROLE_DETACHED)
        return "detached";
    if (enabled)
        return "enabled";
    return supported ? "supported" : "unsupported";
}

static esp_err_t set_tx_power_dbm(int8_t dbm)
{
    esp_openthread_lock_acquire(portMAX_DELAY);
    const otError err = otPlatRadioSetTransmitPower(esp_openthread_get_instance(), dbm);
    esp_openthread_lock_release();
    if (err != OT_ERROR_NONE) {
        ESP_LOGE(TAG, "otPlatRadioSetTransmitPower(%d) failed: %d", dbm, static_cast<int>(err));
        return ESP_FAIL;
    }
    return ESP_OK;
}

// ── uplink telemetry ──────────────────────────────────────────────────────────
// Latest RSSI the PARENT reported measuring of us, from the enhanced-ACK report callback
// below. A sentinel int rather than a std::optional because it is written from the OpenThread
// task and read from the MQTT task: a lock-free atomic<int16_t> load/store is well-defined
// across that boundary, while an optional's value/engaged pair would be two unsynchronised
// stores. INT16_MIN is safely outside the int8_t range every real RSSI arrives in.
static constexpr int16_t UPLINK_RSSI_NONE = INT16_MIN;
static std::atomic<int16_t> s_uplink_rssi_dbm{UPLINK_RSSI_NONE};

// Cumulative counters OpenThread exposes, sampled at the end of the previous read_link_stats().
// LinkStats reports per-cycle deltas, so the differencing happens here and consumers never see
// a running total. Only ever touched from the MQTT task under the OT lock.
struct CounterSnapshot
{
    uint64_t radio_tx_time_us = 0;
    uint64_t radio_rx_time_us = 0;
    uint32_t tx_retries = 0;
    uint32_t tx_cca_failures = 0;
    uint32_t tx_no_ack_expiry = 0;
    bool valid = false;  // false until the first sample; the first cycle reports no deltas
};
static CounterSnapshot s_prev_counters;

// Parent's link-local address, needed as the enhanced-ACK probing destination. OpenThread has
// no getter for it (otThreadGetLinkLocalIp6Address returns OUR address), so it is derived the
// standard way: fe80::/64 plus the modified EUI-64 form of the parent's extended address,
// which is the extended address with the universal/local bit of its first byte inverted.
// Caller must hold the OpenThread lock.
static bool parent_link_local_address(otInstance *ot, otIp6Address &out)
{
    otRouterInfo parent;
    if (otThreadGetParentInfo(ot, &parent) != OT_ERROR_NONE)
        return false;

    out = {};
    out.mFields.m8[0] = 0xfe;
    out.mFields.m8[1] = 0x80;
    std::copy_n(parent.mExtAddress.m8, OT_EXT_ADDRESS_SIZE, out.mFields.m8 + 8);
    out.mFields.m8[8] ^= 0x02;
    return true;
}

// Enhanced-ACK Based Probing report: the parent stamps its own measurement of each frame we
// send into the ACK it returns, which is the ONLY direct evidence this device can get of its
// uplink margin. Runs in OpenThread task context.
static void enh_ack_report_cb(otShortAddress /*aShortAddress*/,
                              const otExtAddress * /*aExtAddress*/,
                              const otLinkMetricsValues *aMetricsValues,
                              void * /*aContext*/)
{
    if (aMetricsValues && aMetricsValues->mMetrics.mRssi)
        s_uplink_rssi_dbm.store(aMetricsValues->mRssiValue, std::memory_order_relaxed);
}

static void enh_ack_mgmt_response_cb(const otIp6Address * /*aSource*/,
                                     otLinkMetricsStatus aStatus,
                                     void * /*aContext*/)
{
    ESP_LOGI(TAG, "enh-ACK probing management response: status %d", static_cast<int>(aStatus));
}

// Registers Enhanced-ACK Based Probing with the current parent. Called on every CHILD
// transition rather than once per boot on purpose: the registration lives in the parent's
// neighbour state, so it does not survive a parent change or a re-attach.
//
// Purely diagnostic — every failure path here is logged and swallowed. In particular a border
// router that is not a Thread 1.2 device answers OT_ERROR_NOT_CAPABLE, which is an expected
// outcome, not an error: uplinkRssiDbm simply stays absent and LinkStats::linkQualityOut plus
// the retry counters remain the only uplink evidence. Nothing here may ever fail an attach.
//
// Caller must already hold the OpenThread lock (process_state_change() does).
static void register_enh_ack_probing(otInstance *ot)
{
    s_uplink_rssi_dbm.store(UPLINK_RSSI_NONE, std::memory_order_relaxed);

    otIp6Address parent_addr;
    if (!parent_link_local_address(ot, parent_addr)) {
        ESP_LOGW(TAG, "enh-ACK probing: parent info unavailable, uplink RSSI stays unknown");
        return;
    }

    otLinkMetrics metrics = {};
    metrics.mRssi = true;
    metrics.mLinkMargin = true;

    const otError err = otLinkMetricsConfigEnhAckProbing(
        ot, &parent_addr, OT_LINK_METRICS_ENH_ACK_REGISTER, &metrics,
        enh_ack_mgmt_response_cb, nullptr, enh_ack_report_cb, nullptr);

    if (err == OT_ERROR_NONE) {
        ESP_LOGI(TAG, "enh-ACK probing registered with parent — uplink RSSI available");
    } else if (err == OT_ERROR_NOT_CAPABLE) {
        ESP_LOGW(TAG, "enh-ACK probing unsupported by parent (not a Thread 1.2 device); "
                      "falling back to link quality out + retry counters");
    } else {
        ESP_LOGW(TAG, "enh-ACK probing registration failed: %d", static_cast<int>(err));
    }
}

// This cycle's view of the link. Called once per MQTT publish window from the MQTT task, so it
// takes the OT lock like set_poll_period() — once, for the whole set, rather than per field.
//
// Note the two RSSIs measure opposite directions (see LinkStats in network_link.h): rssiDbm is
// the parent's transmitter as heard by us, uplinkRssiDbm is us as heard by the parent.
static std::optional<LinkStats> read_link_stats()
{
    otInstance *ot = esp_openthread_get_instance();
    LinkStats stats;

    esp_openthread_lock_acquire(portMAX_DELAY);

    int8_t rssi = 0;
    if (otThreadGetParentLastRssi(ot, &rssi) == OT_ERROR_NONE)
        stats.rssiDbm = rssi;

    otRouterInfo parent;
    if (otThreadGetParentInfo(ot, &parent) == OT_ERROR_NONE)
        stats.linkQualityOut = parent.mLinkQualityOut;

    // Cumulative us since boot; deltas below. Deliberately NOT otRadioTimeStatsReset() — a
    // reset would discard everything the radio does between this read and the next window
    // (the data polls that make up most of a sleepy cycle), which is exactly the time we are
    // trying to account for.
    const otRadioTimeStats *radio = otRadioTimeStatsGet(ot);
    const otMacCounters *mac = otLinkGetCounters(ot);

    CounterSnapshot now;
    now.valid = true;
    if (radio) {
        now.radio_tx_time_us = radio->mTxTime;
        now.radio_rx_time_us = radio->mRxTime;
    }
    if (mac) {
        now.tx_retries = mac->mTxRetry;
        now.tx_cca_failures = mac->mTxErrCca;
        now.tx_no_ack_expiry = mac->mTxDirectMaxRetryExpiry;
    }

    esp_openthread_lock_release();

    // First call of the boot establishes the baseline and reports no deltas — a "delta" against
    // zero would really be a since-boot total and would badly skew the very measurement this
    // instrumentation exists to make.
    if (s_prev_counters.valid) {
        if (radio) {
            stats.radioTxTimeUs = static_cast<uint32_t>(now.radio_tx_time_us - s_prev_counters.radio_tx_time_us);
            stats.radioRxTimeUs = static_cast<uint32_t>(now.radio_rx_time_us - s_prev_counters.radio_rx_time_us);
        }
        if (mac) {
            stats.txRetries = now.tx_retries - s_prev_counters.tx_retries;
            stats.txCcaFailures = now.tx_cca_failures - s_prev_counters.tx_cca_failures;
            stats.txNoAckExpiry = now.tx_no_ack_expiry - s_prev_counters.tx_no_ack_expiry;
        }
    }
    s_prev_counters = now;

    const int16_t uplink = s_uplink_rssi_dbm.load(std::memory_order_relaxed);
    if (uplink != UPLINK_RSSI_NONE)
        stats.uplinkRssiDbm = uplink;

    // Nothing readable at all means detached, not merely uninstrumented — report absence so the
    // caller skips the whole group rather than publishing a row of zeroes.
    if (!stats.rssiDbm && !stats.linkQualityOut && !stats.radioTxTimeUs)
        return std::nullopt;

    ESP_LOGI(TAG, "link: rssi=%d dBm uplink_rssi=%d lqo=%u tx=%lu us rx=%lu us "
                  "retries=%lu cca_fail=%lu no_ack=%lu",
             stats.rssiDbm.value_or(0), stats.uplinkRssiDbm.value_or(0),
             static_cast<unsigned>(stats.linkQualityOut.value_or(0)),
             static_cast<unsigned long>(stats.radioTxTimeUs.value_or(0)),
             static_cast<unsigned long>(stats.radioRxTimeUs.value_or(0)),
             static_cast<unsigned long>(stats.txRetries.value_or(0)),
             static_cast<unsigned long>(stats.txCcaFailures.value_or(0)),
             static_cast<unsigned long>(stats.txNoAckExpiry.value_or(0)));

    return stats;
}

// OTA-download link boost. Deliberately just a faster data-poll cadence, NOT
// rx-on-when-idle: flipping mRxOnWhenIdle mid-attach was tried and hardware-observed to
// black-hole downlink right after the switch (the child stops polling immediately while
// the parent still queues frames for a "sleepy" child until the MLE mode renegotiation —
// and its CSL scheduling — fully lands), which trips esp-mqtt's ~1 s mid-message
// no-progress abort on the very first chunk, every time. Fast polling is the same
// mechanism every ordinary publish window uses, so there is no mode change to renegotiate
// and no new radio state to trust; frame-pending chaining keeps the effective chunk
// throughput far above the nominal period anyway.

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
            // Re-registered on every CHILD transition, not just the boot's first: the
            // registration is neighbour state on the parent and does not survive a re-attach
            // or a parent change. Already inside the OT lock here.
            register_enh_ack_probing(esp_openthread_get_instance());
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
    link.onPublishWindowEnd = [](bool ok) { set_idle_poll_period(ok); };
    // Nested inside a publish window, so the end hook restores the window's fast poll;
    // the window's own end hook then drops back to slow.
    link.onOtaWindowBegin = []() { ESP_LOGI(TAG, "OTA window: poll %lu ms", (unsigned long)POLL_OTA_MS);
                                   set_poll_period(POLL_OTA_MS); };
    link.onOtaWindowEnd = []() { ESP_LOGI(TAG, "OTA window end: poll %lu ms", (unsigned long)POLL_FAST_MS);
                                 set_poll_period(POLL_FAST_MS); };
    link.refresh = refresh_nat64_prefix;
    link.readLinkStats = read_link_stats;
    link.setTxPowerDbm = set_tx_power_dbm;
    link.cslStatus = cslStatus;
    return link;
}
