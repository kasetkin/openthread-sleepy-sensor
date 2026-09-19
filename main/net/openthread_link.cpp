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
#include "ieee802154_rx_at_fix.h"
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

static void set_poll_period(uint32_t ms)
{
    esp_openthread_lock_acquire(portMAX_DELAY);
    otLinkSetPollPeriod(esp_openthread_get_instance(), ms);
    esp_openthread_lock_release();
}

// ── CSL (Coordinated Sampled Listening) ───────────────────────────────────────
// With CSL on, the parent sends our downlink frames into short receive windows we open once per
// CSL period, instead of holding them until our next data poll: bounded downlink latency, paid
// for with a wake plus a few ms of RX every period. It does not replace data polling --
// DataPollSender keeps running either way. Configured by device_config.yaml's csl_period_ms
// (0 = off).
//
// Engaged once per boot, from note_cycle_result(), on the first confirmed publish: never at
// attach and never inside a publish/OTA window (the MQTT task has already closed its window by
// the time a cycle result arrives). Three attempts in Aug 2026 lost downlink after engaging
// (all at a ~10.49 s period), so a fresh engagement is on probation:
// CSL_REVERT_AFTER_FAILED_CYCLES consecutive failures before CSL_TRUST_AFTER_OK_CYCLES
// consecutive successes switch it back off for the rest of the boot. Switching off is local and
// needs no downlink. OT sends the parent no Child Update Request for it, so the parent keeps
// aiming at our (now closed) CSL windows until its CSL timeout (OT default 100 s) expires --
// well inside one sensor cycle. Once trusted, a failed cycle is an ordinary outage and never
// reverts.
//
// OT keeps the period across detach/re-attach, so after a re-attach CSL resumes on its own.
enum class CslState { Off, Probation, Trusted, Reverted };

static constexpr uint32_t CSL_TRUST_AFTER_OK_CYCLES = 3;
static constexpr uint32_t CSL_REVERT_AFTER_FAILED_CYCLES = 2;

// Written once by makeThreadLink(), before any task runs; read-only afterwards.
static uint32_t s_csl_period_us = 0;
// Written by the sensor task (note_cycle_result()), read by the MQTT task (cslStatus()).
static std::atomic<CslState> s_csl_state{CslState::Off};
// Sensor task only.
static uint32_t s_csl_ok_streak = 0;
static uint32_t s_csl_fail_streak = 0;

// otLinkSetCslPeriod() takes a whole number of 160 us units (OT_ERROR_INVALID_ARGS otherwise)
// and silently clamps it to a uint16_t count (~10.49 s). Do both here, where the result can be
// logged, instead of letting a too-long request quietly become a different period.
static uint32_t csl_period_ms_to_us(uint32_t ms)
{
    constexpr uint32_t unit_us = OT_LINK_CSL_PERIOD_TEN_SYMBOLS_UNIT_IN_USEC;
    constexpr uint64_t max_us = uint64_t{UINT16_MAX} * unit_us;
    const uint32_t us = static_cast<uint32_t>(std::min<uint64_t>(uint64_t{ms} * 1000u, max_us));
    return us / unit_us * unit_us;
}

// Sets the CSL period (0 = off). Caller must not hold the OpenThread lock.
static bool set_csl_period_us(uint32_t period_us)
{
    otInstance *ot = esp_openthread_get_instance();

    esp_openthread_lock_acquire(portMAX_DELAY);
    const otError err = otLinkSetCslPeriod(ot, period_us);
    // The OT mainloop sized its select() timeout (up to 10 s) before blocking, and a call from
    // this task only arms OT's CSL alarm without waking it, so the first receive windows would
    // open late. A data poll posts a tasklet, which wakes the loop, and hands the parent our CSL
    // phase right away. Switching off needs no kick: the alarm is simply stopped.
    otError poll_err = OT_ERROR_NONE;
    if (err == OT_ERROR_NONE && period_us != 0)
        poll_err = otLinkSendDataRequest(ot);
    esp_openthread_lock_release();

    if (err != OT_ERROR_NONE) {
        ESP_LOGE(TAG, "otLinkSetCslPeriod(%lu us) failed: %d",
                 static_cast<unsigned long>(period_us), static_cast<int>(err));
        return false;
    }
    if (poll_err != OT_ERROR_NONE)
        ESP_LOGW(TAG, "CSL: wake-up data poll failed: %d (loop wakes within 10 s anyway)",
                 static_cast<int>(poll_err));
    return true;
}

// One line of cumulative MAC counters plus CSL state, to tell apart the ways CSL can break
// downlink: rx data/dup/err_sec show whether the parent's frames reach us (dup climbing = the
// parent retransmits because it doesn't accept our enhanced ACKs); tx abort counts transmits
// cut short when a CSL receive window opens mid-TX (an ESP radio-port quirk); rxat_fix (PART B)
// and rxat_skip (PART A) count the driver workarounds in ieee802154_rx_at_fix.cpp firing.
// Cumulative so it can't disturb read_link_stats()'s per-cycle deltas -- compare two lines.
static void log_csl_diag(const char *reason)
{
    otInstance *ot = esp_openthread_get_instance();
    otMacCounters mac = {};

    esp_openthread_lock_acquire(portMAX_DELAY);
    if (const otMacCounters *counters = otLinkGetCounters(ot))
        mac = *counters;
    const bool enabled = otLinkIsCslEnabled(ot);
    const uint32_t period_us = otLinkGetCslPeriod(ot);
    const uint32_t rx_at_fixes = ieee802154_rx_at_fix_count();
    const uint32_t rx_at_skips = ieee802154_rx_at_skip_count();
    esp_openthread_lock_release();

    ESP_LOGW(TAG, "CSL %s: enabled=%d period=%lu us | tx=%lu poll=%lu retry=%lu abort=%lu "
                  "cca_fail=%lu no_ack=%lu | rx=%lu data=%lu dup=%lu err_sec=%lu err_fcs=%lu "
                  "no_frame=%lu | rxat_fix=%lu rxat_skip=%lu",
             reason, enabled, static_cast<unsigned long>(period_us),
             static_cast<unsigned long>(mac.mTxTotal),
             static_cast<unsigned long>(mac.mTxDataPoll),
             static_cast<unsigned long>(mac.mTxRetry),
             static_cast<unsigned long>(mac.mTxErrAbort),
             static_cast<unsigned long>(mac.mTxErrCca),
             static_cast<unsigned long>(mac.mTxDirectMaxRetryExpiry),
             static_cast<unsigned long>(mac.mRxTotal),
             static_cast<unsigned long>(mac.mRxData),
             static_cast<unsigned long>(mac.mRxDuplicated),
             static_cast<unsigned long>(mac.mRxErrSec),
             static_cast<unsigned long>(mac.mRxErrFcs),
             static_cast<unsigned long>(mac.mRxErrNoFrame),
             static_cast<unsigned long>(rx_at_fixes),
             static_cast<unsigned long>(rx_at_skips));
}

// Called once per sensor cycle with its real outcome (see NetworkLink::noteCycleResult).
static void note_cycle_result(bool ok)
{
    if (s_csl_period_us == 0)
        return;

    switch (s_csl_state.load(std::memory_order_relaxed)) {
        case CslState::Off:
            if (!ok)
                return;
            s_csl_ok_streak = 0;
            s_csl_fail_streak = 0;
            if (set_csl_period_us(s_csl_period_us)) {
                s_csl_state.store(CslState::Probation, std::memory_order_relaxed);
                log_csl_diag("engaged after first confirmed publish, on probation");
            } else {
                s_csl_state.store(CslState::Reverted, std::memory_order_relaxed);
            }
            return;

        case CslState::Probation:
            if (ok) {
                s_csl_fail_streak = 0;
                if (++s_csl_ok_streak >= CSL_TRUST_AFTER_OK_CYCLES) {
                    s_csl_state.store(CslState::Trusted, std::memory_order_relaxed);
                    ESP_LOGI(TAG, "CSL: trusted after %lu confirmed cycles "
                                  "(rxat_fix=%lu rxat_skip=%lu)",
                             static_cast<unsigned long>(s_csl_ok_streak),
                             static_cast<unsigned long>(ieee802154_rx_at_fix_count()),
                             static_cast<unsigned long>(ieee802154_rx_at_skip_count()));
                }
                return;
            }
            s_csl_ok_streak = 0;
            log_csl_diag("cycle failed during probation");
            if (++s_csl_fail_streak >= CSL_REVERT_AFTER_FAILED_CYCLES) {
                set_csl_period_us(0);
                s_csl_state.store(CslState::Reverted, std::memory_order_relaxed);
                log_csl_diag("reverted, off for the rest of this boot");
            }
            return;

        case CslState::Trusted:
            if (!ok)
                log_csl_diag("cycle failed (trusted, not reverting)");
            return;

        case CslState::Reverted:
            return;
    }
}

// "enabled"/"reverted" (see CslState above), else whether the current parent could do CSL at
// all (Mle::IsCslSupported(): attached AND parent is Thread 1.2+).
static std::string_view cslStatus()
{
    otInstance *ot = esp_openthread_get_instance();

    esp_openthread_lock_acquire(portMAX_DELAY);
    const otDeviceRole role = otThreadGetDeviceRole(ot);
    const bool supported = otLinkIsCslSupported(ot);
    const bool enabled = otLinkIsCslEnabled(ot);
    esp_openthread_lock_release();

    if (s_csl_state.load(std::memory_order_relaxed) == CslState::Reverted)
        return "reverted";
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

    s_csl_period_us = csl_period_ms_to_us(cfg.csl_period_ms);
    if (cfg.csl_period_ms != 0)
        ESP_LOGI(TAG, "CSL: csl_period_ms=%lu -> %lu us, engaged after the first confirmed publish",
                 static_cast<unsigned long>(cfg.csl_period_ms),
                 static_cast<unsigned long>(s_csl_period_us));

    NetworkLink link;
    link.start = start;
    link.waitForReady = wait_for_ot_attached;
    link.brokerUri = brokerUri;
    link.waitForBrokerReachable = waitForBrokerReachable;
    link.onPublishWindowBegin = []() { set_poll_period(POLL_FAST_MS); };
    link.onPublishWindowEnd = []() { set_poll_period(POLL_SLOW_MS); };
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
    link.noteCycleResult = note_cycle_result;
    return link;
}
