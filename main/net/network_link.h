#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include "esp_err.h"

// Naive family sniff: every valid IPv6 text form contains ':' and no valid
// IPv4 dotted-quad does. Not a validator — malformed input still fails
// naturally at connect time. Shared by mqtt_sender.cpp and both NetworkLink
// implementations so the family decision is made the same way everywhere.
inline bool looksLikeIpv6(std::string_view addr)
{
    return addr.find(':') != std::string_view::npos;
}

// Picks the MQTT URI scheme for a given transport-security mode. Shared by every
// NetworkLink implementation's brokerUri() so "mqtt" vs "mqtts" is decided in exactly
// one place, mirroring looksLikeIpv6()'s role for address-family sniffing above.
inline std::string_view mqttScheme(bool use_tls)
{
    return use_tls ? "mqtts" : "mqtt";
}

// One publish window's view of the radio link, read in a single pass so the whole set is
// mutually consistent (and, for Thread, costs one OpenThread lock acquisition instead of one
// per field). Every field is optional because the two transports can produce very different
// subsets: Wi-Fi fills only rssiDbm, and even on Thread uplinkRssiDbm stays empty unless the
// border router turns out to speak Thread 1.2 link metrics.
//
// The two RSSI fields point in OPPOSITE directions, which is the whole reason this struct
// exists. rssiDbm is what our receiver measures of the parent's transmitter -- it says nothing
// about how well we are heard, and in particular it cannot respond to our own TX power.
// uplinkRssiDbm is the parent's measurement of US, reported back in enhanced ACKs, and is the
// only direct evidence of our uplink margin. linkQualityOut is the same direction as
// uplinkRssiDbm but far coarser (0-3), and is the fallback when link metrics are unavailable.
//
// The counter-derived fields are PER-CYCLE DELTAS, not the cumulative totals OpenThread
// actually stores -- openthread_link.cpp does the differencing, so consumers can treat every
// field as "this cycle" without knowing which underlying API is cumulative.
struct LinkStats
{
    std::optional<int> rssiDbm;             // downlink: RSSI we measure of the parent
    std::optional<int> uplinkRssiDbm;       // uplink: RSSI the PARENT measures of us
    std::optional<uint8_t> linkQualityOut;  // uplink, coarse: parent's 0-3 grade of our link
    std::optional<uint32_t> radioTxTimeUs;  // radio time spent transmitting
    std::optional<uint32_t> radioRxTimeUs;  // radio time spent receiving
    std::optional<uint32_t> txRetries;      // MAC-layer retransmissions
    std::optional<uint32_t> txCcaFailures;  // transmits abandoned because the channel was busy
    std::optional<uint32_t> txNoAckExpiry;  // transmits that exhausted every retry unacked
};

// Abstracts "how do I reach an IP-connected MQTT broker" over OpenThread or
// Wi-Fi so mqtt_sender.cpp and SensorsTask stay transport-agnostic. A plain
// struct of std::function seams (matching SensorsTask's configureAttachGate/
// configureReadyEvent style) rather than a virtual interface.
struct NetworkLink
{
    // Bring the stack up once at boot.
    std::function<esp_err_t()> start;

    // Blocks up to timeoutMs until usable (OT: CHILD role; Wi-Fi: got address
    // in the selected family). Called once per sensor cycle as the attach gate.
    std::function<bool(uint32_t timeoutMs)> waitForReady;

    // Builds the MQTT broker URI for broker_address (a literal IPv4 or IPv6
    // address — no DNS names). use_tls selects the "mqtt"/"mqtts" scheme.
    std::function<std::string(std::string_view broker_address, uint16_t port, bool use_tls)> brokerUri;

    // Blocks up to timeoutMs until broker_address's family is reachable
    // (OT: NAT64 prefix learned, only for an IPv4 broker; Wi-Fi: always true
    // immediately, since waitForReady plus the boot-time family check already
    // guarantee it).
    std::function<bool(std::string_view broker_address, uint32_t timeoutMs)> waitForBrokerReachable;

    // Publish-window hooks (OT: fast/slow poll period; Wi-Fi: no-op).
    std::function<void()> onPublishWindowBegin;
    std::function<void()> onPublishWindowEnd;

    // OTA-download hooks (OT: a much faster data-poll period so parent-buffered image
    // chunks flow without tripping esp-mqtt's ~1 s mid-message stall abort; Wi-Fi: no-op).
    // Deliberately NOT rx-on-when-idle — see openthread_link.cpp for the hardware-observed
    // downlink black hole that mode switch causes. onOtaWindowEnd MUST run on every abort
    // path: a child left at a 50 ms poll burns the battery ~1000x faster than the slow
    // period. Nested inside a publish window (begin after onPublishWindowBegin, end before
    // onPublishWindowEnd), so the period it restores to is the window's fast one.
    std::function<void()> onOtaWindowBegin;
    std::function<void()> onOtaWindowEnd;

    // Recovery after a failed publish (OT: re-scan NAT64 route; Wi-Fi: reconnect kick).
    std::function<void()> refresh;

    // Sets the radio's transmit power ceiling in dBm (OT: otPlatRadioSetTransmitPower under the
    // OT lock; Wi-Fi: esp_wifi_set_max_tx_power, quarter-dBm units -- a real conversion, done in
    // wifi_link.cpp). Populated on both transports, unlike the diagnostic-only seams that fall
    // back to wifi_link.cpp's shared noop().
    std::function<esp_err_t(int8_t dbm)> setTxPowerDbm;

    // TODO(deferred, see memory note project_blackout_resilience_redesign.md): a lightweight
    // "soft-rejoin" member was designed but not built -- OT: otThreadSetEnabled() false->true
    // to force a fresh MLE detach+attach on the same otInstance; Wi-Fi: esp_wifi_disconnect()+
    // esp_wifi_connect(). Would target OT's own attach state machine getting stuck in a way
    // plain waiting doesn't clear, separate from a genuine no-network blackout (where
    // sensorstask.cpp's reboot supervisor now deliberately never reboots -- see its
    // LP_STALL_REBOOT_THRESHOLD doc comment) and from an LP-core stall (unrelated subsystem).
    // Revisit only on request -- not wired up anywhere yet.

    // This cycle's link telemetry (see LinkStats above), nullopt when the link is unusable
    // altogether (detached/disconnected). Called ONCE per MQTT publish window, so the radio is
    // awake and the readings are fresh -- and so the delta fields cover exactly one cycle.
    // Calling it more than once per cycle would split those deltas across the calls.
    std::function<std::optional<LinkStats>()> readLinkStats;

    // Fired once, idempotently, the first time this boot that waitForReady() succeeds -- for
    // one-time instance-level radio setup that shouldn't be redone on every re-attach (OT: CSL
    // period/timeout negotiation; Wi-Fi: no-op). Idempotency lives in the implementation, not
    // the caller -- mirrors runtime_config_tx_power_note_first_attach()'s "safe to call every
    // successful attach" contract.
    std::function<void()> onFirstAttach;

    // CSL negotiation snapshot with the current parent: "enabled"/"supported"/"unsupported"/
    // "detached" (OT), or "n/a" (Wi-Fi -- no Thread-CSL equivalent). Not folded into LinkStats:
    // it's attach-scoped state, not a per-cycle telemetry delta.
    std::function<std::string_view()> cslStatus;
};

struct NetworkLinkConfig
{
    // Thread dataset (hex TLV). The mesh-local prefix lives inside this blob;
    // there's no separate per-device "static Thread address" — the
    // OMR/global address is Border-Router-owned, not device-configurable.
    // Default members below (rather than bare `std::string x;`) so a caller
    // constructing a Thread-only or Wi-Fi-only config via designated
    // initializers isn't forced to spell out every field (-Wmissing-field-initializers).
    std::string ot_tlv_hex = "";

    // Wi-Fi WPA2 STA credentials.
    std::string wifi_ssid = "";
    std::string wifi_password = "";

    // Which single IP family the Wi-Fi STA brings up: "ipv4" (DHCP) or
    // "ipv6" (SLAAC). The other family is not brought up at all. Explicitly
    // selected, not derived from the broker address, so it survives future
    // DNS-name broker support (a hostname has no family until resolved).
    std::string wifi_address_family = "ipv4";
};

enum class TransportKind { Thread, Wifi };

// Dispatches to makeThreadLink()/makeWifiLink() (declared in
// openthread_link.h/wifi_link.h) based on kind.
NetworkLink makeNetworkLink(TransportKind kind, const NetworkLinkConfig &cfg);
