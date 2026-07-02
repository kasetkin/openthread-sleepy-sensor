#pragma once

#include <cstdint>
#include <functional>
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

    // Recovery after a failed publish (OT: re-scan NAT64 route; Wi-Fi: reconnect kick).
    std::function<void()> refresh;
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
