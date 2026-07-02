#pragma once

#include "network_link.h"

// Builds a NetworkLink backed by Wi-Fi STA (WPA2). Brings up only the single IP family
// selected by cfg.wifi_address_family ("ipv4" -> DHCP, "ipv6" -> SLAAC) — not dual-stack.
NetworkLink makeWifiLink(const NetworkLinkConfig &cfg);
