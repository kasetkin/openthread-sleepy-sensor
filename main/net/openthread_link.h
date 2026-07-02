#pragma once

#include "network_link.h"

// Builds a NetworkLink backed by OpenThread. Reaches an IPv4 broker via NAT64
// (the Border Router's IPv4<->IPv6 translation for the IPv6-only Thread mesh);
// reaches an IPv6 broker directly via the Border Router's normal off-mesh
// IPv6 routing, no NAT64 involved.
NetworkLink makeThreadLink(const NetworkLinkConfig &cfg);
