#include "network_link.h"

#include "openthread_link.h"
#include "wifi_link.h"

NetworkLink makeNetworkLink(TransportKind kind, const NetworkLinkConfig &cfg)
{
    switch (kind) {
    case TransportKind::Wifi:
        return makeWifiLink(cfg);
    case TransportKind::Thread:
    default:
        return makeThreadLink(cfg);
    }
}
