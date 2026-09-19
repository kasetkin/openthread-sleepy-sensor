#pragma once

#include <cstdint>

// Diagnostic counters of the 802.15.4 driver workarounds in ieee802154_rx_at_fix.cpp. Both stay
// 0 with CSL off, and each climbs roughly once per data poll while CSL is on.

// PART B (keep): immediate receives that had to cancel a still-pending CSL receive window first.
uint32_t ieee802154_rx_at_fix_count();

// PART A (backport, delete on ESP-IDF >= v6.0.4 / >= v6.1.1): CSL receive windows skipped because
// they had already ended. When PART A goes, drop this and its rxat_skip log field too.
uint32_t ieee802154_rx_at_skip_count();
