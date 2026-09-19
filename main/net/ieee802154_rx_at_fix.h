#pragma once

#include <cstdint>

// How many times an immediate 802.15.4 receive had to cancel a scheduled CSL receive window
// first -- see ieee802154_rx_at_fix.cpp. Non-zero proves the workaround is linked in and firing
// (roughly once per data poll while CSL is on); stays 0 with CSL off.
uint32_t ieee802154_rx_at_fix_count();
