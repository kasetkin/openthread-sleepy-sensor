#pragma once

#include <cstdint>

// Diagnostic counters of the 802.15.4 workarounds in ieee802154_rx_at_fix.cpp. All stay 0 with
// CSL off; while CSL is on, PARTs A and B each climb roughly once per data poll.

// PART B (keep): immediate receives that had to cancel a still-pending CSL receive window first.
uint32_t ieee802154_rx_at_fix_count();

// PART A (backport, delete on ESP-IDF >= v6.0.4 / >= v6.1.1): CSL receive windows skipped because
// they had already ended. When PART A goes, drop this and its rxat_skip log field too.
uint32_t ieee802154_rx_at_skip_count();

// PART C (backport of OpenThread #13472, delete once ESP-IDF's OpenThread includes it):
// continuous receives the MAC went idle on while CSL was on, which the radio would otherwise have
// kept listening through until the next CSL window. Stays 0 without CSL.
uint32_t ieee802154_idle_rx_stop_count();

// ── Diagnostics (not a workaround; delete once the CSL power question is settled) ─────────────
// Every CSL receive window OpenThread asks for, skipped ones included. Divided by the cycle's
// wall-clock this says whether CSL really runs all cycle (~2 windows/s at a 500 ms period) or
// only around the data polls.
uint64_t ieee802154_rx_at_total_count();

// Summed requested window width, in us. Over ieee802154_rx_at_total_count() it gives the mean
// width, which is what CONFIG_OPENTHREAD_CSL_ACCURACY and the time since the last sync widen.
uint64_t ieee802154_rx_at_window_us();

// Summed arm-ahead time (window start minus now at the call), in us, negatives excluded. The
// driver sets state RX when the window is *armed*, and esp_openthread_sleep.c holds the PM lock
// until the radio reads back SLEEP, so this is the light sleep each window costs us.
uint64_t ieee802154_rx_at_lead_us();

// Windows armed after their start time had already passed (but before their end -- fully expired
// ones are counted by rxat_skip instead). Climbing means we are arming windows we are already
// inside, i.e. late, which is the shape clock drift would have.
uint32_t ieee802154_rx_at_late_count();
