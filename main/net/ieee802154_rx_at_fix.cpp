// Workarounds for two ESP-IDF 802.15.4 driver bugs that leave a CSL receiver deaf right after its
// own data polls (found and hardware-confirmed 2026-09-19 on IDF v6.0.2).
//
// The trigger is OpenThread's CSL re-sync (PR #11601, in the OpenThread snapshot of IDF v6.0.2):
// after a data poll's ACK, SubMac::HandleTransmitDone() calls UpdateCslLastSyncTimestamp() ->
// RestartCslTimerAfterSyncUpdate() while still in its transmit state. That rewinds one CSL period
// and re-schedules the *current* window via otPlatRadioReceiveAt(). Right after, the MAC asks for
// immediate RX (otPlatRadioReceive()) to fetch the frame the parent flagged as pending, and the
// parent sends it ~5 ms after the poll. Both driver bugs below make that receive fail, so every
// downlink frame after a poll got NoAck at the border router while CSL was on. It worked again the
// moment CSL was switched off.
//
// Hooked in via -Wl,--wrap=esp_ieee802154_receive_at / esp_ieee802154_receive (PARTs A, B) and
// -Wl,--wrap=_ZN2ot3Mac6SubMac5SleepEv (PART C) in main/CMakeLists.txt. All callers
// (otPlatRadioReceiveAt()/otPlatRadioReceive() in esp_openthread_radio.c, Mac::UpdateIdleMode()
// in OpenThread's mac.cpp) run in the OpenThread task under its lock; the atomics only let other
// tasks read the diagnostic counters. Inert without CSL: nothing calls receive_at then, and
// SubMac::Sleep() already leaves the radio asleep.
//
// ── PART A: expired receive window ── backport of esp-idf d60495d8 (merged to master as 7eed66d5,
//    "fix(ieee802154): skip receive_at when rx window already expired", July 2026).
//    DELETE PART A once on ESP-IDF >= v6.0.4 or >= v6.1.1: the fix is on release/v6.0,
//    release/v6.1 and master as of 2026-09-19, but NOT in v6.0.3 or v6.1. Verify the driver's
//    ieee802154_receive_at() starts with the same is_target_time_expired() check first. The
//    #pragma message below fires on those versions as a reminder.
//    The window re-scheduled after a poll has usually already ended (at a 500 ms period, ~98 % of
//    polls). This driver still arms it: timer1 fires at once, the window "ends" immediately, and
//    esp_ieee802154_receive_at_done() queues a deferred sleep (EVENT_SLEEP). The port applies it
//    at the end of the same radio processing pass (esp_openthread_radio.c), right after the MAC
//    started its receive for the pending data -- switching the radio off again.
//
// ── PART B: receive() after a still-pending receive_at() ── KEEP: unfixed on esp-idf master
//    as of 2026-09-19; no upstream report existed.
//    ieee802154_receive_at() sets the driver state to RX immediately, although reception only
//    starts later, when timer1 fires at the window. ieee802154_receive() then treats state RX as
//    "already in rx state" and returns without starting anything, so when the re-scheduled window
//    is still in the future the MAC's receive is silently dropped. otPlatRadioReceive() is
//    specified as "transition the radio from Sleep to Receive", and a window that hasn't started
//    isn't receiving. Fix: remember that a window was armed, and on the next immediate receive
//    cancel it with the driver's esp_ieee802154_sleep() first (stops timer1 and the ETM trigger),
//    so RX really starts. Losing that one window costs nothing: the radio is in continuous RX from
//    here until OpenThread schedules the next window. PART B stays correct without PART A: a flag
//    left set for a window the driver itself skipped (or that already ran) is harmless, because
//    sleep() is a no-op on a sleeping radio.
//    Accepted residual risk: if OpenThread asked for immediate RX while a window was actually
//    mid-frame, sleep() would drop that frame. The poll path can't hit this -- the poll's own TX
//    already cancelled any earlier window.
//
// ── PART C: radio left listening after the MAC goes idle ── backport of OpenThread main's
//    SubMac::Sleep() (checked 2026-09-21), which is NOT in the OpenThread snapshot of IDF v6.0.2.
//    DELETE PART C once the IDF's OpenThread has that SubMac::Sleep(): ProcessTimedRx(), then
//    Radio::Sleep() unless a timed receive is in progress.
//    Here, with CSL on, SubMac::Sleep() calls RadioSample() instead, which never calls
//    Radio::Sleep() and leaves it to the next CSL window to switch the radio off. The driver
//    sleeps on its own after a finished TX or a received and ACKed frame, but a continuous
//    receive that ends without a frame (the MAC's wait for data after a poll, its sleep delay
//    between fragments) keeps listening -- holding the PM lock -- for up to a whole CSL period.
//    Hardware-measured 2026-09-21 with CONFIG_PM_PROFILING: the radio held the core awake for 89 %
//    of every CSL publish window (39 % without CSL), in ~35 long stretches instead of hundreds
//    of short ones. Fix: after the real SubMac::Sleep(), put a radio still in RX to sleep, unless
//    a receive window was armed since the last immediate receive -- that RX state is the window's,
//    and like upstream, going idle must not cancel a scheduled window. Otherwise RX can only be a
//    continuous receive the MAC is done with, since arming a window stops any continuous one.
//    This is the driver call SubMac::Sleep() makes at the same point without CSL, so it drops a
//    frame arriving (or an ACK being sent) at that instant exactly as the non-CSL path would.
//
// Re-check all parts on any IDF upgrade (OpenThread #13491 reworks timed RX), and delete this
// file plus the three --wrap flags once the driver and OpenThread are fixed upstream. PART C wraps
// a C++ symbol: if it is renamed, the link fails on __real__ZN2ot3Mac6SubMac5SleepEv; if a call
// moves into sub_mac.cpp itself, the wrap silently stops applying -- check that
// Mac::UpdateIdleMode() still calls __wrap__ZN2ot3Mac6SubMac5SleepEv in the ELF.

#include "ieee802154_rx_at_fix.h"

#include <atomic>

#include "esp_err.h"
#include "esp_idf_version.h"
#include "esp_ieee802154.h"
#include "esp_timer.h"
#include "openthread/error.h"

extern "C" esp_err_t __real_esp_ieee802154_receive(void);
extern "C" esp_err_t __real_esp_ieee802154_receive_at(uint32_t time, uint32_t duration);
extern "C" esp_err_t __wrap_esp_ieee802154_receive(void);
extern "C" esp_err_t __wrap_esp_ieee802154_receive_at(uint32_t time, uint32_t duration);

// ot::Mac::SubMac::Sleep(): a member function returning ot::Error (a typedef of otError), so on
// this ABI it is a plain function taking `this` as its only argument.
extern "C" otError __real__ZN2ot3Mac6SubMac5SleepEv(void *sub_mac);
extern "C" otError __wrap__ZN2ot3Mac6SubMac5SleepEv(void *sub_mac);

// Set when a receive window is armed, cleared by the next immediate receive: "the radio's RX state
// may be a window's, not a continuous receive's". PART B cancels that window before receiving;
// PART C leaves the radio alone while it is set.
static std::atomic<bool> s_rx_at_scheduled{false};
static std::atomic<uint32_t> s_rx_at_fix_count{0};
static std::atomic<uint32_t> s_rx_at_skip_count{0};
static std::atomic<uint32_t> s_idle_rx_stop_count{0};

// Diagnostics only -- see the header. The sums are 64-bit because at a 500 ms period they take
// on the order of 10 ms per window, which would wrap a uint32 of microseconds within the hour.
static std::atomic<uint64_t> s_rx_at_total_count{0};
static std::atomic<uint64_t> s_rx_at_window_us{0};
static std::atomic<uint64_t> s_rx_at_lead_us{0};
static std::atomic<uint32_t> s_rx_at_late_count{0};

// ── PART A (backport of esp-idf d60495d8 -- delete on ESP-IDF >= v6.0.4 / >= v6.1.1) ─────────
#if (ESP_IDF_VERSION_MAJOR == 6 && ESP_IDF_VERSION_MINOR == 0 && ESP_IDF_VERSION_PATCH >= 4) || \
    (ESP_IDF_VERSION_MAJOR == 6 && ESP_IDF_VERSION_MINOR == 1 && ESP_IDF_VERSION_PATCH >= 1) || \
    (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 2, 0))
#pragma message("ieee802154_rx_at_fix.cpp: this ESP-IDF should already contain esp-idf d60495d8 " \
                "(skip expired receive_at windows) -- verify the driver, then delete PART A")
#endif

// Copied from esp-idf d60495d8 (components/ieee802154/private_include/esp_ieee802154_timer.h,
// not reachable from here): wrap-around safe, true when target is at or before now.
static inline bool is_target_time_expired(uint32_t target, uint32_t now)
{
    return (((now - target) & (1 << 31)) == 0);
}

// True when the window has already ended -- the check esp-idf d60495d8 put at the top of
// ieee802154_receive_at(), with the same comment.
static bool rx_at_window_expired(uint32_t time, uint32_t duration)
{
    // If a receive window is specified but it has already elapsed (time + duration is earlier
    // than the current time), this is an expired rx window, so skip it and return directly.
    if (duration) {
        uint32_t current_time = (uint32_t)esp_timer_get_time();
        if (is_target_time_expired(time + duration, current_time)) {
            return true;
        }
    }
    return false;
}
// ── end of PART A ──────────────────────────────────────────────────────────────────────────

esp_err_t __wrap_esp_ieee802154_receive_at(uint32_t time, uint32_t duration)
{
    // Diagnostics: every window asked for, including the ones PART A drops below.
    s_rx_at_total_count.fetch_add(1, std::memory_order_relaxed);

    // PART A -- delete this if-block together with the block above.
    if (rx_at_window_expired(time, duration)) {
        s_rx_at_skip_count.fetch_add(1, std::memory_order_relaxed);
        return ESP_OK;
    }

    // Diagnostics: of the windows actually armed, how wide and how far ahead. Wrap-safe, same
    // 32-bit microsecond timebase the driver itself compares against.
    const int32_t lead_us = static_cast<int32_t>(time - static_cast<uint32_t>(esp_timer_get_time()));
    s_rx_at_window_us.fetch_add(duration, std::memory_order_relaxed);
    if (lead_us > 0)
        s_rx_at_lead_us.fetch_add(static_cast<uint32_t>(lead_us), std::memory_order_relaxed);
    else
        s_rx_at_late_count.fetch_add(1, std::memory_order_relaxed);

    // PART B
    s_rx_at_scheduled.store(true, std::memory_order_relaxed);
    return __real_esp_ieee802154_receive_at(time, duration);
}

// PART B
esp_err_t __wrap_esp_ieee802154_receive(void)
{
    if (s_rx_at_scheduled.exchange(false, std::memory_order_relaxed)) {
        esp_ieee802154_sleep();
        s_rx_at_fix_count.fetch_add(1, std::memory_order_relaxed);
    }
    return __real_esp_ieee802154_receive();
}

// PART C
otError __wrap__ZN2ot3Mac6SubMac5SleepEv(void *sub_mac)
{
    const otError error = __real__ZN2ot3Mac6SubMac5SleepEv(sub_mac);

    if (!s_rx_at_scheduled.load(std::memory_order_relaxed) &&
        esp_ieee802154_get_state() == ESP_IEEE802154_RADIO_RECEIVE) {
        esp_ieee802154_sleep();
        s_idle_rx_stop_count.fetch_add(1, std::memory_order_relaxed);
    }
    return error;
}

uint32_t ieee802154_rx_at_fix_count()
{
    return s_rx_at_fix_count.load(std::memory_order_relaxed);
}

uint32_t ieee802154_rx_at_skip_count()
{
    return s_rx_at_skip_count.load(std::memory_order_relaxed);
}

uint32_t ieee802154_idle_rx_stop_count()
{
    return s_idle_rx_stop_count.load(std::memory_order_relaxed);
}

uint64_t ieee802154_rx_at_total_count()
{
    return s_rx_at_total_count.load(std::memory_order_relaxed);
}

uint64_t ieee802154_rx_at_window_us()
{
    return s_rx_at_window_us.load(std::memory_order_relaxed);
}

uint64_t ieee802154_rx_at_lead_us()
{
    return s_rx_at_lead_us.load(std::memory_order_relaxed);
}

uint32_t ieee802154_rx_at_late_count()
{
    return s_rx_at_late_count.load(std::memory_order_relaxed);
}
