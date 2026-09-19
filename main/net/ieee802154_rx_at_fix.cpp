// Workaround for an ESP-IDF 802.15.4 driver bug that leaves a CSL receiver deaf right after its
// own data polls (found 2026-09-19 on IDF v6.0.2; the driver code is unchanged on esp-idf master
// as of that date, and no upstream report existed).
//
// ieee802154_receive_at() (components/ieee802154/driver/esp_ieee802154_dev.c) sets the driver
// state to RX immediately, although reception only starts later, when timer1 fires at the window.
// ieee802154_receive() then treats state RX as "already in rx state" and returns without starting
// anything. OpenThread walks straight into that on every data poll while CSL is on: after the
// poll's ACK, SubMac::HandleTransmitDone() re-syncs CSL while still in its transmit state
// (UpdateCslLastSyncTimestamp -> RestartCslTimerAfterSyncUpdate, OpenThread PR #11601) and
// schedules the next window via otPlatRadioReceiveAt(); the MAC then asks for immediate RX to
// fetch the frame the parent announced (otPlatRadioReceive()), which the driver silently drops.
// The parent's frame, sent ~5 ms after the poll, meets a deaf radio: NoAck on every downlink
// frame, hardware-confirmed from the border router's log. otPlatRadioReceive() is specified as
// "transition the radio from Sleep to Receive", and a window that hasn't started isn't receiving.
//
// Fix, via -Wl,--wrap in main/CMakeLists.txt: remember that a window was scheduled, and on the
// next immediate receive cancel it with the driver's esp_ieee802154_sleep() first (stops timer1
// and the ETM trigger), so RX really starts. Losing that one window costs nothing: the radio is in
// continuous RX from here until OpenThread schedules the next window. A stale flag (the window
// already ran and the radio went back to sleep) is harmless -- sleep() is then a no-op. Inert
// without CSL: nothing calls receive_at then. Both callers (otPlatRadioReceive()/
// otPlatRadioReceiveAt() in esp_openthread_radio.c) run in the OpenThread task context under its
// lock; the atomics only let other tasks read the counter.
//
// Accepted residual risk: if OpenThread asked for immediate RX while a window was actually
// mid-frame, sleep() would drop that frame. The poll path can't hit this -- the poll's own TX
// already cancelled any earlier window.
//
// Tied to the OpenThread snapshot in IDF v6.0.2: re-check on any IDF upgrade (OpenThread #13491
// reworks timed RX) and delete this file plus the two --wrap flags once the driver is fixed.

#include "ieee802154_rx_at_fix.h"

#include <atomic>

#include "esp_err.h"
#include "esp_ieee802154.h"

extern "C" esp_err_t __real_esp_ieee802154_receive(void);
extern "C" esp_err_t __real_esp_ieee802154_receive_at(uint32_t time, uint32_t duration);
extern "C" esp_err_t __wrap_esp_ieee802154_receive(void);
extern "C" esp_err_t __wrap_esp_ieee802154_receive_at(uint32_t time, uint32_t duration);

static std::atomic<bool> s_rx_at_scheduled{false};
static std::atomic<uint32_t> s_rx_at_fix_count{0};

esp_err_t __wrap_esp_ieee802154_receive_at(uint32_t time, uint32_t duration)
{
    s_rx_at_scheduled.store(true, std::memory_order_relaxed);
    return __real_esp_ieee802154_receive_at(time, duration);
}

esp_err_t __wrap_esp_ieee802154_receive(void)
{
    if (s_rx_at_scheduled.exchange(false, std::memory_order_relaxed)) {
        esp_ieee802154_sleep();
        s_rx_at_fix_count.fetch_add(1, std::memory_order_relaxed);
    }
    return __real_esp_ieee802154_receive();
}

uint32_t ieee802154_rx_at_fix_count()
{
    return s_rx_at_fix_count.load(std::memory_order_relaxed);
}
