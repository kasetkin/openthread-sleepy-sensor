#include "history_log.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <ranges>
#include <span>

#include "esp_timer.h"

namespace {

struct Record {
    int64_t captured_us;
    float temp_c;
    float hum_pct;
};

// ~96 KB (8192 * 12 bytes). From this build's actual idf.py size against build/: DIRAM free at
// link time is 263,344 B; confirmed permanent task-stack overhead (sensors_task 16,384 B +
// event/idle tasks ~3,840 B) leaves ~243 KB before OpenThread/lwIP/mbedTLS's own runtime heap
// use, which isn't visible in that static report -- this budgets conservatively for that
// unknown. At device_config.yaml's worst-case forced-publish cadence
// ((max_skip_cycles+1) x lp_poll_interval_sec = 220 s today), that floors at ~21 days of outage
// coverage, likely much more in practice since most cycles are flagged sooner than the forced
// ceiling. A power of two so wraparound is a mask, not a modulo.
constexpr size_t CAPACITY = 8192;
static_assert((CAPACITY & (CAPACITY - 1)) == 0, "CAPACITY must be a power of two");

// --- pure, size_t-wraparound-aware ring arithmetic ----------------------------------------
// Factored out of history_log_append()/history_log_peek_batch() below so the size_t wrap case
// (head/tail rolling past SIZE_MAX back to 0 -- 32-bit on this target) can be verified at
// compile time via the static_asserts immediately below, instead of trusting it by inspection
// or actually spinning a counter through its full ~4.29 billion range. That range is not the
// safety margin it looks like: append() only fires on a failed publish for a flagged cycle, so
// a well-behaved system would take on the order of centuries to wrap it -- but a bug that
// drove it in a tight loop wouldn't be bounded by that pacing, and this core tops out around a
// few million calls/sec even in a pathological busy-loop, putting a wrap at roughly
// 2^32 / 1e7 ~= 7 minutes in that worst case. The static_asserts below are what actually
// establish correctness across the wrap, not the timing estimate.
constexpr size_t ringIndex(size_t counter)
{
    return counter & (CAPACITY - 1);
}

// Corrects `tail` if the producer has lapped it (more than CAPACITY records appended since
// tail's last known position -- see history_log_append()'s overwrite-oldest doc comment).
// Unsigned subtraction is modular, so `head - tail` is the correct pending count even once
// head has numerically wrapped past tail (see the static_asserts below for worked examples),
// as long as the true, unbounded distance between them never exceeds one full size_t cycle.
constexpr size_t correctedTail(size_t head, size_t tail)
{
    return (head - tail > CAPACITY) ? (head - CAPACITY) : tail;
}

// Compile-time tests: no on-target test runner (same reasoning as
// SensorsValues::convertVoltageToPercent's static_asserts, sensorstask.h), so every build
// verifies the wraparound arithmetic directly rather than trusting it by inspection.
static_assert(ringIndex(0) == 0);
static_assert(ringIndex(CAPACITY - 1) == CAPACITY - 1);
static_assert(ringIndex(CAPACITY) == 0);                    // one full lap, no size_t wrap involved
static_assert(ringIndex(SIZE_MAX) == CAPACITY - 1);          // last valid index just before head wraps
static_assert(static_cast<size_t>(SIZE_MAX) + 1 == 0);       // the wrap itself: well-defined unsigned overflow
static_assert(ringIndex(static_cast<size_t>(SIZE_MAX) + 1) == 0);  // ...and indexing resumes with no gap or repeat

static_assert(correctedTail(100, 50) == 50);                 // ordinary, not lapped: 100-50 <= CAPACITY
static_assert(correctedTail(10000, 0) == 10000 - CAPACITY);  // ordinary, lapped: 10000-0 > CAPACITY
static_assert(correctedTail(2, SIZE_MAX - 5) == SIZE_MAX - 5);       // wrapped, NOT lapped: true distance is 8
static_assert(correctedTail(9000, SIZE_MAX - 5) == 9000 - CAPACITY); // wrapped AND lapped: true distance is 9006

std::array<Record, CAPACITY> s_ring;

// Lock-free SPSC ring buffer: sensors_task is the only writer of head (history_log_append),
// the MQTT publish task is the only writer of tail (history_log_advance) -- matches
// mqtt_sender.cpp's s_discovery_sent_mask/s_task_running/s_last_ok pattern (plain atomics, no
// mutex) rather than introducing a new synchronization idiom. head/tail are monotonically
// increasing counters, not pre-masked indices -- head - tail is always the pending count even
// across a wrap, and head never needs to "know" about tail (see history_log_append()).
std::atomic<size_t> s_head{0};
std::atomic<size_t> s_tail{0};

HistoryLogEntry toEntry(const Record &r, int64_t now_us)
{
    const int64_t agoUs = now_us - r.captured_us;
    return HistoryLogEntry{
        static_cast<uint32_t>(agoUs > 0 ? agoUs / 1'000'000 : 0),
        r.temp_c, r.hum_pct
    };
}

} // namespace

void history_log_append(float temp_c, float hum_pct)
{
    // Unconditional: always write the next slot and advance head, lapping tail if the ring is
    // full. This never touches s_tail (that's the consumer's alone to write) -- the consumer
    // detects and accounts for having been lapped itself, in history_log_peek_batch() below.
    // That keeps this a textbook single-writer-per-variable SPSC buffer even though sensors_task
    // and the MQTT publish task can, in one narrow edge (mqtt_wait_for_idle() timing out while a
    // long OTA session is still finishing a previous cycle's replay), genuinely run concurrently.
    const size_t head = s_head.load(std::memory_order_relaxed);
    s_ring[ringIndex(head)] = Record{esp_timer_get_time(), temp_c, hum_pct};
    s_head.store(head + 1, std::memory_order_release);
}

bool history_log_has_pending()
{
    return s_head.load(std::memory_order_acquire) != s_tail.load(std::memory_order_relaxed);
}

std::span<const HistoryLogEntry> history_log_peek_batch(std::span<HistoryLogEntry> out)
{
    const size_t head = s_head.load(std::memory_order_acquire);
    size_t tail = s_tail.load(std::memory_order_relaxed);  // consumer-owned; no cross-thread write

    // Lapped since our last look: some records were overwritten before we ever replayed them
    // (see history_log_append()'s doc comment -- accepted, would need an outage far beyond this
    // buffer's ~21+ day floor). Jump forward and persist the correction (see correctedTail()'s
    // doc comment above for why this stays correct across a size_t wrap too).
    const size_t corrected = correctedTail(head, tail);
    if (corrected != tail) {
        tail = corrected;
        s_tail.store(tail, std::memory_order_relaxed);
    }

    const size_t pending = head - tail;
    const size_t count = std::min(pending, out.size());
    if (count == 0)
        return {};

    // The pending range is [tail, head) mod CAPACITY -- may wrap the ring's physical end.
    // std::views::concat presents both pieces (the second empty when there's no wrap) as one
    // flat, in-order range, so the loop below never special-cases the seam.
    const size_t tailIdx = ringIndex(tail);
    const size_t firstChunk = std::min(count, CAPACITY - tailIdx);
    const auto chunk1 = std::span(s_ring).subspan(tailIdx, firstChunk);
    const auto chunk2 = std::span(s_ring).subspan(0, count - firstChunk);

    const int64_t now_us = esp_timer_get_time();
    size_t i = 0;
    for (const Record &r : std::views::concat(chunk1, chunk2))
        out[i++] = toEntry(r, now_us);
    return out.first(count);
}

void history_log_advance(size_t count)
{
    s_tail.fetch_add(count, std::memory_order_relaxed);
}
