#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

// RAM-only store-and-forward log for readings that failed to publish live (broker/link
// unreachable) -- so a blackout costs a flat gap in HA's history instead of the readings
// being silently discarded. Deliberately NOT flash-backed: since sensorstask.cpp's reboot
// supervisor now reboots only on a confirmed LP-core stall (never on plain connectivity/broker
// loss -- see its doc comments), this buffer survives essentially an entire outage intact, and
// a RAM ring buffer avoids a flash partition / one-time USB reflash entirely. It does NOT
// survive a reboot (LP stall, OTA success) or a power cycle -- accepted, see sensorstask.cpp's
// call site for why that's rare in practice.
//
// Single producer (sensors_task, via history_log_append), single consumer (the MQTT publish
// task, via history_log_peek_batch/history_log_advance) -- see history_log.cpp for the
// lock-free ring buffer this enables.

// Appends one reading that failed to publish live. Captures esp_timer_get_time() internally as
// the record's timestamp -- no persistent/wall clock is needed: this buffer never survives a
// reboot, so every record that's ever replayed was captured within the same boot session as the
// replay itself, and only ever needs a same-boot time difference (see history_log_peek_batch()).
// If the ring is already full, the oldest not-yet-replayed record is silently overwritten
// (accepted: an outage long enough to matter here is far beyond this buffer's capacity -- see
// the sizing note in history_log.cpp). Call only from sensors_task.
void history_log_append(float temp_c, float hum_pct);

// True if at least one record is waiting to be replayed. Call only from the MQTT publish task.
bool history_log_has_pending();

// One backlog entry as handed to the replay/serialization step. ago_sec is how many seconds
// before the history_log_peek_batch() call that captured it that this record was taken --
// the caller (mqtt_sender.cpp) publishes it as-is; HA computes the true historical timestamp
// as (message arrival time - ago_sec) using its own clock, since the device never has one.
struct HistoryLogEntry {
    uint32_t ago_sec;
    float temp_c;
    float hum_pct;
};

// Writes up to `out.size()` oldest not-yet-replayed entries into `out` (caller-owned scratch
// space, e.g. a std::array -- implicitly converts to a span), oldest first, and returns the
// prefix of `out` actually written (out.first(N)) -- empty if none were pending. The returned
// span borrows `out`'s storage, so it's only valid as long as `out` is; the caller reads it
// immediately (to serialize the batch) rather than holding onto it. Does not itself consume
// anything -- see history_log_advance(). Call only from the MQTT publish task.
std::span<const HistoryLogEntry> history_log_peek_batch(std::span<HistoryLogEntry> out);

// Marks the `count` oldest entries as replayed, freeing their slots for reuse -- call only once
// their batch's PUBACK is confirmed, matching the size of a span previously returned by
// history_log_peek_batch() (or a prefix of it, if only part of a batch was confirmed). Call
// only from the MQTT publish task.
void history_log_advance(size_t count);
