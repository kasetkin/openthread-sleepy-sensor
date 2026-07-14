#include "ota_updater.h"

#include <atomic>
#include <charconv>
#include <cstring>
#include <format>
#include <string>
#include <string_view>

#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
// PSA Crypto is the hash API in mbedtls 4.x (IDF 6.0) — mbedtls/sha256.h went private.
#include "psa/crypto.h"

static const char *TAG = "ota-updater";

// ── tuning ────────────────────────────────────────────────────────────────────
// Battery floor for accepting a download (a multi-minute radio-on session on a nearly-empty
// pack risks brownout mid-flash); manifest "force":true overrides for bench work on USB power.
static constexpr int OTA_MIN_BATTERY_PERCENT = 30;
// Sessions that download at least one chunk always earn a retry (resume makes them forward
// progress); only consecutive attempts with ZERO progress count against this budget, so a
// bad/missing staging can't drain the battery in a retry loop.
static constexpr int OTA_MAX_NO_PROGRESS_ATTEMPTS = 3;
// Per-chunk wait: subscribe -> SUBACK -> retained chunk normally lands in ~1-2 s over Thread.
static constexpr uint32_t OTA_CHUNK_TIMEOUT_MS = 20'000;
// Whole-session cap; hitting it just ends the attempt (resume continues next cycle).
static constexpr uint32_t OTA_SESSION_CAP_MS = 20 * 60'000;

static constexpr EventBits_t OTA_BIT_CHUNK = BIT0;  // expected chunk verified + written
static constexpr EventBits_t OTA_BIT_FAIL  = BIT1;

// ── state ─────────────────────────────────────────────────────────────────────
static const NetworkLink *s_link = nullptr;
static std::string s_topic_manifest, s_topic_image_prefix, s_topic_install, s_topic_installed, s_topic_status;

// Latest staged manifest, filled by the esp-mqtt event task, consumed by the mqtt_pub task.
// Guarded by s_mutex (as are the esp_ota/sha handles below — see handle_image_chunk()).
struct Manifest
{
    std::string version;
    size_t      size = 0;
    size_t      chunk_size = 0;
    uint8_t     sha256[32] = {};
    bool        force = false;
    bool        valid = false;
    bool        differs_from_running = false;

    // Same staged image? (identity = content hash + geometry, not just the version string)
    bool sameImage(const Manifest &o) const
    {
        return size == o.size && chunk_size == o.chunk_size &&
               std::memcmp(sha256, o.sha256, sizeof(sha256)) == 0;
    }
};
static Manifest s_manifest;
static SemaphoreHandle_t s_mutex = nullptr;
static EventGroupHandle_t s_eg = nullptr;

static std::atomic<bool>   s_install_requested{false};
static std::atomic<bool>   s_session_active{false};
static std::atomic<bool>   s_conn_lost{false};   // set by ota_on_mqtt_error() during a session
static std::atomic<int>    s_no_progress_attempts{0};

// Download progress; persists across sessions within one boot so a broken attempt RESUMES
// from the first missing chunk. All guarded by s_mutex while a session is active.
static bool                    s_partial_valid = false;  // handles below hold a resumable partial download
static Manifest                s_partial_manifest;       // identity of that partial download
static size_t                  s_next_chunk = 0;         // first chunk not yet written
static std::atomic<size_t>     s_received{0};            // bytes written so far
static esp_ota_handle_t        s_ota_handle = 0;
static const esp_partition_t  *s_ota_partition = nullptr;
static psa_hash_operation_t    s_sha_op;

// ── helpers ───────────────────────────────────────────────────────────────────
static bool topic_is(const char *topic, size_t topic_len, const std::string &full)
{
    return topic_len == full.size() && std::memcmp(topic, full.data(), topic_len) == 0;
}

// Non-retained, QoS 0 — purely informational; loss is acceptable but not invisible: a
// failed enqueue (dead connection) is logged so the serial trail explains a silent broker.
static void publish_status(esp_mqtt_client_handle_t client, const std::string &json)
{
    int msg_id = -1;
    if (client)
        msg_id = esp_mqtt_client_publish(client, s_topic_status.c_str(), json.c_str(),
                                         static_cast<int>(json.size()), 0, 0);
    ESP_LOGI(TAG, "status%s: %s", msg_id < 0 ? " (NOT delivered — connection down)" : "",
             json.c_str());
}

static bool parse_sha256_hex(std::string_view hex, uint8_t out[32])
{
    if (hex.size() != 64)
        return false;
    for (int i = 0; i < 32; ++i) {
        auto nibble = [](char c) -> int {
            if (c >= '0' && c <= '9') return c - '0';
            if (c >= 'a' && c <= 'f') return c - 'a' + 10;
            if (c >= 'A' && c <= 'F') return c - 'A' + 10;
            return -1;
        };
        const int hi = nibble(hex[2 * i]), lo = nibble(hex[2 * i + 1]);
        if (hi < 0 || lo < 0)
            return false;
        out[i] = static_cast<uint8_t>((hi << 4) | lo);
    }
    return true;
}

// Minimal flat-JSON field extraction, in the same spirit as the project's secrets.yaml /
// calibration.txt hand parsers: the manifest's only producer is tools/ota_push.py, so both
// ends of the format live in this repo and a general JSON library would be dead weight.
// (The payload still has to BE JSON on the wire — HA's update entity templates
// {{ value_json.version }} out of the very same retained message.) Returns the value token
// after `"key":` — the unquoted contents for a string, the bare token for a number/bool.
// No escape handling: none of the manifest fields (git-describe version, decimal sizes, hex
// sha256, boolean force) can contain escapes as ota_push.py emits them.
static std::optional<std::string_view> manifest_field(std::string_view json, std::string_view key)
{
    // Search for the quoted key to avoid matching one field's value against another's name.
    const std::string quoted = std::format("\"{}\"", key);
    size_t pos = json.find(quoted);
    if (pos == std::string_view::npos)
        return std::nullopt;
    pos += quoted.size();
    while (pos < json.size() && (json[pos] == ':' || json[pos] == ' '))
        ++pos;
    if (pos >= json.size())
        return std::nullopt;

    if (json[pos] == '"') {  // string value
        const size_t end = json.find('"', ++pos);
        if (end == std::string_view::npos)
            return std::nullopt;
        return json.substr(pos, end - pos);
    }
    size_t end = pos;  // bare token (number / true / false)
    while (end < json.size() && json[end] != ',' && json[end] != '}' && json[end] != ' ')
        ++end;
    return json.substr(pos, end - pos);
}

static std::optional<size_t> parse_size_field(std::string_view json, std::string_view key)
{
    const auto tok = manifest_field(json, key);
    size_t value = 0;
    if (!tok || std::from_chars(tok->data(), tok->data() + tok->size(), value).ec != std::errc{})
        return std::nullopt;
    return value;
}

static void handle_manifest(const char *data, size_t data_len, size_t total)
{
    if (data_len != total) {
        // Can't happen for a sane manifest (RX buffer is 8.5 KB); refuse rather than
        // stitch together segments for a message that has no business being that big.
        ESP_LOGE(TAG, "manifest larger than RX buffer (%zu bytes) — ignored", total);
        return;
    }
    const std::string_view body(data, data_len);

    const auto version    = manifest_field(body, "version");
    const auto size       = parse_size_field(body, "size");
    const auto chunk_size = parse_size_field(body, "chunk_size");
    const auto sha        = manifest_field(body, "sha256");
    const auto force      = manifest_field(body, "force");

    Manifest m;
    if (!version || version->empty() || !size || *size == 0 ||
        !chunk_size || *chunk_size == 0 || *chunk_size > OTA_MAX_CHUNK_SIZE ||
        !sha || !parse_sha256_hex(*sha, m.sha256)) {
        ESP_LOGE(TAG, "manifest missing/invalid version|size|sha256|chunk_size (max %zu) — ignored",
                 OTA_MAX_CHUNK_SIZE);
        return;
    }
    m.version    = *version;
    m.size       = *size;
    m.chunk_size = *chunk_size;
    m.force      = force && *force == "true";
    m.valid      = true;
    m.differs_from_running = (m.version != esp_app_get_description()->version);

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_manifest = std::move(m);
    xSemaphoreGive(s_mutex);

    ESP_LOGI(TAG, "manifest: version=%s size=%zu chunk_size=%zu force=%d (running=%s, differs=%d)",
             s_manifest.version.c_str(), s_manifest.size, s_manifest.chunk_size, s_manifest.force,
             esp_app_get_description()->version, s_manifest.differs_from_running);
}

// One retained chunk message. Runs in the esp-mqtt event task; blocking on the flash write
// here is fine — the chunk is already fully received, and the next one only starts after
// the session task's next subscribe.
static void handle_image_chunk(size_t index, const char *data, size_t data_len,
                               size_t offset, size_t total)
{
    if (!s_session_active.load())
        return;  // stale delivery after an abort

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    const size_t expected_len = std::min(s_partial_manifest.chunk_size,
                                         s_partial_manifest.size - s_next_chunk * s_partial_manifest.chunk_size);
    bool fail = false;
    if (offset != 0 || total != data_len) {
        // A chunk that fits the RX buffer arrives as exactly one event; anything else means
        // the staging's chunk_size exceeds this build's buffer — a config error, not a link
        // hiccup, so fail loudly rather than let esp-mqtt segment it (see file comment).
        ESP_LOGE(TAG, "chunk %zu spans events (len %zu of %zu at offset %zu) — chunk_size too "
                      "big for RX buffer", index, data_len, total, offset);
        fail = true;
    } else if (index != s_next_chunk) {
        // Strict ordering: chunks are subscribed one at a time, so anything else is a stale
        // retained delivery racing a re-staging. Ignore rather than fail — the expected
        // chunk may still arrive.
        ESP_LOGW(TAG, "ignoring unexpected chunk %zu (waiting for %zu)", index, s_next_chunk);
    } else if (data_len != expected_len) {
        ESP_LOGE(TAG, "chunk %zu is %zu bytes, expected %zu — staging/manifest mismatch",
                 index, data_len, expected_len);
        fail = true;
    } else if (const esp_err_t err = esp_ota_write(s_ota_handle, data, data_len); err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_write failed at chunk %zu: 0x%x", index, err);
        fail = true;
    } else if (psa_hash_update(&s_sha_op, reinterpret_cast<const uint8_t *>(data), data_len)
               != PSA_SUCCESS) {
        ESP_LOGE(TAG, "psa_hash_update failed at chunk %zu", index);
        fail = true;
    } else {
        s_next_chunk = index + 1;
        s_received.fetch_add(data_len);
        xSemaphoreGive(s_mutex);
        xEventGroupSetBits(s_eg, OTA_BIT_CHUNK);
        return;
    }
    xSemaphoreGive(s_mutex);
    if (fail)
        xEventGroupSetBits(s_eg, OTA_BIT_FAIL);
}

// Discard a kept partial download (bad image, or a different image was staged).
// Caller must hold s_mutex.
static void discard_partial_locked()
{
    if (s_ota_handle != 0) {
        esp_ota_abort(s_ota_handle);
        s_ota_handle = 0;
    }
    psa_hash_abort(&s_sha_op);  // safe on an already-terminated operation
    s_partial_valid = false;
    s_next_chunk = 0;
    s_received.store(0);
}

// ── public API ────────────────────────────────────────────────────────────────

void ota_updater_init(std::string_view device_id, const NetworkLink *link)
{
    s_link = link;
    const auto full = [&](std::string_view suffix) {
        return std::format("{}/{}", device_id, suffix);
    };
    s_topic_manifest     = full(OTA_SUFFIX_MANIFEST);
    s_topic_image_prefix = full(OTA_SUFFIX_IMAGE_PREFIX);
    s_topic_install      = full(OTA_SUFFIX_INSTALL);
    s_topic_installed    = full(OTA_SUFFIX_INSTALLED);
    s_topic_status       = full(OTA_SUFFIX_STATUS);
    if (!s_mutex)
        s_mutex = xSemaphoreCreateMutex();
    if (!s_eg)
        s_eg = xEventGroupCreate();
}

const char *ota_topic_manifest()  { return s_topic_manifest.c_str(); }
const char *ota_topic_install()   { return s_topic_install.c_str(); }
const char *ota_topic_installed() { return s_topic_installed.c_str(); }
const char *ota_topic_status()    { return s_topic_status.c_str(); }

void ota_on_mqtt_data(const char *topic, size_t topic_len,
                      const char *data, size_t data_len,
                      size_t offset, size_t total)
{
    if (topic_len == 0)
        return;  // continuation of an oversized message — none of ours is allowed to be one

    const std::string_view t(topic, topic_len);
    if (t.starts_with(s_topic_image_prefix)) {
        const std::string_view index_str = t.substr(s_topic_image_prefix.size());
        size_t index = 0;
        if (std::from_chars(index_str.data(), index_str.data() + index_str.size(), index).ec
            == std::errc{})
            handle_image_chunk(index, data, data_len, offset, total);
        return;
    }
    if (topic_is(topic, topic_len, s_topic_manifest)) {
        handle_manifest(data, data_len, total);
    } else if (topic_is(topic, topic_len, s_topic_install)) {
        // Any non-empty retained payload counts as an install request; the empty
        // payload we publish after success is the MQTT "delete retained" idiom.
        s_install_requested.store(data_len > 0);
        ESP_LOGI(TAG, "install flag: %s", data_len > 0 ? "SET" : "cleared");
    }
}

void ota_on_mqtt_error()
{
    if (!s_session_active.load())
        return;
    s_conn_lost.store(true);
    xEventGroupSetBits(s_eg, OTA_BIT_FAIL);
}

bool ota_update_due()
{
    if (!s_install_requested.load())
        return false;
    if (s_no_progress_attempts.load() >= OTA_MAX_NO_PROGRESS_ATTEMPTS)
        return false;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    const bool due = s_manifest.valid && s_manifest.differs_from_running;
    xSemaphoreGive(s_mutex);
    return due;
}

bool ota_session_in_progress()
{
    return s_session_active.load();
}

void ota_run_session(esp_mqtt_client_handle_t client, std::optional<int> battery_percent)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    const Manifest manifest = s_manifest;  // stable per-session copy
    xSemaphoreGive(s_mutex);

    if (battery_percent.has_value() && *battery_percent < OTA_MIN_BATTERY_PERCENT && !manifest.force) {
        publish_status(client, std::format(
            "{{\"state\":\"deferred\",\"reason\":\"battery {}% < {}%\"}}",
            *battery_percent, OTA_MIN_BATTERY_PERCENT));
        return;  // deliberately NOT an attempt: retried once the battery recovers
    }

    const size_t total_chunks = (manifest.size + manifest.chunk_size - 1) / manifest.chunk_size;

    // ── fresh start or resume ─────────────────────────────────────────────────
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_partial_valid && !manifest.sameImage(s_partial_manifest)) {
        ESP_LOGI(TAG, "staged image changed — discarding partial download (%zu chunks)", s_next_chunk);
        discard_partial_locked();
    }
    if (!s_partial_valid) {
        s_ota_partition = esp_ota_get_next_update_partition(nullptr);
        if (!s_ota_partition || s_ota_partition->size < manifest.size) {
            xSemaphoreGive(s_mutex);
            publish_status(client, std::format(
                "{{\"state\":\"error\",\"reason\":\"image {} bytes exceeds slot\"}}", manifest.size));
            s_no_progress_attempts.fetch_add(1);
            return;
        }
        // OTA_WITH_SEQUENTIAL_WRITES erases flash incrementally as chunks arrive instead of
        // blocking here for a multi-second whole-partition erase.
        if (const esp_err_t err = esp_ota_begin(s_ota_partition, OTA_WITH_SEQUENTIAL_WRITES, &s_ota_handle);
            err != ESP_OK) {
            s_ota_handle = 0;
            xSemaphoreGive(s_mutex);
            publish_status(client, std::format("{{\"state\":\"error\",\"reason\":\"esp_ota_begin 0x{:x}\"}}",
                                               static_cast<unsigned>(err)));
            s_no_progress_attempts.fetch_add(1);
            return;
        }
        // psa_crypto_init() is idempotent; the hash op then streams alongside esp_ota_write().
        s_sha_op = PSA_HASH_OPERATION_INIT;
        if (psa_crypto_init() != PSA_SUCCESS ||
            psa_hash_setup(&s_sha_op, PSA_ALG_SHA_256) != PSA_SUCCESS) {
            discard_partial_locked();
            xSemaphoreGive(s_mutex);
            publish_status(client, "{\"state\":\"error\",\"reason\":\"sha256 setup failed\"}");
            s_no_progress_attempts.fetch_add(1);
            return;
        }
        s_partial_manifest = manifest;
        s_partial_valid = true;
        s_next_chunk = 0;
        s_received.store(0);
    }
    const size_t session_start_chunk = s_next_chunk;
    xSemaphoreGive(s_mutex);

    ESP_LOGI(TAG, "OTA %s -> %s: %zu bytes in %zu chunks of %zu, %s chunk %zu, into %s",
             esp_app_get_description()->version, manifest.version.c_str(), manifest.size,
             total_chunks, manifest.chunk_size,
             session_start_chunk ? "RESUMING at" : "starting at", session_start_chunk,
             s_ota_partition->label);
    publish_status(client, std::format(
        "{{\"state\":\"starting\",\"version\":\"{}\",\"size\":{},\"chunks\":{},\"resume_chunk\":{}}}",
        manifest.version, manifest.size, total_chunks, session_start_chunk));

    s_conn_lost.store(false);
    xEventGroupClearBits(s_eg, OTA_BIT_CHUNK | OTA_BIT_FAIL);
    s_session_active.store(true);

    // rx-on-when-idle for the download; MUST be undone on every exit path below — a child
    // left rx-on burns ~78 mA until the battery dies.
    s_link->onOtaWindowBegin();

    // ── pull chunks strictly in order ─────────────────────────────────────────
    bool all_chunks = false;
    std::string fail_reason;
    const TickType_t session_start_tick = xTaskGetTickCount();
    // Status roughly every 10% — between chunk messages the client is idle, so unlike the
    // single-blob design these actually reach the broker live.
    const size_t status_every = std::max<size_t>(total_chunks / 10, 1);

    for (size_t chunk = session_start_chunk; chunk < total_chunks; ++chunk) {
        const std::string topic = std::format("{}{}", s_topic_image_prefix, chunk);
        // QoS 0: the retained chunk rides an already-reliable TCP stream, and loss just
        // means the 20 s wait below expires and the session resumes next cycle.
        if (esp_mqtt_client_subscribe(client, topic.c_str(), 0) < 0) {
            fail_reason = std::format("subscribe to chunk {} failed", chunk);
            break;
        }
        const EventBits_t bits = xEventGroupWaitBits(s_eg, OTA_BIT_CHUNK | OTA_BIT_FAIL,
                                                     pdTRUE, pdFALSE,
                                                     pdMS_TO_TICKS(OTA_CHUNK_TIMEOUT_MS));
        if (bits & OTA_BIT_FAIL) {
            fail_reason = s_conn_lost.load()
                ? std::format("connection lost at chunk {}", chunk)
                : std::format("chunk {} rejected", chunk);
            break;
        }
        if (!(bits & OTA_BIT_CHUNK)) {
            fail_reason = std::format("chunk {} timed out after {} s", chunk, OTA_CHUNK_TIMEOUT_MS / 1000);
            break;
        }
        if (chunk + 1 == total_chunks) {
            all_chunks = true;
            break;
        }
        if ((chunk + 1) % status_every == 0)
            publish_status(client, std::format(
                "{{\"state\":\"downloading\",\"chunk\":{},\"chunks\":{},\"received\":{}}}",
                chunk + 1, total_chunks, s_received.load()));
        if ((xTaskGetTickCount() - session_start_tick) > pdMS_TO_TICKS(OTA_SESSION_CAP_MS)) {
            fail_reason = "session cap exceeded";
            break;
        }
    }

    s_session_active.store(false);

    // ── finalize or keep for resume ───────────────────────────────────────────
    bool ok = false;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (all_chunks) {
        uint8_t sha[32] = {};
        size_t sha_len = 0;
        if (psa_hash_finish(&s_sha_op, sha, sizeof(sha), &sha_len) != PSA_SUCCESS ||
            sha_len != sizeof(sha) || std::memcmp(sha, manifest.sha256, sizeof(sha)) != 0) {
            fail_reason = "sha256 mismatch";
        } else if (const esp_err_t err = esp_ota_end(s_ota_handle); err != ESP_OK) {
            // esp_ota_end validates the image header/magic on top of our hash check.
            fail_reason = std::format("esp_ota_end 0x{:x}", static_cast<unsigned>(err));
            s_ota_handle = 0;  // esp_ota_end released it, success or not
        } else {
            s_ota_handle = 0;
            if (const esp_err_t berr = esp_ota_set_boot_partition(s_ota_partition); berr != ESP_OK)
                fail_reason = std::format("esp_ota_set_boot_partition 0x{:x}", static_cast<unsigned>(berr));
            else
                ok = true;
        }
        // Complete downloads never resume — good ones reboot, bad ones must restart from 0.
        discard_partial_locked();
    }
    const size_t progressed = s_next_chunk - session_start_chunk;
    xSemaphoreGive(s_mutex);

    // Back to sleepy link mode in BOTH outcomes — on success the parent should see a clean
    // MLE mode transition before we drop off for the reboot.
    s_link->onOtaWindowEnd();

    if (!ok) {
        // A session that fetched even one chunk is forward progress thanks to resume; only
        // consecutive dead-on-arrival sessions burn the attempt budget.
        const int attempts = progressed > 0 ? (s_no_progress_attempts.store(0), 0)
                                            : s_no_progress_attempts.fetch_add(1) + 1;
        publish_status(client, std::format(
            "{{\"state\":\"error\",\"reason\":\"{}\",\"received\":{},\"resume_chunk\":{},\"no_progress_attempts\":{}}}",
            fail_reason, s_received.load(), s_next_chunk, attempts));
        ESP_LOGE(TAG, "OTA attempt failed: %s (will %s)", fail_reason.c_str(),
                 s_partial_valid ? "resume next cycle" : "restart from scratch");
        return;  // retained install flag stays -> retried next cycle (attempt budget applies)
    }

    publish_status(client, std::format("{{\"state\":\"success\",\"version\":\"{}\"}}", manifest.version));
    // Empty retained payload = delete the retained install request, so the new image
    // doesn't immediately see a stale command (its version now matches anyway). QoS 1 +
    // a short drain delay gives both messages a chance to actually leave before reboot.
    esp_mqtt_client_publish(client, s_topic_install.c_str(), "", 0, 1, 1);
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "OTA complete — rebooting into %s", manifest.version.c_str());
    esp_restart();
}
