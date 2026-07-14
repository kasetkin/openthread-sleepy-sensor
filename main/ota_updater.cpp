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
// Battery floor for accepting a download (a ~5 mAh session on a nearly-empty pack risks
// brownout mid-flash); manifest "force":true overrides for bench work on USB power.
static constexpr int OTA_MIN_BATTERY_PERCENT = 30;
// A bad image must not drain the battery in a stage→fail→retry loop: the retained install
// flag persists across cycles, so cap attempts per boot and go quiet until a power cycle
// (or a fixed image, whose different sha/version resets nothing but succeeds).
static constexpr int OTA_MAX_ATTEMPTS_PER_BOOT = 3;
// No-progress watchdog while waiting for broker-streamed segments, and a whole-session cap.
static constexpr uint32_t OTA_STALL_TIMEOUT_MS = 30'000;
static constexpr uint32_t OTA_SESSION_CAP_MS   = 20 * 60'000;

static constexpr EventBits_t OTA_BIT_DONE = BIT0;
static constexpr EventBits_t OTA_BIT_FAIL = BIT1;

// ── state ─────────────────────────────────────────────────────────────────────
static const NetworkLink *s_link = nullptr;
static std::string s_topic_manifest, s_topic_image, s_topic_install, s_topic_installed, s_topic_status;

// Latest staged manifest, filled by the esp-mqtt event task, consumed by the mqtt_pub task.
// Guarded by s_mutex (as are the esp_ota/sha handles below — see ota_on_mqtt_data()).
struct Manifest
{
    std::string version;
    size_t      size = 0;
    uint8_t     sha256[32] = {};
    bool        force = false;
    bool        valid = false;
    bool        differs_from_running = false;
};
static Manifest s_manifest;
static SemaphoreHandle_t s_mutex = nullptr;
static EventGroupHandle_t s_eg = nullptr;

static std::atomic<bool>   s_install_requested{false};
static std::atomic<bool>   s_session_active{false};
static std::atomic<bool>   s_conn_lost{false};   // set by ota_on_mqtt_error() during a session
static std::atomic<size_t> s_received{0};
static std::atomic<int>    s_attempts_this_boot{0};

// Download-session flash/hash state; only touched under s_mutex while s_session_active.
static esp_ota_handle_t        s_ota_handle = 0;
static const esp_partition_t  *s_ota_partition = nullptr;
static psa_hash_operation_t    s_sha_op;
static size_t                  s_expected_size = 0;

// esp-mqtt only presents the topic on the FIRST segment of a message larger than its RX
// buffer; continuations are attributed to whatever topic'd segment came last (safe because
// TCP ordering means one inbound publish completes before the next begins).
static bool s_last_topic_was_image = false;

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
// No escape handling: none of the four fields (git-describe version, decimal size, hex
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

static void handle_manifest(const char *data, size_t data_len, size_t total)
{
    if (data_len != total) {
        // Can't happen for a sane manifest (RX buffer is 4 KB); refuse rather than
        // stitch together segments for a message that has no business being that big.
        ESP_LOGE(TAG, "manifest larger than RX buffer (%zu bytes) — ignored", total);
        return;
    }
    const std::string_view body(data, data_len);

    const auto version = manifest_field(body, "version");
    const auto size    = manifest_field(body, "size");
    const auto sha     = manifest_field(body, "sha256");
    const auto force   = manifest_field(body, "force");

    Manifest m;
    size_t size_val = 0;
    const bool size_ok = size &&
        std::from_chars(size->data(), size->data() + size->size(), size_val).ec == std::errc{} &&
        size_val > 0;
    if (!version || version->empty() || !size_ok || !sha || !parse_sha256_hex(*sha, m.sha256)) {
        ESP_LOGE(TAG, "manifest missing/invalid version|size|sha256 — ignored");
        return;
    }
    m.version = *version;
    m.size    = size_val;
    m.force   = force && *force == "true";
    m.valid   = true;
    m.differs_from_running = (m.version != esp_app_get_description()->version);

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_manifest = std::move(m);
    xSemaphoreGive(s_mutex);

    ESP_LOGI(TAG, "manifest: version=%s size=%zu force=%d (running=%s, differs=%d)",
             s_manifest.version.c_str(), s_manifest.size, s_manifest.force,
             esp_app_get_description()->version, s_manifest.differs_from_running);
}

// One broker-streamed image segment. Runs in the esp-mqtt event task; the flash write
// intentionally blocks it (TCP receive-window backpressure IS the flow control).
static void handle_image_segment(const char *data, size_t data_len, size_t offset, size_t total)
{
    if (!s_session_active.load())
        return;  // late segments after an abort (unsubscribe races the in-flight stream)

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    bool fail = false;
    if (offset == 0 && total != s_expected_size) {
        ESP_LOGE(TAG, "image size %zu != manifest size %zu", total, s_expected_size);
        fail = true;
    } else if (offset != s_received.load()) {
        // esp-mqtt segments arrive strictly in order; a gap means we lost sync (e.g. a
        // second retained publish raced the download). Restarting beats corrupt flash.
        ESP_LOGE(TAG, "segment offset %zu != received %zu — out of sync", offset, s_received.load());
        fail = true;
    } else if (const esp_err_t err = esp_ota_write(s_ota_handle, data, data_len); err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_write failed at %zu: 0x%x", offset, err);
        fail = true;
    } else if (psa_hash_update(&s_sha_op, reinterpret_cast<const uint8_t *>(data), data_len)
               != PSA_SUCCESS) {
        ESP_LOGE(TAG, "psa_hash_update failed at %zu", offset);
        fail = true;
    } else {
        s_received.store(offset + data_len);
    }
    xSemaphoreGive(s_mutex);

    if (fail)
        xEventGroupSetBits(s_eg, OTA_BIT_FAIL);
    else if (s_received.load() == total)
        xEventGroupSetBits(s_eg, OTA_BIT_DONE);
}

// ── public API ────────────────────────────────────────────────────────────────

void ota_updater_init(std::string_view device_id, const NetworkLink *link)
{
    s_link = link;
    const auto full = [&](std::string_view suffix) {
        return std::format("{}/{}", device_id, suffix);
    };
    s_topic_manifest  = full(OTA_SUFFIX_MANIFEST);
    s_topic_image     = full(OTA_SUFFIX_IMAGE);
    s_topic_install   = full(OTA_SUFFIX_INSTALL);
    s_topic_installed = full(OTA_SUFFIX_INSTALLED);
    s_topic_status    = full(OTA_SUFFIX_STATUS);
    if (!s_mutex)
        s_mutex = xSemaphoreCreateMutex();
    if (!s_eg)
        s_eg = xEventGroupCreate();
}

const char *ota_topic_manifest()  { return s_topic_manifest.c_str(); }
const char *ota_topic_image()     { return s_topic_image.c_str(); }
const char *ota_topic_install()   { return s_topic_install.c_str(); }
const char *ota_topic_installed() { return s_topic_installed.c_str(); }
const char *ota_topic_status()    { return s_topic_status.c_str(); }

void ota_on_mqtt_data(const char *topic, size_t topic_len,
                      const char *data, size_t data_len,
                      size_t offset, size_t total)
{
    if (topic_len > 0) {
        if (topic_is(topic, topic_len, s_topic_image)) {
            s_last_topic_was_image = true;
            handle_image_segment(data, data_len, offset, total);
            return;
        }
        s_last_topic_was_image = false;
        if (topic_is(topic, topic_len, s_topic_manifest)) {
            handle_manifest(data, data_len, total);
        } else if (topic_is(topic, topic_len, s_topic_install)) {
            // Any non-empty retained payload counts as an install request; the empty
            // payload we publish after success is the MQTT "delete retained" idiom.
            s_install_requested.store(data_len > 0);
            ESP_LOGI(TAG, "install flag: %s", data_len > 0 ? "SET" : "cleared");
        }
        return;
    }

    // Continuation segment (no topic): belongs to the last topic'd message.
    if (s_last_topic_was_image)
        handle_image_segment(data, data_len, offset, total);
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
    if (s_attempts_this_boot.load() >= OTA_MAX_ATTEMPTS_PER_BOOT)
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

    const int attempt = s_attempts_this_boot.fetch_add(1) + 1;
    s_ota_partition = esp_ota_get_next_update_partition(nullptr);
    if (!s_ota_partition || s_ota_partition->size < manifest.size) {
        publish_status(client, std::format(
            "{{\"state\":\"error\",\"reason\":\"image {} bytes exceeds slot\"}}", manifest.size));
        return;
    }

    ESP_LOGI(TAG, "starting OTA %s -> %s (%zu bytes, attempt %d/%d) into %s",
             esp_app_get_description()->version, manifest.version.c_str(),
             manifest.size, attempt, OTA_MAX_ATTEMPTS_PER_BOOT, s_ota_partition->label);
    publish_status(client, std::format(
        "{{\"state\":\"starting\",\"version\":\"{}\",\"size\":{},\"attempt\":{}}}",
        manifest.version, manifest.size, attempt));

    // OTA_WITH_SEQUENTIAL_WRITES erases flash incrementally as segments arrive instead of
    // blocking here for a multi-second whole-partition erase.
    if (const esp_err_t err = esp_ota_begin(s_ota_partition, OTA_WITH_SEQUENTIAL_WRITES, &s_ota_handle);
        err != ESP_OK) {
        publish_status(client, std::format("{{\"state\":\"error\",\"reason\":\"esp_ota_begin 0x{:x}\"}}",
                                           static_cast<unsigned>(err)));
        return;
    }
    // psa_crypto_init() is idempotent; the hash op then streams alongside esp_ota_write().
    s_sha_op = PSA_HASH_OPERATION_INIT;
    if (psa_crypto_init() != PSA_SUCCESS ||
        psa_hash_setup(&s_sha_op, PSA_ALG_SHA_256) != PSA_SUCCESS) {
        esp_ota_abort(s_ota_handle);
        s_ota_handle = 0;
        publish_status(client, "{\"state\":\"error\",\"reason\":\"sha256 setup failed\"}");
        return;
    }
    s_expected_size = manifest.size;
    s_received.store(0);
    s_conn_lost.store(false);
    xEventGroupClearBits(s_eg, OTA_BIT_DONE | OTA_BIT_FAIL);
    s_session_active.store(true);

    // rx-on-when-idle for the download; MUST be undone on every exit path below — a child
    // left rx-on burns ~78 mA until the battery dies.
    s_link->onOtaWindowBegin();
    // QoS 0: the retained image rides the same TCP stream either way; QoS 1 would only add
    // a pointless 1.8 MB-message PUBACK dance.
    esp_mqtt_client_subscribe(client, s_topic_image.c_str(), 0);

    // ── wait for the broker-streamed download to finish ──────────────────────
    bool ok = false;
    std::string fail_reason = "transfer failed";
    const TickType_t session_start = xTaskGetTickCount();
    size_t last_progress = 0;
    while (true) {
        const EventBits_t bits = xEventGroupWaitBits(s_eg, OTA_BIT_DONE | OTA_BIT_FAIL,
                                                     pdTRUE, pdFALSE,
                                                     pdMS_TO_TICKS(OTA_STALL_TIMEOUT_MS));
        if (bits & OTA_BIT_FAIL) {
            if (s_conn_lost.load())
                fail_reason = std::format("connection lost at offset {}", s_received.load());
            break;
        }
        if (bits & OTA_BIT_DONE) {
            ok = true;
            break;
        }
        const size_t received = s_received.load();
        if (received == last_progress) {
            fail_reason = std::format("no data for {} s at offset {}", OTA_STALL_TIMEOUT_MS / 1000, received);
            break;
        }
        if ((xTaskGetTickCount() - session_start) > pdMS_TO_TICKS(OTA_SESSION_CAP_MS)) {
            fail_reason = "session cap exceeded";
            break;
        }
        last_progress = received;
        publish_status(client, std::format("{{\"state\":\"downloading\",\"received\":{},\"total\":{}}}",
                                           received, manifest.size));
    }

    // Stop accepting segments BEFORE touching the ota handle: late in-flight segments
    // (unsubscribe can't recall what the broker already sent) bail out on this flag, and
    // the mutex below orders us after any segment write already in progress.
    s_session_active.store(false);
    esp_mqtt_client_unsubscribe(client, s_topic_image.c_str());

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (ok) {
        uint8_t sha[32] = {};
        size_t sha_len = 0;
        if (psa_hash_finish(&s_sha_op, sha, sizeof(sha), &sha_len) != PSA_SUCCESS ||
            sha_len != sizeof(sha) || std::memcmp(sha, manifest.sha256, sizeof(sha)) != 0) {
            ok = false;
            fail_reason = "sha256 mismatch";
        } else if (const esp_err_t err = esp_ota_end(s_ota_handle); err != ESP_OK) {
            // esp_ota_end validates the image header/magic on top of our hash check.
            ok = false;
            fail_reason = std::format("esp_ota_end 0x{:x}", static_cast<unsigned>(err));
            s_ota_handle = 0;  // esp_ota_end released it, success or not
        } else {
            s_ota_handle = 0;
            if (const esp_err_t berr = esp_ota_set_boot_partition(s_ota_partition); berr != ESP_OK) {
                ok = false;
                fail_reason = std::format("esp_ota_set_boot_partition 0x{:x}", static_cast<unsigned>(berr));
            }
        }
    }
    if (!ok) {
        psa_hash_abort(&s_sha_op);  // safe on an already-terminated operation
        if (s_ota_handle != 0) {
            esp_ota_abort(s_ota_handle);
            s_ota_handle = 0;
        }
    }
    xSemaphoreGive(s_mutex);

    // Back to sleepy link mode in BOTH outcomes — on success the parent should see a clean
    // MLE mode transition before we drop off for the reboot.
    s_link->onOtaWindowEnd();

    if (!ok) {
        publish_status(client, std::format("{{\"state\":\"error\",\"reason\":\"{}\",\"received\":{}}}",
                                           fail_reason, s_received.load()));
        ESP_LOGE(TAG, "OTA failed: %s", fail_reason.c_str());
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
