#include "mqtt_sender.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <format>
#include <memory>
#include <span>
#include <string>
#include <type_traits>
#include <utility>

#include "esp_app_desc.h"
#include "esp_crt_bundle.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "mqtt_client.h"

#include "ota_updater.h"
#include "history_log.h"

static const char *TAG = "mqtt-sender";

static constexpr EventBits_t BIT_CONNECTED = BIT0;
static constexpr EventBits_t BIT_ALL_ACKED = BIT1;
static constexpr EventBits_t BIT_ERROR     = BIT2;

// Separate, module-level event group used only to signal "no publish cycle in flight".
static constexpr EventBits_t BIT_IDLE = BIT0;

// How long the publish task waits for the broker to become reachable before giving up
// on a cycle (backed by NetworkLink::waitForBrokerReachable — e.g. OT: NAT64 prefix
// learned, only for an IPv4 broker; Wi-Fi: always immediate). After attach a NAT64
// route can land slightly late; a few seconds covers the gap.
static constexpr uint32_t BROKER_REACHABLE_WAIT_MS = 5000;

// Which sensors' HA-discovery configs have been confirmed sent this boot. Per-sensor bits, not
// one bool: each cycle publishes configs only for the values actually present, so a first cycle
// missing one value (humidity disabled, battery ADC failure) must not permanently skip that
// sensor's discovery -- it goes out on the first later cycle that carries the value.
enum DiscoveryBit : uint8_t {
    DISC_TEMP   = 1 << 0,
    DISC_HUM    = 1 << 1,
    DISC_BATT   = 1 << 2,  // covers the Battery + Voltage pair -- always published together
    DISC_UPDATE = 1 << 3,  // HA `update` entity config + retained installed-version -- always published together
    DISC_RSSI   = 1 << 4,  // Signal strength -- owed once a cycle actually carries an RSSI reading
    DISC_DIAG   = 1 << 5,  // Boot count + Reset reason pair -- boot-constant, so always available
};

static MqttConfig s_cfg;
static const NetworkLink *s_link = nullptr;       // set in mqtt_sender_init(); backs the transport
static std::atomic<uint8_t> s_discovery_sent_mask{0};
static std::atomic<bool> s_task_running{false};
static std::atomic<bool> s_last_ok{false};        // true iff the most recent finished cycle connected AND was ACKed
static EventGroupHandle_t s_idle_eg = nullptr;    // created in mqtt_sender_init(); starts idle

// Full "-----BEGIN CERTIFICATE-----...-----END CERTIFICATE-----" PEM, built once in
// mqtt_sender_init() from MqttConfig::tls_ca_cert_b64. esp-mqtt stores the pointer passed
// via broker.verification.certificate raw — it never copies or frees it (confirmed against
// esp-mqtt's source: esp_mqtt_destroy_config() frees host/uri/path/scheme/alpn_protos/etc.
// but not cacert_buf) — so it must outlive every per-cycle client's create/connect/destroy
// lifecycle, not just one start_client() call. A file-scope static that's never freed
// satisfies that. Empty when tls_ca_cert_b64 is empty (falls back to the public CA bundle).
static std::string s_tls_ca_cert_pem;

// Wraps a headerless/footerless base64 body into a parseable PEM certificate — the same
// runtime-wrap trick esp-mqtt's own ssl example uses for CONFIG_BROKER_CERTIFICATE_OVERRIDE,
// proven against this mbedtls PEM parser. Returns "" if body is empty.
static std::string wrap_pem_certificate(std::string_view base64_body)
{
    if (base64_body.empty())
        return {};
    return std::format("-----BEGIN CERTIFICATE-----\n{}\n-----END CERTIFICATE-----\n", base64_body);
}

// ── RAII wrappers for the per-cycle FreeRTOS/esp-mqtt handles ─────────────────
// Both are owned exclusively by run_publish_cycle() below and freed via these deleters on
// every return path (success or early-abort) — see run_publish_cycle()'s doc comment for why
// that function, and not mqtt_publish_task() itself, is where these must live.
using EventGroupPtr = std::unique_ptr<std::remove_pointer_t<EventGroupHandle_t>, decltype(&vEventGroupDelete)>;

struct MqttClientDeleter
{
    void operator()(esp_mqtt_client_handle_t client) const noexcept
    {
        if (!client)
            return;
        esp_mqtt_client_stop(client);
        esp_mqtt_client_destroy(client);
    }
};
using MqttClientPtr = std::unique_ptr<std::remove_pointer_t<esp_mqtt_client_handle_t>, MqttClientDeleter>;

// ── context shared between the publish task and the event handler ─────────────
struct MqttCtx
{
    EventGroupHandle_t eg;  // borrowed from run_publish_cycle()'s EventGroupPtr; never owned here
    std::atomic<int>   expected_acks{0};
    std::atomic<int>   received_acks{0};
};

// One HA sensor entity's discovery config, declaratively. publish_discovery() appends only
// the parts whose field is set, so entities without a device_class or unit (Boot count,
// Reset reason) come out of the same builder as the classic measurement sensors instead of
// needing yet another whole-payload format-string variant.
struct DiscoverySpec
{
    const char *name;        // HA entity name, e.g. "Temperature"
    const char *topic_slug;  // config-topic path segment; matches device_class where one exists
    const char *device_class = nullptr;  // nullptr: omit the field
    const char *state_class  = nullptr;  // nullptr: omit -> HA records no long-term statistics
    const char *unit         = nullptr;  // nullptr: omit (unit-less entity)
    int         precision    = -1;       // suggested_display_precision; <0: omit
    const char *key;         // state-JSON field name, e.g. "t"
    bool        diagnostic = false;      // file under HA's Diagnostic section
};

// HA MQTT-discovery config payload, assembled part-by-part (see publish_discovery()).
// Shown un-escaped for readability, optional parts in [brackets]; in the format strings
// every literal { } is doubled, and Jinja "{{ value_json.X }}" -> "{{{{value_json.{}}}}}".
//   {"name":"<>",["device_class":"<>",]["entity_category":"diagnostic",]
//    ["state_class":"<>",]["unit_of_measurement":"<>",]["suggested_display_precision":N,]
//    ["expire_after":N,]"state_topic":"<id>/state","value_template":"{{value_json.<key>}}",
//    "unique_id":"<id>_<key>","device":{"identifiers":["<id>"],"name":"<name>",...}}
static constexpr std::string_view DISC_PART_HEAD         = "{{\"name\":\"{}\",";
static constexpr std::string_view DISC_PART_DEVICE_CLASS = "\"device_class\":\"{}\",";
static constexpr std::string_view DISC_PART_DIAGNOSTIC   = "\"entity_category\":\"diagnostic\",";
static constexpr std::string_view DISC_PART_STATE_CLASS  = "\"state_class\":\"{}\",";
static constexpr std::string_view DISC_PART_UNIT         = "\"unit_of_measurement\":\"{}\",";
static constexpr std::string_view DISC_PART_PRECISION    = "\"suggested_display_precision\":{},";
static constexpr std::string_view DISC_PART_EXPIRE       = "\"expire_after\":{},";
static constexpr std::string_view DISC_PART_TAIL =
    "\"state_topic\":\"{}/state\","
    "\"value_template\":\"{{{{value_json.{}}}}}\","
    "\"unique_id\":\"{}_{}\","
    "\"device\":{{\"identifiers\":[\"{}\"],\"name\":\"{}\",\"sw_version\":\"{}\","
    "\"manufacturer\":\"Seeed Studio\",\"model\":\"XIAO ESP32-C6\",\"serial_number\":\"{}\"}}"
    "}}";

// HA MQTT `update` entity: gives the device an "Install" button + version pair in HA.
// installed_version comes from the retained <id>/ota/installed topic (plain version string),
// latest_version straight from the retained OTA manifest, and HA's Install click publishes
// a RETAINED command (retain:true below) so the sleeping device can't miss it — see
// ota_updater.h for the topic contract. device_class "firmware" files it with the device's
// firmware section; the device block matches the sensors' so all entities share one HA device.
static constexpr std::string_view UPDATE_DISCOVERY_FMT =
    "{{"
    "\"name\":\"Firmware\","
    "\"device_class\":\"firmware\","
    "\"state_topic\":\"{}\","
    "\"latest_version_topic\":\"{}\","
    "\"latest_version_template\":\"{{{{ value_json.version }}}}\","
    "\"command_topic\":\"{}\","
    "\"payload_install\":\"install\","
    "\"retain\":true,"
    "\"unique_id\":\"{}_fw\","
    "\"device\":{{\"identifiers\":[\"{}\"],\"name\":\"{}\",\"sw_version\":\"{}\","
    "\"manufacturer\":\"Seeed Studio\",\"model\":\"XIAO ESP32-C6\",\"serial_number\":\"{}\"}}"
    "}}";

// The other format strings used below, named (like the DISC_PART_* strings above) so their compile-time
// .size() can size the fixed buffers that follow instead of hand-counting characters.
static constexpr std::string_view DISCOVERY_TOPIC_FMT = "homeassistant/sensor/{}/{}/config";
static constexpr std::string_view UPDATE_DISCOVERY_TOPIC_FMT = "homeassistant/update/{}/firmware/config";
static constexpr std::string_view STATE_TOPIC_FMT      = "{}/state";
static constexpr std::string_view STATE_FMT_BOTH  = "{{\"t\":{:.3g},\"h\":{:.3g}}}";
static constexpr std::string_view STATE_FMT_TEMP  = "{{\"t\":{:.3g}}}";
static constexpr std::string_view STATE_FMT_HUMID = "{{\"h\":{:.3g}}}";
// Appended over the base state JSON's closing '}' when battery data is present (percent, then
// volts) -- re-closes the object, so the result stays valid JSON. Kept as a suffix instead of
// battery variants of the three STATE_FMT_* strings above: that would double them to six.
static constexpr std::string_view STATE_BATT_SUFFIX_FMT = ",\"b\":{},\"v\":{:.3f}}}";
// Same overwrite-the-'}' chaining for the diagnostic values: link RSSI in dBm (only when the
// transport has a reading this cycle), then boot count + reset reason (boot-constant, so
// appended on every state message -- HA's expire_after would otherwise flag the two entities
// unavailable while the rest of the device keeps reporting).
static constexpr std::string_view STATE_RSSI_SUFFIX_FMT = ",\"r\":{}}}";
static constexpr std::string_view STATE_DIAG_SUFFIX_FMT = ",\"bc\":{},\"rr\":\"{}\"}}";

// history_log.h backlog replay -- not part of HA discovery/state, a plain device -> broker
// event stream an HA-side automation consumes (see README's "Blackout data buffering &
// backfill" section). "ago" is seconds before this publish that the reading was captured; HA
// computes the real historical timestamp itself (arrival time - ago), since the device never
// has a wall clock (see history_log.h's doc comment for why).
static constexpr std::string_view BACKFILL_TOPIC_FMT = "{}/backfill";
static constexpr std::string_view BACKFILL_ARRAY_OPEN  = "[";
static constexpr std::string_view BACKFILL_ENTRY_FIRST_FMT = "{{\"ago\":{},\"t\":{:.3g},\"h\":{:.3g}}}";
static constexpr std::string_view BACKFILL_ENTRY_REST_FMT  = ",{{\"ago\":{},\"t\":{:.3g},\"h\":{:.3g}}}";
static constexpr std::string_view BACKFILL_ARRAY_CLOSE = "]";
// Entries per MQTT message and messages per publish cycle -- bounds a long backlog to drain
// across several successful cycles instead of costing one cycle unbounded awake time; both
// tunable like the OTA chunk size above them in spirit. 20 entries keeps BACKFILL_PAYLOAD_BUF
// (below) comfortably under start_client()'s 2048 B out_size; 5 batches/cycle is generous
// progress (100 entries) without risking PUBLISH_TIMEOUT_MS (sensorstask.cpp) on a slow link.
static constexpr size_t BACKFILL_BATCH_SIZE = 20;
static constexpr int    BACKFILL_MAX_BATCHES_PER_CYCLE = 5;

// ── fixed-capacity string building — no heap allocation ────────────────────────
// Every buffer size below is derived, not hand-picked, using one lemma: for a std::format string
// built only from literal text, "{}" placeholders, and "{{"/"}}" escapes, the format string's own
// .size() is always >= the length it contributes to the output once every argument is
// hypothetically stripped to length 0 -- each "{}" placeholder consumes >=2 format-string chars
// but contributes 0 to that baseline, and each "{{"/"}}" escape consumes 2 format-string chars
// but contributes only 1 to the output. So `fmt.size() + (sum of each argument's own max length)`
// is always a safe (if slightly generous) upper bound on the real output length, fully evaluated
// by the compiler -- it can't go stale if a format string above is edited later, unlike a
// hand-counted comment. format_into() below still detects (and logs) a truncation as defense in
// depth, but reaching that path should now require a bug in this derivation, not just an edit
// to one of the format strings above.
static constexpr size_t MAX_DEVICE_ID_LEN   = MQTT_MAX_DEVICE_ID_LEN;    // mqtt_sender.h
static constexpr size_t MAX_DEVICE_NAME_LEN = MQTT_MAX_DEVICE_NAME_LEN;  // mqtt_sender.h
// Longest of each DiscoverySpec-field literal ever passed, at the seven publish_discovery()
// call sites below (Temperature/Humidity/Battery/Voltage/Signal strength/Boot count/Reset reason).
static constexpr size_t MAX_NAME_LEN         = std::max({sizeof("Temperature"), sizeof("Humidity"),
                                                         sizeof("Battery"), sizeof("Voltage"),
                                                         sizeof("Signal strength"), sizeof("Boot count"),
                                                         sizeof("Reset reason")}) - 1;
static constexpr size_t MAX_DEVICE_CLASS_LEN = std::max({sizeof("temperature"), sizeof("humidity"),
                                                         sizeof("battery"), sizeof("voltage"),
                                                         sizeof("signal_strength")}) - 1;
// The config topic's path segment -- device_class where one exists, so its lengths are a
// superset of MAX_DEVICE_CLASS_LEN's plus the class-less sensors' made-up slugs.
static constexpr size_t MAX_TOPIC_SLUG_LEN   = std::max({MAX_DEVICE_CLASS_LEN + 1, sizeof("rssi"),
                                                         sizeof("boot_count"), sizeof("reset_reason")}) - 1;
static constexpr size_t MAX_STATE_CLASS_LEN  = std::max({sizeof("measurement"),
                                                         sizeof("total_increasing")}) - 1;
static constexpr size_t MAX_UNIT_LEN         = std::max({sizeof("°C"), sizeof("%"), sizeof("V"),
                                                         sizeof("dBm")}) - 1;
static constexpr size_t MAX_KEY_LEN          = std::max({sizeof("t"), sizeof("bc"),
                                                         sizeof("rr")}) - 1;  // h/b/v/r are 1 char
static constexpr size_t MAX_PRECISION_LEN    = 1;   // a single decimal digit at every call site
static constexpr size_t MAX_EXPIRE_LEN       = 10;  // uint32_t seconds, at most 10 digits
// {:.3g} (3 significant digits) always needs fewer characters than a full round-trip float --
// reusing common_utils.h's appendNum() bound (sign + up to 17 sig.digits + '.' + 'e' + sign + 3
// exp.digits = 24 chars) rather than deriving a tighter one specific to 3 sig figs. This one
// constant is a reasoned/cited numeric-formatting-width fact, not a sizeof()-derived one -- the
// lemma above only applies to literal string lengths, not to how wide a formatted number can get.
static constexpr size_t MAX_FORMATTED_FLOAT_LEN = 24;

// Widest of the ota/* topic strings ever interpolated into UPDATE_DISCOVERY_FMT below
// ("<device_id>/" + suffix; the three used there are manifest/install/installed).
static constexpr size_t MAX_OTA_TOPIC_LEN = MAX_DEVICE_ID_LEN + 1 /* '/' */ +
    std::max({OTA_SUFFIX_MANIFEST.size(), OTA_SUFFIX_INSTALL.size(), OTA_SUFFIX_INSTALLED.size()});
// esp_app_desc_t::version is a fixed char[32] including its NUL.
static constexpr size_t MAX_SW_VERSION_LEN = sizeof(esp_app_desc_t::version) - 1;

// The chip-unique serial published in the device block: the trailing hex-MAC chars
// addOTMacSuffix() (main.cpp) appends to every device_id -- see serial_from_device_id().
static constexpr size_t MAX_SERIAL_LEN = 2 * MQTT_MAC_ADDRESS_BYTES;

static constexpr size_t MAX_DISCOVERY_TOPIC_LEN =
    DISCOVERY_TOPIC_FMT.size() + MAX_DEVICE_ID_LEN + MAX_TOPIC_SLUG_LEN;
static constexpr size_t MAX_UPDATE_DISCOVERY_TOPIC_LEN =
    UPDATE_DISCOVERY_TOPIC_FMT.size() + MAX_DEVICE_ID_LEN;
static constexpr size_t MAX_STATE_TOPIC_LEN = STATE_TOPIC_FMT.size() + MAX_DEVICE_ID_LEN;
static constexpr size_t MAX_BACKFILL_TOPIC_LEN = BACKFILL_TOPIC_FMT.size() + MAX_DEVICE_ID_LEN;
static constexpr size_t TOPIC_BUF = std::max({MAX_DISCOVERY_TOPIC_LEN, MAX_UPDATE_DISCOVERY_TOPIC_LEN,
                                              MAX_STATE_TOPIC_LEN, MAX_BACKFILL_TOPIC_LEN}) + 1;  // +1 NUL

// The lemma applies per part (each part is itself a format string of literals, {} and {{/}}
// escapes), so the bound for the assembled payload is the sum of every part's size — as if
// every optional part were appended — plus each argument's max length. device_id is
// substituted 3x in DISC_PART_TAIL (state_topic, unique_id, device.identifiers) -- verified
// against the actual format call's argument list, not just eyeballed, after an earlier
// draft of this constant used 2x and format_to_n() silently truncated in a stress test.
static constexpr size_t DISCOVERY_PAYLOAD_BUF = DISC_PART_HEAD.size()
    + DISC_PART_DEVICE_CLASS.size() + DISC_PART_DIAGNOSTIC.size() + DISC_PART_STATE_CLASS.size()
    + DISC_PART_UNIT.size() + DISC_PART_PRECISION.size() + DISC_PART_EXPIRE.size()
    + DISC_PART_TAIL.size()
    + MAX_NAME_LEN + MAX_DEVICE_CLASS_LEN + MAX_STATE_CLASS_LEN + MAX_UNIT_LEN
    + MAX_PRECISION_LEN + MAX_EXPIRE_LEN
    + 3 * MAX_DEVICE_ID_LEN + 2 * MAX_KEY_LEN + MAX_DEVICE_NAME_LEN + MAX_SW_VERSION_LEN
    + MAX_SERIAL_LEN + 1;  // +1 NUL

// UPDATE_DISCOVERY_FMT's argument list: three ota/* topics (state/latest/command), then
// device_id twice (unique_id, identifiers), device name, sw_version, serial — same lemma as above.
static constexpr size_t UPDATE_DISCOVERY_PAYLOAD_BUF = UPDATE_DISCOVERY_FMT.size()
    + 3 * MAX_OTA_TOPIC_LEN + 2 * MAX_DEVICE_ID_LEN + MAX_DEVICE_NAME_LEN
    + MAX_SW_VERSION_LEN + MAX_SERIAL_LEN + 1;  // +1 NUL

// convertVoltageToPercent() clamps to 0..100, so "100" is the widest "b" can ever print.
static constexpr size_t MAX_BATTERY_PCT_LEN = sizeof("100") - 1;
// RSSI arrives as int; both link implementations produce int8 dBm values, but bounding by
// the argument's actual type (INT32_MIN is 11 chars) keeps this reasoned like
// MAX_FORMATTED_FLOAT_LEN above rather than trusting callers.
static constexpr size_t MAX_RSSI_LEN       = sizeof("-2147483648") - 1;
static constexpr size_t MAX_BOOT_COUNT_LEN = 10;  // uint32_t, at most 10 digits

// Each suffix overwrites the previous JSON's closing '}' (net -1), so simply adding every
// suffix's full worst-case length on top of the base keeps this a safe upper bound per the
// lemma above.
static constexpr size_t STATE_BUF = std::max({
    STATE_FMT_BOTH.size()  + 2 * MAX_FORMATTED_FLOAT_LEN,
    STATE_FMT_TEMP.size()  + MAX_FORMATTED_FLOAT_LEN,
    STATE_FMT_HUMID.size() + MAX_FORMATTED_FLOAT_LEN,
}) + STATE_BATT_SUFFIX_FMT.size() + MAX_BATTERY_PCT_LEN + MAX_FORMATTED_FLOAT_LEN
   + STATE_RSSI_SUFFIX_FMT.size() + MAX_RSSI_LEN
   + STATE_DIAG_SUFFIX_FMT.size() + MAX_BOOT_COUNT_LEN + MQTT_MAX_RESET_REASON_LEN
   + 1;  // +1 NUL

// Same lemma, applied to one backfill array: BACKFILL_ENTRY_REST_FMT (the wider of the two --
// leading comma) as every element's bound, ago_sec sized like MAX_EXPIRE_LEN (both a uint32_t
// seconds count, <=10 digits), times BACKFILL_BATCH_SIZE, plus the array brackets.
static constexpr size_t MAX_BACKFILL_ENTRY_LEN = BACKFILL_ENTRY_REST_FMT.size()
    + MAX_EXPIRE_LEN + 2 * MAX_FORMATTED_FLOAT_LEN;
static constexpr size_t BACKFILL_PAYLOAD_BUF = BACKFILL_ARRAY_OPEN.size()
    + BACKFILL_BATCH_SIZE * MAX_BACKFILL_ENTRY_LEN + BACKFILL_ARRAY_CLOSE.size()
    + 1;  // +1 NUL

// Formats into a fixed-capacity std::array via std::format_to_n (no heap allocation) and
// NUL-terminates the result. Returns the formatted length, or 0 (logged) if `buf` was too small
// for this input -- given the derivation above this should be unreachable; the check is
// defense in depth, not an expected path.
template <size_t N, typename... Args>
static size_t format_into(std::array<char, N> &buf, std::format_string<Args...> fmt, Args &&...args)
{
    const auto res = std::format_to_n(buf.data(), N - 1, fmt, std::forward<Args>(args)...);
    const size_t len = static_cast<size_t>(res.size);
    if (len > N - 1) {
        ESP_LOGE(TAG, "formatted string truncated (%zu > %zu chars) -- buffer too small for this input",
                 len, N - 1);
        return 0;
    }
    buf[len] = '\0';
    return len;
}

// format_into()'s append variant: formats starting at buf[offset] (overwriting whatever is
// there), NUL-terminates, and returns the new total length -- or 0 (logged) on truncation,
// matching format_into()'s contract so callers share the same "> 0" success check.
template <size_t N, typename... Args>
static size_t format_append(std::array<char, N> &buf, size_t offset, std::format_string<Args...> fmt, Args &&...args)
{
    const auto res = std::format_to_n(buf.data() + offset, N - 1 - offset, fmt, std::forward<Args>(args)...);
    const size_t len = static_cast<size_t>(res.size);
    if (len > N - 1 - offset) {
        ESP_LOGE(TAG, "appended string truncated (%zu > %zu chars) -- buffer too small for this input",
                 len, N - 1 - offset);
        return 0;
    }
    buf[offset + len] = '\0';
    return offset + len;
}

// ── event handler — ONLY sets event group bits, never touches the client ──────
static void mqtt_event_handler(void *handler_arg, esp_event_base_t /*base*/,
                                int32_t event_id, void *event_data)
{
    auto *ctx = static_cast<MqttCtx *>(handler_arg);
    switch (static_cast<esp_mqtt_event_id_t>(event_id)) {
    case MQTT_EVENT_CONNECTED:
        xEventGroupSetBits(ctx->eg, BIT_CONNECTED);
        break;
    case MQTT_EVENT_PUBLISHED:
        if (ctx->received_acks.fetch_add(1) + 1 >= ctx->expected_acks.load())
            xEventGroupSetBits(ctx->eg, BIT_ALL_ACKED);
        break;
    case MQTT_EVENT_DATA: {
        // Inbound traffic exists solely for OTA (retained manifest/install replies, and the
        // broker-streamed image during a download session) — route it all to ota_updater,
        // which demuxes by topic. Blocking in there (flash writes) is deliberate: it stalls
        // this task's socket reads so TCP backpressure paces the broker.
        const auto *ev = static_cast<esp_mqtt_event_handle_t>(event_data);
        ota_on_mqtt_data(ev->topic, static_cast<size_t>(ev->topic_len),
                         ev->data, static_cast<size_t>(ev->data_len),
                         static_cast<size_t>(ev->current_data_offset),
                         static_cast<size_t>(ev->total_data_len));
        break;
    }
    case MQTT_EVENT_ERROR: {
        const auto *err = static_cast<esp_mqtt_event_handle_t>(event_data)->error_handle;
        if (err) {
            ESP_LOGE(TAG, "MQTT error: type=%d esp_tls=%d errno=%d tls_stack=%d cert_verify_flags=%d",
                     err->error_type, err->esp_tls_last_esp_err, err->esp_transport_sock_errno,
                     err->esp_tls_stack_err, err->esp_tls_cert_verify_flags);
        }
        ota_on_mqtt_error();  // abort an in-flight download promptly (no-op otherwise)
        xEventGroupSetBits(ctx->eg, BIT_ERROR);
        break;
    }
    case MQTT_EVENT_DISCONNECTED:
        // Mid-cycle drop (broker restart, link loss). The publish path's own bounded waits
        // handle it; an in-flight OTA download must abort NOW rather than idle out its
        // 30 s no-progress watchdog against a connection that no longer exists.
        ota_on_mqtt_error();
        break;
    default:
        break;
    }
}

// ── start a fresh client; clears event bits before connecting ─────────────────
static esp_mqtt_client_handle_t start_client(const char *uri, MqttCtx &ctx)
{
    esp_mqtt_client_config_t cfg = {};
    cfg.broker.address.uri       = uri;
    cfg.credentials.username     = s_cfg.username.c_str();
    cfg.credentials.authentication.password = s_cfg.password.c_str();
    // Keepalive OFF, deliberately. The OTA image arrives as ONE multi-minute MQTT message;
    // esp-mqtt pings keepalive/2 after the last control packet and hard-aborts when the
    // PINGRESP misses the deadline (process_keepalive() in mqtt_client.c) — but the broker's
    // PINGRESP is queued behind megabytes of in-flight image on an ordered TCP stream, and
    // esp-mqtt parses one message at a time besides, so ANY finite keepalive shorter than the
    // whole transfer kills the download partway (hardware-observed: 3 silent "starting"-then-
    // dead attempts). Liveness never rested on keepalive anyway: this per-cycle client's every
    // wait is explicitly bounded (connect 5 s/15 s, ACK 4 s, sensorstask's 15 s publish cap,
    // OTA's own 30 s no-progress watchdog), and the client is destroyed at cycle end.
    cfg.session.disable_keepalive = true;
    // Outbox retransmission OFF in practice (default is a hair-trigger 1 s). Over TCP a
    // packet is never lost, only ACKed late — and during an OTA download SUBACKs queue for
    // seconds behind 8 KB chunk deliveries, so 1 s retransmits duplicated every chunk
    // SUBSCRIBE, which made the broker re-send each retained chunk again and again until
    // the duplicates starved the real download (hardware-observed: "ignoring unexpected
    // chunk" x9 storms and 20 s chunk timeouts). Our own bounded waits (4 s publish-ACK,
    // 20 s chunk) remain the real failure detectors.
    cfg.session.message_retransmit_timeout = 30000;
    // RX buffer sized so a max-size OTA image chunk arrives as ONE MQTT_EVENT_DATA event —
    // the property the chunked OTA protocol rests on (see ota_updater.h). Out-buffer stays
    // small separately: the largest outbound message is a ~700 B discovery config, and
    // leaving out_size 0 would clone the big RX size. Heap cost only while a per-cycle
    // client lives.
    cfg.buffer.size              = OTA_MQTT_RX_BUFFER_SIZE;
    cfg.buffer.out_size          = 2048;

    if (s_cfg.use_tls) {
        // Broker is always dialed by literal IP, never a hostname (see MqttConfig::
        // broker_address), so CN/SAN matching against the connect address was never
        // possible — skip it for both the pinned-cert and public-bundle paths below.
        cfg.broker.verification.skip_cert_common_name_check = true;
        if (!s_tls_ca_cert_pem.empty()) {
            // Pinned CA/leaf cert path (secrets.yaml "mqtt_tls_ca_cert"). certificate_len
            // stays 0 ("NUL-terminated string" mode, guaranteed by c_str()) — passing
            // .size() here would switch the parser into DER-length mode and break PEM parsing.
            cfg.broker.verification.certificate = s_tls_ca_cert_pem.c_str();
            cfg.broker.verification.certificate_len = 0;
        } else {
            cfg.broker.verification.crt_bundle_attach = esp_crt_bundle_attach;  // ESP-IDF's public CA bundle
        }
    }

    // Bound TCP ops so stop() returns quickly on failure. TLS needs more headroom than
    // plaintext: a full asymmetric handshake over a WAN link (searching/verifying against
    // the public CA bundle on this core) takes longer than a plaintext TCP connect+CONNACK.
    cfg.network.timeout_ms            = s_cfg.use_tls ? 8000 : 3000;
    cfg.network.disable_auto_reconnect = true;  // we manage reconnects ourselves (one task per cycle)

    ESP_LOGI(TAG, "connecting to %s (tls=%d)", uri, static_cast<int>(s_cfg.use_tls));
    xEventGroupClearBits(ctx.eg, BIT_CONNECTED | BIT_ALL_ACKED | BIT_ERROR);
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(client, MQTT_EVENT_ANY, mqtt_event_handler, &ctx);
    esp_mqtt_client_start(client);
    return client;
}

// The trailing hex-MAC chars addOTMacSuffix() (main.cpp) appends to every device_id. A
// length-bounded suffix rather than a '-' search, so the result's max length stays
// compile-time provable (MAX_SERIAL_LEN) for the payload-buffer bounds above.
static std::string_view serial_from_device_id(std::string_view device_id)
{
    return device_id.size() > MAX_SERIAL_LEN
        ? device_id.substr(device_id.size() - MAX_SERIAL_LEN) : device_id;
}

// Builds and publishes one HA MQTT-discovery config message by chaining the DISC_PART_*
// strings into a fixed-size stack buffer (see format_into()'s doc comment), appending only
// the parts `spec` asks for. expire_after rides on every sensor entity from
// MqttConfig::expire_after_sec (0 omits it) rather than from the spec: it's a device-level
// liveness property, not a per-entity one.
static void publish_discovery(esp_mqtt_client_handle_t client, std::string_view device_id,
                               std::string_view device_name, const DiscoverySpec &spec)
{
    std::array<char, TOPIC_BUF> topicBuf;
    const size_t topicLen = format_into(topicBuf, DISCOVERY_TOPIC_FMT, device_id, spec.topic_slug);

    std::array<char, DISCOVERY_PAYLOAD_BUF> payloadBuf;
    size_t len = format_into(payloadBuf, DISC_PART_HEAD, spec.name);
    if (len > 0 && spec.device_class)
        len = format_append(payloadBuf, len, DISC_PART_DEVICE_CLASS, spec.device_class);
    if (len > 0 && spec.diagnostic)
        len = format_append(payloadBuf, len, DISC_PART_DIAGNOSTIC);
    if (len > 0 && spec.state_class)
        len = format_append(payloadBuf, len, DISC_PART_STATE_CLASS, spec.state_class);
    if (len > 0 && spec.unit)
        len = format_append(payloadBuf, len, DISC_PART_UNIT, spec.unit);
    if (len > 0 && spec.precision >= 0)
        len = format_append(payloadBuf, len, DISC_PART_PRECISION, spec.precision);
    if (len > 0 && s_cfg.expire_after_sec > 0)
        len = format_append(payloadBuf, len, DISC_PART_EXPIRE, s_cfg.expire_after_sec);
    if (len > 0)
        len = format_append(payloadBuf, len, DISC_PART_TAIL,
                            device_id, spec.key,
                            device_id, spec.key,
                            device_id, device_name, esp_app_get_description()->version,
                            serial_from_device_id(device_id));

    if (topicLen == 0 || len == 0)
        return;  // format_into()/format_append() already logged the truncation

    esp_mqtt_client_publish(client, topicBuf.data(), payloadBuf.data(),
                            static_cast<int>(len), 1, 1);
}

// Publishes the HA `update` entity's discovery config plus the retained installed-version
// message it reads its state from — always together (see DISC_UPDATE), so HA never sees a
// version-less update entity. Two QoS-1 messages; callers must account for both ACKs.
static void publish_update_discovery(esp_mqtt_client_handle_t client, std::string_view device_id,
                                     std::string_view device_name)
{
    std::array<char, TOPIC_BUF> topicBuf;
    const size_t topicLen = format_into(topicBuf, UPDATE_DISCOVERY_TOPIC_FMT, device_id);

    std::array<char, UPDATE_DISCOVERY_PAYLOAD_BUF> payloadBuf;
    const char *sw_version = esp_app_get_description()->version;
    const size_t payloadLen = format_into(payloadBuf, UPDATE_DISCOVERY_FMT,
        ota_topic_installed(), ota_topic_manifest(), ota_topic_install(),
        device_id, device_id, device_name, sw_version,
        serial_from_device_id(device_id));

    if (topicLen == 0 || payloadLen == 0)
        return;  // format_into() already logged the truncation

    esp_mqtt_client_publish(client, topicBuf.data(), payloadBuf.data(),
                            static_cast<int>(payloadLen), 1, 1);
    // len 0 => esp-mqtt uses strlen(); retained so HA has installed_version across restarts.
    esp_mqtt_client_publish(client, ota_topic_installed(), sw_version, 0, 1, 1);
}

// Streams history_log.h's backlog (readings that failed to publish live during an outage) over
// the SAME still-open connection, once this cycle's own state publish is confirmed delivered
// (or there was nothing to publish -- see the caller's gating). Mirrors run_ota_if_due()
// below's "stay connected, keep working while the radio's already up" shape, but stays simple:
// unlike OTA there's no reconnect-and-resume here -- a batch that doesn't ACK just stops for
// this cycle (the log entries are untouched, since history_log_advance() only runs after a
// confirmed ACK) and picks up again next successful cycle, same as any other still-pending
// backlog. Bounded to BACKFILL_MAX_BATCHES_PER_CYCLE so a long backlog can't cost one cycle
// unbounded awake time.
static void replay_history_if_pending(esp_mqtt_client_handle_t client, std::string_view device_id,
                                      MqttCtx &ctx, EventGroupHandle_t eg)
{
    std::array<char, TOPIC_BUF> topicBuf;
    const size_t topicLen = format_into(topicBuf, BACKFILL_TOPIC_FMT, device_id);
    if (topicLen == 0)
        return;  // format_into() already logged the truncation

    for (int batchNum = 0; batchNum < BACKFILL_MAX_BATCHES_PER_CYCLE && history_log_has_pending(); ++batchNum) {
        std::array<HistoryLogEntry, BACKFILL_BATCH_SIZE> batch;
        const std::span<const HistoryLogEntry> replayed = history_log_peek_batch(batch);
        if (replayed.empty())
            break;

        std::array<char, BACKFILL_PAYLOAD_BUF> payloadBuf;
        size_t len = format_into(payloadBuf, BACKFILL_ARRAY_OPEN);
        for (size_t i = 0; i < replayed.size() && len > 0; ++i) {
            const HistoryLogEntry &e = replayed[i];
            len = (i == 0)
                ? format_append(payloadBuf, len, BACKFILL_ENTRY_FIRST_FMT, e.ago_sec, e.temp_c, e.hum_pct)
                : format_append(payloadBuf, len, BACKFILL_ENTRY_REST_FMT, e.ago_sec, e.temp_c, e.hum_pct);
        }
        if (len > 0)
            len = format_append(payloadBuf, len, BACKFILL_ARRAY_CLOSE);
        if (len == 0)
            break;  // format_into()/format_append() already logged the truncation

        // Fresh ACK wait, same pattern as the state message earlier in this cycle -- one
        // message, one expected ACK.
        ctx.expected_acks.store(1);
        ctx.received_acks.store(0);
        xEventGroupClearBits(eg, BIT_ALL_ACKED);
        esp_mqtt_client_publish(client, topicBuf.data(), payloadBuf.data(),
                                static_cast<int>(len), 1, 0);

        const bool acked = (xEventGroupWaitBits(eg, BIT_ALL_ACKED, pdFALSE, pdTRUE,
                                                pdMS_TO_TICKS(4000)) & BIT_ALL_ACKED) != 0;
        if (!acked) {
            ESP_LOGW(TAG, "backfill batch (%zu entries) not ACKed — resuming next cycle",
                     replayed.size());
            break;
        }
        history_log_advance(replayed.size());
        ESP_LOGI(TAG, "replayed %zu backlog entries", replayed.size());
    }
}

// Runs the OTA session when one is due, then — the v5 stability core — reconnects and
// resumes IN THIS CYCLE for as long as sessions keep dying by connection loss while making
// progress. v4 ended the cycle on the first abort, so every ~1 s radio hiccup cost the
// 0.5-3 min wait for the next LP-flagged publish cycle (~half the measured 17.6 min total).
// Replaces the caller's client on reconnect; the caller's normal teardown then destroys
// whichever client is current. On success ota_run_session() reboots and never returns.
static void run_ota_if_due(MqttClientPtr &client, MqttCtx &ctx, std::optional<int> battery_percent)
{
    if (!ota_update_due())
        return;
    ota_run_session(client.get(), battery_percent);

    // Loop only on CONNECTION loss: a battery deferral, config error or silent chunk
    // timeout leaves the connection alive — reconnecting can't help those, so they wait
    // for a later cycle. ota_update_due() also goes false once the no-progress budget is
    // spent, bounding this loop alongside the session's own 30 min cap.
    while (ota_update_due() && ota_session_ended_by_connection_loss()) {
        ESP_LOGI(TAG, "OTA interrupted by connection loss — reconnecting in-cycle to resume");
        client.reset();  // stop+destroy the dead client first
        const std::string uri = s_link->brokerUri(s_cfg.broker_address, s_cfg.port, s_cfg.use_tls);
        if (uri.empty())
            return;
        client.reset(start_client(uri.c_str(), ctx));
        const EventBits_t bits = xEventGroupWaitBits(ctx.eg, BIT_CONNECTED | BIT_ERROR,
                                                     pdFALSE, pdFALSE,
                                                     pdMS_TO_TICKS(s_cfg.use_tls ? 15000 : 5000));
        if (!(bits & BIT_CONNECTED)) {
            ESP_LOGW(TAG, "OTA reconnect failed — resuming on a later cycle");
            return;
        }
        ota_run_session(client.get(), battery_percent);
    }
}

// ── publish task — owns the client lifecycle ──────────────────────────────────
struct PublishParams
{
	std::optional<float> temperature;
	std::optional<float> humidity;
	std::optional<int>   battery_percent;
	std::optional<int>   battery_millivolts;
};

// Runs one connect -> publish -> disconnect cycle and reports whether the state message was
// confirmed delivered. Deliberately a separate, ordinary function (not inlined into
// mqtt_publish_task()) so its RAII locals (eg, client below) are freed by an ordinary `return`
// -- mqtt_publish_task() calls this and only afterward calls vTaskDelete(nullptr), which
// self-deletes the calling task WITHOUT unwinding the C++ stack (the task is torn down by the
// scheduler; destructors for anything declared in the frame that calls vTaskDelete(nullptr)
// never run merely because it was called). Declaring eg/client directly in mqtt_publish_task()
// and trusting them to clean up right before its final vTaskDelete(nullptr) would silently leak
// the event group and MQTT client handle every single publish cycle.
static bool run_publish_cycle(const PublishParams &params)
{
    const bool hasTemp = params.temperature.has_value();
    const bool hasHumid = params.humidity.has_value();
    // Both-or-neither: a lone battery value (shouldn't happen -- sensorstask always sets the
    // pair) is ignored rather than published half-formed.
    const bool hasBatt = params.battery_percent.has_value() && params.battery_millivolts.has_value();
    const float temp = params.temperature.value_or(0.0f);
    const float hum  = params.humidity.value_or(0.0f);

    // Declaration order matters: destruction runs in reverse, and client's teardown (below)
    // still needs ctx.eg (a borrowed copy of eg's handle) to be valid, so eg must outlive it.
    EventGroupPtr eg(xEventGroupCreate(), &vEventGroupDelete);
    MqttCtx ctx;
    ctx.eg = eg.get();

    s_link->onPublishWindowBegin();  // OT: fast polls so NAT64 prefix + TCP ACKs arrive promptly; Wi-Fi: no-op

    // Block until the broker's address family is reachable (OT: NAT64 prefix learned,
    // only for an IPv4 broker; Wi-Fi: always immediate — see NetworkLink::waitForBrokerReachable).
    if (!s_link->waitForBrokerReachable(s_cfg.broker_address, BROKER_REACHABLE_WAIT_MS)) {
        ESP_LOGE(TAG, "broker not reachable within %lu ms, skipping cycle",
                 (unsigned long)BROKER_REACHABLE_WAIT_MS);
        s_link->onPublishWindowEnd();
        return false;
    }

    const std::string uri = s_link->brokerUri(s_cfg.broker_address, s_cfg.port, s_cfg.use_tls);
    if (uri.empty()) {
        ESP_LOGE(TAG, "broker_address not set or invalid, cannot connect");
        s_link->onPublishWindowEnd();
        return false;
    }

    MqttClientPtr client(start_client(uri.c_str(), ctx));
    // TLS needs more time than plaintext: a full handshake over a WAN link (vs. plaintext's
    // bare TCP connect+CONNACK) can take several seconds on this core.
    const uint32_t connect_wait_ms = s_cfg.use_tls ? 15000 : 5000;
    const EventBits_t bits = xEventGroupWaitBits(eg.get(), BIT_CONNECTED | BIT_ERROR,
                                                 pdFALSE, pdFALSE, pdMS_TO_TICKS(connect_wait_ms));

    // ── publish if connected ──────────────────────────────────────────────────
    // ok stays false unless we connect, publish a state message, AND the broker ACKs it.
    // This is the signal the sensor task uses for the LED and the reboot supervisor, so it
    // must mean "data actually reached the broker", not merely "the task ran".
    bool ok = false;
    if (bits & BIT_CONNECTED) {
        const std::string_view dev = s_cfg.device_id;
        const std::string_view dev_name = s_cfg.device_name;

        // OTA check rides the publish window: subscribing now means the broker's retained
        // manifest/install replies (if it holds any) arrive while we're waiting for the
        // publish ACKs below — near-zero added awake time on the common no-update cycle.
        // QoS 0: retained delivery over an already-reliable TCP link. See ota_updater.h.
        esp_mqtt_client_subscribe(client.get(), ota_topic_manifest(), 0);
        esp_mqtt_client_subscribe(client.get(), ota_topic_install(), 0);

        // Battery is deliberately absent from hasAny: it only ever rides along on a
        // temperature/humidity publish (see mqtt_send_sensor_data()'s doc comment), so it can
        // neither trigger a cycle nor carry one alone.
        const bool hasAny = hasTemp || hasHumid;

        // Link RSSI is read here, not passed in with the sensor values: it's transport
        // state, and inside the publish window the radio is awake with the connect exchange
        // just refreshed (OT: "last packet from parent" is the CONNACK's frame).
        const std::optional<int> rssi = s_link->readRssiDbm ? s_link->readRssiDbm() : std::nullopt;

        // Discovery configs still owed this boot for the values present in THIS cycle.
        // DISC_UPDATE (the HA update entity + installed-version pair) and DISC_DIAG (the
        // boot-constant Boot count + Reset reason pair) aren't tied to any sensor value,
        // so they're owed on whichever publishing cycle comes first.
        const auto discoveryWant = static_cast<uint8_t>((hasTemp ? DISC_TEMP : 0)
                                                      | (hasHumid ? DISC_HUM : 0)
                                                      | (hasBatt ? DISC_BATT : 0)
                                                      | (rssi ? DISC_RSSI : 0)
                                                      | DISC_DIAG
                                                      | DISC_UPDATE);
        const auto discoveryNeed = hasAny
            ? static_cast<uint8_t>(discoveryWant & ~s_discovery_sent_mask.load()) : uint8_t{0};

        // Expected ACKs must match what we actually publish below: one state message plus one
        // discovery message per still-owed sensor (two for DISC_BATT: Battery and Voltage;
        // two for DISC_DIAG: Boot count and Reset reason; two for DISC_UPDATE: update config
        // and installed-version). A fixed count assuming every discovery is sent would leave
        // BIT_ALL_ACKED forever unset on any cycle that sends fewer, wrongly failing the cycle.
        const int discovery_msgs = ((discoveryNeed & DISC_TEMP) ? 1 : 0)
                                 + ((discoveryNeed & DISC_HUM) ? 1 : 0)
                                 + ((discoveryNeed & DISC_BATT) ? 2 : 0)
                                 + ((discoveryNeed & DISC_RSSI) ? 1 : 0)
                                 + ((discoveryNeed & DISC_DIAG) ? 2 : 0)
                                 + ((discoveryNeed & DISC_UPDATE) ? 2 : 0);
        const int expected = (hasAny ? 1 : 0) + discovery_msgs;

        // Set counters BEFORE publishing so the handler never races ahead
        ctx.expected_acks.store(expected);
        ctx.received_acks.store(0);
        xEventGroupClearBits(eg.get(), BIT_ALL_ACKED);

        // state_class "measurement" is what makes HA record long-term statistics
        // (5-minute min/max/mean beyond the recorder purge window) for an entity;
        // Boot count is a monotonic counter, which is exactly "total_increasing".
        if (discoveryNeed & DISC_TEMP)
            publish_discovery(client.get(), dev, dev_name,
                {.name = "Temperature", .topic_slug = "temperature", .device_class = "temperature",
                 .state_class = "measurement", .unit = "°C", .precision = 1, .key = "t"});
        if (discoveryNeed & DISC_HUM)
            publish_discovery(client.get(), dev, dev_name,
                {.name = "Humidity", .topic_slug = "humidity", .device_class = "humidity",
                 .state_class = "measurement", .unit = "%", .precision = 0, .key = "h"});
        if (discoveryNeed & DISC_BATT) {
            publish_discovery(client.get(), dev, dev_name,
                {.name = "Battery", .topic_slug = "battery", .device_class = "battery",
                 .state_class = "measurement", .unit = "%", .precision = 0, .key = "b",
                 .diagnostic = true});
            publish_discovery(client.get(), dev, dev_name,
                {.name = "Voltage", .topic_slug = "voltage", .device_class = "voltage",
                 .state_class = "measurement", .unit = "V", .precision = 2, .key = "v",
                 .diagnostic = true});
        }
        if (discoveryNeed & DISC_RSSI)
            publish_discovery(client.get(), dev, dev_name,
                {.name = "Signal strength", .topic_slug = "rssi", .device_class = "signal_strength",
                 .state_class = "measurement", .unit = "dBm", .precision = 0, .key = "r",
                 .diagnostic = true});
        if (discoveryNeed & DISC_DIAG) {
            publish_discovery(client.get(), dev, dev_name,
                {.name = "Boot count", .topic_slug = "boot_count",
                 .state_class = "total_increasing", .key = "bc", .diagnostic = true});
            publish_discovery(client.get(), dev, dev_name,
                {.name = "Reset reason", .topic_slug = "reset_reason", .key = "rr",
                 .diagnostic = true});
        }
        if (discoveryNeed & DISC_UPDATE)
            publish_update_discovery(client.get(), dev, dev_name);

        if (hasAny) {
            std::array<char, STATE_BUF> stateBuf;
            size_t stateLen = hasTemp && hasHumid
                ? format_into(stateBuf, STATE_FMT_BOTH, temp, hum)
                : hasTemp
                    ? format_into(stateBuf, STATE_FMT_TEMP, temp)
                    : format_into(stateBuf, STATE_FMT_HUMID, hum);  // hasHumid

            // Battery and RSSI ride along: overwrite the previous JSON's closing '}' with each
            // suffix, which re-closes the object. On a cycle without a value the keys are simply
            // absent -- HA keeps the entities' previous values (until expire_after lapses).
            if (stateLen > 0 && hasBatt) {
                stateLen = format_append(stateBuf, stateLen - 1, STATE_BATT_SUFFIX_FMT,
                                         *params.battery_percent,
                                         static_cast<float>(*params.battery_millivolts) / 1000.0f);
            }
            if (stateLen > 0 && rssi)
                stateLen = format_append(stateBuf, stateLen - 1, STATE_RSSI_SUFFIX_FMT, *rssi);
            // Boot count and reset reason are boot-constant, so they're re-sent on every state
            // message -- otherwise their entities would go stale-then-unavailable under
            // expire_after while the rest of the device keeps reporting.
            if (stateLen > 0)
                stateLen = format_append(stateBuf, stateLen - 1, STATE_DIAG_SUFFIX_FMT,
                                         s_cfg.boot_count, s_cfg.reset_reason);

            std::array<char, TOPIC_BUF> stateTopicBuf;
            const size_t stateTopicLen = format_into(stateTopicBuf, STATE_TOPIC_FMT, dev);

            if (stateLen > 0 && stateTopicLen > 0) {
                esp_mqtt_client_publish(client.get(), stateTopicBuf.data(), stateBuf.data(),
                                        static_cast<int>(stateLen), 1, 0);
                ESP_LOGI(TAG, "sent %s", stateBuf.data());

                // QoS-1 acks over a healthy Thread link return well under a second; cap short
                // so we stop fast-polling (and sleep) promptly instead of idling the radio.
                ok = (xEventGroupWaitBits(eg.get(), BIT_ALL_ACKED, pdFALSE, pdTRUE,
                                          pdMS_TO_TICKS(4000)) & BIT_ALL_ACKED) != 0;
                // Mark discovery sent only on a confirmed cycle: on a failed one the configs may
                // never have reached the broker, and the next successful cycle resends them.
                if (ok)
                    s_discovery_sent_mask.fetch_or(discoveryNeed);
            }
        } else {
            // Not a warning any more: with a pending update, sensorstask deliberately fires
            // value-less cycles every backstop wake so the OTA check below runs promptly.
            ESP_LOGI(TAG, "no sensor values this cycle%s", ota_update_due() ? " (OTA-only cycle)" : "");
        }

        // Same bar as the OTA check below: a data-carrying cycle needs connected AND state
        // ACKed, an OTA-only cycle just needs CONNECTED. Runs before OTA -- replay is quick
        // (a handful of small batches) and shouldn't wait behind a multi-minute OTA session,
        // and running it first means the backlog reaches HA promptly even if that OTA session
        // then reboots the device.
        if (ok || !hasAny)
            replay_history_if_pending(client.get(), dev, ctx, eg.get());

        // A staged update only starts from a healthy cycle: for a data-carrying cycle that
        // means connected AND state ACKed (a flaky link fails fast above instead of kicking
        // off a doomed download); an OTA-only cycle carries nothing to ACK, so CONNECTED is
        // the bar. On success this reboots and never returns; on failure it has restored
        // the sleepy link mode (and possibly replaced `client`) before normal teardown.
        if (ok || !hasAny)
            run_ota_if_due(client, ctx, params.battery_percent);
    } else {
        ESP_LOGE(TAG, "MQTT connection failed, skipping cycle");
    }

    s_link->onPublishWindowEnd();  // OT: back to slow poll until next sensor cycle; Wi-Fi: no-op
    return ok;
    // client, ctx, then eg are destroyed here (in that order) as this ordinary function returns
    // -- esp_mqtt_client_stop()+destroy(), then (trivially) ctx, then vEventGroupDelete().
}

static void mqtt_publish_task(void *arg)
{
    PublishParams paramsCopy;
    {
        // Reconstructs ownership of the heap block mqtt_send_sensor_data() handed across the
        // xTaskCreate() void* boundary. Freed at the end of THIS inner block, deliberately not
        // left to this unique_ptr's destructor firing at the end of mqtt_publish_task() itself
        // -- see run_publish_cycle()'s doc comment for why that wouldn't work (vTaskDelete(nullptr)
        // below never unwinds the stack).
        const std::unique_ptr<PublishParams> params(static_cast<PublishParams *>(arg));
        paramsCopy = *params;
    }

    const bool ok = run_publish_cycle(paramsCopy);

    s_last_ok.store(ok);  // published before BIT_IDLE is set below
    s_task_running.store(false);
    if (s_idle_eg)
    	xEventGroupSetBits(s_idle_eg, BIT_IDLE);  // wake any mqtt_wait_for_idle() caller
    vTaskDelete(nullptr);
}

// ── public API ────────────────────────────────────────────────────────────────

void mqtt_sender_init(const MqttConfig &cfg, const NetworkLink *link)
{
    s_cfg = cfg;
    s_link = link;
    s_tls_ca_cert_pem = wrap_pem_certificate(cfg.tls_ca_cert_b64);  // "" if tls_ca_cert_b64 is empty
    ota_updater_init(s_cfg.device_id, link);  // builds the <device_id>/ota/* topic strings
    if (!s_idle_eg) {
        s_idle_eg = xEventGroupCreate();
        if (s_idle_eg)
        	xEventGroupSetBits(s_idle_eg, BIT_IDLE);  // idle until first publish
    }
}

void mqtt_send_sensor_data(std::optional<float> temperature, std::optional<float> humidity,
                           std::optional<int> battery_percent, std::optional<int> battery_millivolts)
{
    if (s_task_running.exchange(true)) {
        ESP_LOGW(TAG, "previous publish cycle still running, skipping");
        return;
    }

    if (s_idle_eg)
    	xEventGroupClearBits(s_idle_eg, BIT_IDLE);  // mark busy until the task exits

    std::unique_ptr<PublishParams> params(new PublishParams{temperature, humidity,
                                                            battery_percent, battery_millivolts});
    if (xTaskCreate(mqtt_publish_task, "mqtt_pub", 12288, params.get(), 5, nullptr) != pdPASS) {
        ESP_LOGE(TAG, "failed to create mqtt_pub task");
        s_task_running.store(false);
        if (s_idle_eg)
        	xEventGroupSetBits(s_idle_eg, BIT_IDLE);
        return;  // params frees itself here
    }
    (void)params.release();  // ownership now belongs to mqtt_publish_task
}

bool mqtt_is_busy()
{
    return s_task_running.load();
}

bool mqtt_last_publish_succeeded()
{
    return s_last_ok.load();
}

bool mqtt_wait_for_idle(uint32_t timeout_ms)
{
    if (!s_idle_eg) return true;  // never initialised → nothing in flight
    const EventBits_t bits = xEventGroupWaitBits(
        s_idle_eg, BIT_IDLE, pdFALSE, pdTRUE, pdMS_TO_TICKS(timeout_ms));
    return (bits & BIT_IDLE) != 0;
}
