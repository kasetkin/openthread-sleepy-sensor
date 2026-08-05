#include "mqtt_sender.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <charconv>
#include <format>
#include <memory>
#include <span>
#include <string>
#include <type_traits>
#include <utility>

#include "esp_app_desc.h"
#include "esp_crt_bundle.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "mqtt_client.h"

#include "ota_updater.h"
#include "history_log.h"
#include "runtime_config.h"
#include "hp_awake_stats.h"

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
// uint16_t: DISC_NUMBERS/DISC_SWITCH below already fill all 8 bits of a uint8_t, leaving zero
// headroom for anything added later -- widened proactively so the next entity doesn't need a
// second mask-width migration.
enum DiscoveryBit : uint16_t {
    DISC_TEMP    = 1 << 0,
    DISC_HUM     = 1 << 1,
    DISC_BATT    = 1 << 2,  // covers the Battery + Voltage pair -- always published together
    DISC_UPDATE  = 1 << 3,  // HA `update` entity config + retained installed-version -- always published together
    DISC_RSSI    = 1 << 4,  // Signal strength -- owed once a cycle actually carries an RSSI reading
    DISC_DIAG    = 1 << 5,  // Boot count + Reset reason pair -- boot-constant, so always available
    DISC_NUMBERS = 1 << 6,  // all 7 HA `number` entities (calibration/threshold config) -- always published together
    DISC_SWITCH  = 1 << 7,  // the ext_antenna HA `switch` entity -- boot-constant, so always available
    DISC_HEATER  = 1 << 8,  // Heater problem + Heater run count pair -- owed once a heater run has ever completed
    // Uplink instrumentation (LinkStats, network_link.h). Split across three bits rather than
    // grouped into one because the three groups have genuinely different availability, and a
    // bit whose entities can't all be fed would leave HA showing a permanently unavailable
    // entity once expire_after lapsed: radio timings need CONFIG_OPENTHREAD_RADIO_STATS_ENABLE
    // and a second cycle (they're deltas), the MAC/link-quality group needs an attached Thread
    // link, and the uplink RSSI needs a border router that speaks Thread 1.2 link metrics --
    // on a Thread 1.1 mesh, or any Wi-Fi build, DISC_UPLINK is simply never owed at all.
    DISC_RADIO   = 1 << 9,   // Radio TX time + Radio RX time pair
    DISC_LINKQ   = 1 << 10,  // TX retries + CCA failures + TX no-ack expiry + Parent link quality
    DISC_UPLINK  = 1 << 11,  // Uplink signal strength (the parent's RSSI measurement of US)
    DISC_TXPOWER = 1 << 12,  // TX power (active) diagnostic -- boot-constant, so always available
    DISC_AWAKE   = 1 << 13,  // HP awake time diagnostic -- boot-constant, so always available
    // MQTT connect-wait diagnostic -- boot-constant, always available (every cycle attempts a
    // connect). Battery ADC time is a sibling diagnostic added at the same time but deliberately
    // does NOT get its own bit -- it piggybacks on DISC_BATT below, since it's only ever
    // meaningful exactly when battery is (same code block, sensorstask.cpp). NOTE: only
    // `1 << 15` remains free in this uint16_t after this entry -- widen to uint32_t before
    // adding another one, don't silently exhaust the type.
    DISC_MQTTCONN = 1 << 14,
};

static MqttConfig s_cfg;
static const NetworkLink *s_link = nullptr;       // set in mqtt_sender_init(); backs the transport
static std::atomic<uint16_t> s_discovery_sent_mask{0};
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

// ── persistent client for the common (non-OTA) case ────────────────────────────
// See project_light_sleep_power_investigation memory for the full research trail (esp-mqtt
// source + espressif/esp-idf#11883) behind reusing this handle across cycles instead of a
// fresh init()+destroy() every time. esp_mqtt_client_stop()/start() alone never touch the
// buffers/transport list/outbox/config-string duplicates esp_mqtt_client_init() allocates --
// only esp_mqtt_client_destroy() frees them (confirmed against esp-mqtt source) -- so reusing
// the handle eliminates that allocation/heap-fragmentation churn on every cycle. Does NOT
// eliminate the per-cycle FreeRTOS task-creation cost: esp_mqtt_client_start() unconditionally
// spawns a fresh internal task every call regardless of whether the outer handle is reused.
//
// s_persistentCtx/s_persistentEg are ALSO file-scope persistent, not per-cycle stack locals
// like the temporary OTA client's MqttCtx below -- this is load-bearing, not just tidiness:
// esp-mqtt's event registration (esp_event_handler_register_with()) ADDS a handler entry
// rather than replacing one, so re-registering a fresh per-cycle &ctx against a REUSED client
// would accumulate dangling-pointer registrations pointing at freed stack frames once that
// cycle's run_publish_cycle() returns. The event handler is registered exactly once, in
// start_persistent_client() below, the first time the client is created.
//
// Never used for an actual OTA session -- see run_ota_if_due() below, which always connects
// its own separate, OTA_MQTT_RX_BUFFER_SIZE-buffered temporary client instead, so the chunked
// OTA protocol's "one full chunk per MQTT_EVENT_DATA event" invariant can never be broken by
// a manifest+install-request pair that happens to arrive on this persistent connection
// mid-cycle (a real race: subscribing to the OTA topics here is how such a pair is learned
// about in the first place).
static constexpr size_t SENSOR_MQTT_RX_BUFFER_SIZE = 2048;
static EventGroupHandle_t s_persistentEg = nullptr;
static MqttCtx s_persistentCtx;
static esp_mqtt_client_handle_t s_persistentClient = nullptr;

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

// HA `number`/`switch` entities for the HA-tunable-via-MQTT parameters (main/runtime_config.h)
// -- "entity_category":"config" is HA's dedicated bucket for user-editable settings, a sibling
// of DISC_PART_DIAGNOSTIC's "diagnostic" (read-only) above. Unlike the sensor/update families,
// state_topic == command_topic for both: HA's documented pattern for a number/switch entity to
// read its current value back from the very topic it publishes commands to, since
// runtime_config.cpp republishes a retained echo of the applied value on that same topic. No
// value_template: bare scalar payload, not JSON.
static constexpr std::string_view CMD_PART_CONFIG_CAT = "\"entity_category\":\"config\",";
// HA `number` only: forces the plain type-a-value box regardless of range/step. Omitting this
// leaves HA's own "auto" mode in charge, which picks slider vs. box purely from
// (max-min)/step -- an accidental, range-dependent choice (it put the four fine-step
// calibration/threshold entities on sliders while the coarser schedule entities landed on
// boxes) rather than a deliberate one. A box is more usable for typing an exact calibration
// value than dragging a slider, for all 7 number entities uniformly.
static constexpr std::string_view CMD_PART_MODE_BOX = "\"mode\":\"box\",";
static constexpr std::string_view CMD_PART_MINMAXSTEP = "\"min\":{},\"max\":{},\"step\":{},";
static constexpr std::string_view CMD_PART_PAYLOADS    = "\"payload_on\":\"{}\",\"payload_off\":\"{}\",";
static constexpr std::string_view CMD_PART_TAIL =
    "\"state_topic\":\"{}\","
    "\"command_topic\":\"{}\","
    "\"retain\":true,"
    "\"unique_id\":\"{}_{}\","
    "\"device\":{{\"identifiers\":[\"{}\"],\"name\":\"{}\",\"sw_version\":\"{}\","
    "\"manufacturer\":\"Seeed Studio\",\"model\":\"XIAO ESP32-C6\",\"serial_number\":\"{}\"}}"
    "}}";

// The other format strings used below, named (like the DISC_PART_* strings above) so their compile-time
// .size() can size the fixed buffers that follow instead of hand-counting characters.
static constexpr std::string_view DISCOVERY_TOPIC_FMT = "homeassistant/sensor/{}/{}/config";
static constexpr std::string_view BINARY_DISCOVERY_TOPIC_FMT = "homeassistant/binary_sensor/{}/{}/config";
static constexpr std::string_view UPDATE_DISCOVERY_TOPIC_FMT = "homeassistant/update/{}/firmware/config";
static constexpr std::string_view NUMBER_DISCOVERY_TOPIC_FMT = "homeassistant/number/{}/{}/config";
static constexpr std::string_view SWITCH_DISCOVERY_TOPIC_FMT = "homeassistant/switch/{}/{}/config";
static constexpr std::string_view STATE_TOPIC_FMT      = "{}/state";
static constexpr std::string_view STATE_FMT_BOTH  = "{{\"t\":{:.3g},\"h\":{:.3g}}}";
static constexpr std::string_view STATE_FMT_TEMP  = "{{\"t\":{:.3g}}}";
static constexpr std::string_view STATE_FMT_HUMID = "{{\"h\":{:.3g}}}";
// Appended over the base state JSON's closing '}' when battery data is present (percent, then
// volts, then the ADC chain's own wall-clock time) -- re-closes the object, so the result stays
// valid JSON. Kept as a suffix instead of battery variants of the three STATE_FMT_* strings
// above: that would double them to six. adc_time_us is always set alongside the battery fields
// by construction (sensorstask.cpp sets it inside the same readVoltageViaAdc block, timing the
// whole create->read->delete chain) -- value_or(0) at the call site is defensive, not expected.
static constexpr std::string_view STATE_BATT_SUFFIX_FMT = ",\"b\":{:.2f},\"v\":{:.3f},\"at\":{:.2f}}}";
// Same overwrite-the-'}' chaining for the diagnostic values: link RSSI in dBm (only when the
// transport has a reading this cycle), then boot count + reset reason (boot-constant, so
// appended on every state message -- HA's expire_after would otherwise flag the two entities
// unavailable while the rest of the device keeps reporting).
static constexpr std::string_view STATE_RSSI_SUFFIX_FMT = ",\"r\":{}}}";
static constexpr std::string_view STATE_DIAG_SUFFIX_FMT = ",\"bc\":{},\"rr\":\"{}\"}}";
// Heater problem/run-count pair -- absent until the LP core has ever completed a heater run
// (see hasHeater below), same "omit until real data exists" shape as battery/RSSI above.
static constexpr std::string_view STATE_HEATER_SUFFIX_FMT = ",\"hp\":\"{}\",\"hc\":{}}}";
// Uplink instrumentation, same overwrite-the-'}' chaining. All of these are PER-CYCLE values,
// not since-boot totals (openthread_link.cpp does the differencing), which is why the counters
// carry state_class "measurement" rather than "total_increasing" at their discovery sites.
// Radio times are published in milliseconds though LinkStats carries microseconds: a whole
// sleepy cycle's radio time is a few tens of ms at most, and ms keeps the HA graph readable.
// Note "rx" for radio RX time, not the more obvious "rr" -- that key is already reset reason.
static constexpr std::string_view STATE_RADIO_SUFFIX_FMT  = ",\"rt\":{:.2f},\"rx\":{:.2f}}}";
static constexpr std::string_view STATE_LINKQ_SUFFIX_FMT  = ",\"tr\":{},\"cf\":{},\"nk\":{},\"lq\":{}}}";
static constexpr std::string_view STATE_UPLINK_SUFFIX_FMT = ",\"ur\":{}}}";
// TX power actually in effect right now (runtime_config_tx_power_active_dbm()) -- boot-constant
// availability like STATE_DIAG_SUFFIX_FMT above, so appended on every state message.
static constexpr std::string_view STATE_TXPOWER_SUFFIX_FMT = ",\"tp\":{}}}";
// HP-core wall-clock time NOT in light sleep this cycle (hp_awake_stats_get_and_reset_us()) --
// same boot-constant availability, always appended. Float ms, not integer: a quiet cycle's
// awake time is meaningfully sub-millisecond-precise at this scale (see STATE_RADIO_SUFFIX_FMT).
static constexpr std::string_view STATE_AWAKE_SUFFIX_FMT = ",\"aw\":{:.2f}}}";
// Wall-clock time spent waiting for start_client()'s TCP+MQTT CONNECT/CONNACK this cycle --
// same boot-constant availability as STATE_AWAKE_SUFFIX_FMT above (every cycle attempts a
// connect), always appended. Diagnostic for the light-sleep power investigation's
// hp_awake_time residual (see project_light_sleep_power_investigation memory).
static constexpr std::string_view STATE_MQTTCONN_SUFFIX_FMT = ",\"mc\":{:.2f}}}";

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
// Longest of each DiscoverySpec-field literal ever passed, at the nine publish_discovery()
// call sites below (Temperature/Humidity/Battery/Voltage/Signal strength/Boot count/Reset
// reason/Heater problem/Heater run count).
static constexpr size_t MAX_NAME_LEN         = std::max({sizeof("Temperature"), sizeof("Humidity"),
                                                         sizeof("Battery"), sizeof("Voltage"),
                                                         sizeof("Signal strength"), sizeof("Boot count"),
                                                         sizeof("Reset reason"), sizeof("Heater problem"),
                                                         sizeof("Heater run count"),
                                                         sizeof("Radio TX time"), sizeof("Radio RX time"),
                                                         sizeof("TX retries"), sizeof("CCA failures"),
                                                         sizeof("TX no-ack expiry"),
                                                         sizeof("Parent link quality"),
                                                         sizeof("Uplink signal strength"),
                                                         sizeof("TX power (active)"),
                                                         sizeof("HP awake time"),
                                                         sizeof("Battery ADC time"),
                                                         sizeof("MQTT connect time")}) - 1;
static constexpr size_t MAX_DEVICE_CLASS_LEN = std::max({sizeof("temperature"), sizeof("humidity"),
                                                         sizeof("battery"), sizeof("voltage"),
                                                         sizeof("signal_strength"), sizeof("problem")}) - 1;
// The config topic's path segment -- device_class where one exists, so its lengths are a
// superset of MAX_DEVICE_CLASS_LEN's plus the class-less sensors' made-up slugs.
static constexpr size_t MAX_TOPIC_SLUG_LEN   = std::max({MAX_DEVICE_CLASS_LEN + 1, sizeof("rssi"),
                                                         sizeof("boot_count"), sizeof("reset_reason"),
                                                         sizeof("heater_problem"), sizeof("heater_run_count"),
                                                         sizeof("radio_tx_time"), sizeof("radio_rx_time"),
                                                         sizeof("tx_retries"), sizeof("cca_failures"),
                                                         sizeof("tx_no_ack_expiry"),
                                                         sizeof("link_quality_out"),
                                                         sizeof("uplink_rssi"),
                                                         sizeof("tx_power_active"),
                                                         sizeof("hp_awake_time"),
                                                         sizeof("battery_adc_time"),
                                                         sizeof("mqtt_connect_time")}) - 1;
static constexpr size_t MAX_STATE_CLASS_LEN  = std::max({sizeof("measurement"),
                                                         sizeof("total_increasing")}) - 1;
static constexpr size_t MAX_UNIT_LEN         = std::max({sizeof("°C"), sizeof("%"), sizeof("V"),
                                                         sizeof("dBm"), sizeof("ms")}) - 1;
static constexpr size_t MAX_KEY_LEN          = std::max({sizeof("t"), sizeof("bc"), sizeof("rr"),
                                                         sizeof("hp"), sizeof("hc")}) - 1;  // h/b/v/r are 1 char
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

// max() of the two component prefixes: BINARY_DISCOVERY_TOPIC_FMT ("binary_sensor") is
// longer than DISCOVERY_TOPIC_FMT ("sensor"), and both share this one buffer.
static constexpr size_t MAX_DISCOVERY_TOPIC_LEN =
    std::max(DISCOVERY_TOPIC_FMT.size(), BINARY_DISCOVERY_TOPIC_FMT.size())
    + MAX_DEVICE_ID_LEN + MAX_TOPIC_SLUG_LEN;
static constexpr size_t MAX_UPDATE_DISCOVERY_TOPIC_LEN =
    UPDATE_DISCOVERY_TOPIC_FMT.size() + MAX_DEVICE_ID_LEN;
static constexpr size_t MAX_STATE_TOPIC_LEN = STATE_TOPIC_FMT.size() + MAX_DEVICE_ID_LEN;
static constexpr size_t MAX_BACKFILL_TOPIC_LEN = BACKFILL_TOPIC_FMT.size() + MAX_DEVICE_ID_LEN;

// Longest HA entity `name` among the 8 `number` + 1 `switch` config entities (see the
// publish_number_discoveries()/publish_switch_discovery() call sites below).
static constexpr size_t MAX_CFG_NAME_LEN = std::max({sizeof("Temperature offset"), sizeof("Temperature min change"),
    sizeof("Humidity offset"), sizeof("Humidity min change"), sizeof("Max publish gap"),
    sizeof("Heater period"), sizeof("Heater high-RH trigger"), sizeof("External antenna"),
    sizeof("TX power")}) - 1;
// Longest of the 9 cfg/* topic suffixes (runtime_config.h).
static constexpr size_t MAX_CFG_SUFFIX_LEN = std::max({CFG_SUFFIX_TEMP_OFFSET.size(), CFG_SUFFIX_TEMP_MIN_CHANGE.size(),
    CFG_SUFFIX_RH_OFFSET.size(), CFG_SUFFIX_RH_MIN_CHANGE.size(), CFG_SUFFIX_MAX_PUBLISH_GAP_SEC.size(),
    CFG_SUFFIX_HEATER_PERIOD_MIN.size(), CFG_SUFFIX_HEATER_HIGH_RH_MIN.size(), CFG_SUFFIX_EXT_ANTENNA.size(),
    CFG_SUFFIX_TX_POWER_DBM.size()});
// Full "<device_id>/cfg/<suffix>" topic, interpolated 2x into CMD_PART_TAIL (state + command).
static constexpr size_t MAX_CFG_TOPIC_LEN = MAX_DEVICE_ID_LEN + 1 /* '/' */ + MAX_CFG_SUFFIX_LEN;
// unique_id's slug half -- the bare key name (suffix minus the "cfg/" segment).
static constexpr size_t MAX_CFG_SLUG_LEN = MAX_CFG_SUFFIX_LEN - (sizeof("cfg/") - 1);
static constexpr size_t MAX_CFG_UNIT_LEN = std::max({MAX_UNIT_LEN, sizeof("min") - 1});

static constexpr size_t MAX_NUMBER_DISCOVERY_TOPIC_LEN =
    NUMBER_DISCOVERY_TOPIC_FMT.size() + MAX_DEVICE_ID_LEN + MAX_CFG_SLUG_LEN;
static constexpr size_t MAX_SWITCH_DISCOVERY_TOPIC_LEN =
    SWITCH_DISCOVERY_TOPIC_FMT.size() + MAX_DEVICE_ID_LEN + MAX_CFG_SLUG_LEN;

// MAX_CFG_TOPIC_LEN isn't included here: unlike the topics above, the cfg/* command topics
// are never built via format_into() into a TOPIC_BUF-sized local -- they're already-built
// std::strings from runtime_config.cpp's topic accessors, passed straight through as args.
static constexpr size_t TOPIC_BUF = std::max({MAX_DISCOVERY_TOPIC_LEN, MAX_UPDATE_DISCOVERY_TOPIC_LEN,
                                              MAX_STATE_TOPIC_LEN, MAX_BACKFILL_TOPIC_LEN,
                                              MAX_NUMBER_DISCOVERY_TOPIC_LEN, MAX_SWITCH_DISCOVERY_TOPIC_LEN}) + 1;  // +1 NUL

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

// Shared bound for both publish_number_discovery() and publish_switch_discovery(): built from
// DISC_PART_HEAD (name) + CMD_PART_CONFIG_CAT (no args) + CMD_PART_MODE_BOX (no args, number
// only) + CMD_PART_MINMAXSTEP (min/max/step, number only) + DISC_PART_UNIT (unit, number only)
// + CMD_PART_PAYLOADS (payload_on/off, switch only) + CMD_PART_TAIL (state_topic,
// command_topic, unique_id, device block) -- device_id is substituted 2x in CMD_PART_TAIL
// (unique_id, device.identifiers) and slug 1x (unique_id only), same "verify against the
// actual argument list" discipline as DISCOVERY_PAYLOAD_BUF's comment above. Summing every
// part's max as if all applied is a safe (if slightly generous) shared bound, same lemma as
// DISCOVERY_PAYLOAD_BUF. min/max/step reuse MAX_FORMATTED_FLOAT_LEN even for the uint32_t call
// sites -- a safe (if generous) bound either way.
static constexpr size_t CMD_DISCOVERY_PAYLOAD_BUF = DISC_PART_HEAD.size()
    + CMD_PART_CONFIG_CAT.size() + CMD_PART_MODE_BOX.size() + CMD_PART_MINMAXSTEP.size()
    + DISC_PART_UNIT.size() + CMD_PART_PAYLOADS.size() + CMD_PART_TAIL.size()
    + MAX_CFG_NAME_LEN + 3 * MAX_FORMATTED_FLOAT_LEN + MAX_CFG_UNIT_LEN
    + (sizeof("ON") - 1) + (sizeof("OFF") - 1)
    + 2 * MAX_CFG_TOPIC_LEN + 2 * MAX_DEVICE_ID_LEN + MAX_CFG_SLUG_LEN
    + MAX_DEVICE_NAME_LEN + MAX_SW_VERSION_LEN + MAX_SERIAL_LEN
    + 1;  // +1 NUL

// convertVoltageToPercent() clamps to 0..100 with 2 decimal places, so "100.00" is the widest
// "b" can ever print.
static constexpr size_t MAX_BATTERY_PCT_LEN = sizeof("100.00") - 1;
// RSSI arrives as int; both link implementations produce int8 dBm values, but bounding by
// the argument's actual type (INT32_MIN is 11 chars) keeps this reasoned like
// MAX_FORMATTED_FLOAT_LEN above rather than trusting callers.
static constexpr size_t MAX_RSSI_LEN       = sizeof("-2147483648") - 1;
static constexpr size_t MAX_BOOT_COUNT_LEN = 10;  // uint32_t, at most 10 digits
static constexpr size_t MAX_HEATER_RUN_COUNT_LEN = 10;  // uint32_t, at most 10 digits
// Every per-cycle LinkStats counter (radio ms, retries, CCA failures, no-ack expiries, link
// quality) is a uint32_t, so one bound covers them all.
static constexpr size_t MAX_LINK_COUNTER_LEN = 10;
// "OFF" is the longer of the two literals STATE_HEATER_SUFFIX_FMT's "hp" field ever holds.
static constexpr size_t MAX_ON_OFF_LEN = sizeof("OFF") - 1;
// runtime_config_tx_power_active_dbm() returns int8_t (widened to int for formatting), so -128
// bounds it exactly -- unlike MAX_RSSI_LEN above, the source type's real range is known here.
static constexpr size_t MAX_TXPOWER_LEN = sizeof("-128") - 1;

// Each suffix overwrites the previous JSON's closing '}' (net -1), so simply adding every
// suffix's full worst-case length on top of the base keeps this a safe upper bound per the
// lemma above.
static constexpr size_t STATE_BUF = std::max({
    STATE_FMT_BOTH.size()  + 2 * MAX_FORMATTED_FLOAT_LEN,
    STATE_FMT_TEMP.size()  + MAX_FORMATTED_FLOAT_LEN,
    STATE_FMT_HUMID.size() + MAX_FORMATTED_FLOAT_LEN,
}) + STATE_BATT_SUFFIX_FMT.size() + MAX_BATTERY_PCT_LEN + 2 * MAX_FORMATTED_FLOAT_LEN
   + STATE_RSSI_SUFFIX_FMT.size() + MAX_RSSI_LEN
   + STATE_DIAG_SUFFIX_FMT.size() + MAX_BOOT_COUNT_LEN + MQTT_MAX_RESET_REASON_LEN
   + STATE_HEATER_SUFFIX_FMT.size() + MAX_ON_OFF_LEN + MAX_HEATER_RUN_COUNT_LEN
   + STATE_RADIO_SUFFIX_FMT.size() + 2 * MAX_FORMATTED_FLOAT_LEN
   + STATE_LINKQ_SUFFIX_FMT.size() + 4 * MAX_LINK_COUNTER_LEN
   + STATE_UPLINK_SUFFIX_FMT.size() + MAX_RSSI_LEN
   + STATE_TXPOWER_SUFFIX_FMT.size() + MAX_TXPOWER_LEN
   + STATE_AWAKE_SUFFIX_FMT.size() + MAX_FORMATTED_FLOAT_LEN
   + STATE_MQTTCONN_SUFFIX_FMT.size() + MAX_FORMATTED_FLOAT_LEN
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
        // Sibling demux for the <id>/cfg/* topics (see runtime_config.h) -- no-op for any
        // other topic, same "each module ignores what isn't its own" shape as ota_on_mqtt_data.
        runtime_config_on_mqtt_data(ev->topic, static_cast<size_t>(ev->topic_len),
                                    ev->data, static_cast<size_t>(ev->data_len));
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
// Shared config, parameterized only by the RX buffer size (OTA_MQTT_RX_BUFFER_SIZE for the
// temporary OTA client below, SENSOR_MQTT_RX_BUFFER_SIZE for the persistent one) -- every
// other field is identical between the two clients.
static esp_mqtt_client_config_t build_client_config(const char *uri, size_t rx_buffer_size)
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
    // dead attempts). Liveness never rested on keepalive anyway: every wait on either client is
    // explicitly bounded (connect 5 s/15 s, ACK 4 s, sensorstask's 15 s publish cap, OTA's own
    // 30 s no-progress watchdog).
    cfg.session.disable_keepalive = true;
    // Outbox retransmission OFF in practice (default is a hair-trigger 1 s). Over TCP a
    // packet is never lost, only ACKed late — and during an OTA download SUBACKs queue for
    // seconds behind 8 KB chunk deliveries, so 1 s retransmits duplicated every chunk
    // SUBSCRIBE, which made the broker re-send each retained chunk again and again until
    // the duplicates starved the real download (hardware-observed: "ignoring unexpected
    // chunk" x9 storms and 20 s chunk timeouts). Our own bounded waits (4 s publish-ACK,
    // 20 s chunk) remain the real failure detectors.
    cfg.session.message_retransmit_timeout = 30000;
    // rx_buffer_size is OTA_MQTT_RX_BUFFER_SIZE for the temporary OTA client (so a max-size
    // image chunk arrives as ONE MQTT_EVENT_DATA event -- the property the chunked OTA
    // protocol rests on, see ota_updater.h) or the much smaller SENSOR_MQTT_RX_BUFFER_SIZE for
    // the persistent client (CONNACK + 3 SUBACKs + a small retained OTA-manifest/cfg echo, not
    // chunk data -- see its doc comment). Out-buffer stays a single small size either way: the
    // largest outbound message is a ~700 B discovery config, and leaving out_size 0 would
    // clone the (possibly much bigger) RX size.
    cfg.buffer.size              = rx_buffer_size;
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
    cfg.network.disable_auto_reconnect = true;  // we manage reconnects ourselves
    return cfg;
}

// Starts a fresh, temporary, OTA_MQTT_RX_BUFFER_SIZE-buffered client -- used only by
// run_ota_if_due() below, never by the common (non-OTA) path. Owned by the caller via
// MqttClientPtr, destroyed (stop()+destroy()) at the end of its scope -- unlike the
// persistent client, an actual OTA session is rare enough that paying full init/destroy cost
// every time is fine, and it must never share the persistent client's smaller buffer (see
// project_light_sleep_power_investigation memory for why that specific mixup would break the
// chunked protocol).
static esp_mqtt_client_handle_t start_client(const char *uri, MqttCtx &ctx)
{
    esp_mqtt_client_config_t cfg = build_client_config(uri, OTA_MQTT_RX_BUFFER_SIZE);
    ESP_LOGI(TAG, "connecting (OTA) to %s (tls=%d)", uri, static_cast<int>(s_cfg.use_tls));
    xEventGroupClearBits(ctx.eg, BIT_CONNECTED | BIT_ALL_ACKED | BIT_ERROR);
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(client, MQTT_EVENT_ANY, mqtt_event_handler, &ctx);
    esp_mqtt_client_start(client);
    return client;
}

// Lazily creates (once) and returns the persistent, small-buffered client used for every
// non-OTA cycle -- see its doc comment (s_persistentClient et al., above) for the full design
// reasoning. Every call after the first just reuses the existing handle: esp_mqtt_client_set_uri()
// is called every time regardless (not just on first init), since the broker URI can change at
// runtime if OpenThread's NAT64 prefix moves underneath a long-lived client. Returns nullptr on
// a first-time init failure (out of memory) -- the caller's connect-wait will then simply see
// no BIT_CONNECTED, same as any other failed connect.
static esp_mqtt_client_handle_t start_persistent_client(const char *uri)
{
    if (!s_persistentClient) {
        s_persistentEg = xEventGroupCreate();
        if (!s_persistentEg)
            return nullptr;
        s_persistentCtx.eg = s_persistentEg;

        esp_mqtt_client_config_t cfg = build_client_config(uri, SENSOR_MQTT_RX_BUFFER_SIZE);
        s_persistentClient = esp_mqtt_client_init(&cfg);
        if (!s_persistentClient)
            return nullptr;
        esp_mqtt_client_register_event(s_persistentClient, MQTT_EVENT_ANY, mqtt_event_handler, &s_persistentCtx);
    } else {
        esp_mqtt_client_set_uri(s_persistentClient, uri);
    }

    ESP_LOGI(TAG, "connecting (persistent) to %s (tls=%d)", uri, static_cast<int>(s_cfg.use_tls));
    xEventGroupClearBits(s_persistentEg, BIT_CONNECTED | BIT_ALL_ACKED | BIT_ERROR);
    esp_mqtt_client_start(s_persistentClient);
    return s_persistentClient;
}

// The trailing hex-MAC chars addOTMacSuffix() (main.cpp) appends to every device_id. A
// length-bounded suffix rather than a '-' search, so the result's max length stays
// compile-time provable (MAX_SERIAL_LEN) for the payload-buffer bounds above.
static std::string_view serial_from_device_id(std::string_view device_id)
{
    return device_id.size() > MAX_SERIAL_LEN
        ? device_id.substr(device_id.size() - MAX_SERIAL_LEN) : device_id;
}

// Shared payload body for both publish_discovery() and publish_binary_discovery() below --
// factored out because format_into()'s format-string argument must be a compile-time
// constant (std::format_string is consteval-checked), so the two callers can't merge into
// one function with a runtime-chosen topic format string; only the topic-building line
// differs between them, so only that part is duplicated, not this chaining logic.
static size_t build_discovery_payload(std::array<char, DISCOVERY_PAYLOAD_BUF> &payloadBuf,
                                      std::string_view device_id, std::string_view device_name,
                                      const DiscoverySpec &spec)
{
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
    return len;
}

// Builds and publishes one HA `sensor` MQTT-discovery config message, chaining the
// DISC_PART_* strings into a fixed-size stack buffer (see format_into()'s doc comment),
// appending only the parts `spec` asks for. expire_after rides on every sensor entity from
// MqttConfig::expire_after_sec (0 omits it) rather than from the spec: it's a device-level
// liveness property, not a per-entity one.
static void publish_discovery(esp_mqtt_client_handle_t client, std::string_view device_id,
                               std::string_view device_name, const DiscoverySpec &spec)
{
    std::array<char, TOPIC_BUF> topicBuf;
    const size_t topicLen = format_into(topicBuf, DISCOVERY_TOPIC_FMT, device_id, spec.topic_slug);

    std::array<char, DISCOVERY_PAYLOAD_BUF> payloadBuf;
    const size_t len = build_discovery_payload(payloadBuf, device_id, device_name, spec);

    if (topicLen == 0 || len == 0)
        return;  // format_into()/format_append() already logged the truncation

    esp_mqtt_client_publish(client, topicBuf.data(), payloadBuf.data(),
                            static_cast<int>(len), 1, 1);
}

// Same as publish_discovery() above, but files under HA's `binary_sensor` component instead
// of `sensor` -- the payload shape is identical (HA's default payload_on/payload_off,
// "ON"/"OFF", already match the plain {{value_json.<key>}} template as long as the state
// JSON carries that literal string), only the discovery topic's component segment differs.
static void publish_binary_discovery(esp_mqtt_client_handle_t client, std::string_view device_id,
                                     std::string_view device_name, const DiscoverySpec &spec)
{
    std::array<char, TOPIC_BUF> topicBuf;
    const size_t topicLen = format_into(topicBuf, BINARY_DISCOVERY_TOPIC_FMT, device_id, spec.topic_slug);

    std::array<char, DISCOVERY_PAYLOAD_BUF> payloadBuf;
    const size_t len = build_discovery_payload(payloadBuf, device_id, device_name, spec);

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

// One HA `number` entity's discovery config (main/runtime_config.h's live-tunable calibration/
// threshold/schedule parameters). Templated on T (float for the 4 calibration/threshold
// entities, uint32_t for the other 3) so min/max/step format exactly, matching each entity's
// real type. `topic` serves as BOTH state_topic and command_topic (see CMD_PART_TAIL's doc
// comment); unit may be nullptr for a unit-less entity (none currently, but supported). Also publishes
// `current_value` retained on `topic` right after the discovery config -- same "config + state
// together" shape as publish_update_discovery()'s discovery+installed-version pair -- so HA
// shows a real value immediately instead of "Unknown" until the entity is first commanded.
template <typename T>
static void publish_number_discovery(esp_mqtt_client_handle_t client, std::string_view device_id,
                                     std::string_view device_name, const char *name, const char *slug,
                                     const char *topic, T min, T max, T step, const char *unit,
                                     T current_value)
{
    std::array<char, TOPIC_BUF> topicBuf;
    const size_t topicLen = format_into(topicBuf, NUMBER_DISCOVERY_TOPIC_FMT, device_id, slug);

    std::array<char, CMD_DISCOVERY_PAYLOAD_BUF> payloadBuf;
    size_t len = format_into(payloadBuf, DISC_PART_HEAD, name);
    if (len > 0)
        len = format_append(payloadBuf, len, CMD_PART_CONFIG_CAT);
    if (len > 0)
        len = format_append(payloadBuf, len, CMD_PART_MODE_BOX);
    if (len > 0)
        len = format_append(payloadBuf, len, CMD_PART_MINMAXSTEP, min, max, step);
    if (len > 0 && unit)
        len = format_append(payloadBuf, len, DISC_PART_UNIT, unit);
    if (len > 0)
        len = format_append(payloadBuf, len, CMD_PART_TAIL,
                            topic, topic,
                            device_id, slug,
                            device_id, device_name, esp_app_get_description()->version,
                            serial_from_device_id(device_id));

    if (topicLen == 0 || len == 0)
        return;  // format_into()/format_append() already logged the truncation

    esp_mqtt_client_publish(client, topicBuf.data(), payloadBuf.data(),
                            static_cast<int>(len), 1, 1);

    // Bare scalar, not JSON (matches the retained echo runtime_config_apply_pending() itself
    // publishes on a live change) -- a fixed 32-byte stack buffer is a directly-reasoned bound
    // for a to_chars conversion (same reasoning as MAX_FORMATTED_FLOAT_LEN above), not the
    // file's format-string lemma, since this isn't a std::format call.
    std::array<char, 32> valBuf;
    const auto [ptr, ec] = std::to_chars(valBuf.data(), valBuf.data() + valBuf.size(), current_value);
    if (ec == std::errc{})
        esp_mqtt_client_publish(client, topic, valBuf.data(),
                                static_cast<int>(ptr - valBuf.data()), 1, 1);
}

// Retires the pre-rename "Max skip cycles" HA entity (device_config.yaml's `max_skip_cycles`,
// an LP-cycle count, became `max_publish_gap_sec`, wall-clock seconds -- a deliberate rename,
// not a reinterpretation, precisely so an old NVS/retained value is never silently misread in
// the wrong unit). Publishing an empty retained payload to its old discovery topic is MQTT
// discovery's standard removal convention, so HA drops the stale entity instead of showing it
// permanently "unavailable". Safe to publish every discovery cycle indefinitely -- idempotent
// and negligible cost -- so no one-shot guard is needed.
static void retire_old_max_skip_cycles_discovery(esp_mqtt_client_handle_t client, std::string_view device_id)
{
    std::array<char, TOPIC_BUF> topicBuf;
    const size_t topicLen = format_into(topicBuf, NUMBER_DISCOVERY_TOPIC_FMT, device_id, "max_skip_cycles");
    if (topicLen == 0)
        return;  // format_into() already logged the truncation
    esp_mqtt_client_publish(client, topicBuf.data(), "", 0, 1, 1);
}

// Issues all 8 publish_number_discovery() calls -- the ONE place these entities' HA-visible
// names/units/ranges are decided; ranges come straight from runtime_config.h so the clamp
// applied on the device side can never drift from what HA's UI advertises. Current values come
// from runtime_config_current_values(), fetched once here.
static void publish_number_discoveries(esp_mqtt_client_handle_t client, std::string_view device_id,
                                       std::string_view device_name)
{
    const RuntimeConfigValues cur = runtime_config_current_values();

    publish_number_discovery(client, device_id, device_name, "Temperature offset", "temp_offset",
        runtime_config_topic_temp_offset(), TEMP_OFFSET_MIN, TEMP_OFFSET_MAX, TEMP_OFFSET_STEP, "°C",
        cur.temp_offset_c);
    publish_number_discovery(client, device_id, device_name, "Temperature min change", "temp_min_change",
        runtime_config_topic_temp_min_change(), TEMP_MIN_CHANGE_MIN, TEMP_MIN_CHANGE_MAX, TEMP_MIN_CHANGE_STEP, "°C",
        cur.temp_min_change_c);
    publish_number_discovery(client, device_id, device_name, "Humidity offset", "rh_offset",
        runtime_config_topic_rh_offset(), RH_OFFSET_MIN, RH_OFFSET_MAX, RH_OFFSET_STEP, "%",
        cur.rh_offset_pct);
    publish_number_discovery(client, device_id, device_name, "Humidity min change", "rh_min_change",
        runtime_config_topic_rh_min_change(), RH_MIN_CHANGE_MIN, RH_MIN_CHANGE_MAX, RH_MIN_CHANGE_STEP, "%",
        cur.rh_min_change_pct);
    publish_number_discovery(client, device_id, device_name, "Max publish gap", "max_publish_gap_sec",
        runtime_config_topic_max_publish_gap_sec(), MAX_PUBLISH_GAP_SEC_MIN, MAX_PUBLISH_GAP_SEC_MAX,
        MAX_PUBLISH_GAP_SEC_STEP, "s", cur.max_publish_gap_sec);
    publish_number_discovery(client, device_id, device_name, "Heater period", "heater_period_minutes",
        runtime_config_topic_heater_period_minutes(), HEATER_PERIOD_MIN_MINUTES, HEATER_PERIOD_MAX_MINUTES,
        HEATER_PERIOD_STEP_MINUTES, "min", cur.heater_period_minutes);
    publish_number_discovery(client, device_id, device_name, "Heater high-RH trigger", "heater_high_rh_trigger_minutes",
        runtime_config_topic_heater_high_rh_trigger_minutes(), HEATER_HIGH_RH_MIN_MINUTES, HEATER_HIGH_RH_MAX_MINUTES,
        HEATER_HIGH_RH_STEP_MINUTES, "min", cur.heater_high_rh_trigger_minutes);
    publish_number_discovery(client, device_id, device_name, "TX power", "tx_power_dbm",
        runtime_config_topic_tx_power_dbm(), TX_POWER_DBM_MIN, TX_POWER_DBM_MAX, TX_POWER_DBM_STEP,
        "dBm", cur.tx_power_dbm);

    retire_old_max_skip_cycles_discovery(client, device_id);
}

// The ext_antenna HA `switch` entity's discovery config -- same state_topic==command_topic
// shape as the number entities above, with ON/OFF payloads instead of a numeric range. Also
// publishes the current ON/OFF state retained on `topic`, same "config + state together"
// reasoning as publish_number_discovery()'s value publish.
static void publish_switch_discovery(esp_mqtt_client_handle_t client, std::string_view device_id,
                                     std::string_view device_name)
{
    static constexpr const char *NAME = "External antenna";
    static constexpr const char *SLUG = "ext_antenna";
    const char *topic = runtime_config_topic_ext_antenna();
    const bool current_on = runtime_config_current_values().ext_antenna_on;

    std::array<char, TOPIC_BUF> topicBuf;
    const size_t topicLen = format_into(topicBuf, SWITCH_DISCOVERY_TOPIC_FMT, device_id, SLUG);

    std::array<char, CMD_DISCOVERY_PAYLOAD_BUF> payloadBuf;
    size_t len = format_into(payloadBuf, DISC_PART_HEAD, NAME);
    if (len > 0)
        len = format_append(payloadBuf, len, CMD_PART_CONFIG_CAT);
    if (len > 0)
        len = format_append(payloadBuf, len, CMD_PART_PAYLOADS, EXT_ANTENNA_PAYLOAD_ON, EXT_ANTENNA_PAYLOAD_OFF);
    if (len > 0)
        len = format_append(payloadBuf, len, CMD_PART_TAIL,
                            topic, topic,
                            device_id, SLUG,
                            device_id, device_name, esp_app_get_description()->version,
                            serial_from_device_id(device_id));

    if (topicLen == 0 || len == 0)
        return;  // format_into()/format_append() already logged the truncation

    esp_mqtt_client_publish(client, topicBuf.data(), payloadBuf.data(),
                            static_cast<int>(len), 1, 1);

    const std::string_view val = current_on ? EXT_ANTENNA_PAYLOAD_ON : EXT_ANTENNA_PAYLOAD_OFF;
    esp_mqtt_client_publish(client, topic, val.data(), static_cast<int>(val.size()), 1, 1);
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
// Self-contained: connects its own temporary, OTA_MQTT_RX_BUFFER_SIZE-buffered client (see
// start_client() above) rather than sharing whatever client the rest of this cycle used for
// its own purposes -- the persistent client's smaller buffer must never carry chunk data (see
// start_persistent_client()'s doc comment for the race this avoids: a manifest+install-request
// pair can be learned about via the persistent connection's own subscribe, but the actual
// chunk-pulling always happens on this dedicated connection instead). No-op if no update is
// due. On success ota_run_session() reboots and never returns.
static void run_ota_if_due(const std::string &uri, std::optional<float> battery_percent)
{
    if (!ota_update_due())
        return;

    MqttCtx ctx;
    EventGroupPtr eg(xEventGroupCreate(), &vEventGroupDelete);
    if (!eg)
        return;
    ctx.eg = eg.get();

    MqttClientPtr client(start_client(uri.c_str(), ctx));
    EventBits_t bits = xEventGroupWaitBits(ctx.eg, BIT_CONNECTED | BIT_ERROR,
                                           pdFALSE, pdFALSE,
                                           pdMS_TO_TICKS(s_cfg.use_tls ? 15000 : 5000));
    if (!(bits & BIT_CONNECTED)) {
        ESP_LOGW(TAG, "OTA connect failed — resuming on a later cycle");
        return;
    }
    ota_run_session(client.get(), battery_percent);

    // Loop only on CONNECTION loss: a battery deferral, config error or silent chunk
    // timeout leaves the connection alive — reconnecting can't help those, so they wait
    // for a later cycle. ota_update_due() also goes false once the no-progress budget is
    // spent, bounding this loop alongside the session's own 30 min cap.
    while (ota_update_due() && ota_session_ended_by_connection_loss()) {
        ESP_LOGI(TAG, "OTA interrupted by connection loss — reconnecting in-cycle to resume");
        client.reset();  // stop+destroy the dead client first
        const std::string reconnectUri = s_link->brokerUri(s_cfg.broker_address, s_cfg.port, s_cfg.use_tls);
        if (reconnectUri.empty())
            return;
        client.reset(start_client(reconnectUri.c_str(), ctx));
        bits = xEventGroupWaitBits(ctx.eg, BIT_CONNECTED | BIT_ERROR,
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
	std::optional<float> battery_percent;
	std::optional<int>   battery_millivolts;
	std::optional<bool>     heater_problem;
	std::optional<uint32_t> heater_run_count;
	std::optional<uint32_t> adc_time_us;
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
    // Both-or-neither, same reasoning as hasBatt above -- sensorstask always sets the pair.
    const bool hasHeater = params.heater_problem.has_value() && params.heater_run_count.has_value();
    const float temp = params.temperature.value_or(0.0f);
    const float hum  = params.humidity.value_or(0.0f);
    // Battery is deliberately absent from hasAny: it only ever rides along on a
    // temperature/humidity publish (see mqtt_send_sensor_data()'s doc comment), so it can
    // neither trigger a cycle nor carry one alone. Computed up front, not just inside the
    // connected branch below -- it feeds the skip-the-persistent-connection decision next.
    const bool hasAny = hasTemp || hasHumid;

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

    // ok stays false unless we connect, publish a state message, AND the broker ACKs it.
    // This is the signal the sensor task uses for the LED and the reboot supervisor, so it
    // must mean "data actually reached the broker", not merely "the task ran".
    bool ok = false;

    // An already-known-due OTA cycle with nothing else to publish skips the persistent
    // connection entirely -- OTA owns the radio/publish path for this wake (mirrors
    // sensorstask.cpp's "OTA download in progress -- skip ALL sensor work" philosophy on the
    // LP-core side), and going straight to run_ota_if_due()'s own dedicated connection avoids a
    // pointless double-connect. When hasAny is true the persistent connection still runs first
    // regardless of OTA-due-ness, same as always -- real sensor data must never be dropped
    // just because an update also happens to be pending this same wake.
    if (!(ota_update_due() && !hasAny)) {
        // TLS needs more time than plaintext: a full handshake over a WAN link (vs. plaintext's
        // bare TCP connect+CONNACK) can take several seconds on this core.
        const uint32_t connect_wait_ms = s_cfg.use_tls ? 15000 : 5000;
        // Diagnostic for the light-sleep power investigation's hp_awake_time residual (see
        // project_light_sleep_power_investigation memory) -- brackets exactly the connect
        // attempt below, success or failure either way.
        const int64_t mqttConnectStartUs = esp_timer_get_time();
        const esp_mqtt_client_handle_t client = start_persistent_client(uri.c_str());
        const EventBits_t bits = client
            ? xEventGroupWaitBits(s_persistentEg, BIT_CONNECTED | BIT_ERROR,
                                  pdFALSE, pdFALSE, pdMS_TO_TICKS(connect_wait_ms))
            : EventBits_t{0};
        const uint32_t mqttConnectUs = static_cast<uint32_t>(esp_timer_get_time() - mqttConnectStartUs);

        if (bits & BIT_CONNECTED) {
            MqttCtx &ctx = s_persistentCtx;
            const EventGroupHandle_t eg = s_persistentEg;
            const std::string_view dev = s_cfg.device_id;
            const std::string_view dev_name = s_cfg.device_name;

            // OTA check rides the publish window: subscribing now means the broker's retained
            // manifest/install replies (if it holds any) arrive while we're waiting for the
            // publish ACKs below — near-zero added awake time on the common no-update cycle.
            // QoS 0: retained delivery over an already-reliable TCP link. See ota_updater.h.
            esp_mqtt_client_subscribe(client, ota_topic_manifest(), 0);
            esp_mqtt_client_subscribe(client, ota_topic_install(), 0);
            // <id>/cfg/# -- one SUBSCRIBE packet for all 8 HA-tunable-parameter topics (see
            // runtime_config.h), same "ride the publish window" reasoning as the OTA subscribes.
            esp_mqtt_client_subscribe(client, runtime_config_topic_wildcard(), 0);

            // Link telemetry is read here, not passed in with the sensor values: it's transport
            // state, and inside the publish window the radio is awake with the connect exchange
            // just refreshed (OT: "last packet from parent" is the CONNACK's frame). Read exactly
            // ONCE per cycle -- the counter-derived fields are per-cycle deltas, so a second call
            // would split this cycle's radio time across the two.
            const std::optional<LinkStats> link =
                s_link->readLinkStats ? s_link->readLinkStats() : std::nullopt;
            const std::optional<int> rssi = link ? link->rssiDbm : std::nullopt;
            // Same "read exactly once per cycle" discipline as readLinkStats() above -- called
            // unconditionally (not gated on stateLen later) so a truncated/failed buffer never
            // skips resetting the accumulator and silently merges two cycles' awake time into one.
            const uint32_t hpAwakeUs = hp_awake_stats_get_and_reset_us();
            // Grouped exactly as the DISC_RADIO/DISC_LINKQ/DISC_UPLINK bits are, so a group is
            // owed only when every entity in it can actually be fed this cycle.
            const bool hasRadioStats = link && link->radioTxTimeUs && link->radioRxTimeUs;
            const bool hasLinkQuality = link && link->txRetries && link->txCcaFailures
                                     && link->txNoAckExpiry && link->linkQualityOut;
            const bool hasUplinkRssi = link && link->uplinkRssiDbm;

            // Discovery configs still owed this boot for the values present in THIS cycle.
            // DISC_UPDATE (the HA update entity + installed-version pair) and DISC_DIAG (the
            // boot-constant Boot count + Reset reason pair) aren't tied to any sensor value,
            // so they're owed on whichever publishing cycle comes first.
            const auto discoveryWant = static_cast<uint16_t>((hasTemp ? DISC_TEMP : 0)
                                                          | (hasHumid ? DISC_HUM : 0)
                                                          | (hasBatt ? DISC_BATT : 0)
                                                          | (rssi ? DISC_RSSI : 0)
                                                          | (hasHeater ? DISC_HEATER : 0)
                                                          | (hasRadioStats ? DISC_RADIO : 0)
                                                          | (hasLinkQuality ? DISC_LINKQ : 0)
                                                          | (hasUplinkRssi ? DISC_UPLINK : 0)
                                                          | DISC_DIAG
                                                          | DISC_UPDATE
                                                          | DISC_NUMBERS
                                                          | DISC_SWITCH
                                                          | DISC_TXPOWER
                                                          | DISC_AWAKE
                                                          | DISC_MQTTCONN);
            const auto discoveryNeed = hasAny
                ? static_cast<uint16_t>(discoveryWant & ~s_discovery_sent_mask.load()) : uint16_t{0};

            // Expected ACKs must match what we actually publish below: one state message plus one
            // discovery message per still-owed sensor (three for DISC_BATT: Battery, Voltage and
            // Battery ADC time; two for DISC_DIAG: Boot count and Reset reason; two for DISC_UPDATE:
            // update config and installed-version; sixteen for DISC_NUMBERS, discovery config +
            // current-value state per HA `number` entity, 8 entities x 2 messages; two for
            // DISC_SWITCH, discovery config + current-value state; two for DISC_HEATER: Heater
            // problem and Heater run count; two for DISC_RADIO: Radio TX time and Radio RX time;
            // four for DISC_LINKQ: TX retries, CCA failures, TX no-ack expiry and Parent link
            // quality; one for DISC_UPLINK: Uplink signal strength; one for DISC_TXPOWER: TX power
            // (active); one for DISC_AWAKE: HP awake time; one for DISC_MQTTCONN: MQTT connect
            // time). A fixed count assuming every discovery is sent would leave BIT_ALL_ACKED
            // forever unset on any cycle that sends fewer, wrongly failing the cycle.
            const int discovery_msgs = ((discoveryNeed & DISC_TEMP) ? 1 : 0)
                                     + ((discoveryNeed & DISC_HUM) ? 1 : 0)
                                     + ((discoveryNeed & DISC_BATT) ? 3 : 0)
                                     + ((discoveryNeed & DISC_RSSI) ? 1 : 0)
                                     + ((discoveryNeed & DISC_DIAG) ? 2 : 0)
                                     + ((discoveryNeed & DISC_UPDATE) ? 2 : 0)
                                     + ((discoveryNeed & DISC_NUMBERS) ? 16 : 0)
                                     + ((discoveryNeed & DISC_SWITCH) ? 2 : 0)
                                     + ((discoveryNeed & DISC_HEATER) ? 2 : 0)
                                     + ((discoveryNeed & DISC_RADIO) ? 2 : 0)
                                     + ((discoveryNeed & DISC_LINKQ) ? 4 : 0)
                                     + ((discoveryNeed & DISC_UPLINK) ? 1 : 0)
                                     + ((discoveryNeed & DISC_TXPOWER) ? 1 : 0)
                                     + ((discoveryNeed & DISC_AWAKE) ? 1 : 0)
                                     + ((discoveryNeed & DISC_MQTTCONN) ? 1 : 0);
            const int expected = (hasAny ? 1 : 0) + discovery_msgs;

            // Set counters BEFORE publishing so the handler never races ahead
            ctx.expected_acks.store(expected);
            ctx.received_acks.store(0);
            xEventGroupClearBits(eg, BIT_ALL_ACKED);

            // state_class "measurement" is what makes HA record long-term statistics
            // (5-minute min/max/mean beyond the recorder purge window) for an entity;
            // Boot count is a monotonic counter, which is exactly "total_increasing".
            if (discoveryNeed & DISC_TEMP)
                publish_discovery(client, dev, dev_name,
                    {.name = "Temperature", .topic_slug = "temperature", .device_class = "temperature",
                     .state_class = "measurement", .unit = "°C", .precision = 1, .key = "t"});
            if (discoveryNeed & DISC_HUM)
                publish_discovery(client, dev, dev_name,
                    {.name = "Humidity", .topic_slug = "humidity", .device_class = "humidity",
                     .state_class = "measurement", .unit = "%", .precision = 0, .key = "h"});
            if (discoveryNeed & DISC_BATT) {
                publish_discovery(client, dev, dev_name,
                    {.name = "Battery", .topic_slug = "battery", .device_class = "battery",
                     .state_class = "measurement", .unit = "%", .precision = 2, .key = "b",
                     .diagnostic = true});
                publish_discovery(client, dev, dev_name,
                    {.name = "Voltage", .topic_slug = "voltage", .device_class = "voltage",
                     .state_class = "measurement", .unit = "V", .precision = 2, .key = "v",
                     .diagnostic = true});
                // Piggybacks DISC_BATT rather than getting its own bit -- only ever meaningful
                // exactly when battery is (same create->read->delete block, sensorstask.cpp).
                publish_discovery(client, dev, dev_name,
                    {.name = "Battery ADC time", .topic_slug = "battery_adc_time",
                     .state_class = "measurement", .unit = "ms", .precision = 2, .key = "at",
                     .diagnostic = true});
            }
            if (discoveryNeed & DISC_RSSI)
                publish_discovery(client, dev, dev_name,
                    {.name = "Signal strength", .topic_slug = "rssi", .device_class = "signal_strength",
                     .state_class = "measurement", .unit = "dBm", .precision = 0, .key = "r",
                     .diagnostic = true});
            if (discoveryNeed & DISC_DIAG) {
                publish_discovery(client, dev, dev_name,
                    {.name = "Boot count", .topic_slug = "boot_count",
                     .state_class = "total_increasing", .key = "bc", .diagnostic = true});
                publish_discovery(client, dev, dev_name,
                    {.name = "Reset reason", .topic_slug = "reset_reason", .key = "rr",
                     .diagnostic = true});
            }
            if (discoveryNeed & DISC_HEATER) {
                publish_binary_discovery(client, dev, dev_name,
                    {.name = "Heater problem", .topic_slug = "heater_problem", .device_class = "problem",
                     .key = "hp", .diagnostic = true});
                publish_discovery(client, dev, dev_name,
                    {.name = "Heater run count", .topic_slug = "heater_run_count",
                     .state_class = "total_increasing", .key = "hc", .diagnostic = true});
            }
            // Uplink instrumentation. The counter entities are per-cycle deltas, so "measurement"
            // and not "total_increasing" -- HA would otherwise treat each cycle's small count as a
            // counter reset. Radio times are the ones that matter most: mean(rt) is the t_tx term
            // in dI_avg = dI_peak * t_tx / cycle_period, which is what decides whether reducing the
            // radio's +20 dBm default TX power is worth any link margin at all.
            if (discoveryNeed & DISC_RADIO) {
                publish_discovery(client, dev, dev_name,
                    {.name = "Radio TX time", .topic_slug = "radio_tx_time",
                     .state_class = "measurement", .unit = "ms", .precision = 1, .key = "rt",
                     .diagnostic = true});
                publish_discovery(client, dev, dev_name,
                    {.name = "Radio RX time", .topic_slug = "radio_rx_time",
                     .state_class = "measurement", .unit = "ms", .precision = 1, .key = "rx",
                     .diagnostic = true});
            }
            if (discoveryNeed & DISC_LINKQ) {
                publish_discovery(client, dev, dev_name,
                    {.name = "TX retries", .topic_slug = "tx_retries",
                     .state_class = "measurement", .precision = 0, .key = "tr", .diagnostic = true});
                publish_discovery(client, dev, dev_name,
                    {.name = "CCA failures", .topic_slug = "cca_failures",
                     .state_class = "measurement", .precision = 0, .key = "cf", .diagnostic = true});
                publish_discovery(client, dev, dev_name,
                    {.name = "TX no-ack expiry", .topic_slug = "tx_no_ack_expiry",
                     .state_class = "measurement", .precision = 0, .key = "nk", .diagnostic = true});
                publish_discovery(client, dev, dev_name,
                    {.name = "Parent link quality", .topic_slug = "link_quality_out",
                     .state_class = "measurement", .precision = 0, .key = "lq", .diagnostic = true});
            }
            // The parent's OWN measurement of our signal -- the opposite direction from "Signal
            // strength" above, and the only one that responds to our TX power. Absent entirely on a
            // Thread 1.1 border router (enh-ACK probing unsupported), by design.
            if (discoveryNeed & DISC_UPLINK)
                publish_discovery(client, dev, dev_name,
                    {.name = "Uplink signal strength", .topic_slug = "uplink_rssi",
                     .device_class = "signal_strength", .state_class = "measurement", .unit = "dBm",
                     .precision = 0, .key = "ur", .diagnostic = true});
            // The dBm actually in effect right now -- distinct from the cfg/tx_power_dbm number's
            // own retained echo, which can show an outstanding, still-unconfirmed trial value.
            if (discoveryNeed & DISC_TXPOWER)
                publish_discovery(client, dev, dev_name,
                    {.name = "TX power (active)", .topic_slug = "tx_power_active",
                     .state_class = "measurement", .unit = "dBm", .precision = 0, .key = "tp",
                     .diagnostic = true});
            // Total HP-core wall-clock time NOT in light sleep this cycle -- the piece Radio TX/RX
            // time doesn't cover (attach check, JSON encode, ACK wait, DFS ramp). Part of the
            // ~335 uA vs ~35 uA power investigation -- see the plan doc's quantitative model.
            if (discoveryNeed & DISC_AWAKE)
                publish_discovery(client, dev, dev_name,
                    {.name = "HP awake time", .topic_slug = "hp_awake_time",
                     .state_class = "measurement", .unit = "ms", .precision = 2, .key = "aw",
                     .diagnostic = true});
            // Wall-clock time waiting for the TCP+MQTT CONNECT/CONNACK above this cycle -- same
            // power-investigation motivation as DISC_AWAKE above, isolating one of its two
            // likeliest non-radio contributors (see project_light_sleep_power_investigation memory).
            if (discoveryNeed & DISC_MQTTCONN)
                publish_discovery(client, dev, dev_name,
                    {.name = "MQTT connect time", .topic_slug = "mqtt_connect_time",
                     .state_class = "measurement", .unit = "ms", .precision = 2, .key = "mc",
                     .diagnostic = true});
            if (discoveryNeed & DISC_UPDATE)
                publish_update_discovery(client, dev, dev_name);
            if (discoveryNeed & DISC_NUMBERS)
                publish_number_discoveries(client, dev, dev_name);
            if (discoveryNeed & DISC_SWITCH)
                publish_switch_discovery(client, dev, dev_name);

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
                                             static_cast<float>(*params.battery_millivolts) / 1000.0f,
                                             static_cast<float>(params.adc_time_us.value_or(0)) / 1000.0f);
                }
                if (stateLen > 0 && rssi)
                    stateLen = format_append(stateBuf, stateLen - 1, STATE_RSSI_SUFFIX_FMT, *rssi);
                if (stateLen > 0 && hasHeater)
                    stateLen = format_append(stateBuf, stateLen - 1, STATE_HEATER_SUFFIX_FMT,
                                             *params.heater_problem ? "ON" : "OFF",
                                             *params.heater_run_count);
                // us -> ms as float: a quiet sleepy cycle's radio time lands in the hundreds of
                // microseconds, which integer milliseconds would flatten to 0 and destroy exactly
                // the measurement these entities exist to make.
                if (stateLen > 0 && hasRadioStats)
                    stateLen = format_append(stateBuf, stateLen - 1, STATE_RADIO_SUFFIX_FMT,
                                             static_cast<float>(*link->radioTxTimeUs) / 1000.0f,
                                             static_cast<float>(*link->radioRxTimeUs) / 1000.0f);
                if (stateLen > 0 && hasLinkQuality)
                    stateLen = format_append(stateBuf, stateLen - 1, STATE_LINKQ_SUFFIX_FMT,
                                             *link->txRetries, *link->txCcaFailures,
                                             *link->txNoAckExpiry,
                                             static_cast<unsigned>(*link->linkQualityOut));
                if (stateLen > 0 && hasUplinkRssi)
                    stateLen = format_append(stateBuf, stateLen - 1, STATE_UPLINK_SUFFIX_FMT,
                                             *link->uplinkRssiDbm);
                // Boot count and reset reason are boot-constant, so they're re-sent on every state
                // message -- otherwise their entities would go stale-then-unavailable under
                // expire_after while the rest of the device keeps reporting.
                if (stateLen > 0)
                    stateLen = format_append(stateBuf, stateLen - 1, STATE_DIAG_SUFFIX_FMT,
                                             s_cfg.boot_count, s_cfg.reset_reason);
                // TX power in effect right now -- also boot-constant availability (always some
                // value, table max at the very least), so also re-sent every state message.
                if (stateLen > 0)
                    stateLen = format_append(stateBuf, stateLen - 1, STATE_TXPOWER_SUFFIX_FMT,
                                             static_cast<int>(runtime_config_tx_power_active_dbm()));
                // HP awake time -- also boot-constant availability, re-sent every state message.
                if (stateLen > 0)
                    stateLen = format_append(stateBuf, stateLen - 1, STATE_AWAKE_SUFFIX_FMT,
                                             static_cast<float>(hpAwakeUs) / 1000.0f);
                // MQTT connect-wait time -- also boot-constant availability, re-sent every state
                // message (every cycle that reaches here attempted a connect, successful or not).
                if (stateLen > 0)
                    stateLen = format_append(stateBuf, stateLen - 1, STATE_MQTTCONN_SUFFIX_FMT,
                                             static_cast<float>(mqttConnectUs) / 1000.0f);

                std::array<char, TOPIC_BUF> stateTopicBuf;
                const size_t stateTopicLen = format_into(stateTopicBuf, STATE_TOPIC_FMT, dev);

                if (stateLen > 0 && stateTopicLen > 0) {
                    esp_mqtt_client_publish(client, stateTopicBuf.data(), stateBuf.data(),
                                            static_cast<int>(stateLen), 1, 0);
                    ESP_LOGI(TAG, "sent %s", stateBuf.data());

                    // QoS-1 acks over a healthy Thread link return well under a second; cap short
                    // so we stop fast-polling (and sleep) promptly instead of idling the radio.
                    ok = (xEventGroupWaitBits(eg, BIT_ALL_ACKED, pdFALSE, pdTRUE,
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
                replay_history_if_pending(client, dev, ctx, eg);

            // Runtime config changes (see runtime_config.h) are quick -- a handful of small
            // retained publishes, no ACK wait -- so they're serviced here, ahead of OTA, rather
            // than risk being delayed behind a potentially multi-minute OTA session.
            if (ok || !hasAny)
                runtime_config_apply_pending(client);
        } else {
            ESP_LOGE(TAG, "MQTT connection failed, skipping cycle");
        }

        // Never destroyed -- see start_persistent_client()'s doc comment. Unconditional
        // (whether this cycle's connect succeeded or failed): esp-mqtt's internal task, once
        // started, parks itself in a periodically-waking reconnect-wait loop rather than
        // exiting on its own (traced in mqtt_client.c's MQTT_STATE_WAIT_RECONNECT case) --
        // left running between cycles it would repeatedly defeat automatic light sleep.
        if (client)
            esp_mqtt_client_stop(client);
    }

    // A staged update only starts from a healthy cycle: for a data-carrying cycle that means
    // connected AND state ACKed (a flaky link fails fast above instead of kicking off a doomed
    // download); an OTA-only cycle carries nothing to ACK, so !hasAny alone clears the bar.
    // Self-contained (see run_ota_if_due()'s doc comment) -- connects its own dedicated,
    // OTA-sized client regardless of whether the persistent-client branch above ran at all. On
    // success this reboots and never returns.
    if (ok || !hasAny)
        run_ota_if_due(uri, params.battery_percent);

    s_link->onPublishWindowEnd();  // OT: back to slow poll until next sensor cycle; Wi-Fi: no-op
    return ok;
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
                           std::optional<float> battery_percent, std::optional<int> battery_millivolts,
                           std::optional<bool> heater_problem, std::optional<uint32_t> heater_run_count,
                           std::optional<uint32_t> adc_time_us)
{
    if (s_task_running.exchange(true)) {
        ESP_LOGW(TAG, "previous publish cycle still running, skipping");
        return;
    }

    if (s_idle_eg)
    	xEventGroupClearBits(s_idle_eg, BIT_IDLE);  // mark busy until the task exits

    std::unique_ptr<PublishParams> params(new PublishParams{temperature, humidity,
                                                            battery_percent, battery_millivolts,
                                                            heater_problem, heater_run_count,
                                                            adc_time_us});
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
