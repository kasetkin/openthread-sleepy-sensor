#include "mqtt_sender.h"

#include <algorithm>
#include <atomic>
#include <format>
#include <string>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "mqtt_client.h"

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

static MqttConfig s_cfg;
static const NetworkLink *s_link = nullptr;       // set in mqtt_sender_init(); backs the transport
static std::atomic<bool> s_discovery_sent{false};
static std::atomic<bool> s_task_running{false};
static std::atomic<bool> s_last_ok{false};        // true iff the most recent finished cycle connected AND was ACKed
static EventGroupHandle_t s_idle_eg = nullptr;    // created in mqtt_sender_init(); starts idle

// ── context shared between the publish task and the event handler ─────────────
struct MqttCtx
{
    EventGroupHandle_t eg;
    std::atomic<int>   expected_acks{0};
    std::atomic<int>   received_acks{0};
};

// HA MQTT-discovery config payload. Shown un-escaped for readability; in the format
// string every literal { } is doubled, and Jinja "{{ value_json.X }}" -> "{{{{value_json.{}}}}}".
//   {"name":"<>","device_class":"<>","state_topic":"<id>/state",
//    "value_template":"{{value_json.<key>}}","unit_of_measurement":"<>",
//    "unique_id":"<id>_<key>","device":{"identifiers":["<id>"],"name":"<name>"}}
static constexpr std::string_view DISCOVERY_FMT =
    "{{"
    "\"name\":\"{}\","
    "\"device_class\":\"{}\","
    "\"state_topic\":\"{}/state\","
    "\"value_template\":\"{{{{value_json.{}}}}}\","
    "\"unit_of_measurement\":\"{}\","
    "\"unique_id\":\"{}_{}\","
    "\"device\":{{\"identifiers\":[\"{}\"],\"name\":\"{}\"}}"
    "}}";

static std::string discovery_payload(const char *name, const char *device_class,
                                     const char *unit, const char *value_key,
                                     std::string_view device_id, std::string_view device_name)
{
    return std::format(DISCOVERY_FMT,
        name, device_class,
        device_id, value_key,
        unit,
        device_id, value_key,
        device_id, device_name);
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
    case MQTT_EVENT_ERROR: {
        const auto *err = static_cast<esp_mqtt_event_handle_t>(event_data)->error_handle;
        if (err) {
            ESP_LOGE(TAG, "MQTT error: type=%d esp_tls=%d errno=%d",
                     err->error_type, err->esp_tls_last_esp_err, err->esp_transport_sock_errno);
        }
        xEventGroupSetBits(ctx->eg, BIT_ERROR);
        break;
    }
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
    cfg.session.keepalive        = 10;

    cfg.network.timeout_ms            = 3000;   // bound TCP ops so stop() returns quickly on failure
    cfg.network.disable_auto_reconnect = true;  // we manage reconnects ourselves (one task per cycle)

    ESP_LOGI(TAG, "connecting to %s", uri);
    xEventGroupClearBits(ctx.eg, BIT_CONNECTED | BIT_ALL_ACKED | BIT_ERROR);
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(client, MQTT_EVENT_ANY, mqtt_event_handler, &ctx);
    esp_mqtt_client_start(client);
    return client;
}

// ── publish task — owns the client lifecycle ──────────────────────────────────
struct PublishParams
{
	std::optional<float> temperature;
	std::optional<float> humidity;
};

static void mqtt_publish_task(void *arg)
{
    auto *params = static_cast<PublishParams *>(arg);
    const bool hasTemp = params->temperature.has_value();
    const bool hasHumid = params->humidity.has_value();
    const float temp = params->temperature.value_or(0.0);
    const float hum  = params->humidity.value_or(0.0);
    delete params;

    MqttCtx ctx;
    ctx.eg = xEventGroupCreate();

    // Helper for the early-exit paths: mark this cycle failed, release fast poll, free state,
    // mark idle, end task.  s_last_ok must be stored before BIT_IDLE so the waiter sees it.
    auto abort_cycle = [&ctx]() {
        s_last_ok.store(false);
        vEventGroupDelete(ctx.eg);
        s_link->onPublishWindowEnd();
        s_task_running.store(false);
        if (s_idle_eg)
            xEventGroupSetBits(s_idle_eg, BIT_IDLE);
        vTaskDelete(nullptr);
    };

    s_link->onPublishWindowBegin();  // OT: fast polls so NAT64 prefix + TCP ACKs arrive promptly; Wi-Fi: no-op

    // Block until the broker's address family is reachable (OT: NAT64 prefix learned,
    // only for an IPv4 broker; Wi-Fi: always immediate — see NetworkLink::waitForBrokerReachable).
    if (!s_link->waitForBrokerReachable(s_cfg.broker_address, BROKER_REACHABLE_WAIT_MS)) {
        ESP_LOGE(TAG, "broker not reachable within %lu ms, skipping cycle",
                 (unsigned long)BROKER_REACHABLE_WAIT_MS);
        abort_cycle();
        return;
    }

    const std::string uri = s_link->brokerUri(s_cfg.broker_address, s_cfg.port);
    if (uri.empty()) {
        ESP_LOGE(TAG, "broker_address not set or invalid, cannot connect");
        abort_cycle();
        return;
    }

    esp_mqtt_client_handle_t client = start_client(uri.c_str(), ctx);
    EventBits_t bits = xEventGroupWaitBits(ctx.eg, BIT_CONNECTED | BIT_ERROR,
                                           pdFALSE, pdFALSE, pdMS_TO_TICKS(5000));

    // ── publish if connected ──────────────────────────────────────────────────
    // ok stays false unless we connect, publish a state message, AND the broker ACKs it.
    // This is the signal the sensor task uses for the LED and the reboot supervisor, so it
    // must mean "data actually reached the broker", not merely "the task ran".
    bool ok = false;
    if (bits & BIT_CONNECTED) {
        const std::string_view dev = s_cfg.device_id;
        const std::string_view dev_name = s_cfg.device_name;
        const bool hasAny = hasTemp || hasHumid;
        const bool need_discovery = !s_discovery_sent.load() && hasAny;

        // Expected ACKs must match what we actually publish below: one state message plus one
        // discovery message per present value.  A fixed "+2" assumes both T and H discovery are
        // sent; with humidity disabled only T is sent, so BIT_ALL_ACKED would never set and the
        // first cycle would be wrongly counted as a failure.
        const int discovery_msgs = need_discovery ? ((hasTemp ? 1 : 0) + (hasHumid ? 1 : 0)) : 0;
        const int expected = (hasAny ? 1 : 0) + discovery_msgs;

        // Set counters BEFORE publishing so the handler never races ahead
        ctx.expected_acks.store(expected);
        ctx.received_acks.store(0);
        xEventGroupClearBits(ctx.eg, BIT_ALL_ACKED);

        if (need_discovery) {
            if (hasTemp) {
                const std::string t_topic   = "homeassistant/sensor/" + std::string(dev) + "/temperature/config";
                const std::string t_payload = discovery_payload("Temperature", "temperature", "°C", "t", dev, dev_name);
                esp_mqtt_client_publish(client, t_topic.c_str(), t_payload.c_str(), 0, 1, 1);
            }

            if (hasHumid) {
                const std::string h_topic   = "homeassistant/sensor/" + std::string(dev) + "/humidity/config";
                const std::string h_payload = discovery_payload("Humidity", "humidity", "%", "h", dev, dev_name);
                esp_mqtt_client_publish(client, h_topic.c_str(), h_payload.c_str(), 0, 1, 1);
            }
        }

        if (hasAny) {
            std::string state;
            if (hasTemp && hasHumid)
                state = std::format("{{\"t\":{:.3g},\"h\":{:.3g}}}", temp, hum);
            else if (hasTemp)
                state = std::format("{{\"t\":{:.3g}}}", temp);
            else  // hasHumid
                state = std::format("{{\"h\":{:.3g}}}", hum);

            const std::string state_topic = std::string(dev) + "/state";
            esp_mqtt_client_publish(client, state_topic.c_str(), state.c_str(), 0, 1, 0);
            ESP_LOGI(TAG, "sent %s", state.c_str());

            // QoS-1 acks over a healthy Thread link return well under a second; cap short
            // so we stop fast-polling (and sleep) promptly instead of idling the radio.
            ok = (xEventGroupWaitBits(ctx.eg, BIT_ALL_ACKED, pdFALSE, pdTRUE,
                                      pdMS_TO_TICKS(4000)) & BIT_ALL_ACKED) != 0;
            s_discovery_sent.store(true);
        } else {
            ESP_LOGW(TAG, "no sensor values to publish this cycle");
        }
    } else {
        ESP_LOGE(TAG, "MQTT connection failed, skipping cycle");
    }
    s_last_ok.store(ok);  // published before BIT_IDLE is set in the cleanup below

    // ── clean up — safe here because we are NOT in the MQTT event handler ─────
    esp_mqtt_client_stop(client);
    esp_mqtt_client_destroy(client);
    vEventGroupDelete(ctx.eg);
    s_link->onPublishWindowEnd();  // OT: back to slow poll until next sensor cycle; Wi-Fi: no-op
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
    if (!s_idle_eg) {
        s_idle_eg = xEventGroupCreate();
        if (s_idle_eg)
        	xEventGroupSetBits(s_idle_eg, BIT_IDLE);  // idle until first publish
    }
}

void mqtt_send_sensor_data(std::optional<float> temperature, std::optional<float> humidity)
{
    if (s_task_running.exchange(true)) {
        ESP_LOGW(TAG, "previous publish cycle still running, skipping");
        return;
    }
    
    if (s_idle_eg)
    	xEventGroupClearBits(s_idle_eg, BIT_IDLE);  // mark busy until the task exits
    	
    auto *params = new PublishParams{temperature, humidity};
    if (xTaskCreate(mqtt_publish_task, "mqtt_pub", 12288, params, 5, nullptr) != pdPASS) {
        ESP_LOGE(TAG, "failed to create mqtt_pub task");
        delete params;
        s_task_running.store(false);
        if (s_idle_eg)
        	xEventGroupSetBits(s_idle_eg, BIT_IDLE);
    }
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
