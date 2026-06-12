#include "mqtt_sender.h"

#include <atomic>
#include <cstdio>
#include <cstring>
#include <string>

#include "esp_log.h"
#include "esp_openthread.h"
#include "esp_openthread_lock.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "mqtt_client.h"
#include "openthread/link.h"

static const char *TAG = "mqtt-sender";

static constexpr EventBits_t BIT_CONNECTED = BIT0;
static constexpr EventBits_t BIT_ALL_ACKED = BIT1;
static constexpr EventBits_t BIT_ERROR     = BIT2;

static MqttConfig s_cfg;
static std::atomic<bool> s_discovery_sent{false};
static std::atomic<bool> s_task_running{false};

// NAT64 /96 prefix used to reach the IPv4 broker.  Default: IANA well-known 64:ff9b::/96.
// Overridden at runtime via mqtt_sender_set_nat64_prefix() once Thread network data arrives.
static uint8_t s_nat64_prefix[12] = {
    0x00, 0x64, 0xff, 0x9b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// ── context shared between the publish task and the event handler ─────────────
struct MqttCtx {
    EventGroupHandle_t eg;
    std::atomic<int>   expected_acks{0};
    std::atomic<int>   received_acks{0};
};

// Poll periods: fast during the MQTT window so TCP ACKs arrive promptly;
// slow the rest of the time to maximise sleep.
static constexpr uint32_t POLL_FAST_MS = 500;
static constexpr uint32_t POLL_SLOW_MS = 70000;

static void set_poll_period(uint32_t ms) {
    esp_openthread_lock_acquire(portMAX_DELAY);
    otLinkSetPollPeriod(esp_openthread_get_instance(), ms);
    esp_openthread_lock_release();
}

// ── helpers ───────────────────────────────────────────────────────────────────

void mqtt_sender_set_nat64_prefix(const uint8_t *p12) {
    memcpy(s_nat64_prefix, p12, 12);
}

static std::string make_nat64_uri(const std::string &ipv4, uint16_t port) {
    unsigned a, b, c, d;
    if (sscanf(ipv4.c_str(), "%u.%u.%u.%u", &a, &b, &c, &d) != 4) {
        ESP_LOGE(TAG, "bad IPv4 '%s'", ipv4.c_str());
        return {};
    }
    const uint8_t *p = s_nat64_prefix;
    char uri[80];
    // Build full 128-bit IPv6 from 96-bit NAT64 prefix + 32-bit IPv4
    snprintf(uri, sizeof(uri),
             "mqtt://[%x:%x:%x:%x:%x:%x:%02x%02x:%02x%02x]:%u",
             (unsigned)((p[0]<<8)|p[1]),  (unsigned)((p[2]<<8)|p[3]),
             (unsigned)((p[4]<<8)|p[5]),  (unsigned)((p[6]<<8)|p[7]),
             (unsigned)((p[8]<<8)|p[9]),  (unsigned)((p[10]<<8)|p[11]),
             a, b, c, d, port);
    return uri;
}

static std::string discovery_payload(const char *name, const char *device_class,
                                     const char *unit, const char *value_key,
                                     std::string_view device_id) {
    char buf[512];
    snprintf(buf, sizeof(buf),
        "{"
        "\"name\":\"%s\","
        "\"device_class\":\"%s\","
        "\"state_topic\":\"%.*s/state\","
        "\"value_template\":\"{{value_json.%s}}\","
        "\"unit_of_measurement\":\"%s\","
        "\"unique_id\":\"%.*s_%s\","
        "\"device\":{\"identifiers\":[\"%.*s\"],\"name\":\"Sleepy Sensor\"}"
        "}",
        name, device_class,
        static_cast<int>(device_id.size()), device_id.data(), value_key,
        unit,
        static_cast<int>(device_id.size()), device_id.data(), value_key,
        static_cast<int>(device_id.size()), device_id.data());
    return buf;
}

// ── event handler — ONLY sets event group bits, never touches the client ──────
static void mqtt_event_handler(void *handler_arg, esp_event_base_t /*base*/,
                                int32_t event_id, void *event_data) {
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
static esp_mqtt_client_handle_t start_client(const char *uri, MqttCtx &ctx) {
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
struct PublishParams { float temperature; float humidity; };

static void mqtt_publish_task(void *arg) {
    auto *params = static_cast<PublishParams *>(arg);
    const float temp = params->temperature;
    const float hum  = params->humidity;
    delete params;

    MqttCtx ctx;
    ctx.eg = xEventGroupCreate();

    const std::string uri = make_nat64_uri(s_cfg.ha_ipv4, s_cfg.port);
    if (uri.empty()) {
        ESP_LOGE(TAG, "ha_ipv4 not set or invalid, cannot connect");
        vEventGroupDelete(ctx.eg);
        s_task_running.store(false);
        vTaskDelete(nullptr);
        return;
    }

    set_poll_period(POLL_FAST_MS);  // need fast polls so TCP ACKs arrive promptly

    esp_mqtt_client_handle_t client = start_client(uri.c_str(), ctx);
    EventBits_t bits = xEventGroupWaitBits(ctx.eg, BIT_CONNECTED | BIT_ERROR,
                                           pdFALSE, pdFALSE, pdMS_TO_TICKS(5000));

    // ── publish if connected ──────────────────────────────────────────────────
    if (bits & BIT_CONNECTED) {
        const std::string_view dev = s_cfg.device_id;
        const bool need_discovery = !s_discovery_sent.load();
        const int expected = 1 + (need_discovery ? 2 : 0);

        // Set counters BEFORE publishing so the handler never races ahead
        ctx.expected_acks.store(expected);
        ctx.received_acks.store(0);
        xEventGroupClearBits(ctx.eg, BIT_ALL_ACKED);

        if (need_discovery) {
            const std::string t_topic   = "homeassistant/sensor/" + std::string(dev) + "/temperature/config";
            const std::string t_payload = discovery_payload("Temperature", "temperature", "°C", "t", dev);
            esp_mqtt_client_publish(client, t_topic.c_str(), t_payload.c_str(), 0, 1, 1);

            const std::string h_topic   = "homeassistant/sensor/" + std::string(dev) + "/humidity/config";
            const std::string h_payload = discovery_payload("Humidity", "humidity", "%", "h", dev);
            esp_mqtt_client_publish(client, h_topic.c_str(), h_payload.c_str(), 0, 1, 1);
        }

        char state[64];
        snprintf(state, sizeof(state), "{\"t\":%.3g,\"h\":%.3g}", temp, hum);
        const std::string state_topic = std::string(dev) + "/state";
        esp_mqtt_client_publish(client, state_topic.c_str(), state, 0, 1, 0);
        ESP_LOGI(TAG, "sent %s", state);

        xEventGroupWaitBits(ctx.eg, BIT_ALL_ACKED, pdFALSE, pdTRUE, pdMS_TO_TICKS(10000));
        s_discovery_sent.store(true);
    } else {
        ESP_LOGE(TAG, "MQTT connection failed, skipping cycle");
    }

    // ── clean up — safe here because we are NOT in the MQTT event handler ─────
    esp_mqtt_client_stop(client);
    esp_mqtt_client_destroy(client);
    vEventGroupDelete(ctx.eg);
    set_poll_period(POLL_SLOW_MS);  // back to slow poll until next sensor cycle
    s_task_running.store(false);
    vTaskDelete(nullptr);
}

// ── public API ────────────────────────────────────────────────────────────────

void mqtt_sender_init(const MqttConfig &cfg) {
    s_cfg = cfg;
}

void mqtt_send_sensor_data(float temperature, float humidity) {
    if (s_task_running.exchange(true)) {
        ESP_LOGW(TAG, "previous publish cycle still running, skipping");
        return;
    }
    auto *params = new PublishParams{temperature, humidity};
    if (xTaskCreate(mqtt_publish_task, "mqtt_pub", 12288, params, 5, nullptr) != pdPASS) {
        ESP_LOGE(TAG, "failed to create mqtt_pub task");
        delete params;
        s_task_running.store(false);
    }
}
