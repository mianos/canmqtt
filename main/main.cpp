// mqttcan — passive CAN-bus monitor on a Waveshare ESP32-S3-RS485-CAN board.
//
// Built to watch the CAN bus of a BMW R1200GS (K25) without touching it. The
// on-chip TWAI controller runs in hardware listen-only mode: it never transmits
// and never acknowledges, so it cannot perturb the vehicle. (This is not
// paranoia — a published sniffer project's nominally "listen-only" rig
// intermittently transmitted and toggled the bike's rear brake light.)
//
// The K25's CAN IDs are not reliably documented, so this is a discovery
// instrument first: it maintains a live per-ID table (payload, repeat rate, and
// a mask of which bytes have ever changed) and publishes a frame to MQTT only
// when its payload *changes*. Decoding named signals comes later, once IDs are
// confirmed against the actual bike.
//
// Shares its infrastructure components (wifimanager, mqttwrapper, settingsbase,
// webserver, jsonwrapper) with mqttradar / ws-voice / doorbell3.
//
// Wiring — Waveshare ESP32-S3-RS485-CAN to the BMW alarm pre-wire connector
// (part 83300413581):
//   board CAN H   <- connector pin 6 (CAN High)
//   board CAN L   <- connector pin 5 (CAN Low)
//   board V+/V-   <- connector pin 3 (switched 12V) / pin 4 (ground)
// The board takes 7-36V on the screw terminal, and its CAN side is isolated.
// IMPORTANT: leave the board's 120R jumper on NC. The bike's bus is already
// terminated at both ends (~240R measured across the connector = two 120R in
// parallel); adding a third drops the bus to ~80R and you get the classic
// "transceiver pins wiggle but nothing decodes" symptom.
//
// MQTT (cmnd/<name>/...):
//   settings    any subset of the /config JSON (e.g. {"can_bitrate":125000})
//   canreset    {}  clear the frame table + dump ring (baseline before an action)
//   restart     {}
//   reprovision {}  clears Wi-Fi creds, reboots into ESP-Touch v2 provisioning
// Publishes:
//   tele/<name>/frame   {"id":"0x2BC","ext":...,"dlc":8,"data":"...","chg":"0C",...}
//                       on each accepted payload change
//   tele/<name>/stats   bus health + rates, 1/min — the bit-rate diagnostic
//   tele/<name>/init,status   identity + telemetry
// HTTP: /healthz /config /firmware /can/ids /can/dump /can/status /can/reset

#include <cinttypes>
#include <cstring>
#include <cstdlib>
#include <ctime>
#include <regex>
#include <string>

#include "esp_app_desc.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "JsonWrapper.h"
#include "MqttClient.h"
#include "NvsStorageManager.h"
#include "Settings.h"
#include "WebServer.h"
#include "WifiManager.h"

#include "CanBus.h"
#include "CanWebServer.h"
#include "FrameTable.h"

static const char* TAG = "mqttcan";

// CAN transceiver on the Waveshare ESP32-S3-RS485-CAN. From the vendor's
// WS_GPIO.h demo header: TXD2=15, RXD2=16. Do not copy these from other
// Waveshare ESP32-S3 boards — the Touch-LCD-7 uses 20/19 and the Touch-LCD-4
// uses 6/0 for the same peripheral.
static constexpr gpio_num_t kCanTx = GPIO_NUM_15;
static constexpr gpio_num_t kCanRx = GPIO_NUM_16;

namespace {

struct App {
    Settings*    settings;
    MqttClient*  mqtt;
    WiFiManager* wifi;
    CanBus*      bus   = nullptr;  // set once constructed, later in app_main
    FrameTable*  table = nullptr;
};

std::string uptimeString() {
    uint32_t seconds = (uint32_t)(esp_timer_get_time() / 1000000ULL);
    uint32_t days = seconds / 86400; seconds %= 86400;
    uint32_t hours = seconds / 3600; seconds %= 3600;
    uint32_t minutes = seconds / 60;
    return std::to_string(days) + "d " + std::to_string(hours) + "h " +
           std::to_string(minutes) + "m";
}

std::string localIp() {
    char buf[16] = "0.0.0.0";
    esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    esp_netif_ip_info_t ip;
    if (netif && esp_netif_get_ip_info(netif, &ip) == ESP_OK) {
        esp_ip4addr_ntoa(&ip.ip, buf, sizeof(buf));
    }
    return std::string(buf);
}

// ---- MQTT command handlers (cmnd/<name>/<cmd>) ----

// SettingsBase::loadFromJson applies whichever registered fields are present
// and fires their onChange hooks (which is how can_bitrate retunes the live
// controller); it returns void, so the ack is just the resulting settings.
esp_err_t handleSettings(MqttClient* client, const std::string&,
                         const JsonWrapper& d, void* ctx) {
    auto* app = static_cast<App*>(ctx);
    app->settings->loadFromJson(d);
    app->settings->save();
    app->settings->log();
    client->publish("tele/" + app->settings->sensorName + "/settingsack",
                    app->settings->toJson().ToString());
    return ESP_OK;
}

esp_err_t handleCanReset(MqttClient*, const std::string&, const JsonWrapper&, void* ctx) {
    auto* app = static_cast<App*>(ctx);
    if (app->table) app->table->reset();
    return ESP_OK;
}

esp_err_t handleRestart(MqttClient*, const std::string&, const JsonWrapper&, void*) {
    ESP_LOGW(TAG, "restart requested");
    esp_restart();
    return ESP_OK;
}

esp_err_t handleReprovision(MqttClient*, const std::string&, const JsonWrapper&, void* ctx) {
    ESP_LOGW(TAG, "reprovision requested");
    static_cast<App*>(ctx)->wifi->clear();  // clears Wi-Fi creds and restarts
    return ESP_OK;
}

// --- OTA rollback verification (see mqttradar/ws-voice for the rationale) ---
constexpr int OTA_VERIFY_TIMEOUT_MS = 120000;
SemaphoreHandle_t s_got_ip = nullptr;

void onGotIp(void*, esp_event_base_t base, int32_t id, void*) {
    if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP && s_got_ip) {
        xSemaphoreGive(s_got_ip);
    }
}

// Wi-Fi association diagnostics. WiFiManager's own disconnect handler just
// re-calls esp_wifi_connect() without logging, so a device that never joins
// looks completely silent on the console — which is useless when the thing is
// bolted to a motorcycle. Log the reason code for every disconnect, and the
// SSID the driver is actually trying, so a failure names itself.
void onWifiEvent(void*, esp_event_base_t base, int32_t id, void* data) {
    if (base != WIFI_EVENT) return;
    if (id == WIFI_EVENT_STA_DISCONNECTED) {
        auto* e = static_cast<wifi_event_sta_disconnected_t*>(data);
        // Common ones: 201 NO_AP_FOUND (wrong SSID, out of range, or 5GHz-only),
        // 202 AUTH_FAIL / 15 4WAY_HANDSHAKE_TIMEOUT (wrong password),
        // 205 CONNECTION_FAIL, 3 AUTH_LEAVE.
        ESP_LOGW(TAG, "wifi disconnected from '%.*s': reason %d",
                 e->ssid_len, (const char*)e->ssid, e->reason);
    } else if (id == WIFI_EVENT_STA_CONNECTED) {
        auto* e = static_cast<wifi_event_sta_connected_t*>(data);
        ESP_LOGI(TAG, "wifi associated with '%.*s' on channel %d",
                 e->ssid_len, (const char*)e->ssid, e->channel);
    }
}

void otaVerifyTask(void*) {
    const esp_partition_t* running = esp_ota_get_running_partition();
    esp_ota_img_states_t state;
    if (esp_ota_get_state_partition(running, &state) == ESP_OK &&
        state == ESP_OTA_IMG_PENDING_VERIFY) {
        ESP_LOGW(TAG, "OTA: image pending verify; waiting up to %ds for connectivity",
                 OTA_VERIFY_TIMEOUT_MS / 1000);
        if (xSemaphoreTake(s_got_ip, pdMS_TO_TICKS(OTA_VERIFY_TIMEOUT_MS)) == pdTRUE) {
            esp_ota_mark_app_valid_cancel_rollback();
            ESP_LOGI(TAG, "OTA: connectivity confirmed, image marked valid");
        } else {
            ESP_LOGE(TAG, "OTA: no IP within timeout; rolling back to previous image");
            esp_ota_mark_app_invalid_rollback_and_reboot();  // reboots on success
            ESP_LOGE(TAG, "OTA: rollback not possible; keeping current image");
            esp_ota_mark_app_valid_cancel_rollback();
        }
    }
    vTaskDelete(nullptr);
}

// Bus health once a minute. This is the diagnostic that tells you whether the
// configured bit rate is right: frames_rx climbing with bus_errors flat means
// yes; zero frames with bus_errors climbing means no (try 125000 next).
void statsTask(void* arg) {
    auto* app = static_cast<App*>(arg);
    const std::string base = "tele/" + app->settings->sensorName + "/";
    uint32_t lastFrames = 0;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(60000));
        const CanBusCounters& c = app->bus->counters();

        JsonWrapper d;
        d.AddTime();
        d.AddItem("bitrate",     (int)app->bus->bitrate());
        d.AddItem("listen_only", app->bus->listenOnly());
        d.AddItem("frames_rx",   (int)c.framesRx);
        d.AddItem("fps",         (int)((c.framesRx - lastFrames) / 60));
        d.AddItem("drops",       (int)c.drops);
        d.AddItem("rx_errors",   (int)c.rxErrors);
        d.AddItem("tracked_ids", (int)app->table->trackedIds());
        d.AddItem("overflow",    (int)app->table->overflowIds());

        twai_node_status_t st;
        twai_node_record_t rec;
        if (app->bus->info(st, rec) == ESP_OK) {
            d.AddItem("state",      std::string(canErrorStateName(st.state)));
            d.AddItem("bus_errors", (int)rec.bus_err_num);
            d.AddItem("rx_error_count", (int)st.rx_error_count);
        }
        app->mqtt->publish(base + "stats", d.ToString());
        ESP_LOGI(TAG, "bus: %" PRIu32 " frames (%" PRIu32 "/s), %u ids, %" PRIu32 " drops, "
                      "%" PRIu32 " rx_err @ %" PRIu32 " bps",
                 c.framesRx, (c.framesRx - lastFrames) / 60,
                 (unsigned)app->table->trackedIds(), c.drops, c.rxErrors, app->bus->bitrate());

        if (c.framesRx == lastFrames) {
            ESP_LOGW(TAG, "no CAN frames in the last 60s at %" PRIu32 " bps "
                          "- wrong bit rate, or the bus is asleep",
                     app->bus->bitrate());
        }
        lastFrames = c.framesRx;
    }
}

void telemetryTask(void* arg) {
    auto* app = static_cast<App*>(arg);
    const std::string base = "tele/" + app->settings->sensorName + "/";

    // Wait (bounded) for SNTP so the init timestamp is real.
    for (int i = 0; i < 20 && time(nullptr) < 1700000000; ++i) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    // RTC time survives a soft restart, so the SNTP wait passes before Wi-Fi
    // has re-associated; wait (bounded) for an IP too or init reports 0.0.0.0.
    for (int i = 0; i < 40 && localIp() == "0.0.0.0"; ++i) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    const esp_app_desc_t* desc = esp_app_get_description();
    JsonWrapper init;
    init.AddItem("build", std::string(desc->version));
    init.AddItem("built", std::string(desc->date) + " " + std::string(desc->time));
    init.AddTime();
    init.AddItem("hostname", app->settings->sensorName);
    init.AddItem("ip", localIp());
    init.AddItem("bitrate", (int)app->bus->bitrate());
    init.AddItem("listen_only", app->bus->listenOnly());
    init.AddItem("settings", "cmnd/" + app->settings->sensorName + "/settings");
    app->mqtt->publish(base + "init", init.ToString());

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(60000));
        JsonWrapper d;
        d.AddTime();
        d.AddItem("uptime", uptimeString());
        d.AddItem("heap_free", (int)esp_get_free_heap_size());
        d.AddItem("heap_min_free", (int)esp_get_minimum_free_heap_size());
        app->mqtt->publish(base + "status", d.ToString());
    }
}

// Bench only: transmit a few synthetic IDs with slowly-changing payloads so the
// whole ISR -> queue -> table -> change-detect -> MQTT path can be exercised
// with no bus attached. Requires can_listen_only=0 (a listen-only node
// physically cannot transmit), which is refused on a vehicle by policy.
void selfTestTask(void* arg) {
    auto* app = static_cast<App*>(arg);
    ESP_LOGW(TAG, "self-test injector running - synthetic frames, NOT bus traffic");
    uint8_t counter = 0;
    for (;;) {
        // 0x123: a byte that ticks every cycle (should publish every time).
        uint8_t a[8] = {0x01, counter, 0xAA, 0x00, 0x00, 0x00, 0x00, 0x00};
        app->bus->transmit(0x123, false, a, sizeof(a));

        // 0x2BC: constant (should publish once, then only on heartbeat) — the
        // control that proves change-detection is actually suppressing repeats.
        uint8_t b[8] = {0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x00, 0x00, 0x00};
        app->bus->transmit(0x2BC, false, b, sizeof(b));

        // An extended-ID frame: the K25 prior art suggests 29-bit IDs, so make
        // sure that path is exercised too.
        uint8_t c[4] = {counter, 0x00, 0x5A, 0xA5};
        app->bus->transmit(0x18DAF110, true, c, sizeof(c));

        counter++;
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

}  // namespace

extern "C" void app_main(void) {
    esp_log_level_set("*", ESP_LOG_WARN);
    esp_log_level_set("WiFiManager", ESP_LOG_INFO);
    esp_log_level_set("wifi", ESP_LOG_INFO);
    esp_log_level_set("esp_netif_handlers", ESP_LOG_INFO);  // prints "sta ip: ..."
    esp_log_level_set("MqttClient", ESP_LOG_INFO);
    esp_log_level_set("mqttcan", ESP_LOG_INFO);
    esp_log_level_set("canbus", ESP_LOG_INFO);
    esp_log_level_set("frametable", ESP_LOG_INFO);
    esp_log_level_set("settings", ESP_LOG_INFO);

    static NvsStorageManager nvs;      // constructing this initialises NVS flash
    static Settings settings(nvs);
    settings.log();

    // Created before Wi-Fi starts so the got-IP handler can signal it.
    s_got_ip = xSemaphoreCreateBinary();

    // Wi-Fi: ESP-Touch v2 provisioning on first boot, else reconnect. onGotIp
    // feeds OTA rollback verification; publishes queue until MQTT connects.
    static WiFiManager wifi(nvs, onGotIp, nullptr);
    std::string host = settings.sensorName;
    wifi.configSetHostName(host);

    // Regulatory domain. The IDF default "01" caps scanning at channel 11, so an
    // AP on ch12/13 is simply never found (reason 201) however correct the
    // credentials are. Must be set after the Wi-Fi stack is up.
    {
        esp_err_t cr = esp_wifi_set_country_code(settings.wifiCountry.c_str(), true);
        ESP_LOGI(TAG, "wifi country '%s': %s", settings.wifiCountry.c_str(),
                 esp_err_to_name(cr));
    }

    // Association diagnostics (see onWifiEvent). Registered after WiFiManager
    // has initialised the Wi-Fi stack and event loop.
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, onWifiEvent, nullptr);

    // Report what the driver actually loaded from NVS. An empty SSID here means
    // esp_wifi_connect() will fail instantly and silently, which otherwise
    // presents as "provisioned but nothing ever happens".
    {
        wifi_config_t cur = {};
        if (esp_wifi_get_config(WIFI_IF_STA, &cur) == ESP_OK) {
            ESP_LOGI(TAG, "wifi sta config: ssid='%s' (%d chars), password %s",
                     (const char*)cur.sta.ssid,
                     (int)strlen((const char*)cur.sta.ssid),
                     cur.sta.password[0] ? "set" : "EMPTY");
        }
    }

    // Keep the radio awake, as ws-voice does. Power-save doze between DTIM
    // beacons adds latency and, on some APs, makes association flaky.
    esp_wifi_set_ps(WIFI_PS_NONE);

    setenv("TZ", settings.tz.c_str(), 1);
    tzset();
    esp_sntp_config_t sntp_cfg = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    esp_netif_sntp_init(&sntp_cfg);

    // MQTT (anonymous, plain mqtt://, Last-Will on the status topic).
    static std::string uri = "mqtt://" + settings.mqttServer + ":" + std::to_string(settings.mqttPort);
    static std::string statusTopic = "tele/" + settings.sensorName + "/status";
    static std::string lwt = "{\"status\":\"offline\"}";
    static esp_mqtt_client_config_t mcfg = {};
    mcfg.broker.address.uri        = uri.c_str();
    mcfg.credentials.client_id     = settings.sensorName.c_str();
    mcfg.session.last_will.topic   = statusTopic.c_str();
    mcfg.session.last_will.msg     = lwt.c_str();
    mcfg.session.last_will.msg_len = (int)lwt.size();
    mcfg.session.last_will.qos     = 1;
    static MqttClient mqtt(mcfg, settings.sensorName);

    static App app{ &settings, &mqtt, &wifi };

    const std::string b = "cmnd/" + settings.sensorName + "/";
    mqtt.registerHandler(b + "settings",    std::regex(b + "settings"),    handleSettings,    &app);
    mqtt.registerHandler(b + "canreset",    std::regex(b + "canreset"),    handleCanReset,    &app);
    mqtt.registerHandler(b + "restart",     std::regex(b + "restart"),     handleRestart,     &app);
    mqtt.registerHandler(b + "reprovision", std::regex(b + "reprovision"), handleReprovision, &app);
    mqtt.start();

    // --- CAN ---
    static FrameTable table(settings.maxTrackedIds, settings.dumpRingFrames);
    app.table = &table;

    static CanBus bus(kCanTx, kCanRx);
    app.bus = &bus;

    const std::string frameTopic = "tele/" + settings.sensorName + "/frame";

    // Runs on the CanBus worker task, once per received frame. Publishing is
    // gated by FrameTable::observe() so a 500kbit/s firehose becomes a trickle
    // of actual state changes.
    auto onFrame = [frameTopic](const CanFrame& f) {
        uint8_t chg = 0;
        const bool publish = table.observe(f, settings.publishMinMs,
                                           settings.publishHeartbeatMs, &chg);
        if (!publish) return;

        if (settings.logFrames) {
            ESP_LOGI(TAG, "%s %s#%s chg=%02X", f.ext ? "ext" : "std",
                     canIdHex(f.id, f.ext).c_str(), toHex(f.data, f.len).c_str(), chg);
        }
        if (!settings.publishEnable) return;

        JsonWrapper d;
        d.AddItem("id",   "0x" + canIdHex(f.id, f.ext));
        d.AddItem("ext",  f.ext);
        d.AddItem("dlc",  (int)f.len);
        d.AddItem("data", toHex(f.data, f.len));
        d.AddItem("chg",  toHex(&chg, 1));
        d.AddItem("t_us", (int)(f.timestampUs / 1000ULL));
        mqtt.publish(frameTopic, d.ToString());
    };

    const bool wantSelfTest = settings.selfTest != 0;
    if (wantSelfTest && settings.canListenOnly) {
        // Fail loudly rather than silently dropping listen-only: the operator
        // asked for two mutually exclusive things and must not be left guessing
        // which one they got.
        ESP_LOGE(TAG, "self_test=1 needs can_listen_only=0; keeping listen-only and "
                      "skipping the injector");
    }
    const bool selfTest = wantSelfTest && !settings.canListenOnly;

    // RX queue depth: 256 frames of ISR-to-task slack (~6KB). At 500kbit/s a
    // burst can hit ~4000 frames/sec, so this covers ~60ms of worst case —
    // ample, given the worker task only touches RAM. tele/.../stats "drops"
    // reports it if this is ever wrong.
    esp_err_t canErr = bus.start(settings.canBitrate, settings.canListenOnly != 0,
                                 256, onFrame, selfTest);
    if (canErr != ESP_OK) {
        ESP_LOGE(TAG, "CAN bring-up failed: %s - continuing so Wi-Fi/OTA stay reachable "
                      "and the config is fixable remotely", esp_err_to_name(canErr));
    }

    // can_bitrate applies live: disable -> retime -> enable, no reboot. This is
    // deliberate, because 500000 is an educated guess for the K25 rather than a
    // confirmed spec — you want to retune from the saddle, not the workbench.
    settings.onChange("can_bitrate", [] {
        if (settings.canBitrate < 10000 || settings.canBitrate > 1000000) {
            ESP_LOGW(TAG, "can_bitrate %d out of range [10000,1000000]; ignoring",
                     settings.canBitrate);
            return;
        }
        esp_err_t r = bus.setBitrate((uint32_t)settings.canBitrate);
        ESP_LOGW(TAG, "can_bitrate -> %d: %s", settings.canBitrate, esp_err_to_name(r));
    });
    // can_listen_only is deliberately NOT live-applied. Going non-passive means
    // recreating the node, and it must be a considered act with a reboot behind
    // it, not a stray MQTT publish while the bike is running.
    settings.onChange("can_listen_only", [] {
        ESP_LOGW(TAG, "can_listen_only is now %d - saved, but takes effect on next boot",
                 settings.canListenOnly);
    });

    // Web server: /healthz, /reset, /set_hostname plus /firmware, /config,
    // /config/reset, /can/ids, /can/dump, /can/status, /can/reset.
    static WebContext webctx(&wifi);
    static CanWebServer web(&webctx, settings, bus, table);
    web.start();

    xTaskCreate(otaVerifyTask, "ota_verify", 4096, nullptr, 4, nullptr);
    xTaskCreate(telemetryTask, "telemetry", 4096, &app, 4, nullptr);
    if (canErr == ESP_OK) {
        xTaskCreate(statsTask, "can_stats", 4096, &app, 4, nullptr);
        if (selfTest) xTaskCreate(selfTestTask, "can_selftest", 4096, &app, 3, nullptr);
    }

    ESP_LOGI(TAG, "mqttcan started");
}
