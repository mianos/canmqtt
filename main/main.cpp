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
//   mark        {"text":"test high beam"[,"reset":true]}  label the capture
//   output      {"name":"driving","set":"on"|"off"|"toggle"}  drive an output
//                   (local GPIOs and/or coils on an RS485 Modbus node)
//   restart     {}
//   reprovision {}  clears Wi-Fi creds, reboots into ESP-Touch v2 provisioning
// Publishes:
//   tele/<name>/frame   {"id":"0x2BC","ext":...,"dlc":8,"data":"...","chg":"0C",...}
//                       on each accepted payload change
//   tele/<name>/signals {"id":"0x2BC","engine_temp_c":88.5,"gear":"N"} — named
//                       values, whenever one moves past its own deadband
//   tele/<name>/mark    {"seq":7,"text":"test high beam","reset":false,"up_ms":…}
//   tele/<name>/gesture {"gesture":"driving_lights","clicks":3,"action":"driving=on"}
//   tele/<name>/output  {"name":"driving","state":"on","by":"gesture:3"}
//   tele/<name>/modbus  {"link":"down","ok":412,"err":3,...} on link transitions
//   tele/<name>/stats   bus health + rates, 1/min — the bit-rate diagnostic
//   tele/<name>/init,status   identity + telemetry
// HTTP: /healthz /config /firmware /can/ids /can/dump /can/status /can/reset
//       /can/mark /can/inject /can/output /signals

#include <atomic>
#include <cinttypes>
#include <cstring>
#include <cstdlib>
#include <ctime>
#include <regex>
#include <string>
#include <utility>

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

#include "Actions.h"
#include "CanBus.h"
#include "CanWebServer.h"
#include "ModbusBus.h"
#include "FrameTable.h"
#include "SignalTable.h"

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
    SignalTable* signals = nullptr;
    OutputBank*  outputs = nullptr;
    GestureEngine* gestures = nullptr;
    ModbusBus*   modbus  = nullptr;

    // Announces RS485 link transitions. A std::function rather than a direct
    // publish so the topic string stays with the other topics in app_main.
    std::function<void(bool, const ModbusBus::Health&)> modbusReport = nullptr;
};

// Woken by the output change hook so a gesture reaches the RS485 node in
// milliseconds instead of waiting out the heartbeat. Set before the task that
// reads it exists, and only ever assigned once.
TaskHandle_t g_modbusTask = nullptr;

// ---------------------------------------------------------------------------
// Deferred MQTT publishing
// ---------------------------------------------------------------------------
//
// MqttClient::publish can block for ~10s against a stalled broker, and on a
// motorcycle the broker is out of range most of the time. Anything on a
// real-time path must therefore hand its message to this queue rather than
// publish it directly.
//
// This is not tidiness. The RS485 reconciler is what keeps the driving lights
// lit: it re-asserts coil state every second, and the node at the far end
// drops every channel after five seconds of silence. A publish inside that
// task means one corrupted frame flips the link state, the resulting report
// blocks on a broker that is not there, the heartbeat stops, and the lights go
// out at 100km/h. The same argument already applies to the CAN worker (see
// onFrame); it applies here for higher stakes.
//
// Dropping a telemetry message when the queue is full is the correct trade:
// the alternative is blocking a task whose deadline actually matters.
struct PubMsg {
    std::string topic;
    std::string payload;
};

QueueHandle_t g_pubQueue = nullptr;

// Messages refused because the queue was full. Counted here and reported from
// publisherTask, never logged at the point of the drop. publishAsync runs on
// the real-time tasks and the console is UART0, so a log line is a blocking
// write of several milliseconds. In a stalled-broker window the frame path can
// try to publish a hundred times a second; logging each refusal would rebuild
// on the CAN worker, one line at a time, the very stall this queue exists to
// keep off it.
std::atomic<uint32_t> g_pubDropped{0};

void publishAsync(const std::string& topic, const std::string& payload) {
    if (g_pubQueue == nullptr) return;
    auto* msg = new PubMsg{topic, payload};
    if (xQueueSend(g_pubQueue, &msg, 0) != pdTRUE) {
        delete msg;   // queue full: drop it rather than wait
        g_pubDropped.fetch_add(1, std::memory_order_relaxed);
    }
}

void publisherTask(void* arg) {
    auto* app = static_cast<App*>(arg);
    for (;;) {
        PubMsg* msg = nullptr;
        if (xQueueReceive(g_pubQueue, &msg, portMAX_DELAY) != pdTRUE) continue;
        // Blocking here is fine and is the entire point of this task.
        app->mqtt->publish(std::move(msg->topic), std::move(msg->payload));
        delete msg;

        // Once the backlog has cleared, say what the stall cost: one line per
        // episode, from the task that was doing the stalling.
        if (uxQueueMessagesWaiting(g_pubQueue) == 0) {
            const uint32_t dropped = g_pubDropped.exchange(0, std::memory_order_relaxed);
            if (dropped > 0) {
                ESP_LOGW(TAG, "publish stalled; dropped %" PRIu32 " message(s) while the "
                              "broker was unreachable", dropped);
            }
        }
    }
}

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
    if (app->signals) app->signals->resetState();  // or the next report is suppressed as "unchanged"
    return ESP_OK;
}

// cmnd/<name>/mark — {"text":"test high beam"}, optionally "reset":true.
//
// The payload must be a JSON *object*: MqttClient::dispatchEvent parses every
// message and drops anything that is not one before a handler ever runs, so a
// bare-text publish silently goes nowhere.
esp_err_t handleMark(MqttClient*, const std::string&, const JsonWrapper& d, void* ctx) {
    auto* app = static_cast<App*>(ctx);
    if (app->table == nullptr) return ESP_OK;

    std::string text;
    if (!d.GetField("text", text)) {
        ESP_LOGW(TAG, "mark: payload needs a string field 'text'");
        return ESP_OK;
    }
    const size_t b = text.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) {
        ESP_LOGW(TAG, "mark: empty label ignored");
        return ESP_OK;
    }
    text = text.substr(b, text.find_last_not_of(" \t\r\n") - b + 1);

    bool reset = false;
    if (!d.GetField("reset", reset)) {
        int n = 0;
        if (d.GetField("reset", n)) reset = (n != 0);
    }
    // applyMark publishes the ack itself, so the body it returns is discarded.
    applyMark(*app->table, app->signals, *app->mqtt, app->settings->sensorName, text, reset);
    return ESP_OK;
}

// cmnd/<name>/output — {"name":"driving","set":"on"|"off"|"toggle"}.
// Manual control for Node-RED, and the way to exercise a relay from the bench
// without pulling a lever. Same body as POST /can/output; the state change is
// announced on tele/<name>/output by the change hook, not from here.
esp_err_t handleOutput(MqttClient*, const std::string&, const JsonWrapper& d, void* ctx) {
    auto* app = static_cast<App*>(ctx);
    if (app->outputs == nullptr) return ESP_OK;

    std::string name, set;
    if (!d.GetField("name", name) || !d.GetField("set", set)) {
        ESP_LOGW(TAG, "output: payload needs string fields 'name' and 'set'");
        return ESP_OK;
    }
    std::string err;
    if (!applyOutput(*app->outputs, name, set, "mqtt", err)) {
        ESP_LOGW(TAG, "output: %s", err.c_str());
    }
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

// Confirms a freshly OTA'd image once it has proved it can reach the network.
//
// There is deliberately no deadline. Waiting indefinitely looks weaker than
// rolling back on a timer, but it is strictly safer on a vehicle and gives up
// nothing, because the bootloader already covers both real failure modes:
//
//   - an image that actually crashes never gets here at all, and the next boot
//     rolls it back unprompted. That is what CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE
//     does, and it is the whole safety net;
//   - an image that runs but cannot join Wi-Fi is left unconfirmed, so it is
//     rolled back at the next power cycle — on a bike, the end of the ride.
//
// A deadline adds exactly one behaviour on top of that: rebooting a board that
// is working perfectly well, in the middle of a ride, because it happens to be
// out of Wi-Fi range. Here that drops the RS485 heartbeat with the lamps lit
// and leaves the node's 5s failsafe racing a ~3s restart. The lights probably
// survive it. "Probably" is not a good enough reason to keep the timer.
void otaVerifyTask(void*) {
    const esp_partition_t* running = esp_ota_get_running_partition();
    esp_ota_img_states_t state;
    if (esp_ota_get_state_partition(running, &state) == ESP_OK &&
        state == ESP_OTA_IMG_PENDING_VERIFY) {
        ESP_LOGW(TAG, "OTA: image pending verify; waiting for connectivity");
        xSemaphoreTake(s_got_ip, portMAX_DELAY);
        esp_ota_mark_app_valid_cancel_rollback();
        ESP_LOGI(TAG, "OTA: connectivity confirmed, image marked valid");
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

// Everything that actually drives a relay runs here rather than on the CAN
// worker task. The worker only records; a publish or a gpio_config() call in
// that path would stall the RX queue and cost frames. 25ms is well under the
// shortest gesture window and invisible to a rider.
void actionsTask(void* arg) {
    auto* app = static_cast<App*>(arg);
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(25));
        const uint64_t now = (uint64_t)esp_timer_get_time();
        app->gestures->tick(now);   // may switch an output
        app->outputs->tick(now);    // off_when and auto_off expiry
    }
}

// Pushes the desired coil state of every RS485 output node onto the wire.
//
// Deliberately a reconciler and not an event pump. It writes the complete
// coil state of each slave — on every change, and again every
// modbus_period_ms whether anything changed or not. That makes the link
// stateless in the direction that matters: a node that browns out on a pothole,
// resets on a wet connector, or is simply plugged in after the master has
// already booted comes back to the right state within one heartbeat, with no
// handshake, no sequence numbers and no recovery path to get wrong.
//
// It also owns the port's lifecycle, rather than the settings hook doing it:
// mbc_master_delete() while this task sits inside mbc_master_send_request()
// would be a use-after-free, and a settings change arrives on the MQTT task.
void modbusTask(void* arg) {
    auto* app = static_cast<App*>(arg);
    bool lastLinkUp = true;   // so the first failure reports a transition

    for (;;) {
        const bool want = app->settings->modbusEnable != 0;
        if (want && !app->modbus->running()) {
            if (app->modbus->start(app->settings->modbusBaud, app->settings->modbusParity,
                                   (uint32_t)app->settings->modbusTimeoutMs) != ESP_OK) {
                vTaskDelay(pdMS_TO_TICKS(5000));   // retry slowly; nothing else to do
                continue;
            }
            lastLinkUp = true;
            // Enabled with nothing to send is a configuration mistake that is
            // otherwise completely silent: the link reports "up" forever
            // because no transaction ever fails.
            if (!app->outputs->usesModbus()) {
                ESP_LOGW(TAG, "modbus_enable=1 but no output declares any coils - "
                              "add a \"modbus\" block to an output in signals.json");
            }
        } else if (!want && app->modbus->running()) {
            app->modbus->stop();
        }
        if (!app->modbus->running()) {
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
            continue;
        }

        bool allOk = true;
        // What actually goes on the wire, logged only when it changes rather
        // than on every heartbeat. esp-modbus copies this buffer verbatim into
        // the PDU, so this *is* the frame payload: it is the only way to check
        // bit order without a node to ask, and a bit-order slip would swap the
        // lamps rather than fail visibly.
        static std::string lastWire;
        std::string wire;
        for (auto& sc : app->outputs->desiredCoils()) {
            char buf[16];
            snprintf(buf, sizeof(buf), " s%u/%uc=", (unsigned)sc.slave, (unsigned)sc.count);
            wire += buf;
            for (uint8_t b : sc.bits) {
                snprintf(buf, sizeof(buf), "%02X", b);
                wire += buf;
            }
            if (app->modbus->writeCoils(sc.slave, 0, sc.count, sc.bits.data()) != ESP_OK) {
                allOk = false;
            }
        }
        if (wire != lastWire) {
            lastWire = wire;
            ESP_LOGI(TAG, "RS485 coils ->%s", wire.c_str());
        }

        const ModbusBus::Health h = app->modbus->health();
        const bool linkUp = (h.consecErr == 0);
        if (linkUp != lastLinkUp) {
            lastLinkUp = linkUp;
            if (app->modbusReport) app->modbusReport(linkUp, h);
        }

        // Back off once a slave has clearly gone rather than hammering a dead
        // bus: retry briskly for the first few failures, since the usual cause
        // is one corrupted frame and the lights should not wait a whole second
        // for that, then settle to the heartbeat.
        uint32_t waitMs = (uint32_t)app->settings->modbusPeriodMs;
        if (!allOk && h.consecErr < 5) waitMs = 50;
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(waitMs));
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

        // 0x2D0: exercises the enum paths -- nibble maps, the composite ESA
        // key (D4 low + D7 low) and the ignition "default" label (D7 != FF).
        // Expected: ambient 24C, fuel 25.1%, preload norm_1helm, damping
        // soft_smooth, info off, grips off, ignition on.
        uint8_t d[8] = {0x00, 0x80, 0x00, 0x40, 0x2B, 0x04, 0x00, 0xC1};
        app->bus->transmit(0x2D0, false, d, sizeof(d));

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
    esp_log_level_set("signals", ESP_LOG_INFO);
    esp_log_level_set("settings", ESP_LOG_INFO);
    esp_log_level_set("actions", ESP_LOG_INFO);   // which outputs and gestures loaded
    esp_log_level_set("modbus", ESP_LOG_INFO);
    // esp-modbus logs every unanswered request at ERROR. With a node that is
    // unplugged, off, or simply not built yet, that is one line per cycle
    // forever and it buries everything else. The link state is already
    // reported once per transition on tele/<name>/modbus and counted in
    // /can/status, which is the same information without the firehose.
    esp_log_level_set("MB_CONTROLLER_MASTER", ESP_LOG_NONE);

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
    mqtt.registerHandler(b + "mark",        std::regex(b + "mark"),        handleMark,        &app);
    mqtt.registerHandler(b + "output",      std::regex(b + "output"),      handleOutput,      &app);
    mqtt.registerHandler(b + "restart",     std::regex(b + "restart"),     handleRestart,     &app);
    mqtt.registerHandler(b + "reprovision", std::regex(b + "reprovision"), handleReprovision, &app);
    mqtt.start();

    // Before anything that can publish — which now includes the CAN worker, so
    // this has to be up before bus.start() rather than at the end of app_main.
    // 32 messages is far more than the real-time paths generate in a burst, and
    // overflow drops rather than blocks.
    g_pubQueue = xQueueCreate(32, sizeof(PubMsg*));
    xTaskCreate(publisherTask, "publisher", 4096, &app, 3, nullptr);

    // --- CAN ---
    static FrameTable table(settings.maxTrackedIds, settings.dumpRingFrames);
    app.table = &table;

    static CanBus bus(kCanTx, kCanRx);
    app.bus = &bus;

    // Decode table. Lives on its own SPIFFS partition so the mapping can be
    // corrected over the air as IDs are confirmed on the bike; the firmware
    // embeds a copy that is written out on first boot, so a fresh board decodes
    // straight away. A bad table is never fatal -- raw frames keep flowing.
    static SignalTable signals;
    app.signals = &signals;

    // Outputs and gestures come out of the same document. They are what makes
    // this board act rather than only watch -- see Actions.h. The CAN
    // controller is untouched by any of it and stays listen-only.
    static OutputBank   outputs;
    static GestureEngine gestures(outputs);
    static ModbusBus     modbus;
    app.outputs  = &outputs;
    app.gestures = &gestures;
    app.modbus   = &modbus;

    std::string stored;
    if (signalstore::mount()) {
        std::string err;
        if (!signalstore::ensureDefault(err)) {
            ESP_LOGE(TAG, "could not seed signals.json: %s", err.c_str());
        }
        stored = signalstore::read();
        if (stored.empty()) {
            ESP_LOGE(TAG, "no decode table; publishing raw frames only");
        } else if (!signals.loadJson(stored, err)) {
            ESP_LOGE(TAG, "signals.json rejected (%s); publishing raw frames only",
                     err.c_str());
        }
    } else {
        ESP_LOGE(TAG, "signals partition unavailable; publishing raw frames only");
    }
    if (!stored.empty()) {
        // Outputs first: gesture actions are validated against the loaded
        // output names. A bad section here is logged and leaves no outputs
        // configured rather than being fatal -- the same policy as the decode
        // table, and failing closed means no relay is driven.
        std::string err;
        if (!outputs.loadJson(stored, err)) {
            ESP_LOGE(TAG, "outputs section rejected: %s", err.c_str());
        } else if (!gestures.loadJson(stored, err)) {
            ESP_LOGE(TAG, "gestures section rejected: %s", err.c_str());
        }
    }
    outputs.setEnabled(settings.outputsEnable != 0);

    const std::string frameTopic  = "tele/" + settings.sensorName + "/frame";
    const std::string signalTopic = "tele/" + settings.sensorName + "/signals";
    static const std::string outputTopic  = "tele/" + settings.sensorName + "/output";
    static const std::string gestureTopic = "tele/" + settings.sensorName + "/gesture";
    static const std::string modbusTopic  = "tele/" + settings.sensorName + "/modbus";

    // Every output change is announced, whatever caused it, so a Node-RED panel
    // can show the true state rather than assume its own button worked.
    outputs.onChange([](const std::string& name, bool on, const char* by) {
        // Wake the reconciler *first*, before anything that could block, so a
        // stalled broker cannot delay the lights. Safe before the task exists:
        // a null handle means nothing to notify, and the next heartbeat picks
        // the state up anyway.
        if (g_modbusTask != nullptr) xTaskNotifyGive(g_modbusTask);

        JsonWrapper d;
        d.AddItem("name",  name);
        d.AddItem("state", std::string(on ? "on" : "off"));
        d.AddItem("by",    std::string(by));
        publishAsync(outputTopic, d.ToString());
        ESP_LOGI(TAG, "output %s -> %s (%s)", name.c_str(), on ? "on" : "off", by);
    });

    // Only transitions, not every failed cycle: a node that is unplugged over
    // winter should cost one retained-looking message, not one per second.
    app.modbusReport = [](bool up, const ModbusBus::Health& h) {
        JsonWrapper d;
        d.AddItem("link",  std::string(up ? "up" : "down"));
        d.AddItem("ok",    (int)h.ok);
        d.AddItem("err",   (int)h.err);
        d.AddItem("last_error", std::string(esp_err_to_name(h.lastErr)));
        publishAsync(modbusTopic, d.ToString());
        ESP_LOGW(TAG, "RS485 link %s (ok %" PRIu32 ", err %" PRIu32 ", last %s)",
                 up ? "up" : "down", h.ok, h.err, esp_err_to_name(h.lastErr));
    };

    // Reported for every resolved burst including unmapped counts: when a
    // triple click does nothing, this is what tells you the board saw two.
    gestures.onReport([](const std::string& name, int clicks, const std::string& action) {
        JsonWrapper d;
        d.AddItem("gesture", name);
        d.AddItem("clicks",  clicks);
        d.AddItem("action",  action);   // "" when the count is unmapped
        publishAsync(gestureTopic, d.ToString());
    });

    // Runs on the CanBus worker task, once per received frame. Publishing is
    // gated by FrameTable::observe() so a 500kbit/s firehose becomes a trickle
    // of actual state changes.
    //
    // Everything here goes out through publishAsync, never mqtt.publish. This
    // task drains the RX queue *and* feeds the gesture engine, so a stall here
    // costs frames and misses handlebar input. MqttClient::publish is cheap
    // while the broker is known to be disconnected — it buffers and returns —
    // but while it still believes it is connected it writes to the socket with
    // a 10s network timeout. That is precisely the state the board is in as the
    // bike rides out of Wi-Fi range: association gone, TCP not yet given up. A
    // direct publish there would freeze this task for ten seconds, overflow the
    // 256-frame queue, and drop a triple click on the way out of the driveway.
    auto onFrame = [frameTopic, signalTopic](const CanFrame& f) {
        // Decode first, and for every frame — not just the ones that survive the
        // raw-frame filter. The two policies are independent: a signal has its
        // own deadband and min_ms, and gating it behind publish_min_ms would
        // drop real state changes just because the raw frame was throttled.
        //
        // Decoding is also deliberately NOT gated on publishEnable: the
        // gestures that switch the driving lights are fed from here, and
        // turning MQTT publishing off must not turn the handlebar controls off
        // with it. Only the publish below is conditional.
        if (signals.loaded()) {
            std::vector<DecodedSignal> changed = signals.decode(f);
            if (!changed.empty()) {
                JsonWrapper sd;
                sd.AddItem("id", "0x" + canIdHex(f.id, f.ext));
                for (const DecodedSignal& ds : changed) {
                    if (ds.isEnum) sd.AddItem(*ds.name, ds.text);
                    else           sd.AddItem(*ds.name, ds.value);
                    if (settings.logFrames) {
                        ESP_LOGI(TAG, "  %s = %s%s", ds.name->c_str(),
                                 ds.isEnum ? ds.text.c_str()
                                           : std::to_string(ds.value).c_str(),
                                 ds.unit->empty() ? "" : ds.unit->c_str());
                    }
                    // Both of these only record, under a briefly-held mutex.
                    // Nothing on this task is allowed to block: it drains the
                    // RX queue, and a stall here costs frames. The actual
                    // switching happens on the actions task.
                    if (ds.isEnum) {
                        outputs.onSignal(*ds.name, ds.text);
                        gestures.onSignal(*ds.name, ds.text, f.recvUs);
                    }
                }
                if (settings.publishEnable) publishAsync(signalTopic, sd.ToString());
            }
        }

        uint8_t chg = 0;
        // Free-running counters would otherwise make every frame "changed" and
        // republish the ID at its full cyclic rate for the whole ride.
        const uint8_t noise = signals.loaded() ? signals.noiseMask(f.id) : 0;
        const bool publish = table.observe(f, settings.publishMinMs,
                                           settings.publishHeartbeatMs, &chg, noise);
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
        publishAsync(frameTopic, d.ToString());
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
    // The relay kill switch, live. Clearing it forces every output off at once
    // rather than waiting for the next gesture, which is the behaviour you want
    // when something is misbehaving and you are at the side of the road.
    settings.onChange("outputs_enable", [] {
        outputs.setEnabled(settings.outputsEnable != 0);
        if (g_modbusTask != nullptr) xTaskNotifyGive(g_modbusTask);
    });
    // Poking the reconciler is all that happens here; it does the start or
    // stop itself on its next cycle. Doing it from this callback would risk
    // deleting the Modbus context out from under an in-flight transaction,
    // because settings arrive on the MQTT task.
    settings.onChange("modbus_enable", [] {
        ESP_LOGW(TAG, "modbus_enable -> %d", settings.modbusEnable);
        if (g_modbusTask != nullptr) xTaskNotifyGive(g_modbusTask);
    });

    // Web server: /healthz, /reset, /set_hostname plus /firmware, /config,
    // /config/reset, /can/ids, /can/dump, /can/status, /can/reset.
    static WebContext webctx(&wifi);
    static CanWebServer web(&webctx, settings, bus, table, signals, mqtt, outputs, gestures,
                            modbus);
    web.start();

    xTaskCreate(otaVerifyTask, "ota_verify", 4096, nullptr, 4, nullptr);
    xTaskCreate(telemetryTask, "telemetry", 4096, &app, 4, nullptr);
    xTaskCreate(actionsTask,   "actions",   4096, &app, 4, nullptr);
    // Separate from "actions" because a Modbus transaction blocks for up to
    // modbus_timeout_ms, and the gesture tick must keep its 25ms cadence.
    xTaskCreate(modbusTask,    "modbus",    4096, &app, 4, &g_modbusTask);
    if (canErr == ESP_OK) {
        xTaskCreate(statsTask, "can_stats", 4096, &app, 4, nullptr);
        if (selfTest) xTaskCreate(selfTestTask, "can_selftest", 4096, &app, 3, nullptr);
    }

    ESP_LOGI(TAG, "mqttcan started");
}
