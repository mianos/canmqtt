#include "CanWebServer.h"

#include <algorithm>
#include <array>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "CanBus.h"
#include "FrameTable.h"
#include "MqttClient.h"
#include "SignalTable.h"
#include "Settings.h"
#include "WifiManager.h"

#include "esp_timer.h"

namespace {

constexpr const char* TAG = "canweb";

// /config bodies are tiny — bound the input so a hostile Content-Length can't
// trigger a multi-GB std::string allocation. /firmware has its own bound (the
// OTA partition size).
constexpr size_t kMaxJsonBodyBytes = 4 * 1024;

// The decode table is a whole document rather than a settings patch, so it gets
// its own (larger) bound. Still far below the 256KB partition.
constexpr size_t kMaxSignalsBytes = 64 * 1024;

std::string read_request_body(httpd_req_t* req) {
    std::string body;
    body.reserve(req->content_len);
    char buf[256];
    int remaining = req->content_len;
    while (remaining > 0) {
        int got = httpd_req_recv(req, buf, std::min<int>(remaining, static_cast<int>(sizeof(buf))));
        if (got <= 0) break;
        body.append(buf, got);
        remaining -= got;
    }
    return body;
}

esp_err_t send_json(httpd_req_t* req, const JsonWrapper& json) {
    httpd_resp_set_type(req, "application/json");
    std::string out = json.ToString();
    return httpd_resp_sendstr(req, out.c_str());
}

// Labels arrive from a Node-RED button or a shell, so tolerate stray padding
// but treat whitespace-only as no label at all.
std::string trimmed(const std::string& s) {
    const char* ws = " \t\r\n";
    const size_t b = s.find_first_not_of(ws);
    if (b == std::string::npos) return std::string();
    return s.substr(b, s.find_last_not_of(ws) - b + 1);
}

// Pull a positive integer query parameter, e.g. /can/dump?limit=500.
size_t query_limit(httpd_req_t* req, size_t fallback) {
    char query[64];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) return fallback;
    char val[16];
    if (httpd_query_key_value(query, "limit", val, sizeof(val)) != ESP_OK) return fallback;
    long n = strtol(val, nullptr, 10);
    return n > 0 ? static_cast<size_t>(n) : fallback;
}

}  // namespace

JsonWrapper applyMark(FrameTable& table, SignalTable* signals, MqttClient& mqtt,
                      const std::string& sensorName, const std::string& text, bool reset) {
    // Reset first: the label has to outlive the wipe it asked for.
    if (reset) {
        table.reset();
        if (signals != nullptr) signals->resetState();
    }

    const uint64_t upUs = (uint64_t)esp_timer_get_time();
    const uint32_t seq  = table.mark(text.c_str(), upUs);

    JsonWrapper body;
    body.AddItem("seq",   static_cast<int>(seq));
    // Echo what was actually stored, not what was sent, so a truncated label
    // reports itself.
    body.AddItem("text",  text.substr(0, kMarkTextMax));
    body.AddItem("reset", reset);
    // The same monotonic clock the dump orders by: this is what lets a consumer
    // line an MQTT capture up against /can/dump.
    body.AddItem("up_ms", static_cast<int>(upUs / 1000ULL));

    // esp_mqtt_client_publish can block for a few seconds on a stalled network.
    // Acceptable here: this is a ~70-byte QoS 0 payload on a human-paced route.
    mqtt.publish("tele/" + sensorName + "/mark", body.ToString());
    return body;
}

bool applyOutput(OutputBank& outputs, const std::string& name, const std::string& set,
                 const char* by, std::string& errorOut) {
    OutputBank::Level level;
    if (!parseOutputLevel(set, level)) {
        errorOut = "'set' must be on, off or toggle";
        return false;
    }
    if (!outputs.enabled()) {
        errorOut = "outputs are disabled (outputs_enable=0)";
        return false;
    }
    if (!outputs.set(name, level, by)) {
        errorOut = "unknown output '" + name + "'";
        return false;
    }
    errorOut.clear();
    return true;
}

CanWebServer::CanWebServer(WebContext* ctx, Settings& settings, CanBus& bus, FrameTable& table,
                           SignalTable& signals, MqttClient& mqtt, OutputBank& outputs,
                           GestureEngine& gestures, ModbusBus& modbus)
    : WebServer(ctx), settings_(settings), bus_(bus), table_(table), signals_(signals),
      mqtt_(mqtt), outputs_(outputs), gestures_(gestures), modbus_(modbus) {}

esp_err_t CanWebServer::start() {
    esp_err_t r = WebServer::start();
    if (r != ESP_OK) return r;

    struct Route {
        const char*    uri;
        httpd_method_t method;
        esp_err_t (*handler)(httpd_req_t*);
    };
    // The shared WebServer starts httpd with max_uri_handlers = 16 and spends
    // three of them on /reset, /set_hostname and /healthz. Registration past
    // that returns ESP_ERR_HTTPD_HANDLERS_FULL and the route simply is not
    // there — a 404 at runtime with nothing else wrong, which is a miserable
    // thing to debug. The static_assert below turns it into a build error.
    //
    // This table is now exactly full. The next route needs one of: collapse
    // another URI's methods onto a single HTTP_ANY handler the way /signals
    // does below, or raise max_uri_handlers in the shared mianesp webserver
    // component (which four other projects also build against).
    constexpr size_t kMaxUriHandlers = 16;
    constexpr size_t kBaseRoutes     = 3;

    const std::array<Route, 13> routes = {{
        {"/firmware",    HTTP_POST, firmware_post_handler},
        {"/firmware",    HTTP_GET,  firmware_get_handler},
        {"/config",      HTTP_GET,  config_get_handler},
        {"/config",      HTTP_POST, config_post_handler},
        {"/config/reset", HTTP_POST, config_reset_post_handler},
        {"/can/ids",     HTTP_GET,  can_ids_get_handler},
        {"/can/dump",    HTTP_GET,  can_dump_get_handler},
        {"/can/status",  HTTP_GET,  can_status_get_handler},
        {"/can/reset",   HTTP_POST, can_reset_post_handler},
        {"/can/mark",    HTTP_POST, can_mark_post_handler},
        {"/can/inject",  HTTP_POST, can_inject_post_handler},
        {"/can/output",  HTTP_POST, can_output_post_handler},
        // GET and POST share one handler slot via HTTP_ANY (see above); the
        // dispatcher below splits them again. Externally nothing changes.
        {"/signals",     static_cast<httpd_method_t>(HTTP_ANY), signals_any_handler},
    }};
    static_assert(routes.size() + kBaseRoutes <= kMaxUriHandlers,
                  "too many HTTP routes: raise max_uri_handlers in the shared "
                  "WebServer, or httpd will silently drop the last ones");

    for (const Route& route : routes) {
        httpd_uri_t uri = {
            .uri      = route.uri,
            .method   = route.method,
            .handler  = route.handler,
            .user_ctx = this,
        };
        esp_err_t err = httpd_register_uri_handler(server, &uri);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "register %s %s: %s",
                route.method == HTTP_POST ? "POST" : "GET", route.uri, esp_err_to_name(err));
            return err;
        }
    }
    return ESP_OK;
}

void CanWebServer::populate_healthz_fields(WebContext*, JsonWrapper& json) {
    const esp_app_desc_t*  desc    = esp_app_get_description();
    const esp_partition_t* running = esp_ota_get_running_partition();
    json.AddItem("version",   std::string(desc->version));
    json.AddItem("partition", std::string(running->label));
    json.AddItem("heap_free", static_cast<int>(esp_get_free_heap_size()));
    json.AddItem("name",      settings_.sensorName);
    json.AddItem("can_frames", static_cast<int>(table_.totalFrames()));
}

// POST /firmware — raw .bin body. Streams into the inactive OTA slot, sets it as
// the next boot partition, then reboots.
// Deploy: curl --data-binary @build/mqttcan.bin http://<host>/firmware
esp_err_t CanWebServer::firmware_post_handler(httpd_req_t* req) {
    if (req->content_len <= 0) return sendJsonError(req, 400, "Content-Length required");

    const esp_partition_t* target = esp_ota_get_next_update_partition(nullptr);
    if (target == nullptr) return sendJsonError(req, 500, "no OTA partition available");
    if (req->content_len > static_cast<int>(target->size)) {
        return sendJsonError(req, 413, "image larger than OTA partition");
    }
    ESP_LOGI(TAG, "OTA: writing %d bytes to %s @ 0x%" PRIx32,
        req->content_len, target->label, target->address);

    esp_ota_handle_t handle = 0;
    esp_err_t err = esp_ota_begin(target, OTA_WITH_SEQUENTIAL_WRITES, &handle);
    if (err != ESP_OK) return sendJsonError(req, 500, esp_err_to_name(err));

    char buf[1024];
    int remaining = req->content_len;
    int written = 0;
    while (remaining > 0) {
        int got = httpd_req_recv(req, buf, std::min<int>(remaining, static_cast<int>(sizeof(buf))));
        if (got == HTTPD_SOCK_ERR_TIMEOUT) continue;
        if (got <= 0) {
            esp_ota_abort(handle);
            return sendJsonError(req, 400, "request body truncated");
        }
        err = esp_ota_write(handle, buf, got);
        if (err != ESP_OK) {
            esp_ota_abort(handle);
            return sendJsonError(req, 500, esp_err_to_name(err));
        }
        written   += got;
        remaining -= got;
    }

    err = esp_ota_end(handle);
    if (err != ESP_OK) return sendJsonError(req, 400, esp_err_to_name(err));
    err = esp_ota_set_boot_partition(target);
    if (err != ESP_OK) return sendJsonError(req, 500, esp_err_to_name(err));

    ESP_LOGW(TAG, "OTA: %d bytes written to %s; rebooting", written, target->label);
    JsonWrapper resp;
    resp.AddItem("status",    std::string("ok"));
    resp.AddItem("written",   written);
    resp.AddItem("partition", std::string(target->label));
    send_json(req, resp);

    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}

// ota_state is the field to check after an OTA. The image confirms itself only
// once it has an IP and there is no deadline on that, so a board powered off
// while still "pending_verify" boots the *previous* image next time, silently.
// Wait for "valid" before pulling the plug.
static const char* otaStateName(const esp_partition_t* running) {
    esp_ota_img_states_t st;
    if (esp_ota_get_state_partition(running, &st) != ESP_OK) return "unknown";
    switch (st) {
        case ESP_OTA_IMG_NEW:            return "new";
        case ESP_OTA_IMG_PENDING_VERIFY: return "pending_verify";
        case ESP_OTA_IMG_VALID:          return "valid";
        case ESP_OTA_IMG_INVALID:        return "invalid";
        case ESP_OTA_IMG_ABORTED:        return "aborted";
        case ESP_OTA_IMG_UNDEFINED:      return "undefined";
    }
    return "unknown";
}

esp_err_t CanWebServer::firmware_get_handler(httpd_req_t* req) {
    const esp_app_desc_t*  desc    = esp_app_get_description();
    const esp_partition_t* running = esp_ota_get_running_partition();
    JsonWrapper resp;
    resp.AddItem("version",   std::string(desc->version));
    resp.AddItem("idf_ver",   std::string(desc->idf_ver));
    resp.AddItem("date",      std::string(desc->date));
    resp.AddItem("time",      std::string(desc->time));
    resp.AddItem("partition", std::string(running->label));
    resp.AddItem("ota_state", std::string(otaStateName(running)));
    return send_json(req, resp);
}

esp_err_t CanWebServer::config_get_handler(httpd_req_t* req) {
    CanWebServer* self = static_cast<CanWebServer*>(req->user_ctx);
    JsonWrapper resp = self->settings_.toJson();
    return send_json(req, resp);
}

// POST /config — apply any subset of the settings keys, persist, return the new
// full settings. can_bitrate and the publish-policy fields take effect
// immediately; can_listen_only, the capacities and self_test need a reboot.
esp_err_t CanWebServer::config_post_handler(httpd_req_t* req) {
    CanWebServer* self = static_cast<CanWebServer*>(req->user_ctx);
    if (req->content_len > kMaxJsonBodyBytes) return sendJsonError(req, 413, "request body too large");
    std::string body = read_request_body(req);
    if (body.empty()) return sendJsonError(req, 400, "empty body");
    JsonWrapper json = JsonWrapper::Parse(body);
    if (json.Empty()) return sendJsonError(req, 400, "invalid JSON");

    self->settings_.loadFromJson(json);
    self->settings_.save();
    self->settings_.log();

    JsonWrapper resp = self->settings_.toJson();
    return send_json(req, resp);
}

// POST /config/reset — restore every setting to its default and persist.
// Optional body {"wifi": true} also clears Wi-Fi credentials, which reboots the
// device into provisioning (ESP-Touch v2).
esp_err_t CanWebServer::config_reset_post_handler(httpd_req_t* req) {
    CanWebServer* self = static_cast<CanWebServer*>(req->user_ctx);

    bool wipe_wifi = false;
    if (req->content_len > 0) {
        if (req->content_len > kMaxJsonBodyBytes) return sendJsonError(req, 413, "request body too large");
        std::string body = read_request_body(req);
        if (!body.empty()) {
            JsonWrapper json = JsonWrapper::Parse(body);
            if (json.Empty()) return sendJsonError(req, 400, "invalid JSON");
            json.GetField("wifi", wipe_wifi);
        }
    }

    self->settings_.resetToDefaults();
    self->settings_.save();
    self->settings_.log();
    ESP_LOGW(TAG, "config reset to defaults (wipe_wifi=%d)", wipe_wifi);

    JsonWrapper resp;
    resp.AddItem("status",       std::string(wipe_wifi ? "reset+wifi_clear+reboot" : "reset"));
    resp.AddItem("wifi_cleared", wipe_wifi);
    esp_err_t r = send_json(req, resp);

    if (wipe_wifi) {
        if (self->webContext && self->webContext->wifiManager) {
            ESP_LOGW(TAG, "clearing wifi credentials and rebooting");
            vTaskDelay(pdMS_TO_TICKS(500));
            self->webContext->wifiManager->clear();
        }
    }
    return r;
}

// GET /can/ids — the whole frame table. The primary reverse-engineering view:
// one row per CAN ID with its current payload, how often it repeats, and
// crucially `changed` — a bitmask of which payload bytes have ever moved, which
// separates live signal bytes from constant padding. Body built by idsJson().
esp_err_t CanWebServer::can_ids_get_handler(httpd_req_t* req) {
    CanWebServer* self = static_cast<CanWebServer*>(req->user_ctx);
    std::string out = idsJson(self->table_, self->bus_.bitrate());
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, out.c_str());
}

// GET /can/dump?limit=N — recent raw frames in candump's text format, so output
// pastes straight into existing CAN tooling. Bulk data belongs on HTTP, not
// MQTT. Sent in chunks because a full 2048-frame ring is ~90KB of text, too
// much to hold in one buffer alongside everything else.
esp_err_t CanWebServer::can_dump_get_handler(httpd_req_t* req) {
    CanWebServer* self = static_cast<CanWebServer*>(req->user_ctx);
    const size_t limit = query_limit(req, 0);  // 0 = everything retained
    std::vector<CanFrame> frames = self->table_.recentFrames(limit);
    std::vector<Mark>     marks  = self->table_.recentMarks();

    httpd_resp_set_type(req, "text/plain");
    constexpr size_t kFramesPerChunk = 64;
    // dumpText merges marks into one frame list, so each chunk must be handed
    // only the marks that fall inside it — otherwise every chunk re-emits the
    // whole backlog. `mi` walks the mark list once, in step with the frames.
    size_t mi = 0;
    for (size_t i = 0; i < frames.size(); i += kFramesPerChunk) {
        const size_t n    = std::min(kFramesPerChunk, frames.size() - i);
        const bool   last = (i + n >= frames.size());
        size_t mj = mi;
        if (last) {
            mj = marks.size();   // trailing marks belong to the final chunk
        } else {
            const uint64_t until = frames[i + n - 1].recvUs;
            while (mj < marks.size() && marks[mj].recvUs <= until) ++mj;
        }
        std::string chunk = dumpText(
            std::vector<CanFrame>(frames.begin() + i, frames.begin() + i + n),
            std::vector<Mark>(marks.begin() + mi, marks.begin() + mj));
        mi = mj;
        if (httpd_resp_send_chunk(req, chunk.data(), chunk.size()) != ESP_OK) return ESP_FAIL;
    }
    // No frames retained, but labels may still be worth showing.
    if (frames.empty() && !marks.empty()) {
        std::string chunk = dumpText({}, marks);
        if (httpd_resp_send_chunk(req, chunk.data(), chunk.size()) != ESP_OK) return ESP_FAIL;
    }
    return httpd_resp_send_chunk(req, nullptr, 0);
}

// GET /can/status — the bit-rate diagnostic. Frames climbing with bus_errors
// flat means the rate is right; zero frames with bus_errors climbing means it
// is wrong (try 125000 next).
esp_err_t CanWebServer::can_status_get_handler(httpd_req_t* req) {
    CanWebServer* self = static_cast<CanWebServer*>(req->user_ctx);
    const CanBusCounters& c = self->bus_.counters();

    JsonWrapper resp;
    resp.AddItem("bitrate",      static_cast<int>(self->bus_.bitrate()));
    resp.AddItem("listen_only",  self->bus_.listenOnly());
    resp.AddItem("frames_rx",    static_cast<int>(c.framesRx));
    resp.AddItem("drops",        static_cast<int>(c.drops));
    resp.AddItem("rx_errors",    static_cast<int>(c.rxErrors));
    resp.AddItem("state_changes", static_cast<int>(c.stateChanges));
    resp.AddItem("tracked_ids",  static_cast<int>(self->table_.trackedIds()));
    // Non-zero means someone used /can/inject and the table holds fabricated
    // frames. Worth surfacing: it is otherwise indistinguishable from real data.
    resp.AddItem("injected",     static_cast<int>(c.injected));

    // Decode-table state lives here rather than on its own route: httpd's
    // handler table is nearly full (see start()), and "is the decoder loaded
    // and what is it muting" is the same question as "what is the bus doing".
    resp.AddItem("signals_loaded",    self->signals_.loaded());
    resp.AddItem("signals_byte_base", self->signals_.byteBase());
    resp.AddItem("signals_frames",    static_cast<int>(self->signals_.frameCount()));
    resp.AddItem("signals_count",     static_cast<int>(self->signals_.signalCount()));
    const auto noise = self->signals_.noiseMasks();
    if (!noise.empty()) {
        // Flattened to a string because JsonWrapper has no nested-object
        // support: "0x2AC=03,0x3FF=1C".
        std::string flat;
        for (size_t i = 0; i < noise.size(); ++i) {
            char nm[24];
            snprintf(nm, sizeof(nm), "%s0x%s=%02X", i ? "," : "",
                     canIdHex(noise[i].first, noise[i].first > 0x7FF).c_str(), noise[i].second);
            flat += nm;
        }
        resp.AddItem("signals_noise", flat);
    }

    // Outputs and gestures. output_<name> is flattened one key per output for
    // the same reason signals_noise is a string: JsonWrapper has no nested
    // objects. "gestures" of 0 with a table that defines one means the section
    // was rejected at load — check the console.
    resp.AddItem("outputs_enable", self->outputs_.enabled());
    resp.AddItem("outputs",        static_cast<int>(self->outputs_.count()));
    resp.AddItem("gestures",       static_cast<int>(self->gestures_.count()));
    for (const auto& s : self->outputs_.states()) {
        resp.AddItem("output_" + s.first, std::string(s.second ? "on" : "off"));
    }

    // RS485. modbus_link is the one to read: "up" means the last coil write
    // was acknowledged by the node, which is end-to-end confirmation that the
    // pair, the transceivers and the slave's firmware are all alive.
    const ModbusBus::Health mh = self->modbus_.health();
    resp.AddItem("modbus_enable", self->settings_.modbusEnable != 0);
    resp.AddItem("modbus_running", mh.running);
    resp.AddItem("modbus_link",   std::string(!mh.running          ? "off"
                                              : mh.consecErr == 0  ? "up"
                                                                   : "down"));
    resp.AddItem("modbus_ok",     (int)mh.ok);
    resp.AddItem("modbus_err",    (int)mh.err);
    if (mh.lastErr != ESP_OK) {
        resp.AddItem("modbus_last_error", std::string(esp_err_to_name(mh.lastErr)));
    }

    twai_node_status_t st;
    twai_node_record_t rec;
    if (self->bus_.info(st, rec) == ESP_OK) {
        resp.AddItem("state",          std::string(canErrorStateName(st.state)));
        resp.AddItem("tx_error_count", static_cast<int>(st.tx_error_count));
        resp.AddItem("rx_error_count", static_cast<int>(st.rx_error_count));
        resp.AddItem("bus_errors",     static_cast<int>(rec.bus_err_num));
        // In listen-only mode the IDF driver writes 128 into the RX error
        // counter before leaving reset and then freezes it (an errata
        // workaround: error-passive guarantees the node can never emit a
        // dominant error frame). So "passive" and 128 are the healthy reading
        // here, not a fault, and only bus_errors carries information. Say so,
        // because they look alarming and waste a diagnosis otherwise.
        if (self->bus_.listenOnly()) {
            resp.AddItem("state_note", std::string(
                "listen-only pins state=passive and rx_error_count=128 by design; "
                "watch bus_errors instead"));
        }
    }
    return send_json(req, resp);
}

// POST /can/reset — clear the table and ring. The decode workflow is: reset,
// perform exactly one physical action on the bike, then re-read /can/ids and
// see which byte of which ID moved.
esp_err_t CanWebServer::can_reset_post_handler(httpd_req_t* req) {
    CanWebServer* self = static_cast<CanWebServer*>(req->user_ctx);
    self->table_.reset();
    // Decode state too, matching the MQTT canreset command: without this the
    // next value of every signal is suppressed as "unchanged" and the freshly
    // cleared board reports nothing until something physically moves.
    self->signals_.resetState();
    JsonWrapper resp;
    resp.AddItem("status", std::string("cleared"));
    return send_json(req, resp);
}

// POST /can/mark — drop an operator label into the capture:
//   curl -X POST -d '{"text":"test high beam"}' http://mqttcan.local/can/mark
//   curl -X POST -d '{"text":"front brake","reset":true}' .../can/mark
//
// The label is published on tele/<name>/mark and appears inline in /can/dump,
// so a capture read back later says which action produced which change.
esp_err_t CanWebServer::can_mark_post_handler(httpd_req_t* req) {
    CanWebServer* self = static_cast<CanWebServer*>(req->user_ctx);
    if (req->content_len > kMaxJsonBodyBytes) return sendJsonError(req, 413, "request body too large");
    std::string body = read_request_body(req);
    if (body.empty()) return sendJsonError(req, 400, "empty body");
    JsonWrapper json = JsonWrapper::Parse(body);
    if (json.Empty()) return sendJsonError(req, 400, "invalid JSON");

    std::string text;
    if (!json.GetField("text", text)) {
        return sendJsonError(req, 400, json.ContainsField("text")
                                           ? "field 'text' must be a string"
                                           : "missing string field 'text'");
    }
    text = trimmed(text);
    if (text.empty()) return sendJsonError(req, 400, "field 'text' is empty");

    // Accept both true and 1 for 'reset': Node-RED template nodes emit either
    // depending on how the payload was built.
    bool reset = false;
    if (!json.GetField("reset", reset)) {
        int n = 0;
        if (json.GetField("reset", n)) reset = (n != 0);
    }

    JsonWrapper resp = applyMark(self->table_, &self->signals_, self->mqtt_,
                                 self->settings_.sensorName, text, reset);
    return send_json(req, resp);
}

// POST /can/inject — push a synthetic frame through the whole software path
// without touching the bus, so the table, decoder, noise masks and publishers
// can be exercised on a vehicle while the controller stays listen-only.
//
//   {"id":"0x3FF","data":"497FC9BC1E78D001"}
//   {"id":"0x3FF","data":"497FC9BC1E78D001","repeat":5,"increment":2}
//
// `increment` bumps that payload byte by one on each repeat, which is how you
// reproduce a free-running counter and prove a noise mask suppresses it.
esp_err_t CanWebServer::can_inject_post_handler(httpd_req_t* req) {
    CanWebServer* self = static_cast<CanWebServer*>(req->user_ctx);
    if (req->content_len > kMaxJsonBodyBytes) return sendJsonError(req, 413, "request body too large");
    std::string body = read_request_body(req);
    if (body.empty()) return sendJsonError(req, 400, "empty body");
    JsonWrapper json = JsonWrapper::Parse(body);
    if (json.Empty()) return sendJsonError(req, 400, "invalid JSON");

    std::string idStr, dataStr;
    if (!json.GetField("id", idStr))     return sendJsonError(req, 400, "missing string field 'id'");
    if (!json.GetField("data", dataStr)) return sendJsonError(req, 400, "missing string field 'data'");

    const uint32_t id = (uint32_t)strtoul(
        idStr.compare(0, 2, "0x") == 0 ? idStr.c_str() + 2 : idStr.c_str(), nullptr, 16);

    uint8_t data[kCanMaxData] = {};
    if (dataStr.size() % 2 != 0 || dataStr.size() > kCanMaxData * 2) {
        return sendJsonError(req, 400, "'data' must be an even number of hex digits, 16 max");
    }
    const size_t len = dataStr.size() / 2;
    for (size_t i = 0; i < len; ++i) {
        char byteStr[3] = {dataStr[i * 2], dataStr[i * 2 + 1], '\0'};
        char* end = nullptr;
        const unsigned long v = strtoul(byteStr, &end, 16);
        if (end != byteStr + 2) return sendJsonError(req, 400, "'data' is not hex");
        data[i] = (uint8_t)v;
    }

    bool ext = (id > 0x7FF);
    json.GetField("ext", ext);

    int repeat = 1;
    json.GetField("repeat", repeat);
    if (repeat < 1 || repeat > 256) return sendJsonError(req, 400, "'repeat' must be 1..256");

    int increment = -1;
    json.GetField("increment", increment);
    if (increment >= (int)len) return sendJsonError(req, 400, "'increment' is past the end of 'data'");

    int sent = 0;
    for (int n = 0; n < repeat; ++n) {
        if (self->bus_.inject(id, ext, data, len) != ESP_OK) break;
        sent++;
        if (increment >= 0) data[increment]++;
        // The worker task has to actually drain these, and a publish decision
        // depends on the gap between frames, so do not fire them all in one
        // tick.
        if (repeat > 1) vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGW(TAG, "injected %d synthetic frame(s) for 0x%s - table now contains fabricated data",
             sent, canIdHex(id, ext).c_str());

    JsonWrapper resp;
    resp.AddItem("id",       "0x" + canIdHex(id, ext));
    resp.AddItem("injected", sent);
    resp.AddItem("dlc",      static_cast<int>(len));
    resp.AddItem("note",     std::string("synthetic frames, not bus traffic; "
                                         "POST /can/reset afterwards"));
    return send_json(req, resp);
}

// POST /can/output — drive a relay by name, bypassing the gestures:
//   curl -X POST -d '{"name":"driving","set":"on"}' http://mqttcan.local/can/output
//
// This is how a relay is proved out on the bench before a lever is ever pulled,
// and how Node-RED drives the lights directly. The reply carries the state of
// every output, not just the one addressed.
esp_err_t CanWebServer::can_output_post_handler(httpd_req_t* req) {
    CanWebServer* self = static_cast<CanWebServer*>(req->user_ctx);
    if (req->content_len > kMaxJsonBodyBytes) return sendJsonError(req, 413, "request body too large");
    std::string body = read_request_body(req);
    if (body.empty()) return sendJsonError(req, 400, "empty body");
    JsonWrapper json = JsonWrapper::Parse(body);
    if (json.Empty()) return sendJsonError(req, 400, "invalid JSON");

    std::string name, set;
    if (!json.GetField("name", name)) return sendJsonError(req, 400, "missing string field 'name'");
    if (!json.GetField("set", set))   return sendJsonError(req, 400, "missing string field 'set'");

    std::string err;
    if (!applyOutput(self->outputs_, name, set, "http", err)) {
        return sendJsonError(req, 400, err);
    }

    JsonWrapper resp;
    resp.AddItem("status", std::string("ok"));
    for (const auto& s : self->outputs_.states()) {
        resp.AddItem("output_" + s.first, std::string(s.second ? "on" : "off"));
    }
    return send_json(req, resp);
}

// /signals is registered once with HTTP_ANY because httpd's handler table is
// exactly full, so GET and POST cannot each have a slot. This splits them back
// out; anything else gets a 405 rather than being silently treated as a read.
esp_err_t CanWebServer::signals_any_handler(httpd_req_t* req) {
    if (req->method == HTTP_GET)  return signals_get_handler(req);
    if (req->method == HTTP_POST) return signals_post_handler(req);
    return sendJsonError(req, 405, "use GET to read the decode table or POST to replace it");
}

// GET /signals — the decode table as stored. Served straight off the
// filesystem rather than re-serialised from the parsed form, so what you read
// back is byte-for-byte what is in use, comments and all.
esp_err_t CanWebServer::signals_get_handler(httpd_req_t* req) {
    std::string body = signalstore::read();
    if (body.empty()) return sendJsonError(req, 404, "no decode table stored");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, body.c_str());
}

// POST /signals — replace the decode table.
// Deploy: curl --data-binary @data/signals.json http://<host>/signals
//
// The upload is parsed and applied BEFORE it is written to flash: a table that
// does not load is rejected with the parse error and nothing is stored, so a
// bad edit can never leave the device unable to decode after a reboot.
esp_err_t CanWebServer::signals_post_handler(httpd_req_t* req) {
    CanWebServer* self = static_cast<CanWebServer*>(req->user_ctx);
    if (req->content_len <= 0) return sendJsonError(req, 400, "empty body");
    if (req->content_len > kMaxSignalsBytes) return sendJsonError(req, 413, "table too large");

    std::string body;
    body.reserve(req->content_len);
    char buf[512];
    int remaining = req->content_len;
    while (remaining > 0) {
        int got = httpd_req_recv(req, buf, std::min<int>(remaining, (int)sizeof(buf)));
        if (got == HTTPD_SOCK_ERR_TIMEOUT) continue;
        if (got <= 0) return sendJsonError(req, 400, "request body truncated");
        body.append(buf, got);
        remaining -= got;
    }

    std::string err;
    if (!self->signals_.loadJson(body, err)) {
        ESP_LOGW(TAG, "rejected signals upload: %s", err.c_str());
        return sendJsonError(req, 400, err);
    }
    // Outputs before gestures: gesture actions are validated against the output
    // names. Unlike at boot these are hard failures, so a typo in a pin number
    // or an output name is a 400 here rather than a relay that quietly never
    // fires. The decode table is already live at this point, which is the right
    // trade — the table is the thing you cannot afford to lose.
    if (!self->outputs_.loadJson(body, err) || !self->gestures_.loadJson(body, err)) {
        ESP_LOGW(TAG, "rejected signals upload: %s", err.c_str());
        return sendJsonError(req, 400, err);
    }
    if (!signalstore::write(body, err)) {
        // The new table is live but unsaved; say so rather than report success.
        return sendJsonError(req, 500, "table applied but not saved: " + err);
    }

    ESP_LOGW(TAG, "decode table replaced: %u frames / %u signals",
             (unsigned)self->signals_.frameCount(), (unsigned)self->signals_.signalCount());
    JsonWrapper resp;
    resp.AddItem("status",  std::string("ok"));
    resp.AddItem("frames",   (int)self->signals_.frameCount());
    resp.AddItem("signals",  (int)self->signals_.signalCount());
    resp.AddItem("outputs",  (int)self->outputs_.count());
    resp.AddItem("gestures", (int)self->gestures_.count());
    resp.AddItem("bytes",    (int)body.size());
    return send_json(req, resp);
}
