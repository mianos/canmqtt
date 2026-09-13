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
#include "SignalTable.h"
#include "Settings.h"
#include "WifiManager.h"

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

CanWebServer::CanWebServer(WebContext* ctx, Settings& settings, CanBus& bus, FrameTable& table,
                           SignalTable& signals)
    : WebServer(ctx), settings_(settings), bus_(bus), table_(table), signals_(signals) {}

esp_err_t CanWebServer::start() {
    esp_err_t r = WebServer::start();
    if (r != ESP_OK) return r;

    struct Route {
        const char*    uri;
        httpd_method_t method;
        esp_err_t (*handler)(httpd_req_t*);
    };
    const std::array<Route, 12> routes = {{
        {"/firmware",    HTTP_POST, firmware_post_handler},
        {"/firmware",    HTTP_GET,  firmware_get_handler},
        {"/config",      HTTP_GET,  config_get_handler},
        {"/config",      HTTP_POST, config_post_handler},
        {"/config/reset", HTTP_POST, config_reset_post_handler},
        {"/can/ids",     HTTP_GET,  can_ids_get_handler},
        {"/can/dump",    HTTP_GET,  can_dump_get_handler},
        {"/can/status",  HTTP_GET,  can_status_get_handler},
        {"/can/reset",   HTTP_POST, can_reset_post_handler},
        {"/signals",        HTTP_GET,  signals_get_handler},
        {"/signals",        HTTP_POST, signals_post_handler},
        {"/signals/status", HTTP_GET,  signals_status_get_handler},
    }};

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

esp_err_t CanWebServer::firmware_get_handler(httpd_req_t* req) {
    const esp_app_desc_t*  desc    = esp_app_get_description();
    const esp_partition_t* running = esp_ota_get_running_partition();
    JsonWrapper resp;
    resp.AddItem("version",   std::string(desc->version));
    resp.AddItem("idf_ver",   std::string(desc->idf_ver));
    resp.AddItem("date",      std::string(desc->date));
    resp.AddItem("time",      std::string(desc->time));
    resp.AddItem("partition", std::string(running->label));
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

    httpd_resp_set_type(req, "text/plain");
    constexpr size_t kFramesPerChunk = 64;
    for (size_t i = 0; i < frames.size(); i += kFramesPerChunk) {
        const size_t n = std::min(kFramesPerChunk, frames.size() - i);
        std::string chunk = dumpText(std::vector<CanFrame>(frames.begin() + i,
                                                           frames.begin() + i + n));
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

    twai_node_status_t st;
    twai_node_record_t rec;
    if (self->bus_.info(st, rec) == ESP_OK) {
        resp.AddItem("state",          std::string(canErrorStateName(st.state)));
        resp.AddItem("tx_error_count", static_cast<int>(st.tx_error_count));
        resp.AddItem("rx_error_count", static_cast<int>(st.rx_error_count));
        resp.AddItem("bus_errors",     static_cast<int>(rec.bus_err_num));
    }
    return send_json(req, resp);
}

// POST /can/reset — clear the table and ring. The decode workflow is: reset,
// perform exactly one physical action on the bike, then re-read /can/ids and
// see which byte of which ID moved.
esp_err_t CanWebServer::can_reset_post_handler(httpd_req_t* req) {
    CanWebServer* self = static_cast<CanWebServer*>(req->user_ctx);
    self->table_.reset();
    JsonWrapper resp;
    resp.AddItem("status", std::string("cleared"));
    return send_json(req, resp);
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
    if (!signalstore::write(body, err)) {
        // The new table is live but unsaved; say so rather than report success.
        return sendJsonError(req, 500, "table applied but not saved: " + err);
    }

    ESP_LOGW(TAG, "decode table replaced: %u frames / %u signals",
             (unsigned)self->signals_.frameCount(), (unsigned)self->signals_.signalCount());
    JsonWrapper resp;
    resp.AddItem("status",  std::string("ok"));
    resp.AddItem("frames",  (int)self->signals_.frameCount());
    resp.AddItem("signals", (int)self->signals_.signalCount());
    resp.AddItem("bytes",   (int)body.size());
    return send_json(req, resp);
}

// GET /signals/status — is a table loaded, what does it cover, and which byte
// convention is it using. The ids list is the quick way to see whether the IDs
// in the table match the ones /can/ids is actually seeing on the bike.
esp_err_t CanWebServer::signals_status_get_handler(httpd_req_t* req) {
    CanWebServer* self = static_cast<CanWebServer*>(req->user_ctx);
    std::vector<uint32_t> ids = self->signals_.knownIds();
    std::sort(ids.begin(), ids.end());

    std::string out = "{\"loaded\":";
    out += self->signals_.loaded() ? "true" : "false";
    out += ",\"byte_base\":" + std::to_string(self->signals_.byteBase());
    out += ",\"frames\":" + std::to_string(self->signals_.frameCount());
    out += ",\"signals\":" + std::to_string(self->signals_.signalCount());
    out += ",\"ids\":[";
    for (size_t i = 0; i < ids.size(); ++i) {
        if (i) out += ',';
        out += "\"0x" + canIdHex(ids[i], ids[i] > 0x7FF) + "\"";
    }
    out += "]}";
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, out.c_str());
}
