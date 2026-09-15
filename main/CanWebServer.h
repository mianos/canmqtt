#pragma once

#include "esp_err.h"
#include "esp_http_server.h"

#include "JsonWrapper.h"
#include "WebServer.h"

struct Settings;
class CanBus;
class FrameTable;
class MqttClient;
class SignalTable;

// Record an operator label and announce it on tele/<sensorName>/mark, returning
// the body both transports reply with. Shared by POST /can/mark and the MQTT
// `mark` command so the two cannot drift apart.
//
// `reset` clears the frame table first — deliberately opt-in, and deliberately
// before the label is stored, since a reset afterwards would discard it.
// `signals` may be null; when present its last-reported state is cleared
// alongside the table, or the next decode is suppressed as "unchanged".
JsonWrapper applyMark(FrameTable& table, SignalTable* signals, MqttClient& mqtt,
                      const std::string& sensorName,
                      const std::string& text, bool reset);

// mqttcan HTTP control surface, layered on the shared WebServer base
// (/reset, /set_hostname, /healthz). Adds:
//   POST /firmware      raw .bin body -> inactive OTA slot -> reboot
//   GET  /firmware      running image version / partition
//   GET  /config        current settings as JSON
//   POST /config        apply + persist a subset of settings
//   POST /config/reset  restore settings to defaults (optional wifi wipe)
//   GET  /can/ids       the per-ID frame table — the main discovery view
//   GET  /can/dump      recent raw frames as candump-style text (?limit=N)
//   GET  /can/status    bit rate, error state, counters
//   POST /can/reset     clear the frame table and dump ring
//   POST /can/mark      drop an operator label into the capture
//   POST /can/inject    push a synthetic frame through the software path
//                       (no bus access, so it is safe while listen-only)
//   GET  /signals       the decode table currently in use (JSON)
//   POST /signals       replace it (validated before it is stored)
//   GET  /signals/status  whether a table is loaded, and what it covers
// Handlers recover this instance from req->user_ctx.
class CanWebServer : public WebServer {
public:
    CanWebServer(WebContext* ctx, Settings& settings, CanBus& bus, FrameTable& table,
                 SignalTable& signals, MqttClient& mqtt);

    esp_err_t start() override;

protected:
    void populate_healthz_fields(WebContext* ctx, JsonWrapper& json) override;

private:
    static esp_err_t firmware_post_handler(httpd_req_t* req);
    static esp_err_t firmware_get_handler(httpd_req_t* req);
    static esp_err_t config_get_handler(httpd_req_t* req);
    static esp_err_t config_post_handler(httpd_req_t* req);
    static esp_err_t config_reset_post_handler(httpd_req_t* req);
    static esp_err_t can_ids_get_handler(httpd_req_t* req);
    static esp_err_t can_dump_get_handler(httpd_req_t* req);
    static esp_err_t can_status_get_handler(httpd_req_t* req);
    static esp_err_t can_reset_post_handler(httpd_req_t* req);
    static esp_err_t can_mark_post_handler(httpd_req_t* req);
    static esp_err_t can_inject_post_handler(httpd_req_t* req);
    static esp_err_t signals_get_handler(httpd_req_t* req);
    static esp_err_t signals_post_handler(httpd_req_t* req);
    static esp_err_t signals_status_get_handler(httpd_req_t* req);

    Settings&    settings_;
    CanBus&      bus_;
    FrameTable&  table_;
    SignalTable& signals_;
    MqttClient&  mqtt_;
};
