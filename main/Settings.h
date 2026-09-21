#pragma once

#include "SettingsBase.h"

// mqttcan settings schema — persistence/reset/log machinery lives in mianesp's
// settingsbase. Member initialisers are the compiled-in defaults; missing or
// unparseable NVS config falls back to them (logged).
//
// SettingsBase only stores std::string and int fields, so booleans are modelled
// as 0/1 ints.
struct Settings : SettingsBase {
    std::string mqttServer = "mqtt2.mianos.com";
    int         mqttPort   = 1883;
    std::string sensorName = "mqttcan";
    std::string tz         = "AEST-10AEDT,M10.1.0,M4.1.0/3";

    // Wi-Fi regulatory domain. ESP-IDF defaults to "01" (worldwide), which
    // allows only channels 1-11 — so an AP on channel 12 or 13 is invisible and
    // association fails with reason 201 NO_AP_FOUND even though the SSID and
    // password are correct. AU (like most of the world outside North America)
    // permits 1-13, so default to that and keep it settable.
    std::string wifiCountry = "AU";

    // --- CAN bus ---
    // 500000 is a well-corroborated first guess for the R1200GS, but NOT a
    // confirmed BMW spec for the K25: a 2008 R1200GS Adventure got error-free
    // frames at 500k with an MCP2515 and failed at 125k/250k/1M, and an R1200R
    // tapped at the OBD connector worked with SocketCAN at 500000. No source
    // states the rate authoritatively. It is therefore settable at runtime and
    // applies live (see the can_bitrate onChange hook in main.cpp) — if the bus
    // shows zero frames and a climbing error count, try 125000 next.
    int canBitrate = 500000;

    // 1 ⇒ the TWAI controller is put in hardware listen-only mode: it never
    // transmits and never acknowledges, so it cannot perturb the vehicle bus.
    // Prior art matters here — someone's nominally "listen-only" sniffer
    // intermittently transmitted and toggled their bike's rear brake light.
    // Deliberately NOT live-applied: a change is persisted but only takes
    // effect on the next boot, and coming up non-passive is logged at WARN.
    int canListenOnly = 1;

    // --- Publish policy ---
    // A 500kbit/s bus can carry thousands of frames/sec, so publishing every
    // frame would flatten the broker. Instead a frame is published only when
    // its payload *changes*, floored at publishMinMs per ID, with an optional
    // slow heartbeat so a static ID still reports in. Same decimation idea as
    // mqttradar's presencePeriodSec, applied per CAN ID.
    int publishMinMs       = 200;    // per-ID floor between change publishes
    int publishHeartbeatMs = 60000;  // republish an unchanged ID this often (0 = never)
    int publishEnable      = 1;      // 0 ⇒ observe only; /can/ids still fills in

    // 1 ⇒ also ESP_LOGI each published frame to the console. Mirrors
    // mqttradar's PrintEP sitting alongside its MQTT publisher: invaluable on
    // the bench (and on the bike over USB serial) where there is no broker to
    // watch. Costs a line of UART per published frame, so leave it off in
    // normal operation.
    int logFrames = 0;

    // --- Capacity ---
    // Both are read once at startup (the tables are allocated from them), so a
    // change needs a reboot.
    int maxTrackedIds  = 256;   // distinct CAN IDs in the frame table
    // Raw frames retained for GET /can/dump. The K25 was measured at ~1200
    // frames/s (12 IDs, most cycling every 9-10ms), so the old 2048 held only
    // 1.7 seconds — useless for "label an action, then read the dump", which
    // is the whole workflow. 32768 frames is 1MB and ~27 seconds; it lands in
    // PSRAM (8MB, almost entirely free) and is only touched by the worker task
    // and HTTP handlers, never the ISR.
    int dumpRingFrames = 32768;

    // --- Outputs ---
    // Master kill switch for the GPIO outputs driven by signals.json's
    // "gestures" section. Clearing it forces every output off immediately and
    // makes further gestures inert, without editing the decode table — which is
    // what you want when a relay starts misbehaving at the side of the road.
    // Applies live.
    int outputsEnable = 1;

    // --- RS485 / Modbus RTU master ---
    // Drives outputs that live away from this board — a MOSFET node at the
    // headlight rather than a relay module under the seat.
    //
    // Default off: with no slave answering, every cycle costs a full response
    // timeout and logs a failure, which is noise on a board that has no node
    // wired to it yet. Turn it on once the node exists. Applies live — the
    // reconciler task owns the port's lifecycle and starts or stops it on the
    // next cycle, so nothing else can be mid-transaction when it happens.
    int modbusEnable = 0;

    // Both take effect when the port next starts, i.e. toggle modbus_enable
    // after changing them. 19200 is Modbus RTU's conventional default and is
    // untroubled by the length of a motorcycle; there is no reason to go
    // faster for four coils.
    int         modbusBaud   = 19200;
    std::string modbusParity = "none";   // none | even | odd

    // How often the master re-asserts coil state when nothing has changed. A
    // change is written immediately (the reconciler is woken by the output
    // change hook), so this is purely the heartbeat that resyncs a slave which
    // rebooted or missed a frame — and, more importantly, the clock the slave
    // measures its own comms watchdog against. The slave must hold its
    // watchdog well above this: see the receiver notes in the README.
    int modbusPeriodMs = 1000;

    // Per-transaction response timeout. A four-coil write at 19200 baud is
    // over in ~10ms, so 200ms is generous; it exists to bound how long a dead
    // slave stalls the reconciler task.
    int modbusTimeoutMs = 200;

    // --- Bench self-test ---
    // 1 ⇒ transmit synthetic frames to ourselves to exercise the whole
    // ISR→ring→table→MQTT path with no bus attached. Requires canListenOnly=0
    // (a listen-only node physically cannot transmit) and is refused otherwise.
    // Never enable this on the vehicle.
    int selfTest = 0;

    explicit Settings(NvsStorageManager& nvs) : SettingsBase(nvs) {
        field("mqtt_server", mqttServer);
        field("mqtt_port",   mqttPort);
        field("sensor_name", sensorName);
        field("tz",          tz);
        field("wifi_country", wifiCountry);
        field("can_bitrate",     canBitrate);
        field("can_listen_only", canListenOnly);
        field("publish_min_ms",       publishMinMs);
        field("publish_heartbeat_ms", publishHeartbeatMs);
        field("publish_enable",       publishEnable);
        field("log_frames",           logFrames);
        field("max_tracked_ids",  maxTrackedIds);
        field("dump_ring_frames", dumpRingFrames);
        field("outputs_enable",   outputsEnable);
        field("modbus_enable",     modbusEnable);
        field("modbus_baud",       modbusBaud);
        field("modbus_parity",     modbusParity);
        field("modbus_period_ms",  modbusPeriodMs);
        field("modbus_timeout_ms", modbusTimeoutMs);
        field("self_test",        selfTest);
        load();
    }
};
