#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// Turning watched CAN signals into GPIO output, so auxiliary driving lights can
// be switched from the handlebars with a multi-click of an existing control.
//
// Nothing here touches the CAN controller. The TWAI node stays in hardware
// listen-only mode; the only thing that leaves the board is a relay drive level
// on a GPIO. Making mqttcan an actuator must not make it a bus participant.
//
// Threading invariant, and the reason the design looks the way it does:
// onSignal() runs on the CanBus worker task, which must never block — a stall
// there overflows the RX queue and drops frames. So onSignal() only ever
// records state under a short-held mutex. Every side effect (driving a pin,
// firing the change callback, publishing to MQTT) happens in tick(), called
// from a slow timer task, or from an HTTP/MQTT handler that can afford to
// block. Actuation is therefore never on the frame path.

// One named output: a set of GPIOs driven together, i.e. a pair of lamps on
// two relays that are always on or off as one.
//
// JSON (a top-level "outputs" object in signals.json):
//   "driving": {
//     "gpios": [4, 5],              which pins; empty or absent = disabled
//     "active_low": false,          true for relay boards that close on a low
//     "auto_off_ms": 0,             force off after this long on (0 = never)
//     "off_when": {"signal": "ignition", "is": "off"}
//   }
//
// Boot state is always off and nothing is persisted: a reboot mid-ride leaves
// the auxiliary lights dark, which is the safe failure. The bike's own
// headlight is on its own circuit and is unaffected either way.
class OutputBank {
public:
    enum class Level { Off, On, Toggle };

    // name, new state, and what caused it ("gesture:3", "mqtt", "auto_off", ...)
    using ChangeFn = std::function<void(const std::string&, bool, const char*)>;

    OutputBank();
    ~OutputBank();

    // Parse the "outputs" section. A missing section is not an error — it means
    // no outputs, which is the shipped default. A malformed one is, so that a
    // typo is rejected by POST /signals rather than discovered the first time a
    // lever is pulled. On success the previous pins are released and the new
    // ones claimed; on failure the previous configuration keeps running.
    bool loadJson(const std::string& json, std::string& errorOut);

    bool has(const std::string& name) const;

    // Drive an output. Returns false if the name is unknown or outputs are
    // disabled. Fires the change callback (outside the lock) if the state moved.
    bool set(const std::string& name, Level level, const char* by);

    // Feed decoded signals in, for "off_when". Records only; tick() acts.
    void onSignal(const std::string& signal, const std::string& text);

    // Apply anything onSignal() queued and expire auto_off timers. Call from a
    // timer task, not the frame path.
    void tick(uint64_t nowUs);

    // The outputs_enable kill switch. Clearing it forces everything off.
    void setEnabled(bool enabled);
    bool enabled() const;

    size_t count() const;
    std::vector<std::pair<std::string, bool>> states() const;

    void onChange(ChangeFn fn) { change_ = std::move(fn); }

private:
    struct Output {
        std::string             name;
        std::vector<gpio_num_t> gpios;
        bool                    activeLow  = false;
        uint32_t                autoOffMs  = 0;
        std::string             offWhenSignal;   // empty = no interlock
        std::string             offWhenIs;

        bool     on         = false;
        uint64_t onSinceUs  = 0;
        bool     offPending = false;   // set by onSignal, applied by tick
    };

    struct Event {
        std::string name;
        bool        on;
        const char* by;
    };

    // Drives the pins and updates state. Caller holds the lock; the returned
    // event must be published after releasing it, because MQTT publishing can
    // block for seconds.
    bool applyLocked(Output& o, bool on, const char* by, Event& evOut);
    void driveLocked(const Output& o, bool on);
    void releaseLocked();

    mutable SemaphoreHandle_t lock_;
    std::vector<Output>       outputs_;
    bool                      enabled_ = true;
    ChangeFn                  change_;
};

// Counts short pulses of a decoded signal and fires an output action when the
// burst ends.
//
// JSON (a top-level "gestures" array in signals.json):
//   {"name": "driving_lights", "signal": "high_beam", "trigger": "on",
//    "window_ms": 1000, "max_hold_ms": 600, "min_gap_ms": 60,
//    "actions": {"1": {"output": "driving", "set": "off"},
//                "3": {"output": "driving", "set": "on"}}}
//
// A click is a *complete short pulse*, not a transition: the signal must reach
// `trigger` and leave it again within max_hold_ms, and the click is counted on
// release. That is what keeps ordinary use of the control out of the way —
// holding the high beam on for a mile is one transition but never a click.
// A hold longer than max_hold_ms also cancels any sequence in progress, so
// "hold it on" is a reliable way to abort a miscount.
//
// The window runs from the *last* release rather than the first, so a slow
// triple click still resolves as three instead of being truncated at two.
//
// A burst also resolves *immediately* once the count reaches the highest one
// the table maps, since no further click could change the outcome. That is what
// lets window_ms be generous: the top gesture stays instant while a lower count
// waits out the window to be sure nothing more is coming. Without it the window
// has to be short to keep the action responsive, and a short window splits a
// burst whenever the rider is slow or the bus loses a pulse between frames.
//
// Signal names are the decode table's own, so any enum signal can drive this
// without a firmware change.
class GestureEngine {
public:
    // gesture name, click count, and the action taken ("" when the count is
    // unmapped). Reported for every resolved burst, including unmapped counts —
    // that is the view that says "I saw 2" when you meant 3.
    using ReportFn = std::function<void(const std::string&, int, const std::string&)>;

    explicit GestureEngine(OutputBank& outputs);
    ~GestureEngine();

    // Load the "gestures" section. Must be called after OutputBank::loadJson on
    // the same document: action targets are checked against the loaded outputs
    // so a misspelled output name is rejected at upload time.
    bool loadJson(const std::string& json, std::string& errorOut);

    // Called on the CanBus worker task for every decoded signal. Records only.
    void onSignal(const std::string& signal, const std::string& text, uint64_t recvUs);

    // Resolve windows that have gone quiet and fire their actions.
    void tick(uint64_t nowUs);

    size_t count() const;
    void   onReport(ReportFn fn) { report_ = std::move(fn); }

private:
    struct Action {
        std::string       output;
        OutputBank::Level level = OutputBank::Level::Off;
    };

    struct Gesture {
        std::string name;
        std::string signal;
        std::string trigger;
        uint32_t    windowMs  = 1000;
        uint32_t    maxHoldMs = 600;
        uint32_t    minGapMs  = 60;
        std::vector<std::pair<int, Action>> actions;   // click count -> action
        int         maxClicks = 0;   // highest mapped count; reaching it fires early

        // Runtime. lastText is the engine's own view of the signal, deliberately
        // not SignalTable's: resetState() (from canreset/mark) makes the next
        // report of every signal unconditional, and reading that as a transition
        // would fabricate a click.
        bool        haveLast = false;
        std::string lastText;
        bool        pressed       = false;
        uint64_t    pressedUs     = 0;
        int         clicks        = 0;
        uint64_t    lastReleaseUs = 0;
        bool        fireNow       = false;   // count hit maxClicks; resolve on the next tick
    };

    struct Fired {
        std::string name;
        int         clicks;
        std::string action;      // "" when unmapped
        bool        hasAction;
        std::string output;
        OutputBank::Level level;
    };

    mutable SemaphoreHandle_t lock_;
    OutputBank&               outputs_;
    std::vector<Gesture>      gestures_;
    ReportFn                  report_;
};

// "on" | "off" | "toggle" -> Level. Returns false for anything else.
bool parseOutputLevel(const std::string& s, OutputBank::Level& out);
