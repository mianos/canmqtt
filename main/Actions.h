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

// One named output: a set of targets driven together, i.e. a pair of lamps
// that are always on or off as one. A target is either a local GPIO or a coil
// on a Modbus slave reached over RS485; an output can carry both, which is how
// you bench-test against a relay board and drive the real node from the same
// table.
//
// JSON (a top-level "outputs" object in signals.json):
//   "driving": {
//     "gpios": [4, 5],              local pins; empty or absent = none
//     "modbus": {"slave": 1, "coils": [0, 1]},   remote coils; absent = none
//     "active_low": false,          true for relay boards that close on a low
//     "auto_off_ms": 0,             force off after this long on (0 = never)
//     "off_when": {"signal": "ignition", "is": "off"}
//   }
//
// active_low applies to `gpios` only. A coil carries the logical state and the
// slave decides its own polarity — putting the inversion in two places is how
// you end up with lights that are on when the table says off.
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

    // The desired coil state of every Modbus slave any output refers to,
    // packed for FC 0x0F.
    //
    // This is the whole master/slave protocol: not "send an event when
    // something changes" but "here is what the coils should be, now". The
    // reconciler writes it on every change and again on a slow heartbeat, so a
    // slave that browned out, missed a frame, or was plugged in ten minutes
    // late converges on the next cycle with no resync logic anywhere. Events
    // would need retries, acknowledgement and a recovery path; state needs
    // none of them.
    //
    // A slave's vector spans coil 0 through the highest address any output
    // claims on it, and coils in that span which no output claims are written
    // off. The master owns the slave's whole coil space — a shared slave is
    // not a supported arrangement, and silently leaving gaps alone would be a
    // worse surprise than saying so.
    struct SlaveCoils {
        uint8_t              slave;
        uint16_t             count;   // coils 0..count-1
        std::vector<uint8_t> bits;    // packed LSB-first, Modbus wire order
    };
    std::vector<SlaveCoils> desiredCoils() const;

    // True if any output has at least one Modbus coil, i.e. whether the RS485
    // reconciler has anything to do.
    bool usesModbus() const;

    void onChange(ChangeFn fn) { change_ = std::move(fn); }

private:
    struct Coil {
        uint8_t  slave;
        uint16_t addr;
    };

    struct Output {
        std::string             name;
        std::vector<gpio_num_t> gpios;
        std::vector<Coil>       coils;
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
//    "actions": {"1": {"output": ["driving0", "driving1"], "set": "off"},
//                "3": {"output": ["driving0", "driving1"], "set": "on"}}}
//
// "output" is one output name or a list of them. A list is how one gesture
// switches several lamps while each lamp stays its own output, individually
// addressable over HTTP and MQTT: outputs may not share a coil, so a combined
// output alongside per-lamp ones is not an option. "toggle" flips each listed
// output on its own, so lamps already out of step stay out of step.
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
        std::vector<std::string> outputs;
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
        std::vector<std::string> outputs;
        OutputBank::Level level;
    };

    mutable SemaphoreHandle_t lock_;
    OutputBank&               outputs_;
    std::vector<Gesture>      gestures_;
    ReportFn                  report_;
};

// "on" | "off" | "toggle" -> Level. Returns false for anything else.
bool parseOutputLevel(const std::string& s, OutputBank::Level& out);
