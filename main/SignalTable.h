#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "CanBus.h"

// One decoded signal that has actually changed and is worth reporting.
struct DecodedSignal {
    const std::string* name;   // points into the SignalDef; valid while the table lives
    const std::string* unit;
    std::string        text;   // enum label, for mapped signals
    double             value;  // scaled numeric, for unmapped signals
    bool               isEnum;
};

// Turns raw CAN frames into named vehicle signals using a JSON description
// loaded from the filesystem, so the decode table can be corrected over the
// air as IDs are confirmed on the bike — without a firmware rebuild.
//
// Schema (see data/signals.json):
//   version        integer; a newer embedded default replaces a stored table
//   byte_base      which data[] index "D1" means; 0 => Dn is data[n] (the
//                  community sheet's convention), 1 => D1 is data[0]
//   frames.<ID>.signals[]
//     bytes[]      little-endian byte list, least-significant first:
//                  (D3*256 + D2) is [2,3]
//     nibble       "high" | "low" | absent
//     parts[]      {byte, nibble} list, for keys built from several nibbles
//                  (the ESA damping pair); joined with "," to index map
//     scale/offset value = raw * scale + offset
//     map{}        uppercase-hex key -> label; presence makes it an enum
//     default      label for any code missing from map (else unmapped_0xNN)
//     invalid      raw value meaning "no reading"; the signal is not reported
//     deadband     suppress numeric reports smaller than this
//     min_ms       per-signal floor between reports
//   noise.<ID>     hex byte mask of free-running counters to ignore when
//                  deciding whether a raw frame changed, e.g. "1C"
//
// Reporting is change-driven, matching FrameTable's policy for raw frames:
// an enum reports when its label changes, a number when it moves further than
// its deadband, and both respect min_ms.
class SignalTable {
public:
    SignalTable();
    ~SignalTable();

    // Parse a JSON description. Replaces any previously loaded table.
    // Returns false and keeps the old table on a parse error.
    bool loadJson(const std::string& json, std::string& errorOut);

    // Decode a frame and return only the signals that changed enough to report.
    std::vector<DecodedSignal> decode(const CanFrame& f);

    // Forget every signal's last-reported value, so the next frame of each ID
    // reports afresh. Pairs with FrameTable::reset().
    void resetState();

    // Payload bytes for this ID that must not count as a change, from the
    // table's "noise" section. 0 when the ID has no entry.
    uint8_t  noiseMask(uint32_t id) const;

    // Every muted ID and its mask, for /signals/status — the only way to
    // confirm the "noise" section parsed without waiting for bus traffic.
    std::vector<std::pair<uint32_t, uint8_t>> noiseMasks() const;

    bool     loaded() const { return loaded_; }
    size_t   frameCount() const;
    size_t   signalCount() const;
    int      byteBase() const { return byteBase_; }
    // Names of the IDs in the table, for /signals/status.
    std::vector<uint32_t> knownIds() const;

private:
    struct Part {
        uint8_t byteIdx;   // already resolved to a data[] index
        int8_t  nibble;    // -1 whole byte, 0 low, 1 high
    };

    struct SignalDef {
        std::string name;
        std::string unit;
        std::vector<uint8_t> bytes;  // resolved data[] indices, LSB first
        int8_t  nibble  = -1;
        double  scale   = 1.0;
        double  offset  = 0.0;
        int     decimals = 0;
        double  deadband = 0.0;
        uint32_t minMs  = 0;
        bool     haveInvalid = false;
        uint32_t invalid = 0;
        std::vector<Part> parts;                             // composite key
        std::unordered_map<std::string, std::string> map;    // enum
        std::string mapDefault;                              // "" => unmapped_0xNN

        // Last reported state.
        bool        haveLast  = false;
        double      lastValue = 0.0;
        std::string lastText;
        uint64_t    lastPubUs = 0;
    };

    struct FrameDef {
        std::string            name;
        std::vector<SignalDef> signals;
    };

    // Pull the raw integer a signal is built from. Returns false if the frame
    // is too short for the bytes it names.
    static bool rawValue(const SignalDef& s, const CanFrame& f, uint32_t& out);
    static bool nibbleOf(const Part& p, const CanFrame& f, uint8_t& out);

    mutable SemaphoreHandle_t lock_;
    std::unordered_map<uint32_t, FrameDef> frames_;
    std::unordered_map<uint32_t, uint8_t>  noise_;
    int  byteBase_ = 0;
    bool loaded_   = false;
};

// Storage for the decode table: a SPIFFS partition holding signals.json.
// mount() formats on first use; ensureDefault() writes the firmware's built-in
// copy if the file is missing or carries a lower "version" than the build,
// so a fresh board decodes without an upload and a reflash carries table
// corrections with it. Bump "version" in an uploaded table to protect it.
namespace signalstore {
bool        mount();
std::string read();                       // "" if absent/unreadable
bool        write(const std::string& s, std::string& errorOut);
bool        ensureDefault(std::string& errorOut);
std::string defaultJson();                // the embedded build-time copy
}  // namespace signalstore
