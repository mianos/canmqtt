#pragma once

#include <cstdint>
#include <string>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// Forward declared rather than including CanBus.h. That header reaches IDF's
// hal/assert.h, which under picolibc redefines __noreturn as [[noreturn]];
// picolibc's own stdlib.h then uses __noreturn as a *trailing* attribute,
// where [[noreturn]] appertains to the type and is a hard error. Pulling
// CanBus.h in from here dragged that ordering into every translation unit that
// includes this header. Only a reference is needed, so don't.
struct CanFrame;

// Recovers a wall clock from a cyclic counter on the CAN bus.
//
// Why this exists: the board runs from the bike's switched 12V and is out of
// Wi-Fi range for almost every minute it is powered, so SNTP sets the clock in
// the garage and never again. The board's own PCF85063 RTC does not rescue
// that -- its VDD is the main 3V3 rail with no backup cell fitted (confirmed
// against the Waveshare schematic: no battery, no holder, not even an
// unpopulated footprint), so it loses time at every ignition-off exactly like
// the ESP32's internal RTC does. The only clock that is guaranteed to be alive
// whenever this board is alive belongs to the bike.
//
// The candidate is 3FF D2-D4: a 24-bit counter measured on the vehicle
// incrementing by exactly 1 every 1000ms. It is currently in the noise mask
// because nobody had a use for it. What it counts *from* is the open question,
// and it decides whether this is a clock or a stopwatch:
//
//   seconds since local midnight   -> a real time-of-day, and this module's
//                                     whole reason for existing. Values stay
//                                     under 86400, so D2 only ever holds 0x00
//                                     or 0x01.
//   seconds since ignition on      -> a ride timer. Starts near zero every
//                                     time the key turns.
//   seconds since battery connect  -> a session timer that survives ignition
//                                     cycles and wraps after 194 days.
//
// All three are useful and all three look identical in a single reading, so
// this module deliberately does not guess. In the default "observe" mode it
// only reports the raw value and both interpretations, which is what turns one
// glance at /can/status into a settled answer.
class CanClock {
public:
    enum class Mode {
        Observe,   // read and report only; never touches the system clock
        Tod,       // treat the field as seconds since local midnight
    };

    struct Reading {
        bool     valid    = false;   // a frame carrying the field has arrived
        uint32_t raw      = 0;       // the field, big-endian across its bytes
        uint64_t atUs     = 0;       // monotonic time of that read
        uint32_t updates  = 0;       // times the value changed
    };

    CanClock();
    ~CanClock();

    // Parses the optional top-level "clock" section. A document without one
    // leaves the clock unconfigured, which is not an error.
    bool loadJson(const std::string& json, std::string& errorOut);

    // Called for every received frame, on the CAN worker task. Records only --
    // never sets the system clock, never publishes, never blocks beyond a
    // briefly-held mutex. Anything slower belongs on the actions task.
    void onFrame(const CanFrame& f);

    bool    configured() const;
    Mode    mode() const;
    uint32_t id() const;
    Reading reading() const;

    // The field read as seconds since local midnight, advanced by however long
    // ago the reading was taken. Returns false when nothing has been seen yet,
    // or when the value is too large to be a time of day.
    bool timeOfDay(uint32_t& secondsOut) const;

    // How far the time-of-day reading sits from the system clock's own local
    // time of day, in seconds. This is the measurement that identifies the
    // counter: near zero and it *is* the wall clock.
    //
    // Only meaningful once SNTP has run, so it returns false until the system
    // clock is real — which is exactly the condition of the test that settles
    // it: ignition on, stationary, in Wi-Fi range.
    bool offsetVsLocal(int& offsetSecondsOut, uint32_t& localTodOut) const;

private:
    mutable SemaphoreHandle_t lock_;
    bool     configured_ = false;
    Mode     mode_       = Mode::Observe;
    uint32_t id_         = 0;
    bool     ext_        = false;
    uint8_t  bytes_[4]   = {};
    uint8_t  nbytes_     = 0;
    Reading  last_;
};

// "1d 04h 22m 13s" for the elapsed reading, "19:42:13" for the time-of-day one.
std::string canClockElapsedString(uint32_t seconds);
std::string canClockTodString(uint32_t secondsSinceMidnight);
