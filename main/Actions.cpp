#include "Actions.h"

#include <algorithm>
#include <cinttypes>
#include <cstdlib>
#include <cstring>

#include "cJSON.h"
#include "esp_log.h"
#include "esp_timer.h"

namespace {

constexpr const char* TAG = "actions";

struct LockGuard {
    SemaphoreHandle_t h;
    explicit LockGuard(SemaphoreHandle_t s) : h(s) { xSemaphoreTake(h, portMAX_DELAY); }
    ~LockGuard() { xSemaphoreGive(h); }
};

double numberOr(const cJSON* o, const char* key, double def) {
    const cJSON* n = cJSON_GetObjectItem(o, key);
    return cJSON_IsNumber(n) ? n->valuedouble : def;
}

std::string stringOr(const cJSON* o, const char* key, const char* def) {
    const cJSON* n = cJSON_GetObjectItem(o, key);
    return cJSON_IsString(n) ? std::string(n->valuestring) : std::string(def);
}

// Why a pin is unusable on this board, or nullptr if it is fine. A bad table
// arrives over the air, so this has to reject rather than let gpio_config()
// take out the flash interface and brick the thing until someone gets a cable
// onto a bike.
//
// The PSRAM range is the one that bites: this is an ESP32-S3R8 built with
// CONFIG_SPIRAM_MODE_OCT, so the octal PSRAM interface consumes GPIO33-37 on
// top of the usual quad-SPI flash pins.
const char* gpioRejectReason(int pin) {
    if (pin < 0 || pin > 48)                              return "outside 0..48";
    if (!GPIO_IS_VALID_OUTPUT_GPIO(pin))                  return "not output-capable";
    if (pin == 15 || pin == 16)                           return "the CAN transceiver";
    if (pin >= 26 && pin <= 37)                           return "SPI flash / octal PSRAM";
    if (pin == 0 || pin == 3 || pin == 45 || pin == 46)   return "a strapping pin";
    if (pin == 19 || pin == 20)                           return "USB D-/D+";
    if (pin == 43 || pin == 44)                           return "the console UART";
    return nullptr;
}

}  // namespace

bool parseOutputLevel(const std::string& s, OutputBank::Level& out) {
    if (s == "on")     { out = OutputBank::Level::On;     return true; }
    if (s == "off")    { out = OutputBank::Level::Off;    return true; }
    if (s == "toggle") { out = OutputBank::Level::Toggle; return true; }
    return false;
}

// ---------------------------------------------------------------------------
// OutputBank
// ---------------------------------------------------------------------------

OutputBank::OutputBank() { lock_ = xSemaphoreCreateMutex(); }

OutputBank::~OutputBank() {
    if (lock_) {
        { LockGuard g(lock_); releaseLocked(); }
        vSemaphoreDelete(lock_);
    }
}

void OutputBank::driveLocked(const Output& o, bool on) {
    const int level = (on != o.activeLow) ? 1 : 0;
    for (gpio_num_t p : o.gpios) gpio_set_level(p, level);
}

void OutputBank::releaseLocked() {
    for (Output& o : outputs_) {
        driveLocked(o, false);
        for (gpio_num_t p : o.gpios) gpio_reset_pin(p);
    }
    outputs_.clear();
}

bool OutputBank::loadJson(const std::string& json, std::string& errorOut) {
    cJSON* root = cJSON_Parse(json.c_str());
    if (root == nullptr) { errorOut = "outputs: document does not parse"; return false; }

    const cJSON* sect = cJSON_GetObjectItem(root, "outputs");
    if (sect == nullptr) {
        // No outputs section at all: the shipped default, and not an error.
        cJSON_Delete(root);
        LockGuard g(lock_);
        releaseLocked();
        errorOut.clear();
        return true;
    }
    if (!cJSON_IsObject(sect)) {
        errorOut = "'outputs' must be an object";
        cJSON_Delete(root);
        return false;
    }

    std::vector<Output> built;
    std::vector<int>    claimed;

    const cJSON* item = nullptr;
    cJSON_ArrayForEach(item, sect) {
        if (item->string == nullptr) continue;
        if (item->string[0] == '_') continue;         // "_note" and friends
        if (!cJSON_IsObject(item)) {
            errorOut = std::string("output '") + item->string + "' must be an object";
            cJSON_Delete(root);
            return false;
        }

        Output o;
        o.name       = item->string;
        o.activeLow  = cJSON_IsTrue(cJSON_GetObjectItem(item, "active_low"));
        o.autoOffMs  = (uint32_t)numberOr(item, "auto_off_ms", 0);

        const cJSON* gpios = cJSON_GetObjectItem(item, "gpios");
        if (gpios != nullptr && !cJSON_IsArray(gpios)) {
            errorOut = o.name + ": 'gpios' must be an array";
            cJSON_Delete(root);
            return false;
        }
        const cJSON* g = nullptr;
        cJSON_ArrayForEach(g, gpios) {
            if (!cJSON_IsNumber(g)) {
                errorOut = o.name + ": 'gpios' must contain numbers";
                cJSON_Delete(root);
                return false;
            }
            const int pin = (int)g->valuedouble;
            if (const char* why = gpioRejectReason(pin)) {
                errorOut = o.name + ": GPIO" + std::to_string(pin) + " is " + why;
                cJSON_Delete(root);
                return false;
            }
            if (std::find(claimed.begin(), claimed.end(), pin) != claimed.end()) {
                errorOut = o.name + ": GPIO" + std::to_string(pin) + " is used twice";
                cJSON_Delete(root);
                return false;
            }
            claimed.push_back(pin);
            o.gpios.push_back((gpio_num_t)pin);
        }

        const cJSON* when = cJSON_GetObjectItem(item, "off_when");
        if (cJSON_IsObject(when)) {
            o.offWhenSignal = stringOr(when, "signal", "");
            o.offWhenIs     = stringOr(when, "is", "");
            if (o.offWhenSignal.empty() || o.offWhenIs.empty()) {
                errorOut = o.name + ": 'off_when' needs both 'signal' and 'is'";
                cJSON_Delete(root);
                return false;
            }
        }
        built.push_back(std::move(o));
    }
    cJSON_Delete(root);

    LockGuard g(lock_);
    releaseLocked();                       // drives old pins inactive first
    outputs_ = std::move(built);
    for (Output& o : outputs_) {
        // Order matters: park the pin at the inactive level *before* it becomes
        // an output, or an active-high relay gets a brief close as the driver
        // enables the pad. The pull is set to match, which also holds the line
        // during the window where the pin is floating on reset — though a
        // physical pull-down at the relay input is the real fix for that.
        for (gpio_num_t p : o.gpios) {
            gpio_reset_pin(p);
            gpio_set_level(p, o.activeLow ? 1 : 0);
            gpio_config_t cfg = {};
            cfg.pin_bit_mask = 1ULL << p;
            cfg.mode         = GPIO_MODE_OUTPUT;
            cfg.pull_up_en   = o.activeLow ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
            cfg.pull_down_en = o.activeLow ? GPIO_PULLDOWN_DISABLE : GPIO_PULLDOWN_ENABLE;
            cfg.intr_type    = GPIO_INTR_DISABLE;
            gpio_config(&cfg);
            gpio_set_level(p, o.activeLow ? 1 : 0);
        }
        o.on = false;
        ESP_LOGI(TAG, "output '%s': %u gpio(s), active_%s, auto_off %" PRIu32 "ms",
                 o.name.c_str(), (unsigned)o.gpios.size(),
                 o.activeLow ? "low" : "high", o.autoOffMs);
    }
    errorOut.clear();
    return true;
}

bool OutputBank::applyLocked(Output& o, bool on, const char* by, Event& evOut) {
    if (o.on == on) return false;
    driveLocked(o, on);
    o.on        = on;
    o.onSinceUs = on ? (uint64_t)esp_timer_get_time() : 0;
    o.offPending = false;
    evOut = {o.name, on, by};
    return true;
}

bool OutputBank::set(const std::string& name, Level level, const char* by) {
    Event ev;
    bool  changed = false;
    bool  found   = false;
    {
        LockGuard g(lock_);
        if (!enabled_) return false;
        for (Output& o : outputs_) {
            if (o.name != name) continue;
            found = true;
            const bool want = (level == Level::Toggle) ? !o.on : (level == Level::On);
            changed = applyLocked(o, want, by, ev);
            break;
        }
    }
    // Outside the lock on purpose: this ends up in MqttClient::publish, which
    // can block for seconds on a stalled network.
    if (changed && change_) change_(ev.name, ev.on, ev.by);
    return found;
}

void OutputBank::onSignal(const std::string& signal, const std::string& text) {
    LockGuard g(lock_);
    for (Output& o : outputs_) {
        if (o.on && !o.offWhenSignal.empty() &&
            o.offWhenSignal == signal && o.offWhenIs == text) {
            o.offPending = true;
        }
    }
}

void OutputBank::tick(uint64_t nowUs) {
    std::vector<Event> events;
    {
        LockGuard g(lock_);
        for (Output& o : outputs_) {
            if (!o.on) continue;
            const char* by = nullptr;
            if (o.offPending) {
                by = "off_when";
            } else if (o.autoOffMs != 0 &&
                       (nowUs - o.onSinceUs) >= (uint64_t)o.autoOffMs * 1000ULL) {
                by = "auto_off";
            }
            if (by == nullptr) continue;
            Event ev;
            if (applyLocked(o, false, by, ev)) events.push_back(ev);
        }
    }
    for (const Event& ev : events) {
        ESP_LOGW(TAG, "output '%s' off (%s)", ev.name.c_str(), ev.by);
        if (change_) change_(ev.name, ev.on, ev.by);
    }
}

void OutputBank::setEnabled(bool enabled) {
    std::vector<Event> events;
    {
        LockGuard g(lock_);
        if (enabled_ == enabled) return;
        enabled_ = enabled;
        if (!enabled) {
            for (Output& o : outputs_) {
                Event ev;
                if (applyLocked(o, false, "disabled", ev)) events.push_back(ev);
            }
        }
    }
    ESP_LOGW(TAG, "outputs %s", enabled ? "enabled" : "disabled - all forced off");
    for (const Event& ev : events) {
        if (change_) change_(ev.name, ev.on, ev.by);
    }
}

bool OutputBank::enabled() const { LockGuard g(lock_); return enabled_; }

bool OutputBank::has(const std::string& name) const {
    LockGuard g(lock_);
    for (const Output& o : outputs_) if (o.name == name) return true;
    return false;
}

size_t OutputBank::count() const { LockGuard g(lock_); return outputs_.size(); }

std::vector<std::pair<std::string, bool>> OutputBank::states() const {
    LockGuard g(lock_);
    std::vector<std::pair<std::string, bool>> out;
    out.reserve(outputs_.size());
    for (const Output& o : outputs_) out.emplace_back(o.name, o.on);
    return out;
}

// ---------------------------------------------------------------------------
// GestureEngine
// ---------------------------------------------------------------------------

GestureEngine::GestureEngine(OutputBank& outputs) : outputs_(outputs) {
    lock_ = xSemaphoreCreateMutex();
}

GestureEngine::~GestureEngine() {
    if (lock_) vSemaphoreDelete(lock_);
}

bool GestureEngine::loadJson(const std::string& json, std::string& errorOut) {
    cJSON* root = cJSON_Parse(json.c_str());
    if (root == nullptr) { errorOut = "gestures: document does not parse"; return false; }

    const cJSON* sect = cJSON_GetObjectItem(root, "gestures");
    if (sect == nullptr) {
        cJSON_Delete(root);
        LockGuard g(lock_);
        gestures_.clear();
        errorOut.clear();
        return true;
    }
    if (!cJSON_IsArray(sect)) {
        errorOut = "'gestures' must be an array";
        cJSON_Delete(root);
        return false;
    }

    std::vector<Gesture> built;
    const cJSON* item = nullptr;
    cJSON_ArrayForEach(item, sect) {
        if (!cJSON_IsObject(item)) {
            errorOut = "each gesture must be an object";
            cJSON_Delete(root);
            return false;
        }
        Gesture gd;
        gd.name      = stringOr(item, "name", "");
        gd.signal    = stringOr(item, "signal", "");
        gd.trigger   = stringOr(item, "trigger", "");
        gd.windowMs  = (uint32_t)numberOr(item, "window_ms", 1000);
        gd.maxHoldMs = (uint32_t)numberOr(item, "max_hold_ms", 600);
        gd.minGapMs  = (uint32_t)numberOr(item, "min_gap_ms", 60);
        if (gd.name.empty() || gd.signal.empty() || gd.trigger.empty()) {
            errorOut = "gesture needs 'name', 'signal' and 'trigger'";
            cJSON_Delete(root);
            return false;
        }
        if (gd.windowMs < 100 || gd.windowMs > 10000) {
            errorOut = gd.name + ": 'window_ms' must be 100..10000";
            cJSON_Delete(root);
            return false;
        }

        const cJSON* actions = cJSON_GetObjectItem(item, "actions");
        if (!cJSON_IsObject(actions)) {
            errorOut = gd.name + ": 'actions' must be an object keyed by click count";
            cJSON_Delete(root);
            return false;
        }
        const cJSON* a = nullptr;
        cJSON_ArrayForEach(a, actions) {
            if (a->string == nullptr || a->string[0] == '_') continue;
            char* end = nullptr;
            const long clicks = strtol(a->string, &end, 10);
            if (end == a->string || *end != '\0' || clicks < 1 || clicks > 10) {
                errorOut = gd.name + ": action key '" + a->string +
                           "' is not a click count of 1..10";
                cJSON_Delete(root);
                return false;
            }
            Action act;
            act.output = stringOr(a, "output", "");
            if (!parseOutputLevel(stringOr(a, "set", ""), act.level)) {
                errorOut = gd.name + ": action " + a->string +
                           " needs \"set\" of on, off or toggle";
                cJSON_Delete(root);
                return false;
            }
            // Checked here so a misspelled output is a 400 on upload rather
            // than a gesture that silently does nothing on the bike. This is
            // why outputs must be loaded before gestures.
            if (!outputs_.has(act.output)) {
                errorOut = gd.name + ": action " + a->string +
                           " targets unknown output '" + act.output + "'";
                cJSON_Delete(root);
                return false;
            }
            gd.actions.emplace_back((int)clicks, act);
        }
        built.push_back(std::move(gd));
    }
    cJSON_Delete(root);

    {
        LockGuard g(lock_);
        gestures_ = std::move(built);
    }
    for (const Gesture& gd : gestures_) {
        ESP_LOGI(TAG, "gesture '%s': %s == '%s', window %" PRIu32 "ms, %u action(s)",
                 gd.name.c_str(), gd.signal.c_str(), gd.trigger.c_str(),
                 gd.windowMs, (unsigned)gd.actions.size());
    }
    errorOut.clear();
    return true;
}

void GestureEngine::onSignal(const std::string& signal, const std::string& text,
                             uint64_t recvUs) {
    LockGuard g(lock_);
    for (Gesture& gd : gestures_) {
        if (gd.signal != signal) continue;

        // The first sighting establishes a baseline rather than counting as a
        // transition, which is what stops a canreset/mark from fabricating a
        // click. But if that first sighting is itself the trigger value, the
        // control really did just move into the pressed state, so fall through
        // and start the press — otherwise the leading edge of the very first
        // burst after boot is swallowed and a triple click counts as two.
        // (Measured: it does exactly that.) Booting with the control already
        // held is the harmless case, since the hold then exceeds max_hold_ms
        // and cancels.
        if (!gd.haveLast) {
            gd.haveLast = true;
            gd.lastText = text;
            if (text != gd.trigger) continue;
        } else {
            if (text == gd.lastText) continue;
            gd.lastText = text;
        }

        if (text == gd.trigger) {
            gd.pressed   = true;
            gd.pressedUs = recvUs;
            continue;
        }
        if (!gd.pressed) continue;
        gd.pressed = false;

        const uint64_t held = recvUs - gd.pressedUs;
        if (held > (uint64_t)gd.maxHoldMs * 1000ULL) {
            // Deliberate use of the control, not a click. Cancel anything in
            // progress so "hold it on" is a predictable way to abort a miscount.
            if (gd.clicks != 0) {
                ESP_LOGI(TAG, "gesture '%s': held %" PRIu32 "ms, sequence cancelled",
                         gd.name.c_str(), (uint32_t)(held / 1000ULL));
            }
            gd.clicks = 0;
            continue;
        }
        if (gd.clicks != 0 &&
            (recvUs - gd.lastReleaseUs) < (uint64_t)gd.minGapMs * 1000ULL) {
            continue;   // implausibly fast repeat; treat as contact bounce
        }
        gd.clicks++;
        gd.lastReleaseUs = recvUs;
    }
}

void GestureEngine::tick(uint64_t nowUs) {
    std::vector<Fired> fired;
    {
        LockGuard g(lock_);
        for (Gesture& gd : gestures_) {
            if (gd.clicks == 0 || gd.pressed) continue;
            if ((nowUs - gd.lastReleaseUs) < (uint64_t)gd.windowMs * 1000ULL) continue;

            Fired f;
            f.name      = gd.name;
            f.clicks    = gd.clicks;
            f.hasAction = false;
            f.level     = OutputBank::Level::Off;
            for (const auto& kv : gd.actions) {
                if (kv.first != gd.clicks) continue;
                f.hasAction = true;
                f.output    = kv.second.output;
                f.level     = kv.second.level;
                f.action    = f.output + "=" +
                              (kv.second.level == OutputBank::Level::On     ? "on"
                             : kv.second.level == OutputBank::Level::Off    ? "off"
                                                                            : "toggle");
                break;
            }
            gd.clicks = 0;
            fired.push_back(std::move(f));
        }
    }
    // Acting and reporting happen outside the lock: OutputBank::set() takes its
    // own, and the report callback publishes to MQTT.
    for (const Fired& f : fired) {
        ESP_LOGI(TAG, "gesture '%s': %d click(s) -> %s",
                 f.name.c_str(), f.clicks, f.hasAction ? f.action.c_str() : "(unmapped)");
        if (f.hasAction) {
            const std::string by = "gesture:" + std::to_string(f.clicks);
            outputs_.set(f.output, f.level, by.c_str());
        }
        if (report_) report_(f.name, f.clicks, f.action);
    }
}

size_t GestureEngine::count() const { LockGuard g(lock_); return gestures_.size(); }
