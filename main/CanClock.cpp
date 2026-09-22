// Standard library first, before anything that reaches IDF's hal/assert.h:
// under picolibc that header redefines __noreturn as [[noreturn]], and
// picolibc's stdlib.h then uses __noreturn as a trailing attribute, where it
// appertains to the type and is a hard error. Getting the C library in first
// means its own definition is already settled.
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>

#include <cJSON.h>

#include "esp_log.h"
#include "esp_timer.h"

#include "CanBus.h"
#include "CanClock.h"

namespace {

constexpr const char* TAG = "canclock";

struct LockGuard {
    SemaphoreHandle_t h;
    explicit LockGuard(SemaphoreHandle_t s) : h(s) { xSemaphoreTake(h, portMAX_DELAY); }
    ~LockGuard() { xSemaphoreGive(h); }
};

constexpr uint32_t kSecondsPerDay = 86400;

}  // namespace

CanClock::CanClock() { lock_ = xSemaphoreCreateMutex(); }

CanClock::~CanClock() {
    if (lock_) vSemaphoreDelete(lock_);
}

bool CanClock::loadJson(const std::string& json, std::string& errorOut) {
    cJSON* root = cJSON_Parse(json.c_str());
    if (root == nullptr) { errorOut = "clock: document does not parse"; return false; }

    const cJSON* sect = cJSON_GetObjectItem(root, "clock");
    if (sect == nullptr) {
        // No clock section: the shipped default until the counter is
        // identified, and not an error.
        cJSON_Delete(root);
        LockGuard g(lock_);
        configured_ = false;
        errorOut.clear();
        return true;
    }
    if (!cJSON_IsObject(sect)) {
        errorOut = "'clock' must be an object";
        cJSON_Delete(root);
        return false;
    }

    const cJSON* idItem = cJSON_GetObjectItem(sect, "id");
    if (!cJSON_IsString(idItem) || idItem->valuestring == nullptr) {
        errorOut = "clock: 'id' must be a hex string like \"3FF\"";
        cJSON_Delete(root);
        return false;
    }
    char* end = nullptr;
    const uint32_t id = (uint32_t)strtoul(idItem->valuestring, &end, 16);
    if (end == idItem->valuestring) {
        errorOut = std::string("clock: cannot parse id '") + idItem->valuestring + "'";
        cJSON_Delete(root);
        return false;
    }

    const cJSON* bytes = cJSON_GetObjectItem(sect, "bytes");
    if (!cJSON_IsArray(bytes)) {
        errorOut = "clock: 'bytes' must be an array";
        cJSON_Delete(root);
        return false;
    }
    uint8_t buf[4] = {};
    uint8_t n = 0;
    const cJSON* b = nullptr;
    cJSON_ArrayForEach(b, bytes) {
        if (!cJSON_IsNumber(b)) {
            errorOut = "clock: 'bytes' entries must be integers";
            cJSON_Delete(root);
            return false;
        }
        const int v = b->valueint;
        if (v < 0 || v >= (int)kCanMaxData) {
            errorOut = "clock: byte index out of range 0.." + std::to_string(kCanMaxData - 1);
            cJSON_Delete(root);
            return false;
        }
        // Four bytes is the widest that fits the uint32 the field is read into,
        // and nothing plausible needs more than three.
        if (n >= 4) {
            errorOut = "clock: at most 4 bytes";
            cJSON_Delete(root);
            return false;
        }
        buf[n++] = (uint8_t)v;
    }
    if (n == 0) {
        errorOut = "clock: 'bytes' is empty";
        cJSON_Delete(root);
        return false;
    }

    Mode mode = Mode::Observe;
    const cJSON* modeItem = cJSON_GetObjectItem(sect, "mode");
    if (cJSON_IsString(modeItem) && modeItem->valuestring != nullptr) {
        const std::string m = modeItem->valuestring;
        if (m == "observe") {
            mode = Mode::Observe;
        } else if (m == "tod") {
            // Reserved, not implemented, and rejected rather than accepted
            // quietly. Accepting it would be a setting that claims to set the
            // clock and does not — the same silent-no-op trap as a mismatched
            // register address, and it would be discovered on the bike months
            // later. Same policy as a bad output or gesture: fail at upload.
            //
            // What is missing is not the plumbing but a date. A
            // seconds-since-midnight field carries no day, so making this real
            // needs a date source (last-known epoch persisted to NVS, restored
            // on a coldboot) and an honest time_source on everything derived
            // from it. Not worth building against an unconfirmed hypothesis.
            errorOut = "clock: mode \"tod\" is reserved and not implemented yet - "
                       "a seconds-since-midnight field carries no date, so it needs "
                       "a persisted date source first. Use \"observe\" and read "
                       "clock_vs_local from /can/status";
            cJSON_Delete(root);
            return false;
        } else {
            errorOut = "clock: 'mode' must be \"observe\"";
            cJSON_Delete(root);
            return false;
        }
    }

    const bool ext = cJSON_IsTrue(cJSON_GetObjectItem(sect, "ext"));
    cJSON_Delete(root);

    {
        LockGuard g(lock_);
        configured_ = true;
        mode_       = mode;
        id_         = id;
        ext_        = ext;
        std::memcpy(bytes_, buf, sizeof(buf));
        nbytes_     = n;
        last_       = Reading{};
    }

    ESP_LOGI(TAG, "clock field: id 0x%03" PRIX32 " bytes %u wide, mode %s",
             id, (unsigned)n, mode == Mode::Tod ? "tod" : "observe");
    errorOut.clear();
    return true;
}

void CanClock::onFrame(const CanFrame& f) {
    LockGuard g(lock_);
    if (!configured_) return;
    if (f.id != id_ || f.ext != ext_) return;

    uint32_t v = 0;
    for (uint8_t i = 0; i < nbytes_; ++i) {
        const uint8_t idx = bytes_[i];
        if (idx >= f.len) return;          // short frame: not our field
        v = (v << 8) | f.data[idx];
    }

    if (last_.valid && last_.raw == v) {
        // Same second, next cyclic repeat. Keep the timestamp of the *first*
        // frame carrying this value: that is the instant the counter ticked,
        // and it is what makes the sub-second offset meaningful.
        return;
    }
    last_.valid   = true;
    last_.raw     = v;
    last_.atUs    = (uint64_t)esp_timer_get_time();
    last_.updates++;
}

bool CanClock::configured() const { LockGuard g(lock_); return configured_; }
CanClock::Mode CanClock::mode() const { LockGuard g(lock_); return mode_; }
uint32_t CanClock::id() const { LockGuard g(lock_); return id_; }

CanClock::Reading CanClock::reading() const {
    LockGuard g(lock_);
    return last_;
}

bool CanClock::timeOfDay(uint32_t& secondsOut) const {
    Reading r;
    {
        LockGuard g(lock_);
        if (!configured_ || !last_.valid) return false;
        r = last_;
    }
    if (r.raw >= kSecondsPerDay) return false;

    const uint64_t ageUs = (uint64_t)esp_timer_get_time() - r.atUs;
    secondsOut = (uint32_t)((r.raw + ageUs / 1000000ULL) % kSecondsPerDay);
    return true;
}

bool CanClock::offsetVsLocal(int& offsetSecondsOut, uint32_t& localTodOut) const {
    uint32_t tod = 0;
    if (!timeOfDay(tod)) return false;

    const time_t now = time(nullptr);
    if (now < 1700000000) return false;   // clock never set; nothing to compare

    struct tm lt;
    localtime_r(&now, &lt);
    const int localTod = lt.tm_hour * 3600 + lt.tm_min * 60 + lt.tm_sec;

    // Wrap into -43200..+43200 so a reading either side of midnight reads as a
    // small offset rather than a whole day out.
    int diff = (int)tod - localTod;
    if (diff >  43200) diff -= 86400;
    if (diff < -43200) diff += 86400;

    offsetSecondsOut = diff;
    localTodOut      = (uint32_t)localTod;
    return true;
}

std::string canClockElapsedString(uint32_t seconds) {
    char buf[40];
    const uint32_t d = seconds / 86400; seconds %= 86400;
    const uint32_t h = seconds / 3600;  seconds %= 3600;
    const uint32_t m = seconds / 60;
    snprintf(buf, sizeof(buf), "%" PRIu32 "d %02" PRIu32 "h %02" PRIu32 "m %02" PRIu32 "s",
             d, h, m, seconds % 60);
    return std::string(buf);
}

std::string canClockTodString(uint32_t s) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%02" PRIu32 ":%02" PRIu32 ":%02" PRIu32,
             (s / 3600) % 24, (s / 60) % 60, s % 60);
    return std::string(buf);
}
