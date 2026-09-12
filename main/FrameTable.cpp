#include "FrameTable.h"

#include <algorithm>
#include <cinttypes>
#include <cstdio>
#include <cstring>

#include "esp_log.h"

namespace {
constexpr const char* TAG = "frametable";

size_t roundUpPow2(size_t n) {
    size_t p = 1;
    while (p < n) p <<= 1;
    return p;
}

// Fibonacci hash: the 29-bit IDs on a vehicle bus cluster tightly (the K25
// reportedly uses ~19 of them), and a plain bitmask would collide badly.
size_t hashId(uint32_t id) {
    return static_cast<size_t>((id * 2654435761U) >> 16);
}

struct LockGuard {
    SemaphoreHandle_t h;
    explicit LockGuard(SemaphoreHandle_t s) : h(s) { xSemaphoreTake(h, portMAX_DELAY); }
    ~LockGuard() { xSemaphoreGive(h); }
};
}  // namespace

std::string toHex(const uint8_t* data, size_t len) {
    static const char* kDigits = "0123456789ABCDEF";
    std::string out;
    out.reserve(len * 2);
    for (size_t i = 0; i < len; ++i) {
        out.push_back(kDigits[data[i] >> 4]);
        out.push_back(kDigits[data[i] & 0x0F]);
    }
    return out;
}

std::string canIdHex(uint32_t id, bool ext) {
    char buf[12];
    snprintf(buf, sizeof(buf), ext ? "%08" PRIX32 : "%03" PRIX32, id);
    return std::string(buf);
}

std::string idsJson(const FrameTable& table, uint32_t bitrate) {
    std::vector<IdRecord> rows = table.snapshot();
    std::string out;
    out.reserve(rows.size() * 192 + 128);
    out += "{\"count\":" + std::to_string(rows.size()) +
           ",\"total_frames\":" + std::to_string(table.totalFrames()) +
           ",\"overflow\":" + std::to_string(table.overflowIds()) +
           ",\"bitrate\":" + std::to_string(bitrate) +
           ",\"ids\":[";
    bool first = true;
    for (const IdRecord& r : rows) {
        if (!first) out += ',';
        first = false;
        char head[176];
        snprintf(head, sizeof(head),
                 "{\"id\":\"0x%s\",\"ext\":%s,\"dlc\":%u,\"count\":%" PRIu32
                 ",\"changes\":%" PRIu32 ",\"published\":%" PRIu32 ",\"changed\":\"%02X\"",
                 canIdHex(r.id, r.ext).c_str(), r.ext ? "true" : "false",
                 (unsigned)r.len, r.count, r.changes, r.published, r.changedMask);
        out += head;
        out += ",\"data\":\"" + toHex(r.data, r.len) + "\"";
        // Bus time between first and last sighting, plus the mean period. A
        // 100ms-cyclic ID and a one-shot event ID look very different here,
        // which is a useful first classification when decoding.
        const uint64_t spanUs = r.lastSeenUs - r.firstSeenUs;
        out += ",\"span_ms\":" + std::to_string(spanUs / 1000ULL);
        if (r.count > 1) {
            out += ",\"period_ms\":" + std::to_string(spanUs / 1000ULL / (r.count - 1));
        }
        out += '}';
    }
    out += "]}";
    return out;
}

std::string dumpText(const std::vector<CanFrame>& frames) {
    std::string out;
    out.reserve(frames.size() * 44);
    for (const CanFrame& f : frames) {
        char line[96];
        snprintf(line, sizeof(line), "(%llu.%06llu) can0 %s#%s%s\n",
                 (unsigned long long)(f.timestampUs / 1000000ULL),
                 (unsigned long long)(f.timestampUs % 1000000ULL),
                 canIdHex(f.id, f.ext).c_str(),
                 f.rtr ? "R" : "",
                 toHex(f.data, f.len).c_str());
        out += line;
    }
    return out;
}

FrameTable::FrameTable(size_t maxIds, size_t dumpRingFrames) {
    lock_ = xSemaphoreCreateMutex();
    // 2x headroom keeps the load factor at/below 0.5 so linear probing stays
    // short even if the bus turns out to use far more IDs than expected.
    const size_t cap = roundUpPow2(std::max<size_t>(maxIds, 8) * 2);
    slots_.resize(cap);
    mask_ = cap - 1;
    ring_.resize(std::max<size_t>(dumpRingFrames, 16));
    ESP_LOGI(TAG, "%u ID slots, %u-frame dump ring (%u bytes)",
             (unsigned)cap, (unsigned)ring_.size(),
             (unsigned)(cap * sizeof(IdRecord) + ring_.size() * sizeof(CanFrame)));
}

FrameTable::~FrameTable() {
    if (lock_) vSemaphoreDelete(lock_);
}

// Caller holds the lock.
IdRecord* FrameTable::find(uint32_t id, bool ext) {
    size_t i = hashId(id) & mask_;
    for (size_t probes = 0; probes <= mask_; ++probes) {
        IdRecord& r = slots_[i];
        if (!r.used) {
            r.used = true;
            r.id   = id;
            r.ext  = ext;
            return &r;
        }
        if (r.id == id && r.ext == ext) return &r;
        i = (i + 1) & mask_;
    }
    return nullptr;  // table full
}

bool FrameTable::observe(const CanFrame& f, uint32_t minIntervalMs, uint32_t heartbeatMs,
                         uint8_t* changedMaskOut) {
    LockGuard g(lock_);
    totalFrames_++;

    // Raw ring first, so /can/dump reflects everything regardless of the
    // publish decision below.
    ring_[ringHead_] = f;
    ringHead_ = (ringHead_ + 1) % ring_.size();
    if (ringCount_ < ring_.size()) ringCount_++;

    IdRecord* r = find(f.id, f.ext);
    if (r == nullptr) {
        overflowIds_++;
        return false;
    }

    const bool  first   = (r->count == 0);
    const bool  differs = first || r->len != f.len ||
                          std::memcmp(r->data, f.data, kCanMaxData) != 0;

    if (!first) {
        for (size_t i = 0; i < kCanMaxData; ++i) {
            if (r->data[i] != f.data[i]) r->changedMask |= (1u << i);
        }
    }

    r->len = f.len;
    std::memcpy(r->data, f.data, kCanMaxData);
    r->count++;
    if (differs && !first) r->changes++;
    // All of this is the monotonic clock, never the hardware RX timestamp: that
    // one wraps every ~72 minutes, which would corrupt period_ms and fire
    // spurious heartbeats partway through any long session. See CanFrame.
    if (first) r->firstSeenUs = f.recvUs;
    r->lastSeenUs = f.recvUs;
    if (changedMaskOut) *changedMaskOut = r->changedMask;

    // A brand-new ID always publishes: that first sighting is the interesting
    // event, and waiting for it to change would hide a static ID entirely.
    const bool payloadNew = first || std::memcmp(r->prev, f.data, kCanMaxData) != 0;
    const uint64_t sinceUs = f.recvUs - r->lastPubUs;
    const bool throttled = !first && sinceUs < static_cast<uint64_t>(minIntervalMs) * 1000ULL;
    const bool heartbeat = heartbeatMs > 0 && !first &&
                           sinceUs >= static_cast<uint64_t>(heartbeatMs) * 1000ULL;

    if ((payloadNew && !throttled) || heartbeat) {
        std::memcpy(r->prev, f.data, kCanMaxData);
        r->lastPubUs = f.recvUs;
        r->published++;
        return true;
    }
    return false;
}

std::vector<IdRecord> FrameTable::snapshot() const {
    LockGuard g(lock_);
    std::vector<IdRecord> out;
    for (const IdRecord& r : slots_) {
        if (r.used) out.push_back(r);
    }
    std::sort(out.begin(), out.end(),
              [](const IdRecord& a, const IdRecord& b) { return a.id < b.id; });
    return out;
}

std::vector<CanFrame> FrameTable::recentFrames(size_t limit) const {
    LockGuard g(lock_);
    const size_t n = (limit == 0 || limit > ringCount_) ? ringCount_ : limit;
    std::vector<CanFrame> out;
    out.reserve(n);
    // ringHead_ is the next write slot, so the oldest of the n most recent
    // frames sits n places behind it.
    size_t i = (ringHead_ + ring_.size() - n) % ring_.size();
    for (size_t k = 0; k < n; ++k) {
        out.push_back(ring_[i]);
        i = (i + 1) % ring_.size();
    }
    return out;
}

void FrameTable::reset() {
    LockGuard g(lock_);
    std::fill(slots_.begin(), slots_.end(), IdRecord{});
    ringHead_    = 0;
    ringCount_   = 0;
    totalFrames_ = 0;
    overflowIds_ = 0;
    ESP_LOGW(TAG, "frame table and dump ring cleared");
}

size_t FrameTable::trackedIds() const {
    LockGuard g(lock_);
    size_t n = 0;
    for (const IdRecord& r : slots_) {
        if (r.used) n++;
    }
    return n;
}
