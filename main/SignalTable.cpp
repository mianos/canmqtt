#include "SignalTable.h"

#include <cmath>
#include <cstdio>
#include <cstring>

#include "cJSON.h"
#include "esp_log.h"
#include "esp_spiffs.h"

namespace {
constexpr const char* TAG = "signals";

struct LockGuard {
    SemaphoreHandle_t h;
    explicit LockGuard(SemaphoreHandle_t s) : h(s) { xSemaphoreTake(h, portMAX_DELAY); }
    ~LockGuard() { xSemaphoreGive(h); }
};

int8_t parseNibble(const cJSON* o) {
    const cJSON* n = cJSON_GetObjectItem(o, "nibble");
    if (!cJSON_IsString(n)) return -1;
    if (strcmp(n->valuestring, "high") == 0) return 1;
    if (strcmp(n->valuestring, "low") == 0) return 0;
    return -1;
}

double numberOr(const cJSON* o, const char* key, double def) {
    const cJSON* n = cJSON_GetObjectItem(o, key);
    return cJSON_IsNumber(n) ? n->valuedouble : def;
}

std::string stringOr(const cJSON* o, const char* key, const char* def) {
    const cJSON* n = cJSON_GetObjectItem(o, key);
    return cJSON_IsString(n) ? std::string(n->valuestring) : std::string(def);
}

// Uppercase hex without leading zeros beyond what the width needs. Enum map
// keys in the JSON are written the way a human reads them off a dump ("0F",
// "CF", "B"), so nibbles format as one digit and bytes as two.
std::string hexKey(uint32_t v, int digits) {
    char buf[12];
    snprintf(buf, sizeof(buf), "%0*X", digits, (unsigned)v);
    return std::string(buf);
}
}  // namespace

SignalTable::SignalTable() { lock_ = xSemaphoreCreateMutex(); }

SignalTable::~SignalTable() {
    if (lock_) vSemaphoreDelete(lock_);
}

bool SignalTable::loadJson(const std::string& json, std::string& errorOut) {
    cJSON* root = cJSON_Parse(json.c_str());
    if (root == nullptr) {
        const char* e = cJSON_GetErrorPtr();
        errorOut = std::string("parse error near: ") + (e ? std::string(e).substr(0, 40) : "?");
        return false;
    }

    const int base = (int)numberOr(root, "byte_base", 1);
    if (base != 0 && base != 1) {
        errorOut = "byte_base must be 0 or 1";
        cJSON_Delete(root);
        return false;
    }

    const cJSON* framesJson = cJSON_GetObjectItem(root, "frames");
    if (!cJSON_IsObject(framesJson)) {
        errorOut = "missing 'frames' object";
        cJSON_Delete(root);
        return false;
    }

    std::unordered_map<uint32_t, FrameDef> built;
    const cJSON* frameJson = nullptr;
    cJSON_ArrayForEach(frameJson, framesJson) {
        if (frameJson->string == nullptr) continue;
        const uint32_t id = (uint32_t)strtoul(frameJson->string, nullptr, 16);

        FrameDef fd;
        fd.name = stringOr(frameJson, "name", frameJson->string);

        const cJSON* sigs = cJSON_GetObjectItem(frameJson, "signals");
        if (!cJSON_IsArray(sigs)) continue;

        const cJSON* sigJson = nullptr;
        cJSON_ArrayForEach(sigJson, sigs) {
            SignalDef sd;
            sd.name     = stringOr(sigJson, "name", "");
            if (sd.name.empty()) continue;
            sd.unit     = stringOr(sigJson, "unit", "");
            sd.nibble   = parseNibble(sigJson);
            sd.scale    = numberOr(sigJson, "scale", 1.0);
            sd.offset   = numberOr(sigJson, "offset", 0.0);
            sd.decimals = (int)numberOr(sigJson, "decimals", 0);
            sd.deadband = numberOr(sigJson, "deadband", 0.0);
            sd.minMs    = (uint32_t)numberOr(sigJson, "min_ms", 0);

            const cJSON* bytes = cJSON_GetObjectItem(sigJson, "bytes");
            if (cJSON_IsArray(bytes)) {
                const cJSON* b = nullptr;
                cJSON_ArrayForEach(b, bytes) {
                    if (!cJSON_IsNumber(b)) continue;
                    const int idx = (int)b->valuedouble - base;
                    if (idx < 0 || idx >= (int)kCanMaxData) {
                        errorOut = sd.name + ": byte index out of range for byte_base " +
                                   std::to_string(base);
                        cJSON_Delete(root);
                        return false;
                    }
                    sd.bytes.push_back((uint8_t)idx);
                }
            }

            const cJSON* parts = cJSON_GetObjectItem(sigJson, "parts");
            if (cJSON_IsArray(parts)) {
                const cJSON* p = nullptr;
                cJSON_ArrayForEach(p, parts) {
                    Part pt;
                    const int idx = (int)numberOr(p, "byte", -1) - base;
                    if (idx < 0 || idx >= (int)kCanMaxData) {
                        errorOut = sd.name + ": part byte index out of range";
                        cJSON_Delete(root);
                        return false;
                    }
                    pt.byteIdx = (uint8_t)idx;
                    pt.nibble  = parseNibble(p);
                    sd.parts.push_back(pt);
                }
            }

            const cJSON* map = cJSON_GetObjectItem(sigJson, "map");
            if (cJSON_IsObject(map)) {
                const cJSON* m = nullptr;
                cJSON_ArrayForEach(m, map) {
                    if (m->string && cJSON_IsString(m)) {
                        sd.map[m->string] = m->valuestring;
                    }
                }
            }

            if (sd.bytes.empty() && sd.parts.empty()) continue;
            fd.signals.push_back(std::move(sd));
        }
        if (!fd.signals.empty()) built[id] = std::move(fd);
    }
    cJSON_Delete(root);

    if (built.empty()) {
        errorOut = "no usable frames in table";
        return false;
    }

    size_t nsig = 0;
    for (const auto& kv : built) nsig += kv.second.signals.size();

    {
        LockGuard g(lock_);
        frames_   = std::move(built);
        byteBase_ = base;
        loaded_   = true;
    }
    ESP_LOGI(TAG, "loaded %u frames / %u signals (byte_base %d)",
             (unsigned)frames_.size(), (unsigned)nsig, base);
    errorOut.clear();
    return true;
}

bool SignalTable::nibbleOf(const Part& p, const CanFrame& f, uint8_t& out) {
    if (p.byteIdx >= f.len) return false;
    const uint8_t b = f.data[p.byteIdx];
    out = (p.nibble == 1) ? (b >> 4) : (p.nibble == 0) ? (b & 0x0F) : b;
    return true;
}

bool SignalTable::rawValue(const SignalDef& s, const CanFrame& f, uint32_t& out) {
    uint32_t v = 0;
    for (size_t i = 0; i < s.bytes.size(); ++i) {
        if (s.bytes[i] >= f.len) return false;   // frame shorter than the table expects
        v |= (uint32_t)f.data[s.bytes[i]] << (8 * i);  // little-endian: first byte is LSB
    }
    if (s.bytes.size() == 1) {
        if (s.nibble == 1) v >>= 4;
        else if (s.nibble == 0) v &= 0x0F;
    }
    out = v;
    return true;
}

std::vector<DecodedSignal> SignalTable::decode(const CanFrame& f) {
    std::vector<DecodedSignal> out;
    LockGuard g(lock_);

    auto it = frames_.find(f.id);
    if (it == frames_.end()) return out;

    for (SignalDef& s : it->second.signals) {
        std::string key;
        uint32_t raw = 0;

        if (!s.parts.empty()) {
            // Composite key: each part rendered as one hex digit (nibble) or
            // two (whole byte), joined with "," to match the JSON map keys.
            bool ok = true;
            for (size_t i = 0; i < s.parts.size(); ++i) {
                uint8_t nv = 0;
                if (!nibbleOf(s.parts[i], f, nv)) { ok = false; break; }
                if (i) key += ',';
                key += hexKey(nv, s.parts[i].nibble < 0 ? 2 : 1);
            }
            if (!ok) continue;
        } else {
            if (!rawValue(s, f, raw)) continue;
            if (!s.map.empty()) {
                key = hexKey(raw, s.nibble < 0 ? (int)(s.bytes.size() * 2) : 1);
            }
        }

        const bool isEnum = !s.map.empty();
        const uint64_t now = f.recvUs;   // monotonic; never the wrapping hw stamp

        if (isEnum) {
            auto m = s.map.find(key);
            // An unmapped code is still worth surfacing -- it means the bike is
            // using a value the table does not know about, which is exactly the
            // kind of gap this project exists to close.
            const std::string text = (m != s.map.end()) ? m->second : ("unmapped_0x" + key);
            if (s.haveLast && text == s.lastText) continue;
            if (s.haveLast && s.minMs &&
                (now - s.lastPubUs) < (uint64_t)s.minMs * 1000ULL) continue;
            s.haveLast  = true;
            s.lastText  = text;
            s.lastPubUs = now;
            out.push_back({&s.name, &s.unit, text, 0.0, true});
        } else {
            double v = (double)raw * s.scale + s.offset;
            if (s.decimals >= 0) {
                const double p = std::pow(10.0, s.decimals);
                v = std::round(v * p) / p;
            }
            if (s.haveLast && std::fabs(v - s.lastValue) <= s.deadband) continue;
            if (s.haveLast && s.minMs &&
                (now - s.lastPubUs) < (uint64_t)s.minMs * 1000ULL) continue;
            s.haveLast  = true;
            s.lastValue = v;
            s.lastPubUs = now;
            out.push_back({&s.name, &s.unit, std::string(), v, false});
        }
    }
    return out;
}

void SignalTable::resetState() {
    LockGuard g(lock_);
    for (auto& kv : frames_) {
        for (SignalDef& s : kv.second.signals) {
            s.haveLast  = false;
            s.lastValue = 0.0;
            s.lastText.clear();
            s.lastPubUs = 0;
        }
    }
}

size_t SignalTable::frameCount() const {
    LockGuard g(lock_);
    return frames_.size();
}

size_t SignalTable::signalCount() const {
    LockGuard g(lock_);
    size_t n = 0;
    for (const auto& kv : frames_) n += kv.second.signals.size();
    return n;
}

std::vector<uint32_t> SignalTable::knownIds() const {
    LockGuard g(lock_);
    std::vector<uint32_t> ids;
    ids.reserve(frames_.size());
    for (const auto& kv : frames_) ids.push_back(kv.first);
    return ids;
}

// --------------------------------------------------------------------------
// signalstore: the SPIFFS-backed copy of signals.json
// --------------------------------------------------------------------------

// Build-time copy of data/signals.json, embedded so a freshly flashed board
// decodes immediately instead of needing an upload first.
extern const char signals_json_start[] asm("_binary_signals_json_start");
extern const char signals_json_end[]   asm("_binary_signals_json_end");

namespace signalstore {
namespace {
constexpr const char* kBase = "/signals";
constexpr const char* kPath = "/signals/signals.json";
bool s_mounted = false;
}  // namespace

std::string defaultJson() {
    return std::string(signals_json_start, (size_t)(signals_json_end - signals_json_start));
}

bool mount() {
    if (s_mounted) return true;
    esp_vfs_spiffs_conf_t conf = {};
    conf.base_path              = kBase;
    conf.partition_label        = "signals";
    conf.max_files              = 2;
    conf.format_if_mount_failed = true;   // a blank partition on a new board

    esp_err_t err = esp_vfs_spiffs_register(&conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spiffs mount: %s", esp_err_to_name(err));
        return false;
    }
    s_mounted = true;
    size_t total = 0, used = 0;
    if (esp_spiffs_info("signals", &total, &used) == ESP_OK) {
        ESP_LOGI(TAG, "spiffs mounted: %u/%u bytes used", (unsigned)used, (unsigned)total);
    }
    return true;
}

std::string read() {
    if (!s_mounted) return std::string();
    FILE* fp = fopen(kPath, "rb");
    if (!fp) return std::string();
    std::string out;
    char buf[512];
    size_t n;
    while ((n = fread(buf, 1, sizeof(buf), fp)) > 0) out.append(buf, n);
    fclose(fp);
    return out;
}

bool write(const std::string& s, std::string& errorOut) {
    if (!s_mounted) { errorOut = "filesystem not mounted"; return false; }
    // Write to a temporary file and rename, so an interrupted upload cannot
    // leave a truncated table behind -- the previous one stays usable.
    const char* tmp = "/signals/signals.tmp";
    FILE* fp = fopen(tmp, "wb");
    if (!fp) { errorOut = "cannot open temp file"; return false; }
    const size_t wrote = fwrite(s.data(), 1, s.size(), fp);
    fclose(fp);
    if (wrote != s.size()) {
        remove(tmp);
        errorOut = "short write (filesystem full?)";
        return false;
    }
    remove(kPath);
    if (rename(tmp, kPath) != 0) {
        remove(tmp);
        errorOut = "rename failed";
        return false;
    }
    errorOut.clear();
    return true;
}

bool ensureDefault(std::string& errorOut) {
    if (!read().empty()) return true;
    ESP_LOGW(TAG, "no stored table; writing the built-in default");
    return write(defaultJson(), errorOut);
}

}  // namespace signalstore
