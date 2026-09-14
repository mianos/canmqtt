#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "CanBus.h"

// What we know about one CAN ID seen on the bus.
struct IdRecord {
    uint32_t id          = 0;
    bool     used        = false;
    bool     ext         = false;
    uint8_t  len         = 0;
    uint8_t  data[kCanMaxData] = {};  // most recent payload
    uint8_t  prev[kCanMaxData] = {};  // payload as of the last publish

    uint32_t count       = 0;   // frames seen
    uint32_t changes     = 0;   // times the payload differed from the previous frame
    uint32_t published   = 0;   // times we emitted this ID

    // Bit N set => byte N has been observed to change at least once. This is
    // the single most useful column when reverse engineering: it separates the
    // live signal bytes from constant padding and counters.
    uint8_t  changedMask = 0;

    uint64_t firstSeenUs = 0;
    uint64_t lastSeenUs  = 0;
    uint64_t lastPubUs   = 0;
};

// An operator-supplied label dropped into the capture, e.g. "test high beam",
// pushed just before performing the action it names. Decoding works by
// correlation, and without these the record of *which* action produced a change
// exists only in the operator's head.
inline constexpr size_t kMarkTextMax = 63;

struct Mark {
    uint64_t recvUs = 0;   // monotonic, the same clock as CanFrame::recvUs
    uint32_t seq    = 0;   // unique for the life of the boot; survives reset()
    char     text[kMarkTextMax + 1] = {};
};

// Live per-ID view of the bus, plus a rolling window of raw frames.
//
// observe() runs on the CanBus worker task and decides whether a frame is worth
// publishing; the snapshot accessors run on HTTP worker threads. A single mutex
// covers both tables — contention is a non-issue because the web handlers are
// the only other toucher and they run rarely.
//
// The publish policy exists because a 500kbit/s bus can carry thousands of
// frames per second: emitting each one would flatten the broker and bury the
// signal. Instead an ID is published when its payload *changes* (floored at
// minIntervalMs per ID) or when its heartbeat falls due. Same decimation idea
// as mqttradar's presencePeriodSec, applied per CAN ID.
class FrameTable {
public:
    FrameTable(size_t maxIds, size_t dumpRingFrames);
    ~FrameTable();

    // Fold a frame in. Returns true if the caller should publish it, and (when
    // non-null) hands back that ID's accumulated changed-byte mask so the
    // publisher doesn't have to re-find the record.
    bool observe(const CanFrame& f, uint32_t minIntervalMs, uint32_t heartbeatMs,
                 uint8_t* changedMaskOut = nullptr);

    // Copy of every populated record, ascending by ID.
    std::vector<IdRecord> snapshot() const;

    // Raw frames, oldest first, capped at `limit` (0 = all retained).
    std::vector<CanFrame> recentFrames(size_t limit = 0) const;

    // Record an operator label at `recvUs` and return the sequence number it
    // was given. Text longer than kMarkTextMax is truncated.
    uint32_t mark(const char* text, uint64_t recvUs);

    // Retained labels, oldest first — mirrors recentFrames().
    std::vector<Mark> recentMarks() const;

    // Forget everything, labels included: a label pointing at frames that have
    // been discarded misleads more than it helps. Sequence numbers are *not*
    // rewound, so a seq identifies one label for the whole boot.
    void reset();

    size_t   trackedIds() const;
    uint32_t totalFrames() const { return totalFrames_; }
    uint32_t overflowIds() const { return overflowIds_; }

private:
    // Open addressing with linear probing on a power-of-two table: no
    // allocation on the hot path, and an ID's slot is stable for its lifetime.
    IdRecord* find(uint32_t id, bool ext);

    mutable SemaphoreHandle_t lock_;
    std::vector<IdRecord>     slots_;
    size_t                    mask_ = 0;

    std::vector<CanFrame> ring_;
    size_t                ringHead_  = 0;
    size_t                ringCount_ = 0;

    // Far rarer than frames, so a small fixed ring is plenty.
    static constexpr size_t kMarkRing = 32;
    Mark     marks_[kMarkRing] = {};
    size_t   markHead_  = 0;
    size_t   markCount_ = 0;
    uint32_t markSeq_   = 0;

    uint32_t totalFrames_ = 0;
    uint32_t overflowIds_ = 0;  // frames dropped because the ID table was full
};

// "00A1FF00" — uppercase, no separators, candump-compatible.
std::string toHex(const uint8_t* data, size_t len);

// "2BC" for standard frames, "18DAF110" for extended — the widths candump uses.
std::string canIdHex(uint32_t id, bool ext);

// Report bodies. These live here rather than in the HTTP handlers because they
// are views of the table, not web concerns — which also makes them callable
// from a bench self-check.
//
// idsJson: the per-ID table. Assembled as a string rather than through
// JsonWrapper because JsonWrapper has no array or nested-object support (see
// the static_assert in its addItemInternal).
std::string idsJson(const FrameTable& table, uint32_t bitrate);

// dumpText: `(sec.usec) can0 2BC#00A1FF00` per line, candump's format, with any
// operator labels merged in as `# MARK <seq> <text>` at their place in the
// sequence. Both inputs must be ordered by recvUs, which recentFrames() and
// recentMarks() both are.
//
// Labels carry no timestamp of their own on purpose: the frame timestamps above
// come from the TWAI hardware counter, which wraps about every 72 minutes, and
// printing a second unrelated clock into the same file invites misreading.
// Position in the stream is the information. Use the `up_ms` field on the MQTT
// `mark` topic to line a capture up against wall time.
std::string dumpText(const std::vector<CanFrame>& frames,
                     const std::vector<Mark>& marks = {});
