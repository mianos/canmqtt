#pragma once

#include <cstdint>
#include <functional>

#include "esp_err.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "hal/gpio_types.h"

// Max payload bytes we ever store per frame. The ESP32-S3 has no
// SOC_TWAI_SUPPORT_FD, so classic CAN's 8 is the hard ceiling here — a longer
// frame is not representable on this silicon.
inline constexpr size_t kCanMaxData = 8;

// One received CAN frame, flattened out of twai_frame_t by the RX ISR. Kept
// small because every frame is memcpy'd through a FreeRTOS queue from ISR
// context.
//
// Two clocks, on purpose:
//   timestampUs — the controller's hardware RX timestamp. Precise, so it is the
//                 right thing for inter-frame timing in /can/dump. But it comes
//                 off a 1MHz counter that wraps roughly every 72 minutes, so it
//                 must NOT be used for any duration arithmetic.
//   recvUs      — esp_timer_get_time(), captured in the ISR. Monotonic 64-bit
//                 microseconds since boot (~584,000 years of range), so all the
//                 publish-throttle, heartbeat and period maths uses this.
struct CanFrame {
    uint64_t timestampUs;        // hardware RX timestamp (1MHz, wraps ~72min)
    uint64_t recvUs;             // monotonic since boot — use this for durations
    uint32_t id;                 // 11- or 29-bit arbitration ID
    uint8_t  data[kCanMaxData];
    uint8_t  len;                // real payload length, from twaifd_dlc2len()
    bool     ext;                // 29-bit extended ID
    bool     rtr;                // remote-transmission-request frame
};

// Counters that only the ISR writes and only reporting reads. Not atomic:
// they're monotonic uint32 on a 32-bit target, so a torn read is impossible and
// a stale read is harmless for telemetry.
struct CanBusCounters {
    uint32_t framesRx;   // frames handed to the worker task
    uint32_t drops;      // frames lost because the queue was full
    uint32_t rxErrors;   // on_error callbacks
    uint32_t stateChanges;
};

// Passive monitor for the on-chip TWAI controller.
//
// Built on the node-based esp_twai.h API (ESP-IDF >= 5.4; the older
// driver/twai.h model is deprecated in v6 and cannot express this). The API's
// shape dictates the architecture: twai_node_receive_from_isr() is callable
// ONLY from inside the on_rx_done callback — there is no task-level receive —
// so the ISR flattens each frame into a FreeRTOS queue and a worker task drains
// it. The queue lives in internal DRAM (xQueueCreate's default), which is what
// we want: a cache-safe ISR may not touch PSRAM.
//
// In listen-only mode the controller neither transmits nor acknowledges, at the
// hardware level. That is the whole point on a vehicle bus.
class CanBus {
public:
    // Called on the worker task for every received frame, in arrival order.
    using FrameHandler = std::function<void(const CanFrame&)>;

    CanBus(gpio_num_t tx, gpio_num_t rx);
    ~CanBus();

    // Bring up the controller and start the worker task. queueDepth frames of
    // ISR-to-task buffering; selfTest enables the controller's self-test flag
    // so transmit() does not require an acknowledgement from another node
    // (bench use only, and mutually exclusive with listenOnly).
    esp_err_t start(uint32_t bitrate, bool listenOnly, size_t queueDepth,
                    FrameHandler handler, bool selfTest = false);

    // Change bit rate live: disable -> reconfig timing -> enable. The node must
    // be disabled to retime it, so a few frames are missed across the switch.
    esp_err_t setBitrate(uint32_t bitrate);

    // Transmit a frame, blocking until the hardware has finished with it (the
    // driver keeps only pointers to the frame and its buffer — see the .cpp).
    // Fails with ESP_ERR_NOT_SUPPORTED when listen-only, which is the desired
    // outcome everywhere except the bench self-test.
    esp_err_t transmit(uint32_t id, bool ext, const uint8_t* data, size_t len);

    // Live controller state: error state, tx/rx error counts, cumulative bus
    // errors. Reading frames alongside a flat bus_err_num means the bit rate is
    // right; zero frames with a climbing count means it is wrong.
    esp_err_t info(twai_node_status_t& status, twai_node_record_t& record) const;

    const CanBusCounters& counters() const { return counters_; }
    uint32_t bitrate() const { return bitrate_; }
    bool listenOnly() const { return listenOnly_; }

private:
    static bool IRAM_ATTR onRxDone(twai_node_handle_t handle,
                                   const twai_rx_done_event_data_t* edata, void* ctx);
    static bool IRAM_ATTR onError(twai_node_handle_t handle,
                                  const twai_error_event_data_t* edata, void* ctx);
    static bool IRAM_ATTR onStateChange(twai_node_handle_t handle,
                                        const twai_state_change_event_data_t* edata, void* ctx);
    static void workerTask(void* arg);

    esp_err_t createNode(uint32_t bitrate, bool listenOnly, bool selfTest);

    gpio_num_t         tx_;
    gpio_num_t         rx_;
    twai_node_handle_t node_ = nullptr;
    QueueHandle_t      queue_ = nullptr;
    TaskHandle_t       task_ = nullptr;
    FrameHandler       handler_;

    // TX staging. Must outlive the queued transfer because the driver holds
    // pointers into it, and is serialised by txLock_ so only one transfer is
    // ever in flight against it.
    SemaphoreHandle_t  txLock_ = nullptr;
    twai_frame_t       txFrame_ = {};
    uint8_t            txBuf_[kCanMaxData] = {};

    uint32_t           bitrate_ = 0;
    bool               listenOnly_ = true;
    bool               selfTest_ = false;
    CanBusCounters     counters_ = {};
};

// "active" / "warning" / "passive" / "bus_off", for logs and telemetry.
const char* canErrorStateName(twai_error_state_t s);
