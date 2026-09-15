#include "CanBus.h"

#include <cinttypes>
#include <cstring>

#include "esp_heap_caps.h"
#include "esp_memory_utils.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"

namespace {
constexpr const char* TAG = "canbus";

// Every frame is memcpy'd through the queue from ISR context, so the size is
// part of the design rather than an accident — 256 frames is 8KB of internal
// DRAM. Fails the build if a field is added carelessly.
static_assert(sizeof(CanFrame) == 32, "CanFrame grew; re-check RX queue sizing");

// Worker task: priority above the telemetry/web tasks (they're 4-5) so a burst
// of frames drains promptly, but below the TWAI ISR itself.
constexpr int kWorkerPriority = 6;
constexpr int kWorkerStack    = 4096;

const char* stateName(twai_error_state_t s) {
    switch (s) {
        case TWAI_ERROR_ACTIVE:  return "active";
        case TWAI_ERROR_WARNING: return "warning";
        case TWAI_ERROR_PASSIVE: return "passive";
        case TWAI_ERROR_BUS_OFF: return "bus_off";
        default:                 return "unknown";
    }
}
}  // namespace

CanBus::CanBus(gpio_num_t tx, gpio_num_t rx) : tx_(tx), rx_(rx) {
    txLock_ = xSemaphoreCreateMutex();
}

CanBus::~CanBus() {
    if (node_) {
        twai_node_disable(node_);
        twai_node_delete(node_);
    }
    if (task_)   vTaskDelete(task_);
    if (queue_)  vQueueDeleteWithCaps(queue_);
    if (txLock_) vSemaphoreDelete(txLock_);
}

// RX ISR. Mirrors the pattern in ESP-IDF's twai_utils dump example: build a
// local frame pointing at a local buffer, receive into it, push a flattened
// copy onto the queue. No allocation, no logging, no blocking.
bool IRAM_ATTR CanBus::onRxDone(twai_node_handle_t handle,
                                const twai_rx_done_event_data_t*, void* ctx) {
    auto* self = static_cast<CanBus*>(ctx);
    if (self == nullptr || self->queue_ == nullptr) return false;

    uint8_t      buf[kCanMaxData];
    twai_frame_t rx = {};
    rx.buffer     = buf;
    rx.buffer_len = sizeof(buf);

    if (twai_node_receive_from_isr(handle, &rx) != ESP_OK) return false;

    CanFrame f;
    f.timestampUs = rx.header.timestamp;
    f.recvUs      = (uint64_t)esp_timer_get_time();  // monotonic; see CanFrame
    f.id          = rx.header.id;
    f.ext         = rx.header.ide != 0;
    f.rtr         = rx.header.rtr != 0;

    uint16_t n = twaifd_dlc2len(rx.header.dlc);
    if (n > kCanMaxData) n = kCanMaxData;  // unreachable without FD; belt and braces
    f.len = static_cast<uint8_t>(n);
    std::memcpy(f.data, buf, n);
    if (n < kCanMaxData) std::memset(f.data + n, 0, kCanMaxData - n);

    BaseType_t woken = pdFALSE;
    if (xQueueSendFromISR(self->queue_, &f, &woken) == pdTRUE) {
        self->counters_.framesRx++;
    } else {
        self->counters_.drops++;
    }
    return woken == pdTRUE;
}

bool IRAM_ATTR CanBus::onError(twai_node_handle_t, const twai_error_event_data_t*, void* ctx) {
    auto* self = static_cast<CanBus*>(ctx);
    if (self) self->counters_.rxErrors++;
    return false;
}

bool IRAM_ATTR CanBus::onStateChange(twai_node_handle_t,
                                     const twai_state_change_event_data_t*, void* ctx) {
    auto* self = static_cast<CanBus*>(ctx);
    if (self) self->counters_.stateChanges++;
    return false;
}

void CanBus::workerTask(void* arg) {
    auto* self = static_cast<CanBus*>(arg);
    CanFrame f;
    for (;;) {
        if (xQueueReceive(self->queue_, &f, portMAX_DELAY) == pdTRUE && self->handler_) {
            self->handler_(f);
        }
    }
}

// Build, configure and enable the hardware node at `bitrate`. Factored out
// because a live bit-rate change recreates the node rather than retiming it:
// twai_node_reconfig_timing() takes the *advanced* timing struct (brp/prop_seg/
// tseg/sjw), and turning a plain bit rate into those needs
// twai_node_timing_calc_param() plus the peripheral's source clock and register
// limits — all of which live behind esp_private/. Recreating the node lets the
// driver do that conversion itself from the public bit_timing.bitrate field.
esp_err_t CanBus::createNode(uint32_t bitrate, bool listenOnly, bool selfTest) {
    twai_onchip_node_config_t cfg = {};
    cfg.io_cfg.tx                = tx_;
    cfg.io_cfg.rx                = rx_;
    cfg.io_cfg.quanta_clk_out    = GPIO_NUM_NC;
    cfg.io_cfg.bus_off_indicator = GPIO_NUM_NC;
    cfg.bit_timing.bitrate       = bitrate;
    // Hardware RX timestamps at 1MHz — free, and the only trustworthy ordering
    // source once frames have been through a queue.
    cfg.timestamp_resolution_hz  = 1000000;
    cfg.tx_queue_depth           = selfTest ? 4 : 1;
    cfg.flags.enable_listen_only = listenOnly ? 1 : 0;
    // Self-test needs BOTH flags, and they do different things:
    //   enable_self_test — transmission does not require an acknowledgement, so
    //                      a lone node on a dead bus can transmit at all.
    //   enable_loopback  — the controller receives back the frames it sends.
    // Without loopback the injector transmits happily and the RX path never
    // fires (observed on-device: zero frames, no errors), which makes the whole
    // exercise pointless. With both, the bench test needs no bus, no
    // transceiver activity and no second node.
    cfg.flags.enable_self_test   = selfTest ? 1 : 0;
    cfg.flags.enable_loopback    = selfTest ? 1 : 0;

    esp_err_t err = twai_new_node_onchip(&cfg, &node_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_new_node_onchip(%" PRIu32 " bps): %s", bitrate, esp_err_to_name(err));
        node_ = nullptr;
        return err;
    }

    // No acceptance filter is configured: mask filter 0 defaults to accept-all,
    // which is exactly right for discovery. The S3 has SOC_TWAI_MASK_FILTER_NUM
    // == 1 and no range filter, so there is nothing else to set up anyway.
    twai_event_callbacks_t cbs = {};
    cbs.on_rx_done      = onRxDone;
    cbs.on_error        = onError;
    cbs.on_state_change = onStateChange;
    err = twai_node_register_event_callbacks(node_, &cbs, this);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "register callbacks: %s", esp_err_to_name(err));
        twai_node_delete(node_);
        node_ = nullptr;
        return err;
    }

    err = twai_node_enable(node_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_node_enable: %s", esp_err_to_name(err));
        twai_node_delete(node_);
        node_ = nullptr;
        return err;
    }

    bitrate_    = bitrate;
    listenOnly_ = listenOnly;
    selfTest_   = selfTest;
    return ESP_OK;
}

esp_err_t CanBus::start(uint32_t bitrate, bool listenOnly, size_t queueDepth,
                        FrameHandler handler, bool selfTest) {
    if (node_ != nullptr) return ESP_ERR_INVALID_STATE;
    if (listenOnly && selfTest) {
        ESP_LOGE(TAG, "self-test needs to transmit; refusing with listen-only set");
        return ESP_ERR_INVALID_ARG;
    }

    handler_ = std::move(handler);

    // Pinned to internal DRAM, not left to the general heap. With
    // CONFIG_SPIRAM_USE_MALLOC a plain xQueueCreate() only lands internal while
    // the allocation stays under CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL (16KB by
    // default) — so at today's 256 frames it happens to, and at 512+ it would
    // silently move to PSRAM. That matters twice: PSRAM is slow to touch from
    // an ISR running at several thousand frames a second, and it is outright
    // illegal if TWAI_ISR_CACHE_SAFE is ever enabled (cache off during the
    // ISR). Same call the TWAI driver uses for its own queues.
    queue_ = xQueueCreateWithCaps(queueDepth, sizeof(CanFrame),
                                  MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (queue_ == nullptr) {
        ESP_LOGE(TAG, "failed to allocate a %u-frame internal-DRAM RX queue",
                 (unsigned)queueDepth);
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "RX queue: %u frames x %u B at %p (%s)", (unsigned)queueDepth,
             (unsigned)sizeof(CanFrame), (void*)queue_,
             esp_ptr_internal(queue_) ? "internal DRAM" : "PSRAM - UNEXPECTED");

    // The worker task must exist before the node is enabled, or the first
    // frames arrive with nothing draining the queue.
    if (xTaskCreate(workerTask, "can_rx", kWorkerStack, this, kWorkerPriority, &task_) != pdPASS) {
        ESP_LOGE(TAG, "failed to create worker task");
        vQueueDeleteWithCaps(queue_);
        queue_ = nullptr;
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = createNode(bitrate, listenOnly, selfTest);
    if (err != ESP_OK) {
        vTaskDelete(task_);
        task_ = nullptr;
        vQueueDeleteWithCaps(queue_);
        queue_ = nullptr;
        return err;
    }

    if (listenOnly) {
        ESP_LOGI(TAG, "listening on TX=%d RX=%d at %" PRIu32 " bps (listen-only: no TX, no ACK)",
                 (int)tx_, (int)rx_, bitrate);
    } else {
        // Loud on purpose. A non-passive node can perturb a vehicle bus; this
        // line is the evidence in the log that it was a deliberate choice.
        ESP_LOGW(TAG, "*** NODE IS NOT LISTEN-ONLY *** TX=%d RX=%d at %" PRIu32 " bps, self_test=%d "
                      "- this node CAN transmit and acknowledge. Do not use on a vehicle.",
                 (int)tx_, (int)rx_, bitrate, selfTest ? 1 : 0);
    }
    return ESP_OK;
}

// Tear the node down and rebuild it at the new rate. A few frames are missed
// across the switch; the worker task and queue are untouched, so nothing else
// in the pipeline notices. On failure we rebuild at the previous rate so the
// monitor is never left dead after a bad guess.
esp_err_t CanBus::setBitrate(uint32_t bitrate) {
    if (node_ == nullptr) return ESP_ERR_INVALID_STATE;
    if (bitrate == bitrate_) return ESP_OK;

    const uint32_t previous = bitrate_;
    const bool listenOnly = listenOnly_;
    const bool selfTest   = selfTest_;

    esp_err_t err = twai_node_disable(node_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "disable for retime: %s", esp_err_to_name(err));
        return err;
    }
    err = twai_node_delete(node_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "delete for retime: %s", esp_err_to_name(err));
        twai_node_enable(node_);  // put it back the way it was
        return err;
    }
    node_ = nullptr;

    err = createNode(bitrate, listenOnly, selfTest);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "could not come up at %" PRIu32 " bps (%s); reverting to %" PRIu32,
                 bitrate, esp_err_to_name(err), previous);
        esp_err_t back = createNode(previous, listenOnly, selfTest);
        if (back != ESP_OK) {
            ESP_LOGE(TAG, "revert to %" PRIu32 " bps also failed: %s - CAN is now down, "
                          "restart to recover", previous, esp_err_to_name(back));
        }
        return err;
    }

    ESP_LOGW(TAG, "bit rate changed %" PRIu32 " -> %" PRIu32 " bps", previous, bitrate);
    return ESP_OK;
}

// Transmit, blocking until the hardware is finished with our buffers.
//
// The blocking is not laziness, it is required. twai_node_transmit() does NOT
// copy: the driver's TX queue is created with sizeof(twai_frame_t *) and it
// stores `p_curr_tx = frame`, so it keeps only POINTERS to the caller's
// twai_frame_t and to its payload buffer. Both must stay alive until the
// transfer actually completes — the ISR dereferences them when it starts the
// transmission. Passing stack locals here panics with LoadProhibited inside
// twai_hal_format_frame() (observed on-device). So: persistent members, one
// transfer in flight at a time, and wait for done before returning.
esp_err_t CanBus::transmit(uint32_t id, bool ext, const uint8_t* data, size_t len) {
    if (node_ == nullptr) return ESP_ERR_INVALID_STATE;
    if (len > kCanMaxData) return ESP_ERR_INVALID_ARG;

    xSemaphoreTake(txLock_, portMAX_DELAY);
    std::memcpy(txBuf_, data, len);
    txFrame_             = {};
    txFrame_.header.id   = id;
    txFrame_.header.ide  = ext ? 1 : 0;
    txFrame_.header.dlc  = twaifd_len2dlc(static_cast<uint16_t>(len));
    txFrame_.buffer      = txBuf_;
    txFrame_.buffer_len  = len;

    esp_err_t err = twai_node_transmit(node_, &txFrame_, 100);
    if (err == ESP_OK) {
        err = twai_node_transmit_wait_all_done(node_, 100);
    }
    xSemaphoreGive(txLock_);
    return err;
}

esp_err_t CanBus::inject(uint32_t id, bool ext, const uint8_t* data, size_t len) {
    if (queue_ == nullptr) return ESP_ERR_INVALID_STATE;
    if (len > kCanMaxData) return ESP_ERR_INVALID_ARG;

    // Same shape the ISR builds, so nothing downstream can tell the difference.
    // Both clocks come from esp_timer here: there is no hardware capture for a
    // frame that was never on the wire, and using the monotonic value for both
    // keeps /can/dump ordering sane.
    CanFrame f = {};
    const uint64_t now = (uint64_t)esp_timer_get_time();
    f.timestampUs = now;
    f.recvUs      = now;
    f.id          = id;
    f.len         = (uint8_t)len;
    f.ext         = ext;
    f.rtr         = false;
    std::memcpy(f.data, data, len);

    if (xQueueSend(queue_, &f, 0) != pdTRUE) {
        counters_.drops++;
        return ESP_ERR_NO_MEM;
    }
    counters_.framesRx++;
    counters_.injected++;
    return ESP_OK;
}

esp_err_t CanBus::info(twai_node_status_t& status, twai_node_record_t& record) const {
    if (node_ == nullptr) return ESP_ERR_INVALID_STATE;
    return twai_node_get_info(node_, &status, &record);
}

const char* canErrorStateName(twai_error_state_t s) { return stateName(s); }
