#include "ModbusBus.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "mbcontroller.h"

namespace {

constexpr const char* TAG = "modbus";

// MB_FUNC_WRITE_MULTIPLE_COILS. Spelled out rather than pulled from the
// component's internal mb_proto.h, which is not on the public include path.
constexpr uint8_t kFnWriteMultipleCoils = 0x0F;

uart_parity_t parseParity(const std::string& s) {
    if (s == "even") return UART_PARITY_EVEN;
    if (s == "odd")  return UART_PARITY_ODD;
    return UART_PARITY_DISABLE;
}

}  // namespace

ModbusBus::~ModbusBus() { stop(); }

esp_err_t ModbusBus::start(int baud, const std::string& parity, uint32_t responseToutMs) {
    if (ctx_ != nullptr) return ESP_OK;

    mb_communication_info_t comm = {};
    comm.ser_opts.port             = (uart_port_t)modbusbus::kUartPort;
    comm.ser_opts.mode             = MB_RTU;
    comm.ser_opts.baudrate         = (uint32_t)baud;
    comm.ser_opts.parity           = parseParity(parity);
    comm.ser_opts.uid              = 0;              // unused for a master
    comm.ser_opts.response_tout_ms = responseToutMs;
    comm.ser_opts.data_bits        = UART_DATA_8_BITS;
    comm.ser_opts.stop_bits        = UART_STOP_BITS_1;

    esp_err_t err = mbc_master_create_serial(&comm, &ctx_);
    if (err != ESP_OK || ctx_ == nullptr) {
        ESP_LOGE(TAG, "mbc_master_create_serial: %s", esp_err_to_name(err));
        ctx_ = nullptr;
        return err == ESP_OK ? ESP_FAIL : err;
    }

    // Ordering is load-bearing and matches the component's own example: the
    // pins and the half-duplex mode must be set on the port *after* the
    // controller has created the UART driver and *before* the stack starts.
    err = uart_set_pin((uart_port_t)modbusbus::kUartPort, modbusbus::kPinTxd,
                       modbusbus::kPinRxd, modbusbus::kPinDe, UART_PIN_NO_CHANGE);
    if (err == ESP_OK) {
        err = uart_set_mode((uart_port_t)modbusbus::kUartPort, UART_MODE_RS485_HALF_DUPLEX);
    }
    if (err == ESP_OK) {
        err = mbc_master_start(ctx_);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "RS485 bring-up failed: %s", esp_err_to_name(err));
        mbc_master_delete(ctx_);
        ctx_ = nullptr;
        return err;
    }

    health_           = Health{};
    health_.running   = true;
    // Counts as "heard from" at startup so the first cycle is not reported as
    // a link that has been down since the epoch.
    health_.lastOkUs  = (uint64_t)esp_timer_get_time();
    ESP_LOGI(TAG, "RS485 master up: %d baud, parity %s, TX=%d RX=%d DE=%d (half duplex)",
             baud, parity.c_str(), modbusbus::kPinTxd, modbusbus::kPinRxd, modbusbus::kPinDe);
    return ESP_OK;
}

void ModbusBus::stop() {
    if (ctx_ == nullptr) return;
    mbc_master_delete(ctx_);
    ctx_             = nullptr;
    health_.running  = false;
    ESP_LOGW(TAG, "RS485 master stopped");
}

esp_err_t ModbusBus::writeCoils(uint8_t slave, uint16_t start, uint16_t count, uint8_t* bits) {
    if (ctx_ == nullptr) return ESP_ERR_INVALID_STATE;

    mb_param_request_t req = {};
    req.slave_addr = slave;
    req.command    = kFnWriteMultipleCoils;
    req.reg_start  = start;
    req.reg_size   = count;

    const esp_err_t err = mbc_master_send_request(ctx_, &req, bits);
    if (err == ESP_OK) {
        health_.ok++;
        health_.consecErr = 0;
        health_.lastOkUs  = (uint64_t)esp_timer_get_time();
    } else {
        health_.err++;
        health_.consecErr++;
        health_.lastErr = err;
    }
    return err;
}
