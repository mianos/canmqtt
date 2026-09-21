#pragma once

#include <cstdint>
#include <string>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

// Modbus RTU master on the board's RS485 port, for driving output nodes that
// are not physically next to this board.
//
// Why this exists: the driving lights are on the front of the bike and the
// monitor is under the seat. Running two relay-trigger wires the length of the
// frame means two more unshielded conductors next to a CAN tap and an ignition
// harness, and a relay board out in the weather. One shielded twisted pair
// carrying RS485 to a MOSFET node at the headlight is the better arrangement,
// and it scales: the same pair can carry four channels, or eight, with no more
// copper.
//
// This does NOT make the board a CAN participant. The TWAI controller stays in
// hardware listen-only mode. RS485 is a private bus with exactly two nodes on
// it — this master and our own slave — and shares nothing with the vehicle.
//
// Pins are fixed by the board, not configurable: the RS485 transceiver is
// hard-wired to these. From Waveshare's own WS_GPIO.h for the
// ESP32-S3-RS485-CAN (TXD1 17, RXD1 18, TXD1EN 21).
//
// GPIO21 is the transceiver's driver-enable, and it is driven as the UART's
// RTS in UART_MODE_RS485_HALF_DUPLEX rather than by hand. That matters: the
// peripheral asserts DE before the first bit and releases it after the last
// stop bit has physically left the shift register. Toggling DE from software
// races the FIFO and truncates the tail of a frame, which on Modbus shows up
// as an intermittent CRC error under load rather than an obvious failure.
// (Sources online disagree about whether this board needs manual DE control;
// the vendor's own driver settles it — it uses RS485 half-duplex mode.)
namespace modbusbus {
constexpr int kUartPort = 1;    // UART0 is the console
constexpr int kPinTxd   = 17;
constexpr int kPinRxd   = 18;
constexpr int kPinDe    = 21;   // transceiver DE, driven as RTS by the UART
}  // namespace modbusbus

class ModbusBus {
public:
    struct Health {
        bool     running   = false;
        uint32_t ok        = 0;   // successful transactions
        uint32_t err       = 0;   // failed transactions
        uint32_t consecErr = 0;   // current run of failures; 0 == link is good
        uint64_t lastOkUs  = 0;   // esp_timer stamp of the last good transaction
        esp_err_t lastErr  = ESP_OK;
    };

    ~ModbusBus();

    // Bring up the UART and the Modbus master. parity is "none", "even" or
    // "odd" — Modbus RTU's spec default is even, but both ends here are ours,
    // so "none" is the shipped default and any conformant slave can be matched
    // by changing the setting.
    //
    // Lifecycle is owned by a single task (see modbusTask in main.cpp). Do not
    // call start/stop/writeCoils from more than one task: the underlying
    // handle is not guarded, and a transaction blocks for the response timeout.
    esp_err_t start(int baud, const std::string& parity, uint32_t responseToutMs);
    void      stop();
    bool      running() const { return ctx_ != nullptr; }

    // FC 0x0F, write multiple coils. Blocks until the slave answers or the
    // response timeout expires. `bits` is packed LSB-first — coil `start+n`
    // lives in bit (n % 8) of byte (n / 8), which is Modbus wire order, so the
    // buffer goes out untouched.
    //
    // A successful return is real confirmation, not just "we transmitted":
    // FC 0x0F's normal response echoes the address and quantity, so ESP_OK
    // means the slave parsed and accepted the write. That is why nothing here
    // reads the coils back.
    esp_err_t writeCoils(uint8_t slave, uint16_t start, uint16_t count, uint8_t* bits);

    Health health() const { return health_; }

private:
    void*  ctx_ = nullptr;
    Health health_;
};
