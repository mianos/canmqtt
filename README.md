# mqttcan

Passive CAN-bus monitor for a **BMW R1200GS (K25)**, on a **Waveshare
ESP32-S3-RS485-CAN** board. ESP-IDF v6.0.1.

It watches the bike's bus without touching it, maintains a live per-ID table of
what it sees, and publishes only the frames whose payload actually *changes*.
Shares its infrastructure components (`wifimanager`, `mqttwrapper`,
`settingsbase`, `webserver`, `jsonwrapper`) with mqttradar / ws-voice / doorbell3.

The K25's CAN IDs are **not** reliably documented — the one public project for
this bike ([`4G-Gregg/BMW-GS-CAN-Sniffer`](https://github.com/4G-Gregg/BMW-GS-CAN-Sniffer))
notes that the community spreadsheet it was based on contained errors. So this is
a **discovery instrument first**: identify IDs against your own bike by
correlation, then add decoding.

## Safety

The TWAI controller runs in **hardware listen-only mode**: it never transmits and
never acknowledges. This is not excess caution — an experimenter on a BMW forum
found their nominally "listen-only" sniffer intermittently transmitted and
toggled the bike's rear brake light.

`can_listen_only` defaults to 1 and is deliberately **not** live-applied: a
change is persisted but only takes effect on the next boot, and a node coming up
non-passive logs a `*** NODE IS NOT LISTEN-ONLY ***` warning. Never put a
non-passive build on the vehicle.

## Wiring

Tap the **BMW alarm pre-wire connector, part 83300413581**, which conveniently
carries both power and the bus:

| Connector pin | Signal | Board terminal |
|---|---|---|
| 6 | CAN High | CAN **H** |
| 5 | CAN Low | CAN **L** |
| 3 | Switched 12V | **V+** (7–36V input) |
| 4 | Ground | **V−** |

**Leave the board's 120Ω jumper on `NC`.** The bike's bus is already terminated
at both ends — someone measured ~240Ω across this connector, which is two 120Ω
terminators in parallel. Adding a third drops the bus to ~80Ω and gives you the
classic "transceiver pins wiggle but nothing decodes" symptom. Keep the tap a
short stub off the twisted pair.

The board's CAN side is galvanically isolated (digital + power isolation, TVS,
surge and ESD protection), which is what makes this safe to hang off vehicle
wiring.

### Board facts

Confirmed on-device during bring-up:

- CAN TWAI **TX = GPIO15, RX = GPIO16** (vendor's `WS_GPIO.h`: `TXD2 15`,
  `RXD2 16`). Do **not** copy these from other Waveshare ESP32-S3 boards — the
  Touch-LCD-7 uses 20/19 and the Touch-LCD-4 uses 6/0 for the same peripheral.
- 16MB flash, **8MB octal PSRAM** (ESP32-S3R8 — the boot log confirms
  `octal_psram: Found 8MB PSRAM device`).
- 7–36V screw-terminal input, or USB-C.

## Bit rate

Default **500000**, and settable at runtime.

500 kbit/s is a well-corroborated first guess but **not a confirmed BMW spec for
the K25**, so treat it as a starting point:

- A 2008 R1200GS Adventure (K25 family) with an MCP2515 got error-free frames at
  500k and **failed at 125k, 250k and 1M**. The same report saw only ~19 distinct
  IDs, with frame lengths implying **extended (29-bit) IDs** — so nothing here
  assumes 11-bit.
- An R1200R tapped at the OBD connector worked with SocketCAN at `bitrate 500000`.

**Diagnosing it:** watch `tele/mqttcan/stats` or `GET /can/status`.

| Symptom | Meaning |
|---|---|
| `frames_rx` climbing, `bus_errors` flat | bit rate is correct |
| `frames_rx` zero, `bus_errors` climbing | wrong bit rate — try 125000 |
| `frames_rx` zero, `bus_errors` zero | bus asleep, or not actually connected |

Retune live, no reflash and no reboot:

```sh
mosquitto_pub -h mqtt2.mianos.com -t cmnd/mqttcan/settings -m '{"can_bitrate":125000}'
```

Listen-only means a wrong guess can never disturb the bike, so sweeping rates is
safe.

## Publish policy

A 500 kbit/s bus can carry thousands of frames per second; publishing all of them
would flatten the broker and bury the signal. So an ID is published when its
payload **changes**, floored at `publish_min_ms` per ID, with an optional slow
heartbeat so a static ID still reports in. Same decimation idea as mqttradar's
`presencePeriodSec`, applied per CAN ID.

### MQTT

Commands — `cmnd/mqttcan/…`:

| Topic | Payload | Effect |
|---|---|---|
| `settings` | any subset of the `/config` JSON | apply + persist |
| `canreset` | `{}` | clear the frame table + dump ring |
| `restart` | `{}` | reboot |
| `reprovision` | `{}` | clear Wi-Fi creds, reboot into ESP-Touch v2 |

Publishes — `tele/mqttcan/…`:

| Topic | Content |
|---|---|
| `frame` | `{"id":"0x2BC","ext":false,"dlc":8,"data":"DEADBEEF00000000","chg":"0C","t_us":…}` on each accepted change |
| `stats` | bus health, 1/min — the bit-rate diagnostic |
| `init`, `status` | identity + uptime/heap |
| `settingsack` | full settings after a `settings` command |

### HTTP

| Route | Purpose |
|---|---|
| `GET /can/ids` | **the per-ID table — the main discovery view** |
| `GET /can/dump?limit=N` | recent raw frames as `candump` text |
| `GET /can/status` | bit rate, error state, counters |
| `POST /can/reset` | clear table + ring |
| `GET`/`POST /config`, `POST /config/reset` | settings |
| `GET`/`POST /firmware` | OTA (raw `.bin` body) |
| `GET /healthz`, `POST /reset`, `POST /set_hostname` | from the shared `WebServer` base |

## Decoding workflow

`changed` in `/can/ids` is a bitmask of which payload bytes have **ever** moved.
That one column separates live signal bytes from constant padding, and it is the
fastest way in:

```sh
curl -X POST http://mqttcan.local/can/reset   # clean baseline
# perform exactly ONE action: left indicator, front brake, ignition off, ...
curl http://mqttcan.local/can/ids | jq .
```

Look for the ID whose `changed` mask gained a bit. Repeat per action and record
findings. `period_ms` also classifies quickly: a ~100ms cyclic ID behaves very
differently from a one-shot event ID.

Treat any community ID map as a hypothesis to verify against your own bike, not
ground truth.

## Build, flash, monitor

```sh
export IDF_PATH=~/.espressif/v6.0.1/esp-idf
. "$IDF_PATH/export.sh"
./build.sh                                   # or: idf.py build
idf.py -p /dev/cu.usbmodem14401 flash monitor
```

The first flash must be over serial (it writes the partition table). After that,
update over the air:

```sh
curl --data-binary @build/mqttcan.bin http://mqttcan.local/firmware
```

A freshly-OTA'd image boots `PENDING_VERIFY` and is only marked valid once Wi-Fi
is back, so an image that boots but can't reach the network rolls itself back.

## Bench self-test

There is a built-in injector for exercising the whole pipeline with **no bus, no
transceiver activity and no second CAN node**:

```jsonc
{"self_test": 1, "can_listen_only": 0, "log_frames": 1}
```

then reboot. It transmits three synthetic IDs every 250ms — one with a ticking
byte, one constant (the control that proves change-detection suppresses
repeats), and one extended 29-bit ID — and `log_frames` prints each published
frame to the console, so none of this needs a broker.

It requires `can_listen_only=0` (a listen-only node physically cannot transmit)
and is refused otherwise with a logged error. **Never enable it on the vehicle.**

## Implementation notes

Two things about ESP-IDF v6's TWAI driver cost real debugging time and are worth
knowing before editing `CanBus.cpp`.

**The driver model changed.** Every CAN example online uses the legacy
`driver/twai.h` (`twai_driver_install`, `twai_receive`, `TWAI_MODE_LISTEN_ONLY`),
which in 6.0.1 still exists but emits a deprecation `#warning`. This project uses
the current node-based `esp_twai.h` / `esp_twai_onchip.h`. Consequences:

- **RX is ISR-callback-only.** `twai_node_receive_from_isr()` is callable *only*
  from inside `on_rx_done` — there is no task-level receive. Hence the
  architecture: ISR flattens each frame into a FreeRTOS queue (internal DRAM), a
  worker task drains it.
- Bit-rate changes **recreate the node** rather than retiming it.
  `twai_node_reconfig_timing()` takes the *advanced* timing struct
  (brp/prop_seg/tseg/sjw), and converting a plain bit rate into those needs
  `twai_node_timing_calc_param()` plus the peripheral clock and register limits —
  all behind `esp_private/`. Recreating the node lets the driver do that
  conversion itself from the public `bit_timing.bitrate` field.

**`twai_node_transmit()` does not copy.** Its TX queue is created with
`sizeof(twai_frame_t *)` and it stores `p_curr_tx = frame`, so it keeps only
*pointers* to the caller's `twai_frame_t` and its payload buffer. Both must stay
alive until the transfer completes — the ISR dereferences them when it starts
transmitting. Passing stack locals panics with `LoadProhibited` inside
`twai_hal_format_frame()` (observed on-device). `CanBus::transmit()` therefore
uses persistent members, serialises with a mutex, and waits for
`twai_node_transmit_wait_all_done()` before returning.

**Self-test needs two flags, not one.** `enable_self_test` only removes the ACK
requirement; `enable_loopback` is what feeds transmitted frames back to RX.
With only the former, the injector transmits happily and the RX path never fires.

**ESP32-S3 silicon limits:** one TWAI controller, a single mask filter
(`filter_id` always 0), no range filter, and no CAN FD — classic CAN only, which
is all the K25 needs. No acceptance filter is configured at all: mask filter 0
defaults to accept-all, which is what discovery wants.

**`CONFIG_TWAI_ISR_IN_IRAM=y`** is set deliberately. On a busy bus, an ISR living
in flash is starved whenever the cache is disabled for a flash write (an NVS
settings save, or a `POST /firmware` OTA) and frames are silently dropped. If
`stats.drops` ever shows losses during an OTA, the escalation is
`TWAI_ISR_CACHE_SAFE`, which additionally requires the callbacks *and* `user_ctx`
in IRAM — which is also why the RX queue stays in internal DRAM rather than PSRAM.

## Layout

| File | Role |
|---|---|
| `main/main.cpp` | wiring: Wi-Fi, MQTT, tasks, settings hooks, self-test injector |
| `main/CanBus.{h,cpp}` | TWAI node, listen-only, ISR → queue → worker task |
| `main/FrameTable.{h,cpp}` | per-ID table, change detection, dump ring, report bodies |
| `main/CanWebServer.{h,cpp}` | HTTP surface on the shared `WebServer` base |
| `main/Settings.h` | schema on mianesp's `SettingsBase` |
| `partitions.csv` | 16MB, dual 3MB OTA slots |

## Sources

- [Waveshare ESP32-S3-RS485-CAN wiki](https://www.waveshare.com/wiki/ESP32-S3-RS485-CAN)
- [ESPHome board notes — CAN GPIO15/16, 16MB/8MB](https://github.com/Sleeper85/esphome-yambms/blob/main/documents/README/Board_Waveshare_ESP32-S3-RS485-CAN.md)
- [OpenELAB — 7–36V input, isolation](https://openelab.io/products/waveshare-industrial-esp32-s3-control-board-rs485-can)
- [4G-Gregg/BMW-GS-CAN-Sniffer — 2010 R1200GS, alarm connector pinout](https://github.com/4G-Gregg/BMW-GS-CAN-Sniffer)
- [Largiader — BMW motorcycle CAN-bus; the marketed "Single-Wire System" is still a two-wire CAN](https://largiader.com/articles/canbus/)
- [BMW R1200R forum — SocketCAN at 500000, ~240Ω termination measurement](https://www.r1200rforum.com/threads/canbus-hack.7890/)
- [K1600 forum — 2008 R1200GS Adv: 500k works, 125k/250k/1M fail; ~19 IDs](https://www.k1600forum.com/threads/canbus-adventure-begins.3440/)
