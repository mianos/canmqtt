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
| `signals` | decoded named values, e.g. `{"id":"0x2BC","engine_temp_c":88.5,"gear":"N"}` |
| `stats` | bus health, 1/min — the bit-rate diagnostic |
| `init`, `status` | identity + uptime/heap |
| `settingsack` | full settings after a `settings` command |

### Watching it with mosquitto

`brew install mosquitto` (or your distro's `mosquitto-clients`). These assume:

```sh
export B=mqtt2.mianos.com
export N=mqttcan
```

Everything the board says, topic shown per line. Start here when you first plug
into the bike:

```sh
mosquitto_sub -h $B -v -t "tele/$N/#"
```

Decoded signals only:

```sh
mosquitto_sub -h $B -t "tele/$N/signals" | jq -c .
```

Raw frames narrowed to one ID — this is what you want while correlating an
action with a frame, e.g. pulling the clutch and watching `10C`:

```sh
mosquitto_sub -h $B -t "tele/$N/frame" | jq -c 'select(.id == "0x10C")'
```

Frames as `candump`-ish text with the changed-byte mask, which is the fastest
way to see which byte a lever or button lives in:

```sh
mosquitto_sub -h $B -t "tele/$N/frame" | jq -r '"\(.id)#\(.data) chg=\(.chg)"'
```

Bus health, once a minute. Rising `rx_errors`/`bus_errors` with no frames means
the bit rate is wrong; all zeros and no frames means a quiet bus:

```sh
mosquitto_sub -h $B -t "tele/$N/stats" | jq .
```

Log a whole ride, with broker timestamps, for decoding afterwards:

```sh
mosquitto_sub -h $B -v -F '%I %t %p' -t "tele/$N/frame" | tee ride-$(date +%F).log
```

Commands:

```sh
# clean baseline before performing exactly one action
mosquitto_pub -h $B -t "cmnd/$N/canreset" -m '{}'

# live setting change; the ack lands on tele/$N/settingsack
mosquitto_pub -h $B -t "cmnd/$N/settings" -m '{"can_bitrate":125000}'

# plain reboot — the safe one, unlike HTTP POST /reset
mosquitto_pub -h $B -t "cmnd/$N/restart" -m '{}'
```

Watch a command and its effect together:

```sh
mosquitto_sub -h $B -v -t "tele/$N/signals" -t "tele/$N/settingsack"
```

Add `-u user -P pass` if the broker wants credentials. Add `-d` to any
`mosquitto_sub` when a topic looks silent — the SUBACK tells you whether it is a
broker ACL or the board genuinely not publishing.

### HTTP

| Route | Purpose |
|---|---|
| `GET /can/ids` | **the per-ID table — the main discovery view** |
| `GET /can/dump?limit=N` | recent raw frames as `candump` text |
| `GET /signals` | the decode table in use |
| `POST /signals` | replace it (validated before storing) |
| `GET /signals/status` | loaded?, byte_base, frame/signal counts, known IDs |
| `GET /can/status` | bit rate, error state, counters |
| `POST /can/reset` | clear table + ring |
| `GET`/`POST /config`, `POST /config/reset` | settings |
| `GET`/`POST /firmware` | OTA (raw `.bin` body) |
| `GET /healthz`, `POST /reset`, `POST /set_hostname` | from the shared `WebServer` base. **`/reset` wipes the Wi-Fi credentials** and reboots into provisioning; it is not a restart. To reboot after a setting that needs one, use `cmnd/<name>/restart` over MQTT or power-cycle |

## Decoding: named signals

Raw hex is for discovery; once an ID is understood it should report as a named
value. `data/signals.json` describes how, and lives on its own SPIFFS partition
so the mapping can be corrected **over the air** as IDs are confirmed against
the bike — no firmware rebuild. The firmware embeds a copy and writes it out on
first boot, so a freshly flashed board decodes immediately.

```sh
curl http://<host>/signals/status    # loaded? how many frames/signals? which IDs?
curl http://<host>/signals           # the table currently in use
curl --data-binary @data/signals.json http://<host>/signals   # replace it
```

An upload is **parsed and applied before it is written to flash**: a table that
does not load is rejected with the parse error and nothing is stored, so a bad
edit cannot leave the device unable to decode after a reboot.

Decoded values publish to `tele/<name>/signals` as a flat object of whatever
changed in that frame:

```json
{"id":"0x2BC","engine_temp_c":88.5,"gear":"N"}
```

Reporting is change-driven, independent of the raw-frame filter: an enum reports
when its label changes, a number when it moves further than its `deadband`, and
both respect a per-signal `min_ms`. That is what stops `rpm` from flooding the
broker while still reporting `gear` the instant it shifts. A code that is not in
a signal's `map` reports as `unmapped_0xNN` rather than being dropped — an
unknown code is exactly the kind of gap this project exists to close.

### Schema

| key | meaning |
|---|---|
| `version` | integer; a reflash replaces a stored table whose version is lower than the build's, so bump it on a table you upload and want to keep |
| `byte_base` | which `data[]` index `D1` means; `0` ⇒ `Dn` is `data[n]` (the community sheet's convention, and the default) |
| `bytes[]` | little-endian byte list, least-significant first: `(D3*256+D2)` is `[2,3]` |
| `nibble` | `"high"` / `"low"` / absent |
| `parts[]` | `{byte, nibble}` list, for keys built from several nibbles (ESA damping); joined with `,` |
| `scale`, `offset` | `value = raw * scale + offset` |
| `map{}` | uppercase-hex key → label; its presence makes the signal an enum |
| `default` | label for a code missing from `map`; without it the code reports as `unmapped_0xNN` |
| `deadband`, `min_ms` | suppress chatter on analog signals |

### Provenance, and what is still unverified

The table is the K25 subset of the community
[BMW Motorrad CAN sheet](https://docs.google.com/spreadsheets/d/1tUrOES5fQZa92Robr6uP8v2dzQDq9ohHjUiTU3isqdc)
(Keith Conger et al.). Its Key tab defines the payload as `D0`–`D7`, so
`D1` is `data[1]`; the 2013 ancestor of the sheet numbered the same fields
`byte1`–`byte8`, and the
[Arduino sniffer](https://github.com/4G-Gregg/BMW-GS-CAN-Sniffer) written
against a 2010 K25 from that ancestor reads heated grips from `data[7]` and
throttle from `data[1]`, which settles the indexing. That sniffer's source is
the only evidence here of code that actually ran on a K25, and it agrees with
the sheet on high beam, turn signals, info button, heated grips, ABS button and
brake levers.

It disagrees on three nibbles, and the sheet has two internal slips. Each is
flagged with a `_note` in the JSON and encoded as follows:

- **`10C` clutch**: sheet says high nibble of `D4`, sniffer reads the low
  nibble, same `6`/`A` values. Sheet kept. Pull the clutch and see which
  nibble moves.
- **`294` ABS state**: sheet says low nibble of `D1`, sniffer reads the high
  nibble, same `5`/`B` values. Sheet kept.
- **`3FF` ambient light**: sheet `B` dark / `7` light, sniffer `7` dark /
  `3` light. Sheet kept; anything else surfaces as `unmapped_0xN`.
- **`2D0` D7** carries heated grips in the high nibble (`C`/`D`/`E`, sniffer
  confirmed) and the sheet's separate ignition row (`FF` off / `DF` on) is the
  same byte seen with the grips on low. Ignition is therefore `FF` ⇒ off,
  anything else ⇒ on, via `default`; grips report `unavailable` on `F`.
- **`10C` throttle position** says `D6(High Nibble)` with a `/255` formula; the
  2013 sheet had it as a whole byte and so does the K50 row. Whole byte used.
- **`2BC` gear** moved from the low nibble (2013) to the high nibble (current,
  eight models). High nibble used.

Front wheel speed on `294` has carried `~0.06` since 2013 while the rear on
`2A8` is exact. No GPS is needed to fix it: at steady speed in a straight line
the two wheels must agree, so `scale = 0.06 × rear_raw / front_raw`. Treat every
mapping as a hypothesis until you have confirmed it on your own bike.

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
in IRAM.

**The RX queue is pinned to internal DRAM**, via
`xQueueCreateWithCaps(..., MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)` rather than a
plain `xQueueCreate()`. This is enforced rather than incidental: with
`CONFIG_SPIRAM_USE_MALLOC=y` the general heap only keeps an allocation internal
while it stays under `CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL` (16KB default), so at
the current 256 frames × 32 B = 8KB it would land internal by luck, and a larger
queue depth would silently migrate to PSRAM. PSRAM is slow to touch from an ISR
running at thousands of frames a second, and illegal outright under
`TWAI_ISR_CACHE_SAFE`. A `static_assert` pins `sizeof(CanFrame)` at 32 bytes so
the sizing above stays honest, and the queue's placement is logged at startup.

## Layout

| File | Role |
|---|---|
| `main/main.cpp` | wiring: Wi-Fi, MQTT, tasks, settings hooks, self-test injector |
| `main/CanBus.{h,cpp}` | TWAI node, listen-only, ISR → queue → worker task |
| `main/FrameTable.{h,cpp}` | per-ID table, change detection, dump ring, report bodies |
| `main/SignalTable.{h,cpp}` | JSON decode table, per-signal change detection, SPIFFS store |
| `data/signals.json` | the decode table (embedded as the built-in default) |
| `main/CanWebServer.{h,cpp}` | HTTP surface on the shared `WebServer` base |
| `main/Settings.h` | schema on mianesp's `SettingsBase` |
| `partitions.csv` | 16MB, dual 3MB OTA slots + 256KB `signals` SPIFFS |

## Sources

- [Waveshare ESP32-S3-RS485-CAN wiki](https://www.waveshare.com/wiki/ESP32-S3-RS485-CAN)
- [ESPHome board notes — CAN GPIO15/16, 16MB/8MB](https://github.com/Sleeper85/esphome-yambms/blob/main/documents/README/Board_Waveshare_ESP32-S3-RS485-CAN.md)
- [OpenELAB — 7–36V input, isolation](https://openelab.io/products/waveshare-industrial-esp32-s3-control-board-rs485-can)
- [4G-Gregg/BMW-GS-CAN-Sniffer — 2010 R1200GS, alarm connector pinout](https://github.com/4G-Gregg/BMW-GS-CAN-Sniffer)
- [Largiader — BMW motorcycle CAN-bus; the marketed "Single-Wire System" is still a two-wire CAN](https://largiader.com/articles/canbus/)
- [BMW R1200R forum — SocketCAN at 500000, ~240Ω termination measurement](https://www.r1200rforum.com/threads/canbus-hack.7890/)
- [K1600 forum — 2008 R1200GS Adv: 500k works, 125k/250k/1M fail; ~19 IDs](https://www.k1600forum.com/threads/canbus-adventure-begins.3440/)
