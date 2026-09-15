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

**500000, confirmed on the vehicle** on 2026-09-15: twelve IDs decoded cleanly
at that rate with measured cycle times matching the community sheet (`10C` 9 ms,
`2BC` 99 ms, `3F8` 999 ms). Settable at runtime.

### Reading `/can/status` in listen-only mode

`state` reads `passive` and `rx_error_count` reads `128` permanently, and
**neither is a fault**. The IDF driver writes 128 into the RX error counter
before leaving reset whenever listen-only is set, as an errata workaround: an
error-passive node can only ever emit recessive bits, which guarantees it cannot
disturb the bus with a dominant error frame. The counters are then frozen. Only
`bus_errors` and `frames_rx` carry information. The response says so in a
`state_note` field, because the raw numbers look alarming and cost a diagnosis
otherwise.

The three cases that matter:

| `frames_rx` | `bus_errors` | Meaning |
|---|---|---|
| climbing | flat | healthy |
| zero | climbing | wrong bit rate |
| zero | flat | nothing on the wire: ignition off, or the tap is not connected |

The original note on the rate, kept because it is still only one bike's worth of
evidence:

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
| `mark` | `{"text":"test high beam"}`, optionally `"reset":true` | label the capture — see [Test annotations](#test-annotations) |
| `output` | `{"name":"driving","set":"on"\|"off"\|"toggle"}` | drive a GPIO relay — see [Driving lights](#driving-lights) |
| `restart` | `{}` | reboot |
| `reprovision` | `{}` | clear Wi-Fi creds, reboot into ESP-Touch v2 |

Every command payload must be a **JSON object**. The shared `MqttClient` parses
each message and discards anything that is not one before a handler runs, so
publishing bare text to `cmnd/mqttcan/mark` does nothing at all, silently.

Publishes — `tele/mqttcan/…`:

| Topic | Content |
|---|---|
| `frame` | `{"id":"0x2BC","ext":false,"dlc":8,"data":"DEADBEEF00000000","chg":"0C","t_us":…}` on each accepted change |
| `signals` | decoded named values, e.g. `{"id":"0x2BC","engine_temp_c":88.5,"gear":"N"}` |
| `mark` | `{"seq":7,"text":"test high beam","reset":false,"up_ms":1234567}` |
| `gesture` | `{"gesture":"driving_lights","clicks":3,"action":"driving=on"}` on every resolved click burst, **including unmapped counts** (`"action":""`) |
| `output` | `{"name":"driving","state":"on","by":"gesture:3"}` on every output change, whatever caused it |
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

Labels alongside the frames they explain, which is the capture you want when
something else has to reconstruct what you were doing:

```sh
mosquitto_sub -h $B -v -t "tele/$N/mark" -t "tele/$N/frame"
```

Commands:

```sh
# label the capture; add "reset":true to clear the table in the same call
mosquitto_pub -h $B -t "cmnd/$N/mark" -m '{"text":"test high beam"}'

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
| `GET /can/status` | bit rate, error state, counters, **and decode-table state** — see the listen-only caveat below |
| `POST /can/reset` | clear table + ring (and decode state) |
| `POST /can/mark` | label the capture; `{"text":"…"[,"reset":true]}` |
| `POST /can/inject` | push a synthetic frame through the software path; safe while listen-only |
| `POST /can/output` | drive a GPIO relay directly; `{"name":"driving","set":"on"}` |
| `GET`/`POST /config`, `POST /config/reset` | settings |
| `GET`/`POST /firmware` | OTA (raw `.bin` body) |
| `GET /healthz`, `POST /reset`, `POST /set_hostname` | from the shared `WebServer` base. **`/reset` wipes the Wi-Fi credentials** and reboots into provisioning; it is not a restart. To reboot after a setting that needs one, use `cmnd/<name>/restart` over MQTT or power-cycle |

The shared `WebServer` starts httpd with `max_uri_handlers = 16` and spends
three, so **this table is exactly full**. Adding a route means collapsing
another URI's methods onto a single `HTTP_ANY` handler — which is what `/signals`
already does, invisibly from outside — or raising the limit in the mianesp
`webserver` component that four other projects also build against. A
`static_assert` in `CanWebServer::start()` turns overflow into a build error,
because httpd otherwise just drops the last route and you get a 404 with nothing
else wrong.

## Decoding: named signals

Raw hex is for discovery; once an ID is understood it should report as a named
value. `data/signals.json` describes how, and lives on its own SPIFFS partition
so the mapping can be corrected **over the air** as IDs are confirmed against
the bike — no firmware rebuild. The firmware embeds a copy and writes it out on
first boot, so a freshly flashed board decodes immediately.

```sh
curl http://<host>/can/status        # loaded? how many frames/signals? what is muted?
curl http://<host>/signals           # the table currently in use
curl --data-binary @data/signals.json http://<host>/signals   # replace it
```

Frames can be decoded off the bike, using the same table the board runs, which
is the quickest way to read a log someone pasted at you:

```sh
tools/decode.py < capture.log
mosquitto_sub -h $B -t "tele/$N/frame" | tools/decode.py
```

It accepts either the JSON the board publishes or bare `2BC#FF534E...` candump
text.

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
| `invalid` | raw value meaning "no reading"; the signal is not reported at all |
| `deadband`, `min_ms` | suppress chatter on analog signals |
| `noise.<ID>` | hex mask of payload bytes that must not count as a frame change |

### Free-running counters

Several IDs carry a byte that increments every cycle. `3FF` D2–D4 is a seconds
counter, measured on the bike stepping by exactly one every 1000 ms. Change
detection is defenceless against these: the payload is always different, so the
ID republishes at its full cyclic rate for the entire ride and buries the real
events.

`noise` mutes them per ID. Bit N mutes byte N, so `"3FF": "1C"` excludes D2, D3
and D4 while leaving ambient light on D1 and the odometer on D5–D7 live:

```json
"noise": { "3FF": "1C", "2AC": "03" }
```

Muted bytes are still stored and still counted in the `changed` mask — hiding a
counter from the publish decision should not hide the fact that it is moving.
`/can/ids` shows the mask as a `noise` field on the affected row, and
`/can/status` reports `signals_noise` (e.g. `0x2AC=03,0x3FF=1C`), which is the
way to confirm the section parsed without waiting for bus traffic.

Verified on the board with [`/can/inject`](#frame-injection), listen-only, on
the bike:

| Injected | Frames in | Published |
|---|---|---|
| `3FF` ticking D2 (masked) | 8 | 1 |
| `3FF` ticking D5 (odometer) | 8 | 8 |
| `2AC` ticking D0 (masked) | 6 | 1 |
| `2AC` ticking D4 | 6 | 6 |

To find a candidate, look for an ID in `/can/ids` whose `published` count is
close to its `count`, then watch the byte in `/can/dump`.

### Dump ring size

The bus runs at roughly 1200 frames/s (twelve IDs, most cycling every 9–10 ms),
so `dump_ring_frames` buys about a frame per millisecond. The default is 32768,
which is 1 MB out of 8 MB of PSRAM and holds about 27 seconds — enough to label
an action and then go read the dump. The old 2048 held 1.7 seconds, which was
not.

The setting is read once at startup, so a change needs a power cycle. On the
bike that happens on every ignition cycle. An over-large value halves itself
until it fits rather than throwing, because the value lives in NVS and not in
the image: a failed allocation would otherwise boot-loop a board that even an
OTA rollback could not rescue.

### Confirmed on the vehicle

Read from the bike on 2026-09-15, ignition on, engine off, stationary.
`byte_base: 0` is settled three independent ways:

- `2BC` gave engine 24.75 °C and air 24.75 °C. A cold engine sits at ambient, so
  the two agreeing is the check. `byte_base: 1` yields 138 °C air on the same
  frame.
- Gear read `N` on a parked bike; `byte_base: 1` leaves it unmapped.
- The odometer appears twice, at `3F8` D1–D3 and `3FF` D5–D7, and both decode to
  118904 km. Two frames, different offsets, same number.

Also confirmed: wheel speeds, throttle and rpm all zero; brake levers, high
beam, indicators, heated grips and info button all off; lamp faults none;
ignition on; fuel 28.6 % to reserve; ambient 24 °C from `2D0` agreeing with
`2BC`. `2AC` exists on this bus and is in no decode table. No ESA and no tyre
pressure senders are fitted, both of which now report as such rather than
inventing a value.

### Confirmed by operating the switches

Each of these was cleared to a fresh baseline, the control operated, and the
changed byte read back. The bike agrees with the table:

| Action | ID | Byte | Off | On |
|---|---|---|---|---|
| High beam | `130` | D6 low nibble | `A` | `9` |
| Left indicator | `130` | D7 | `CF` | `D7` |
| Front brake | `294` | D6 | `03` | `07` |
| Heated grips | `2D0` | D7 high nibble | `C` | `D` low, `E` high |
| Side stand | `10C` | D5 **low** nibble | `9` up | `5` down |

Engine running added rpm on `10C` D2–D3, reading 1304 at a fast cold idle.

### Two negative results worth keeping

**The horn is not on the bus.** Repeated presses moved no byte of any of the
twelve IDs, including `2AC` with its mask temporarily lifted. The body module
evidently reads the switch and drives the horn locally without telling anyone.
Do not go looking for it.

**`2A0` is unused on the K25**, despite the community sheet listing it as the
left-hand switch cluster for this model. It sits at all `FF` with a changed mask
of `00` indefinitely, through horn, indicator and beam operation.

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

Four enums did not map against the real bus, and they are exactly the ones the
two sources disagreed about. This is the shortlist for the next session, and it
is what [test annotations](#test-annotations) exist for: label the action, do it,
diff the byte.

- **`10C` clutch**: sheet says high nibble of `D4`, sniffer reads the low
  nibble, same `6`/`A` values. The bike read `0x16`, so the **low** nibble holds
  `6` = clutch out, favouring the sniffer. Sheet still in place pending a test.
- **`294` ABS state**: sheet says low nibble of `D1`, sniffer reads the high
  nibble, same `5`/`B` values. The bike read `0x5F`, so the **high** nibble
  holds `5` = on, again favouring the sniffer.
- **`10C` side stand and ASC**: read `0xE9` on `D5`, matching neither source's
  codes. `D5` is the only byte of `10C` that moved during the whole capture, so
  the state is certainly in there.
- **`3FF` ambient light**: sheet `B` dark / `7` light, sniffer `7` dark /
  `3` light. The bike read `9` in daylight, so neither is right.

Three of the sheet's nibble claims have now been tested against the bike and all
three were wrong, so treat the rest as unverified rather than merely uncertain.
The side stand was wrong twice over: wrong nibble *and* inverted values.
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

## Test annotations

Decoding is correlation, so a capture is only as good as the record of what you
were doing while it was taken. Push a label just before each action and it lands
in the capture next to the frames it explains:

```sh
curl -X POST -d '{"text":"test high beam"}' http://mqttcan.local/can/mark
```

The label goes two places. It publishes on `tele/mqttcan/mark`, interleaved with
`frame` and `signals` in a subscriber's stream, and it is kept in a 32-entry ring
that `GET /can/dump` merges inline:

```
(1234.567890) can0 130#CF00000000000000
# MARK 7 test high beam
(1234.612340) can0 130#D700000000000000
```

That is the form to hand an LLM when asking it to confirm a mapping: the label
and the frames it caused are in one file, in order.

Add `"reset":true` to clear the frame table first, which makes the per-test loop
a single call. It is opt-in rather than automatic because `changed` is
cumulative and is the most valuable thing the table holds — you want labels
during a ride without throwing that away, and you want an "off" marker that does
not erase what the "on" marker was measuring. `POST /can/reset` still exists for
a baseline with no label.

Labels are truncated at 63 characters and the response echoes what was actually
stored. `seq` is unique for the life of the boot and is not rewound by a reset,
so it names one label unambiguously. `up_ms` is the monotonic clock the dump
orders by, which is how you line an MQTT capture up against `/can/dump`.

### From Node-RED

An inject node per test, into a template node emitting

```json
{"text":"test high beam","reset":true}
```

then an mqtt-out node on `cmnd/mqttcan/mark`. Set the template's output to
*parsed JSON* or the mqtt-out node to send a string — either way the payload on
the wire has to be a JSON object, not bare text. An http-request node doing
`POST http://mqttcan.local/can/mark` works identically and returns the ack, which
is easier to debug against.

## Decoding workflow

`changed` in `/can/ids` is a bitmask of which payload bytes have **ever** moved.
That one column separates live signal bytes from constant padding, and it is the
fastest way in:

```sh
# clean baseline + a label, in one call
curl -X POST -d '{"text":"left indicator","reset":true}' http://mqttcan.local/can/mark
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

## Frame injection

`POST /can/inject` pushes a synthetic frame into the RX queue exactly where the
ISR would, so the worker task, frame table, noise masks, decoder and publishers
all handle it as real. It touches **no hardware**, so unlike the bench self-test
below it works while listen-only and is safe to use on the vehicle:

```sh
curl -X POST -d '{"id":"0x2BC","data":"FF5341000026F861"}' http://mqttcan.local/can/inject
```

`repeat` sends several copies and `increment` bumps one payload byte each time,
which reproduces a free-running counter. That is how the noise masks were
verified on the bike without ever leaving listen-only:

```sh
# D2 is inside 3FF's "1C" mask -> 8 frames in, 1 published
curl -X POST -d '{"id":"0x3FF","data":"497FC9BC1E78D001","repeat":8,"increment":2}' \
  http://mqttcan.local/can/inject

# D5 is the odometer, not masked -> 8 frames in, 8 published
curl -X POST -d '{"id":"0x3FF","data":"497FC9BC1E78D001","repeat":8,"increment":5}' \
  http://mqttcan.local/can/inject
```

Set `publish_min_ms` to 0 first, or the 200 ms floor will suppress the control
case too and both look identical.

Injected frames are **indistinguishable from real ones downstream**, which is
the point and also the hazard: they go into the frame table and out to MQTT like
anything else. `/can/status` reports an `injected` count so the provenance is
visible, and it stays set for the rest of the boot even after `POST /can/reset`,
because "this board has had fabricated data in it" is the fact worth keeping.
Clear the table afterwards, and remember a power cycle resets the counter.

## Driving lights

The one feature that makes this board *act* rather than only watch: click the
high beam three times inside a second and a pair of auxiliary driving lights
comes on, one click turns them off. No extra switch on the bars, no tap into the
loom beyond the relay feed.

The CAN side is unaffected. The TWAI controller stays in hardware listen-only
mode; the only thing that leaves the board is a logic level on a GPIO.

### What a click is

A click is a **complete short pulse**, not a transition:

```
high_beam -> "on"                        pulse starts
high_beam -> "off" within max_hold_ms    click counted, on release
...quiet for window_ms...                burst resolves, action fires
```

Counting on release with a hold limit is what keeps ordinary use out of the way:
holding the high beam on for a mile is one transition but never a click. A hold
longer than `max_hold_ms` also **cancels** a burst in progress, so "hold it on"
is a reliable way to abort a miscount.

`window_ms` runs from the **last** release, not the first, so a slow triple
still resolves as three rather than being truncated at two. The cost is that a
single click takes `window_ms` to act — the board cannot know a second one is
not coming.

**A single flash-to-pass switches the driving lights off.** That is deliberate
and was chosen knowingly; it is the price of `1 = off`. Change the `"1"` key to
`"2"` in the table if it becomes a nuisance on the road.

One inherent limit: `130` is cyclic, so a flash shorter than one frame period
never reaches the decoder at all. Nothing in firmware can recover that.

### Configuration

Both sections live in `data/signals.json`, so the pin assignment and the gesture
map are correctable over the air exactly like a byte mapping — `POST /signals`,
no reflash.

```jsonc
"outputs": {
  "driving": {
    "gpios": [],                   // the relay pins; empty configures nothing
    "active_low": false,           // true if the relay board closes on a low
    "auto_off_ms": 0,              // battery backstop; 0 = never
    "off_when": {"signal": "ignition", "is": "off"}
  }
},
"gestures": [
  {
    "name": "driving_lights",
    "signal": "high_beam",         // any enum signal in the decode table
    "trigger": "on",               // the value that counts as "pressed"
    "window_ms": 1000,
    "max_hold_ms": 600,
    "min_gap_ms": 60,              // contact-bounce floor
    "actions": {
      "1": {"output": "driving", "set": "off"},
      "3": {"output": "driving", "set": "on"}
    }
  }
]
```

`set` is `on`, `off` or `toggle`. A click count with no entry is published on
`tele/…/gesture` and otherwise ignored, which is how you find out the board saw
two when you meant three.

Nothing about this is specific to the high beam — `signal` is just a name from
the decode table, so the info button or the indicators can drive an output with
no firmware change.

### Where the GPIOs actually are: header P1

The board looks like it has no usable I/O — screw terminals, USB-C and one
SH1.0 UART plug (whose four wires are GND/VCC and the *console* UART, not spare
pins). There is an **internal 2×10 header, P1**, and it breaks out twelve free
GPIOs. It is inside the case, so reaching it means drilling or reprinting the
enclosure. This is not in the wiki page or the product listing; it came out of
[the schematic PDF](https://files.waveshare.com/wiki/ESP32-S3-RS485-CAN/ESP32-S3-RS485-CAN-Schematic.pdf).

| Pin | Net | Pin | Net |
|---|---|---|---|
| 1 | 3V3 | 2 | 5V |
| 3 | GND | 4 | GND |
| 5 | TXD (console) | 6 | D_P (USB) |
| 7 | RXD (console) | 8 | D_N (USB) |
| 9 | **IO3** ⚠ strapping | 10 | **IO14** |
| 11 | **IO4** | 12 | **IO13** |
| 13 | **IO5** | 14 | **IO12** |
| 15 | **IO6** | 16 | **IO11** |
| 17 | **IO7** | 18 | **IO10** |
| 19 | **IO8** | 20 | **IO9** |

The schematic's own GPIO allocation table assigns nothing to IO3–IO14 on this
board — only IO15/16 (CAN), IO17/18/21 (RS485) and IO43/44 (console) are spoken
for. So any of IO4–IO14 is fair game. **IO3 is a strapping pin** and the loader
rejects it.

In use here: `IO4` on pin 11 → relay `IN1`, `IO5` on pin 13 → relay `IN2`,
pin 3 or 4 → relay `DC−`.

### Wiring, and the pins you cannot use

`gpios` is validated on load and a bad pin is a 400 on `POST /signals`, not a
brick. Rejected on this board:

| Pins | Why |
|---|---|
| 15, 16 | the CAN transceiver |
| 26–37 | SPI flash **and the octal PSRAM** — this is an ESP32-S3R8 built with `CONFIG_SPIRAM_MODE_OCT`, so the range is wider than the usual quad-SPI one |
| 0, 3, 45, 46 | strapping pins |
| 19, 20 | USB D−/D+ |
| 43, 44 | console UART |

**Fit a pull-down at the relay input** (10 kΩ to ground for an active-high
module). ESP32-S3 pins float during reset and the first moments of boot, so
without it the lights flash on every reboot. The firmware parks each pin at its
inactive level before enabling the pad and sets a matching internal pull, but
that cannot cover the window before the firmware runs. Optocoupler isolation
does not help — a floating input can still bias the LED into conduction.

On a jumper-selectable relay module, set the trigger jumpers to **H**
(high-level) and leave `active_low` false. Low-level trigger is unreliable from
3.3 V logic: the module pulls the optocoupler anode to 5 V, so a logic high
still leaves ~1.7 V across an LED whose forward drop is ~1.2–1.4 V, and the
relay may never release.

Coil power comes from the bike's fused accessory circuit, not from the board.
There is no `VIN`/`5V` pin to borrow on the outside of this board — it is a
7–36 V screw terminal — so use a 12 V-coil module, or a 5 V one with its own
supply. P1 pin 2 does carry 5 V, but its current headroom is unspecified, so
do not run coils from it. Ground is already common: the board's logic is
referenced to `V−`, which is bike ground.

Feed the relay coils from the bike's fused accessory circuit, not from the
board. Boot state is always off and nothing is persisted, so a crash or reboot
mid-ride leaves the auxiliary lights dark — the bike's own headlight is on its
own circuit and unaffected.

### Control and the kill switch

```sh
curl -X POST -d '{"name":"driving","set":"on"}'     http://mqttcan.local/can/output
curl -X POST -d '{"name":"driving","set":"toggle"}' http://mqttcan.local/can/output
mosquitto_pub -h $B -t cmnd/$N/output -m '{"name":"driving","set":"off"}'

mosquitto_sub -h $B -v -t "tele/$N/gesture" -t "tele/$N/output"
```

`outputs_enable` is a live settings field. Clearing it forces every output off
**immediately** rather than at the next gesture, and makes further gestures
inert, without editing the decode table:

```sh
mosquitto_pub -h $B -t cmnd/$N/settings -m '{"outputs_enable":0}'
```

### Testing it without touching the bike

`POST /can/inject` feeds the decoder synthetic frames, so the whole gesture path
can be exercised on the vehicle while the controller stays listen-only. Leave
`gpios` empty and watch the MQTT side first.

```sh
ON='{"id":"0x130","data":"00000000000009CF"}'   # D6 low nibble 9 = high beam on
OFF='{"id":"0x130","data":"0000000000000ACF"}'  # A = off
click() { curl -sX POST -d "$ON" $B/can/inject; curl -sX POST -d "$OFF" $B/can/inject; }

click; click; click; sleep 1.5   # -> output_driving "on"
click;             sleep 1.5     # -> "off"
click; click;      sleep 1.5     # -> unchanged, clicks:2 is unmapped
```

Confirmed on-device 2026-09-15 on a freshly booted board: three clicks on, one
click off, two clicks unmapped, and a 2-second hold correctly counted as no
click at all. `/can/status` reports `outputs`, `gestures`, `outputs_enable` and
one `output_<name>` per output.

## Bench self-test

The older injector transmits real frames on the transceiver, for exercising the
controller itself with **no bus and no second CAN node**. Prefer `/can/inject`
above unless you specifically need the hardware path:

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
