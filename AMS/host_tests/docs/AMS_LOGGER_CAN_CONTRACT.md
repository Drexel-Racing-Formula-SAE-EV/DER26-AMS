# AMS Dashboard/Logger CAN Contract


Authored by Mahad Faisal, 2026.
This contract is for a passive in-car ESP32 dashboard/logger. The ESP32 may
display, log, and stream telemetry, but it must not be required for BMS_OK,
charger control, ADBMS SPI/isoSPI communication, AIR control, or any other AMS
safety output.

## Bus Assumptions

| Item | Value |
|---|---:|
| CAN type | Classic CAN 2.0 |
| AMS CAN peripheral | STM32 CAN1 |
| Current firmware bitrate | 250 kbit/s |
| AMS CAN pins | PD0 RX, PD1 TX |
| Logger role | Passive listener |
| Logger transmit requirement | None |

The STM32 CAN timing in `Core/Src/main.c` is currently:

| Field | Value |
|---|---:|
| APB1 CAN clock | 54 MHz |
| Prescaler | 12 |
| Sync | 1 TQ |
| TimeSeg1 | 15 TQ |
| TimeSeg2 | 2 TQ |
| Bitrate | 54 MHz / 12 / 18 = 250 kbit/s |

Do not reuse a 500 kbit/s ESP32/MCP2515 setup without changing its CAN timing.

## Safety Boundary

| Rule | Requirement |
|---|---|
| Unplug safety | AMS behavior must be identical if the ESP32 is missing. |
| No safety authority | ESP32 must not gate BMS_OK, charger enable, AIRs, or fault latches. |
| No ADBMS bus access | ESP32 must not share the ADBMS SPI/isoSPI bus. |
| First implementation | Receive-only CAN logging. |
| Future commands | Only in explicit service/debug firmware modes, never race/charge default. |

## Existing ECU Telemetry

The existing ECU stream remains unchanged:

| ID | Type | Rate | Payload |
|---:|---|---:|---|
| `0x069` | Standard | `CAN_FREQ`, currently 2 Hz | Packet number plus three 16-bit words |

Packets `0..61` contain status, cell voltage, partial temperature, and fan data.
That stream is kept for compatibility.

## Logger Summary/Diagnostic Frames

The dashboard/logger summary and diagnostic stream uses standard IDs `0x690..0x69F`, plus bench-focused diagnostic IDs `0x6A6..0x6A7`.
Multi-byte values are big-endian.

| ID | Frame | Bytes |
|---:|---|---|
| `0x690` | Heartbeat/status | `version`, `seq`, `state`, status flags, validity flags, current flags, uptime seconds |
| `0x691` | Fault reasons | voltage reason, voltage latched reason, temp reason, temp pending reason, temp latched reason, current reason, current latched reason, current mode |
| `0x692` | Pack electrical | pack voltage 0.1 V/bit, current 0.1 A/bit signed, min cell mV, max cell mV |
| `0x693` | Temperature/fan | max temp 0.1 C/bit signed, min temp, avg temp, max fan duty %, temp flags |
| `0x694` | Voltage health | max cell location, min cell location, usable/updated/stale/PEC-fail counts |
| `0x695` | Temperature health | max temp location, min temp location, usable/updated/stale/invalid counts |
| `0x696` | Charger | target voltage 0.1 V/bit, target current 0.1 A/bit, read voltage 0.1 V/bit, charger flags, charger raw flags |
| `0x697` | Current detail | current 0.1 A/bit signed, selected range, measurement reason, current reason, latched reason, pending ms |
| `0x698` | ADBMS6830 link | last HAL status, last xfer status, last op, error count, PEC fail mask, command-counter mismatch mask |
| `0x699` | ADBMS6830 counters | error count, command-counter error count, PEC pass mask, last command bytes |
| `0x69A` | ADBMS2950/APM link | last HAL status, last xfer status, last op, error count, PEC fail mask, debug enabled, IC count |
| `0x69B` | Task health | stale heartbeat mask, seen heartbeat mask, safety-stale mask, heartbeat fault flags, logger heartbeat count |
| `0x69C` | CAN diagnostics | error code, bus-off count, error count, recovery count, bus-off/recovery flags |
| `0x69D` | Safety diagnostics | reset flags, last panic reason, panic count, BMS output block count, safety flags |
| `0x69E` | Watchdog diagnostics | watchdog flags, last block reason, feed count, block count, last feed age |
| `0x69F` | ADBMS diagnostics | scan count, diag counters, last diag status, ADBMS/HIL flags |
| `0x6A6` | Current ADC diagnostics | high/low ADC counts, selected range, measurement reason, current ADC flags, zero-cal capture count |
| `0x6A7` | Charger detail | read current, disable reason mask, last TX status, TX/RX/fail counters |


### `0x6A6` Current ADC Flags

| Byte | Bit | Meaning |
|---:|---:|---|
| 6 | 0 | High/800 A ADC sample fresh |
| 6 | 1 | Low/50 A ADC sample fresh |
| 6 | 2 | Last ADC read OK |
| 6 | 3 | Current measurement valid |
| 6 | 4 | Current sensor fault active |
| 6 | 5 | Software zero-current calibration active |
| 7 | all | Zero-current calibration capture count, saturated to 255 |

### `0x690` Status Flags

| Byte | Bit | Meaning |
|---:|---:|---|
| 3 | 0 | BMS_OK state |
| 3 | 1 | AIR state |
| 3 | 2 | IMD OK |
| 3 | 3 | hard fault |
| 3 | 4 | soft fault |
| 3 | 5 | charger fault |
| 3 | 6 | CAN bus fault |
| 3 | 7 | BMS_OK output inhibited |
| 4 | 0 | voltage valid |
| 4 | 1 | current valid |
| 4 | 2 | temp valid |
| 4 | 3 | voltage read fault |
| 4 | 4 | temp read fault |
| 4 | 5 | current sensor fault |
| 4 | 6 | voltage fault latched |
| 4 | 7 | temp fault latched |
| 5 | 0 | current fault |
| 5 | 1 | current sensor fault |
| 5 | 2 | current warning |
| 5 | 3 | current pending |
| 5 | 4 | current confirmed |
| 5 | 5 | current latched |
| 5 | 6 | fuse fault |
| 5 | 7 | estimator fault |

### `0x696` Charger Flags

| Byte | Bit | Meaning |
|---:|---:|---|
| 6 | 0 | charger hardware fail |
| 6 | 1 | charger overtemp fail |
| 6 | 2 | charger input voltage fail |
| 6 | 3 | charger voltage sense fail |
| 6 | 4 | charger communication fail |
| 6 | 5 | AMS temp charge stop |
| 6 | 6 | AMS voltage charge stop |
| 6 | 7 | AMS command disables charger |

### `0x69B` Task Health

Heartbeat bit order:

| Bit | Task/path |
|---:|---|
| 0 | ADBMS voltage path |
| 1 | current sensor task |
| 2 | temperature sensing path |
| 3 | CAN task |
| 4 | logger/dashboard telemetry path |

Payload:

| Bytes | Meaning |
|---:|---|
| 0-1 | stale heartbeat mask |
| 2-3 | seen heartbeat mask |
| 4-5 | safety-critical stale mask |
| 6 bit 0 | safety heartbeat fault |
| 6 bit 1 | logger/dashboard heartbeat fault |
| 7 | saturated logger heartbeat counter |


### `0x69C` CAN Diagnostics

Payload:

| Bytes | Meaning |
|---:|---|
| 0-3 | Last HAL CAN error code, big-endian uint32 |
| 4 | Saturated bus-off counter |
| 5 | Saturated CAN error counter |
| 6 | Saturated CAN recovery counter |
| 7 bit 0 | bus-off fault active |
| 7 bit 1 | recovery pending |

The dashboard decodes this frame for diagnostics only. It remains a passive
listener and must not acknowledge, clear, or mask AMS CAN faults.

### `0x69D` Safety Diagnostics

| Bytes | Meaning |
|---:|---|
| 0-3 | RCC reset flags, big-endian uint32 |
| 4 | Saturated last panic reason |
| 5 | Saturated retained panic/reset count |
| 6 | Saturated BMS_OK output block count |
| 7 bit 0 | safety panic active |
| 7 bit 1 | BMS_OK output inhibited |
| 7 bit 2 | balancing inhibited |
| 7 bit 3 | BMS_OK state |
| 7 bit 4 | hard fault active |
| 7 bit 5 | safety task heartbeat fault |
| 7 bit 6 | logger heartbeat fault |

### `0x69E` Watchdog Diagnostics

| Bytes | Meaning |
|---:|---|
| 0 bit 0 | watchdog runtime gate enabled |
| 0 bit 1 | hardware IWDG started |
| 0 bit 2 | feed gate currently OK |
| 1 | last watchdog block reason |
| 2-3 | saturated watchdog feed count |
| 4-5 | saturated watchdog block count |
| 6-7 | last feed age in 0.1 s/bit, `0xFFFF` if never fed |

### `0x69F` ADBMS Diagnostics

| Bytes | Meaning |
|---:|---|
| 0-1 | saturated ADBMS scan count |
| 2 | saturated status diagnostic count |
| 3 | saturated config diagnostic count |
| 4 | saturated open-wire diagnostic count |
| 5 | last ADBMS diagnostic HAL status |
| 6 bit 0 | ADBMS diagnostic fault |
| 6 bit 1 | config fault |
| 6 bit 2 | status fault |
| 6 bit 3 | open-wire fault |
| 6 bit 4 | ADBMS scan active |
| 6 bit 5 | HIL ADBMS-image replacement compiled in |
| 7 bit 0 | HIL measurement frame fresh |
| 7 bit 1 | HIL truth frame fresh |
| 7 bit 2 | HIL summary frame fresh |

## Dashboard-Decoded Non-Logger Frames

The dashboard also decodes these standard IDs when present, but they are not
required for normal in-car logging:

| ID | Frame | Payload |
|---:|---|---|
| `0x200` | HIL measurement | plant pack voltage/current/surface temp and counter |
| `0x201` | HIL truth | true SoC/core temp/counter/plant step |
| `0x202` | HIL summary | min/max cell voltage and max/avg temp |
| `0x421` | estimator status | active instance, flags, SoC, innovation, R0 |

Known non-dashboard frames such as ECU packet `0x069`, HIL image frames
`0x210/0x211`, and extended charger frames are counted as ignored, not unknown.
This keeps `unknown_frames` useful for real ID/contract mistakes.

## Logger Detail Frames

The detail stream exports all 75 cell voltages and all 120 thermistors. It is
intended for SD logging and WiFi dashboards. Multi-byte values are big-endian.

| ID | Frame | Payload |
|---:|---|---|
| `0x6A0` | Cell detail | `seg`, `start_cell`, three cell voltages in mV |
| `0x6A1` | Temp detail | `seg`, `start_sensor`, three temps in 0.1 C/bit signed |
| `0x6A2` | Voltage masks | `seg`, updated mask, usable mask, stale mask |
| `0x6A3` | Temp masks A | `seg`, 24-bit updated mask, 24-bit usable mask |
| `0x6A4` | Temp masks B | `seg`, 24-bit stale mask, 24-bit invalid mask |
| `0x6A5` | Voltage PEC | `seg`, PEC fail mask, PEC fail count |

Invalid or unusable temperatures are encoded as `0x8000`, not zero. Zero is a
plausible real temperature and must not be used as a missing-data value.

Invalid or unusable cell voltages are encoded as `0 mV`. Consumers should use
the voltage masks before treating a voltage as valid.

## Frame Count

At each non-charge CAN task tick, AMS sends:

| Stream | Frames |
|---|---:|
| Existing ECU `0x069` stream | 62 |
| Logger summaries/diagnostics | 18 |
| Logger cell details | 25 |
| Logger temp details | 40 |
| Logger masks/diagnostics | 20 |
| Total without estimator frame | 165 |

In charge mode, AMS still sends the logger stream, then sends the charger command
frame on extended ID `0x1806E5F4`.

At 250 kbit/s and `CAN_FREQ = 2`, this is acceptable for bench/dashboard logging,
but any future vehicle-wide bus load change should be checked before raising the
rate.

## ESP32 Dashboard Plan

Recommended hardware:

| Option | Recommendation |
|---|---|
| ESP32 native TWAI + CAN transceiver | Preferred for a fresh in-car logger |
| ESP32 + MCP2515 | Acceptable if already available; configure for 250 kbit/s |
| UART from AMS | Debug-only fallback, not primary dashboard logging |

Recommended ESP32 tasks:

| Task | Role |
|---|---|
| CAN RX | Receive and timestamp frames; no transmit required |
| Decoder | Maintain latest AMS state from `0x690..0x6A7`, safety/CAN/watchdog diagnostics `0x69C..0x69F`, current ADC `0x6A6`, charger detail `0x6A7`, estimator status `0x421`, and optional HIL frames `0x200..0x202` |
| SD logger | Write raw CAN and decoded CSV |
| WiFi dashboard | Stream latest decoded state over WebSocket or UDP |
| Watchdog/status | Show stale logger data if heartbeat sequence stops |

The ESP32 should log raw frames even if the decoder has a bug. Raw CAN logs are
the recovery path for post-run analysis.
