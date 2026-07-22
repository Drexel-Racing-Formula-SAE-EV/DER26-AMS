# DER26 ESP32 AMS Dashboard/Logger

This is a passive in-car ESP32 WiFi dashboard/logger for the DER26 AMS CAN
logger stream. It is separate from `HiL/esp32_plant`: the plant node is a bench
simulation source, while this project only listens to the real AMS CAN bus and
serves decoded telemetry over WiFi.

## Safety Boundary

| Rule | Requirement |
|---|---|
| CAN role | Receive-only listener |
| AMS authority | No BMS_OK, charger, AIR, fault-latch, SPI, or isoSPI control |
| Failure behavior | Unplugging this ESP32 must not change AMS behavior |
| Logger data | Treat as telemetry/debug, not safety authority |

Do not wire this ESP32 to the ADBMS SPI/isoSPI path. It belongs only on the
low-voltage CAN bus.

## Hardware

The wiring matches the existing ESP32 plant-node MCP2515 reference so the same
adapter can be reused for dashboard bring-up.

| ESP32 pin | MCP2515 module pin | Notes |
|---:|---|---|
| GPIO23 | SI / MOSI | SPI MOSI |
| GPIO19 | SO / MISO | SPI MISO |
| GPIO18 | SCK | SPI clock |
| GPIO5 | CS | SPI chip select |
| GPIO4 | INT | Optional; current driver polls instead |
| 3V3 | VCC | Use a 3.3 V-compatible module or level shifting |
| GND | GND | Common low-voltage ground |
| CAN_H | CAN_H | Connect to AMS low-voltage CAN bus |
| CAN_L | CAN_L | Connect to AMS low-voltage CAN bus |

The AMS CAN firmware is currently configured for **250 kbit/s**. This project
configures the MCP2515 for 250 kbit/s using an 8 MHz crystal. Do not reuse a
500 kbit/s MCP2515 sketch without changing timing.

Verify the CAN bus already has proper termination. Do not add an extra
termination resistor unless the physical bus needs it.

## WiFi Dashboard

The ESP32 starts a WiFi access point:

| Field | Value |
|---|---|
| SSID | `DER26_AMS_DASH` |
| Password | `der26amslogger` |
| URL | `http://192.168.4.1/` |

Endpoints:

| Endpoint | Purpose |
|---|---|
| `/` | Browser dashboard |
| `/api/state` | Full decoded JSON state |
| `/api/snapshot.csv` | One-line CSV snapshot |

The web UI marks the data stale if the AMS heartbeat is not updated within
1500 ms. While the page is open, the browser also stores a rolling CSV log of
the latest dashboard samples and exposes a **Download CSV log** button. This is
useful for quick in-car/pit telemetry capture. It is not a persistent SD-card
logger; if the browser disconnects, only the ESP32's latest decoded state
remains available.

## Build

From this folder with ESP-IDF activated:

```bash
idf.py set-target esp32
idf.py build
```

Flash and monitor:

```bash
idf.py -p <PORT> flash monitor
```

Expected monitor output after boot:

```text
MCP2515 ready: 250 kbit/s @ 8 MHz
WiFi AP started: SSID=DER26_AMS_DASH password=der26amslogger
Open http://192.168.4.1/
AMS dashboard/logger running
```


The decoder includes the expanded AMS diagnostic frames:

| Frame | Data exposed |
|---:|---|
| `0x69C` | CAN error code, bus-off counter, recovery counter, bus-off/recovery flags |
| `0x69D` | reset flags, panic reason/count, BMS_OK block count, safety flags |
| `0x69E` | watchdog runtime/hardware/feed-gate state, feed/block counts, last-feed age |
| `0x69F` | ADBMS scan/diagnostic counters, last diagnostic status, ADBMS/HIL flags |
| `0x6A6` | current ADC high/low counts, selected range, measurement reason, ADC validity flags, zero-cal capture count |
| `0x6A7` | charger read current, disable reason mask, TX/RX/fail counters |
| `0x6A8..0x6AB` | temperature/fan and voltage plausibility diagnostics |
| `0x6AC` | RTOS heap free/min-free, stack warning mask, minimum stack high-water mark, RTOS flags |
| `0x421` | estimator active instance, flags, SoC, innovation, R0 |
| `0x684` | 1 s DCL/current, discharge power, flags, binding and limiting segment |
| `0x685` | 1 s CCL/current, charge power, flags, binding and limiting segment |
| `0x686` | capacity SoH, conservative lower bound, resistance growth and confidence |
| `0x687` | 0.1/10/30 s discharge and charge current envelope |
| `0x689` | Advisory mission profile, fuse utilization, thermal readiness and R0 bootstrap progress |

`0x688` is the CRC/counter-protected ECU-to-AMS mission request. It is not a
logger authority path. The four-frame `0x684`-`0x687` bundle remains the only
required fail-zero power contract; loss of advisory `0x689` does not invalidate
the hard bundle.
| `0x200..0x202` | optional HIL plant measurement/truth/summary frames when present |

Known non-dashboard frames such as ECU compatibility packet `0x069`, HIL image
frames `0x210/0x211`, and extended charger frames are counted as ignored rather
than unknown. This keeps `/api/state.unknown_frames` useful for real CAN contract
mistakes. The ESP32 dashboard remains receive-only and never clears AMS faults. Stack overflow, malloc failure, and RTOS assert are fatal on the AMS side; low stack/heap watermark is displayed as warning telemetry.

## Host Decoder Test

The CAN decoder is pure C and can be tested without ESP-IDF:

```bash
make -C tests test
```

This checks the AMS logger frame layout, stale heartbeat detection, invalid
temperature sentinel handling, masks, safety/watchdog/ADBMS diagnostics,
estimator/HIL decode, SoP/SoH CRC/version/counter/freshness handling, and clean
handling of ignored versus unknown frames.

The dashboard is not a safety consumer. It decodes each power-frame stream for
display and marks the power object stale if any required frame is older than
250 ms. The production ECU contract is stricter: it stages all four frames as
one same-counter bundle, limits skew to 50 ms, and requires two consecutive
complete bundles after startup or a fault. See
`../../Docs/SOP_SOH_CAN_CONTRACT.md`.

## Bring-Up Checklist

1. Flash the ESP32 dashboard/logger.
2. Power the MCP2515 module from a compatible 3.3 V supply.
3. Confirm the MCP2515 crystal is 8 MHz, or update CNF timing if it is not.
4. Connect only to the low-voltage CAN bus: CAN_H, CAN_L, and appropriate
   low-voltage ground reference.
5. Boot the AMS and confirm CAN is running at 250 kbit/s.
6. Connect a laptop or phone to `DER26_AMS_DASH`.
7. Open `http://192.168.4.1/`.
8. Confirm `/api/state` shows `stale:false`, increasing `rx_frames`, and
   heartbeat `version:1`.
9. If stale remains true, check CAN_H/CAN_L polarity, termination, MCP2515
   crystal frequency, and bitrate.

## Contract Source

The decoded frame definitions come from:

```text
AMS/host_tests/docs/AMS_LOGGER_CAN_CONTRACT.md
```

If AMS CAN IDs, bit flags, scaling, or invalid sentinels change, update the
contract, this decoder, and the host decoder tests together.
