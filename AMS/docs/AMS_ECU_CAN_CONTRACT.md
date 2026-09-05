# AMS to ECU CAN Contract

This document is the bench-test CAN contract for the DER26 AMS compact telemetry frames.
It is written for firmware members who need to understand what the ECU can safely consume
from AMS without parsing the full legacy cell/temperature telemetry stream.

## Scope

These frames are for staged bench testing and early ECU integration.
They summarize AMS measurement health and pack state.
They do **not** claim that full accumulator bring-up is complete.

Out of scope for this contract right now:

- AIR+ / AIR- / precharge sequencing
- firmware-owned IMD PWM decoding
- inverter torque command logic
- charge/discharge current limit calculation
- full HV-ready accumulator state machine

The hardwired shutdown path remains the authority for safety items that are not yet owned
by AMS firmware.

`AIR_CONTROL_MCU` is currently command/control-voltage sense only. Any legacy/logger
field named `AIR_STATE` carries that raw control-net indication; it does not prove AIR+,
AIR- or precharge physical contact state. Future auxiliary-contact supervision is defined
in `AIR_CONTACTOR_MONITORING.md` and must receive a versioned CAN assignment before the
ECU consumes it as physical feedback.

## CAN format

- CAN type: Standard 11-bit identifiers
- DLC: 8 bytes
- Byte order for 16-bit fields: big-endian, MSB first
- Send rate: 10 Hz target
- ECU freshness check: use the rolling sequence byte in frame `0x680`

Firmware freezes all four compact-frame inputs before sending a bundle, so a
mid-bundle task update cannot mix old and new thermal/fan/fault state. Only
`0x680` currently carries the bundle sequence, however. A future SoP contract
must carry a common sequence and source-measurement timestamp in every
multi-frame publication; this bench contract must not be represented as that
future authoritative SoP interface.

The ECU should treat these compact frames as the high-priority AMS heartbeat. The older
paged telemetry and logger frames may still be transmitted, but ECU torque gating should
not require parsing those slower bulk frames.

## Frame 0x680: AMS status and fault summary

| Byte | Field | Meaning |
|---:|---|---|
| 0 | Protocol version | Currently `1` |
| 1 | Sequence | Rolling 8-bit counter, increments once per compact frame group |
| 2 | AMS state | Raw `app_state_t` value from AMS firmware |
| 3 | Primary flags | See bit table below |
| 4 | Fault / diagnostic flags | See bit table below |
| 5 | Voltage fault reason | Raw `voltage_fault_reason_t` value |
| 6 | Temperature fault reason | Raw `temp_fault_reason_t` value |
| 7 | Current fault reason | Raw `current_fault_reason_t` value |

### Byte 3 flags

| Bit | Name | ECU use |
|---:|---|---|
| 0 | `BMS_OK` | Required true before ECU allows torque |
| 1 | `BMS_OUTPUT_INHIBIT` | If true, treat AMS safety output as intentionally inhibited |
| 2 | `HARD_FAULT` | If true, block torque |
| 3 | `SOFT_FAULT` | If true, block or derate torque depending on ECU policy |
| 4 | `VOLTAGE_VALID` | Required true before ECU trusts voltage fields |
| 5 | `CURRENT_VALID` | Required true before ECU trusts current fields |
| 6 | `TEMP_VALID` | Required true before ECU trusts thermal fields |
| 7 | `CANBUS_FAULT` | AMS has detected CAN TX/RX health issues |

### Byte 4 flags

| Bit | Name | ECU use |
|---:|---|---|
| 0 | `VOLTAGE_FAULT` | If true, block torque |
| 1 | `TEMP_FAULT` | If true, block torque |
| 2 | `CURRENT_FAULT` | If true, block torque |
| 3 | Reserved / IMD firmware supervision not implemented | Do **not** use as IMD OK |
| 4 | `CHARGER_FAULT` | Charge-path fault indication |
| 5 | `ADBMS_DIAG_FAULT` | ADBMS diagnostic fault indication |
| 6 | `TASK_HEARTBEAT_FAULT` | Internal RTOS task heartbeat fault |
| 7 | `LOGGER_HEARTBEAT_FAULT` | Logger/telemetry heartbeat fault |

Important: byte 4 bit 3 is deliberately held at `0` until AMS firmware actually decodes
and validates the IMD input. During the current bench stage, IMD supervision is offloaded
to the hardwired safety/shutdown path. ECU firmware must not treat this bit as IMD OK.

## Frame 0x681: Electrical summary

| Bytes | Field | Units / encoding |
|---:|---|---|
| 0-1 | Pack voltage | unsigned, 0.1 V/count |
| 2-3 | Pack current | signed, 0.1 A/count |
| 4-5 | Minimum cell voltage | unsigned, mV |
| 6-7 | Maximum cell voltage | unsigned, mV |

ECU rules:

- Only trust pack voltage and min/max cell voltage if `0x680` byte 3 bit 4 is true.
- Only trust pack current if `0x680` byte 3 bit 5 is true.
- Any voltage/current fault bit should block torque even if the numeric fields look normal.

## Frame 0x682: Thermal and fan summary

| Bytes | Field | Units / encoding |
|---:|---|---|
| 0-1 | Maximum temperature | signed, 0.1 deg C/count |
| 2-3 | Minimum temperature | signed, 0.1 deg C/count |
| 4-5 | Filtered average temperature | signed, 0.1 deg C/count |
| 6 | Maximum fan command | unsigned percent, 0-100 |
| 7 | Thermal/fan flags | See bit table below |

### Byte 7 flags

| Bit | Name | ECU use |
|---:|---|---|
| 0 | `TEMP_WARNING` | Thermal warning/early derate candidate |
| 1 | `TEMP_FAN_MAX` | AMS has commanded max fan behavior |
| 2 | `TEMP_CHARGE_STOP` | Charging should be stopped |
| 3 | `TEMP_OVERTEMP_PENDING` | Overtemperature debounce/pending state |
| 4 | `OVERTEMP_FAULT` | Block torque/charge |
| 5 | `SEVERE_OVERTEMP_FAULT` | Hard block torque/charge |
| 6 | `FAN_FAULT` | Cooling actuator/path fault |
| 7 | `TEMP_INVALID_OR_READ_FAULT` | Do not trust thermal fields |

ECU rules:

- Only trust thermal fields if `0x680` byte 3 bit 6 is true and `0x682` byte 7 bit 7 is false.
- Any overtemperature or severe overtemperature fault should block torque.
- Fan information is diagnostic/status data; it is not a substitute for temperature validity.

## Frame 0x683: Measurement health locations

| Byte | Field | Meaning |
|---:|---|---|
| 0 | Max-voltage segment | Segment index containing max cell voltage |
| 1 | Max-voltage cell | Cell index containing max cell voltage |
| 2 | Min-voltage segment | Segment index containing min cell voltage |
| 3 | Min-voltage cell | Cell index containing min cell voltage |
| 4 | Max-temperature segment | Segment index containing max temperature |
| 5 | Max-temperature sensor | Sensor index containing max temperature |
| 6 | Usable cell count | Saturated to 255 |
| 7 | Usable temperature sensor count | Saturated to 255 |

ECU rules:

- Use this frame for debugging, dashboard details, and sanity checks.
- Do not use this frame alone to decide torque enable. Torque gating should primarily use
  `0x680` validity/fault flags plus `0x681` and `0x682` numeric summaries.

## Minimum ECU torque gate from AMS

Before ECU sends nonzero torque or inverter enable, it should require:

- `0x680` is fresh; sequence must be changing at the expected rate.
- `BMS_OK == 1`.
- `HARD_FAULT == 0`.
- `VOLTAGE_VALID == 1`, `CURRENT_VALID == 1`, and `TEMP_VALID == 1`.
- `VOLTAGE_FAULT == 0`, `CURRENT_FAULT == 0`, and `TEMP_FAULT == 0`.
- `ADBMS_DIAG_FAULT == 0`.
- `TASK_HEARTBEAT_FAULT == 0`.
- Thermal fatal flags in `0x682` are clear.

Do not use any reserved bit as a safety-good signal.

## Frame 0x68B: Current source and freshness diagnostic

This fifth compact frame is transmitted with every `0x680-0x683` bundle. It is
observability data and does not independently grant ECU torque authority.

| Byte | Field | Meaning |
|---:|---|---|
| 0 | Current source | `0` unavailable, `1` DHAB/current-sensor path, `2` ADBMS2950/APM path |
| 1 | Current quality | `0` invalid, `1` valid without proven calibration, `2` calibrated primary, `3` calibrated primary plus valid redundant sample |
| 2 | Physical boundary valid | `1` only when the selected source is current-valid and fresh |
| 3 | Source epoch | Boot/source epoch; currently `1` |
| 4-5 | Sample sequence | Unsigned rolling current-window sequence for canonical DHAB data; source-sample sequence for live fallback/APM, big-endian |
| 6-7 | Sample age | Unsigned milliseconds, saturated at 65535, big-endian |

For the standalone ADBMS2950 evaluation image, source `2` is selected only after
the ADBMS2950 health object reports a valid current sample. A stale or invalid
sample is transmitted as source `0`, quality `0`, and boundary-valid `0`.

The ECU records this frame in decoded and raw logs. It must continue to use
`0x680` current validity and the authoritative power bundle for torque gating.

From firmware v0.5.22 / contract suffix CURRENT2, canonical DHAB quality, age and
sequence all come from the same immutable current window as the electrical
frame. Valid DHAB data without a confident calibration record has quality 1.
Canonical data expires when its latest sample exceeds 100 ms, independently
of the slower whole-measurement cache timeout. Live fallback without snapshot
provenance never claims calibration. IDs, payload size and power authority are
unchanged; log decoders should display quality 1 as valid/unproven calibration.

## Passive logger/diagnostic stream 0x690-0x6C0

AMS also publishes a read-only telemetry stream for ECU SD logging and external
dashboard/logger nodes. This stream includes detailed faults, ADBMS link health,
cell/temperature detail, RTOS/CAN/watchdog diagnostics, snapshot metadata, and
ADBMS2950 samples, CAN scheduler diagnostics, passive estimator voltage-product
comparison, and test-day estimator/SoP/fuse tuning. Its payload contracts are
documented in `AMS_LOGGER_CAN_CONTRACT_V1.md` and
`AMS_TUNING_CAN_SD_CONTRACT.md`.

These frames are never required for BMS_OK, charger control, ADBMS operation, or
ECU torque permission. A congested bus suppresses detail traffic after a compact
bundle failure so compact safety/status traffic retains priority.
