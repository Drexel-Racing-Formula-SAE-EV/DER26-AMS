# Charger CAN Bring-Up Guide

This guide is for low-voltage bench validation of the plug-in charger CAN path.
Do not connect high voltage until the charger command, charger status frame,
BMS_OK behavior, and charge inhibit behavior are understood on a CAN sniffer.

## Expected Protocol

The firmware uses the TC/Elcon-style extended CAN charger protocol:

| Direction | Extended CAN ID | Period | Payload |
|---|---:|---:|---|
| AMS/BMS to charger | `0x1806E5F4` | `1000 ms` | max terminal voltage, max charge current, control byte |
| Charger to AMS/BMS | `0x18FF50E5` | about `1000 ms` | output voltage, output current, status flags |

AMS command payload:

| Byte | Meaning | Scaling |
|---:|---|---|
| 0..1 | max allowable charging terminal voltage | big-endian, `0.1 V/bit` |
| 2..3 | max allowable charging current | big-endian, `0.1 A/bit` |
| 4 | control | `0 = enable/start`, `1 = battery protection / close output` |
| 5..7 | reserved | currently `0` |

Current firmware targets:

| Setting | Value |
|---|---:|
| `CHARGE_MAX_VOLTAGE` | `312.0 V` |
| `CHARGE_MAX_CURRENT` | `10.0 A` |
| command period | `1000 ms` |
| status RX timeout | `5000 ms` |

The physical CM200DC/charger configuration still must be confirmed to match
the same CAN IDs, baud rate, voltage limit, current limit, and control-byte
polarity.

## Low-Voltage Test Order

1. Power the AMS and charger low-voltage side only.
2. Confirm the CAN bus is at `250 kbit/s`.
3. Attach a CAN sniffer to the AMS/charger CAN bus.
4. Use the CLI to enter charge state:
   ```text
   state charge
   charger
   bmsok status
   ```
5. Confirm AMS transmits extended ID `0x1806E5F4` once per second.
6. Confirm bytes `0..4` while charge is allowed:
   ```text
   0C 30 00 64 00
   ```
   This means `312.0 V`, `10.0 A`, enable/start.
7. Confirm the charger responds on extended ID `0x18FF50E5`.
8. Run `charger` again and verify `rx_count`, `read_voltage`, `read_current`,
   `flags`, and `rx_age_ms` update.
9. Force a safe inhibit condition and confirm byte 4 becomes `01`.
   Good first tests are:
   ```text
   bmsok inhibit
   charger
   ```
10. Return to a safe non-charge state when finished:
    ```text
    state discharge
    bmsok inhibit
    charger
    ```

## What To Look For

If the charger does not start:

| Symptom | Likely area |
|---|---|
| No `0x1806E5F4` frame | AMS CAN TX, charge state, CAN task, BMS_OK gates |
| `0x1806E5F4` present but no ACK/status | wiring, termination, baud rate, charger config, charger power |
| Byte 4 is `01` | AMS is intentionally disabling charge; check `charger` disable mask |
| `tx_fail:1` | AMS could not place the charger command on CAN |
| `rx_comm:1` | AMS has not received charger status for more than `5000 ms` |
| charger flags nonzero | charger is reporting its own hardware/status fault |
| target current is not expected | confirm `CHARGE_MAX_CURRENT` and charger configuration |

## CLI Fields

The `charger` CLI command prints:

| Field | Meaning |
|---|---|
| target voltage/current | command values the AMS is sending |
| read voltage/current | latest parsed charger status values |
| raw flags | byte 4 from the charger status frame |
| `rx_comm` | charger status timeout |
| `tx_fail` | AMS CAN transmit failure for charger command |
| `txfail` | count of failed charger command transmissions |
| `disable_mask` | bitmask of AMS reasons for sending disable byte `1` |
| CAN IDs / byte4 values | quick check against the sniffer |

## Firmware Safety Behavior

In charge state the charger command is sent before the telemetry burst. If the
charger command transmit fails, firmware sets `tx_fail`, marks charger control
faulted, increments `tx_fail_count`, and drops BMS_OK.

CAN auto-retransmission is enabled in STM32 CAN init so arbitration loss or a
transient failed attempt is retried by the CAN peripheral.

Charger disable does not always mean BMS_OK should be forced low. A voltage
charge-stop condition sends charger byte 4 as `01` so the charger backs off, but
it leaves BMS_OK under the normal measurement/fault gates so controlled
balancing can continue. Safety-critical charger disable reasons still force
BMS_OK low, including charger hardware/status fault, charger command TX failure,
hard fault, invalid voltage/current data, voltage/temp/current fault, and
temperature charge-stop.
