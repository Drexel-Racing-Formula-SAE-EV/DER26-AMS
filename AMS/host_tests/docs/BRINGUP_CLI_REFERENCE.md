# Bring-Up CLI Reference

Authored by Mahad Faisal, 2026.

The `bringup` command is a staged bench-debug layer on top of the lower-level
`status`, `spi`, `apm`, `charger`, `current`, `volt`, `temp`, `fault`, and
`bmsok` commands. It summarizes whether the next hardware step is sensible; it
does not release BMS_OK, clear safety latches, send extra charger commands, or
override normal safety gating.

## Command Summary

| Command | Phase | What It Answers |
|---|---|---|
| `bringup board` | LV board-only | Did the firmware boot, is SPI6 mode 3, is BMS_OK inhibited, and is DHAB zero-current alive if powered? |
| `bringup adbms6830` | SMB/ADBMS6830 chain | Are SPI mode, CS path, response bytes, PEC, SID, status, and diagnostic health moving in the right direction? |
| `bringup apm2950` | APM debug-only | Is the ADBMS2950/APM probe observable while remaining explicitly non-safety-gating? |
| `bringup charger-lv` | Charger low-voltage CAN | Are the charger CAN IDs, command frame, and `BYTE5/data[4]` polarity clear for sniffer validation? |
| `bringup charger-battery` | Charger with safe battery path | Are BMS_OK, fresh charger RX, voltage, current, temp, and charger gates clean enough for a battery-connected charger test? |
| `bringup ready` | BMS_OK release review | Would the normal safety gates allow release? This command does not run `bmsok release`. |
| `bringup snapshot` | Any phase | Compact state/fault snapshot for logs. |
| `bringup evidence` | Any phase | List of CLI outputs and bench artifacts to capture before changing phase. |

## First Board-Only Flow

Use this when the AMS board is powered from low voltage and is not connected to
an accumulator.

```text
status
bringup board
bmsok status
fault
```

Expected:

```text
BMS_OK remains 0
output_inhibit remains 1 in AMS_HW_BRINGUP builds
spi6=PASS
voltage/temp may be not_ready without cells
current_zero=PASS if the DHAB sensor is LV-powered and sitting at mid-scale
```

If `current_zero=WARN`, check whether the harness actually powers the DHAB. A
warning is acceptable for a board-only harness that omits the current sensor
excitation; it is not acceptable if the DHAB is powered and should be reporting
mid-scale zero current.

## ADBMS6830 / SMB Flow

The old firmware state was reportedly not communicating, so this is the first
real proof point after flashing the fixes.

```text
spi clear
spi enable
spi pins
spi cspins both 10
spi cs b pulse 10
spi preset normal
spi scope
spi coldwake
spi probea
spi probeb
spi sid
spi stat
spi cfgchk
bringup adbms6830
```

Use `spi pins; spi cspins both 10; spi cs b pulse 10` first when the goal is
pin diagnosis. `spi pins` prints the compiled GPIO mapping. `spi cspins both
10` pulses PE4 and PF4 as separate candidate CS_B pins so the schematic-note
conflict can be settled on a scope. `spi cs b pulse 10` then pulses the actual
runtime CS_B path used by the ADBMS driver. After the CS_B pin is proven, use
`spi preset normal; spi scope` for hardware diagnosis rather than firmware
diagnosis. It creates repeatable CS_B/SPI6 traffic using the real RDCFGA command
and readback clocks, then prints the expected MCU probe pins. If MISO is stuck,
run `spi preset cmd; spi scope` to prove the
MCU-to-ADBMS6822 side without depending on a response. Use
`spi preset pattern; spi scope` only as a signal-path check; it transmits a
visible `AA 55 FF 00 69 96 12 34` pattern, not a valid ADBMS transaction.
`spi toggle` cycles normal -> command-only -> pattern -> CS_A readback.

Candidate pin helper:

```text
spi cspins alt 10   alternates PE4 then PF4
spi cspins both 10  pulses PE4 block, then PF4 block
spi cspins pe4 10   pulses PE4 only
spi cspins pf4 10   pulses PF4 only
```

Key interpretations:

| `bringup adbms6830` Field | Meaning |
|---|---|
| `mode=PASS` | SPI6 is CPOL high, CPHA 2-edge, MSB first. |
| `response=NO_READ` | No read transaction has been attempted or recorded yet. |
| `response=FAIL all_zero` | MISO/response path looks stuck low or not driven. |
| `response=FAIL all_ff` | MISO/response path looks stuck high, floating, or no device response. |
| `response=PASS changing` | Readback bytes are changing; now inspect PEC/SID/status quality. |
| `pec=PASS` | Last read PEC fail mask is clear for the expected SMB mask. |
| `sid=PASS` | All expected SMB ICs have valid serial IDs. |
| `stat=PASS` | All expected SMB ICs have valid status registers. |
| `health` masks | Sticky evidence of config mismatch, PEC failures, or command-counter mismatch. |

## ADBMS2950 / APM Flow

Only use this after the ADBMS6830 chain is understood.

```text
apm clear
apm enable
apm probe
apm status
bringup apm2950
```

`bringup apm2950` must continue to say `DEBUG_ONLY_NON_GATING` until CS routing,
SPI response, PEC, current sign, shunt value, VBAT divider, and scaling are
bench-proven and intentionally integrated into safety logic.

## Charger Split

Low-voltage charger CAN test, no battery:

```text
state charge
bringup charger-lv
charger
bmsok inhibit
bringup charger-lv
```

Check the CAN sniffer for extended ID `0x1806E5F4` and allow frame:

```text
0C 30 00 64 00
```

That is `312.0 V`, `10.0 A`, and `BYTE5/data[4] = 0`, meaning allow/start. A
disable command should end in `01`.

Battery/accumulator charger test:

```text
bringup ready
bringup charger-battery
```

Do not use `charger-battery` as permission by itself. It is a firmware-side
readiness summary that still depends on the approved HV/battery bench procedure,
contactor state, charger configuration, and physical safety controls.

## Evidence Packet

Before moving from one bench phase to the next, save:

```text
status
bringup board
fault
spi status
bringup adbms6830
current
volt
temp
bringup ready
charger
bringup charger-lv
```

Also keep CAN sniffer logs, logic-analyzer captures, power-supply current
limits, harness configuration, and photos of the wiring/setup used for that
specific result.
