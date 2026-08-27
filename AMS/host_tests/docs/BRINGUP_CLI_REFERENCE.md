# Bring-Up CLI Reference

Authored by Mahad Faisal, 2026.

The `bringup` command is a staged bench-debug layer on top of the lower-level
`status`, `spi`, `apm`, `charger`, `current`, `volt`, `temp`, `fan`, `fault`, `can`, `wdg`, and
`bmsok` commands. It summarizes whether the next hardware step is sensible; it
does not release BMS_OK, clear safety latches, send extra charger commands, or
override normal safety gating.

## Command Summary

| Command | Phase | What It Answers |
|---|---|---|
| `bringup board` | LV board-only | Did the firmware boot, is SPI6 mode 3, is BMS_OK inhibited, and is DHAB zero-current alive if powered? |
| `bringup adbms6830` | SMB/ADBMS6830 chain | Are SPI mode, CS path, response bytes, PEC, SID, status, and diagnostic health moving in the right direction? |
| `bringup apm2950` | Final-ring APM | Is the String-B ADBMS2950 identity/config/sample path observable while remaining explicitly non-safety-gating? |
| `bringup charger-lv` | Charger low-voltage CAN | Are the charger CAN IDs, command frame, and `BYTE5/data[4]` polarity clear for sniffer validation? |
| `bringup charger-battery` | Charger with safe battery path | Are BMS_OK, fresh charger RX, voltage, current, temp, and charger gates clean enough for a battery-connected charger test? |
| `bringup ready` | BMS_OK release review | Would the normal safety gates allow release? This command does not run `bmsok release`. |
| `bringup snapshot` | Any phase | Compact state/fault snapshot for logs. |
| `bringup evidence` | Any phase | List of CLI outputs and bench artifacts to capture before changing phase. |
| `balance inhibit` | Resistor-ladder/bench | Clears balance PWM/DCC and blocks automatic balancing until `balance release`. |


## Safety/Fault Infrastructure CLI

Use these commands during bench evidence capture and after abnormal resets:

```text
fault resetcause
fault panic
fault log
can diag
wdg status
```

`fault resetcause` prints decoded RCC reset flags. `fault panic` prints the
last `.noinit` panic record, including CFSR/HFSR/MMFAR/BFAR. `fault log` prints
the small RAM fault-event ring. `can diag` reports the last HAL CAN error,
bus-off counter, recovery counter, and pending recovery state. `wdg status`
reports whether the optional IWDG build flag is compiled in and whether runtime
feeding is enabled.

`wdg enable` only has effect in builds compiled with `AMS_ENABLE_IWDG=1`. Do not
enable the watchdog during first ADBMS bench probing unless the UART, ADBMS,
current, temp, and heartbeat paths are already stable.

Optional destructive test commands such as `fault inject hardfault`,
`fault inject busfault`, `fault inject canbusoff`, and `wdg stopfeed` are compiled
only when `AMS_FAULT_INJECTION_CLI=1`. Keep that flag disabled for normal bench
and competition firmware.

## First Board-Only Flow

Use this when the AMS board is powered from low voltage and is not connected to
an accumulator.

```text
status
bringup board
bmsok status
balance status
balance inhibit
fault
```

Expected:

```text
BMS_OK remains 0
output_inhibit remains 1 in AMS_HW_BRINGUP builds
balance_inhibit remains 1 in AMS_HW_BRINGUP builds
spi6=PASS
voltage/temp may be not_ready without cells
current_zero=PASS if the DHAB sensor is LV-powered and sitting at mid-scale
```

If `current_zero=WARN`, check whether the harness actually powers the DHAB. A
warning is acceptable for a board-only harness that omits the current sensor
excitation; it is not acceptable if the DHAB is powered and should be reporting
mid-scale zero current.

Current-sense ADC mapping, rechecked from nets/connectors and Cube config:

| Signal | AMS input connector | MCU breakout input connector | STM32 pin | Cube/HAL ADC path | Firmware use |
|---|---:|---:|---|---|---|
| `C_SENSE_L_MCU` / `C_SNS_L` | `J4 pin 6` | `J601 pin 6` / `ADC2` | `PC0` | `ADC2_IN10` / `ADC_CHANNEL_10` | DHAB 50 A channel |
| `C_SENSE_H_MCU` / `C_SNS_H` | `J4 pin 2` | `J601 pin 2` / `ADC1` | `PA3` | `ADC1_IN3` / `ADC_CHANNEL_3` | DHAB 800 A channel |

Use `current` during board bring-up. It prints this map before the raw ADC
counts so UART logs show which physical ADC path was being interpreted. After the
DHAB is powered and the pack current is known to be zero, capture software offset
with:

```text
bmsok inhibit
state start
current zero
current zero status
```

`current zero` is refused unless BMS_OK output is inhibited, BMS_OK is low, and
the AMS is not in charge or discharge state. Use `current zero clear` to remove
the software offset.


## Temperature / Fan Diagnostics

Use these during ADBMS temperature-bus bring-up:

```text
tempbus idle
tempbus scan
tempsns 0 0
temp
fan
```

`tempbus idle` is non-driving and only checks the resting GPIO4/SDA and
GPIO5/SCL levels. `tempbus scan` actively probes `0x4C` through `0x4F` with
control byte `0x00`, records raw ACK/PEC/counter evidence, leaves all mux
switches open, and publishes no temperature sample.

`temp` now prints usable/updated/stale/invalid counts plus open, short,
implausible-jump, and rate-rise diagnostic counts. The open/short/stale/invalid
conditions are safety-relevant because they make the temperature set unusable.
The jump/rate masks are diagnostic telemetry only and do not change the existing
over-temperature threshold policy.

`fan` prints commanded duty, reason, state, driver-set failure count, and each
fan-zone PWM duty. There is no tach/RPM feedback in the current AMS hardware, so
this command proves only what firmware commanded, not actual blade rotation.

```text
fan command 0%     -> cool/off
fan command 25%+   -> ramp/minimum cooling
fan command 100%   -> temp invalid, temp fault, or fan-max condition
```

## ADBMS6830 / SMB Flow

The old firmware state was reportedly not communicating, so this is the first
real proof point after flashing the fixes.

```text
spi clear
spi enable
balance inhibit
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

`spi probea` performs the five-device ADBMS6830 RDCFGA read from String A.
`spi probeb` is intentionally routed through the one-device ADBMS2950 RDSID
reader on String B; it does not run the five-packet SMB parser backwards.

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

The v0.3.6 evaluation release is a dedicated standalone topology:

```text
STM32 SPI6 / CS_B -> EVAL-ADBMS6822 -> one EVAL-ADBMS2950-BASIC
```

The SMB is intentionally absent. Run:

```text
apm clear
apm status
apm sid
apm refup
apm config
apm flags
apm raw
apm sample
apm redundant
apm eeprom
apm scope sid 20
apm scope sample 20
apm status
bringup apm2950
```

The evaluation image selects `EVAL_BASIC_50uR` after successful startup. The
DER final-board profile remains available through `apm profile der`; use
`apm profile eval` to restore evaluation-board scaling after changing it.

`apm eeprom` performs a non-destructive pointer probe to the evaluation
board's EEPROM at address `0x50`. It sends the control byte and address pointer
`0x00`, releases SDA on both ACK clocks, issues STOP, and prints the raw
pre/post `RDCOMM` payloads. It does not transmit an EEPROM data byte.

`apm redundant` temporarily starts synchronized I1/I2 and VB1/VB2 conversion,
reads one coherent snapshot, reports normalized channel disagreement, and then
restores the normal mixed-ring I1/VB1 continuous mode. A successful command is
diagnostic evidence only; APM measurements remain advisory and non-gating.

`bringup apm2950` must say `STANDALONE_EVAL` and `ADVISORY_NON_GATING` in
this release. A final-ring build must instead say `FINAL_RING`. Do not make APM
data authoritative until CS routing, PEC/counter integrity, current sign, shunt
value, VBAT divider, scaling and fault policy are bench-proven and intentionally
integrated into safety logic.

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
fault resetcause
fault panic
fault log
can diag
wdg status
spi status
bringup adbms6830
current
volt
temp
fan
bringup ready
charger
bringup charger-lv
```

Also keep CAN sniffer logs, logic-analyzer captures, power-supply current
limits, harness configuration, and photos of the wiring/setup used for that
specific result.

## Single-SMB ADBMS Diagnostic Suite

These commands are service diagnostics for the isolated single-SMB bench image.
They serialize against the normal ADBMS task with the recursive SPI mutex. Long
commands temporarily pause periodic ADBMS traffic and should only be used with
BMS output inhibited and balancing disabled.

```text
spi cdump
spi csoak [1-1000]
spi snapshot
spi cfgrepeat [1-1000]
spi timing [all|c|s|aux] [1-1000]
spi recovery [idle_ms 0-10000]
spi rawdump
spi faults
```

| Command | Purpose |
|---|---|
| `spi cdump` | One standalone C-ADC conversion using `ADCV` with redundancy, discharge, and open-wire switches disabled. Polls `PLADC`, then prints raw `RDCVA..RDCVF`, PEC/counter integrity, and decoded C1-C15 values. |
| `spi csoak 1000` | Repeats the standalone C conversion and reports valid count, min/average/max, peak-to-peak spread, largest sample jump, HAL/PEC/counter failures, conversion timing, pack sum, and cell delta. |
| `spi snapshot` | Captures SID, CFGA/CFGB readback, fresh C/S comparison, CSFLT, references/status, non-driving temperature-bus state, readiness, and fault classes in one log. |
| `spi cfgrepeat 100` | Repeats safe CFGA/CFGB write/readback verification. It refuses to run if any discharge or PWM shadow is nonzero. |
| `spi timing all 20` | Profiles C-, S-, and AUX-conversion completion with `PLADC`, `PLSADC`, and `PLAUX1` polling. Reports min/average/max completion time and poll clocks. |
| `spi recovery 2000` | Holds the ADBMS bus idle for the requested interval, then wakes the chain and verifies SID, configuration, references/status, and one C-ADC image. It does not clear application latches or make the SMB ready. |
| `spi rawdump` | Reads every relevant ADBMS6830 register group as raw hex before decoding. |
| `spi faults` | Prints active and boot-latched ADBMS fault-class masks with named causes. |

The existing independent S-channel commands remain available:

```text
spi sdump
spi srepeat 20
spi csdump
```

The diagnostic commands do not intentionally change BMS_OK, readiness, fault
latches, or balancing authority. They do update the driver's most recent raw
register image and transport counters, so save the complete output with the
bench setup and firmware revision.

### Voltage authority modes

Standard builds remain fail-closed and require C/S redundancy:

```c
#define AMS_VOLTAGE_MODE AMS_VOLTAGE_MODE_REDUNDANT_CS
```

A temporary C-only MVP mode exists as an explicit compile-time option:

```c
#define AMS_VOLTAGE_MODE AMS_VOLTAGE_MODE_C_ONLY_MVP
```

C-only mode never activates automatically. It keeps collecting and reporting S
and CSFLT data, marks voltage authority as degraded, and ignores only the known
S-redundancy fault for voltage authority. PEC, command counter, C-code validity,
staleness, configuration, reference, status, identity, topology, OV/UV, and
communication faults remain blocking. Temperature validity remains a separate
requirement, and balancing must remain disabled while temperature data is not
valid.

A vehicle-profile C-only build is rejected unless the build explicitly defines:

```text
AMS_C_ONLY_MVP_RELEASE_REVIEWED=1
```

This acknowledgement is not hardware validation; it only prevents an accidental
vehicle build from silently using degraded voltage authority.

## Single-SMB Resilience and Explainability Suite

These additions are intended to make a one-SMB bring-up build explain why it
is or is not authoritative. They do not bypass the existing S-channel,
temperature, communication, configuration, or balancing safety gates.

### Read-only status commands

```text
spi lifecycle
spi ages
spi authority
spi events
spi lockdiag
balance shadow
spi tempemu [temperature_C]
```

| Command | Purpose |
|---|---|
| `spi lifecycle` | Prints the formal ADBMS state (`offline`, `waking`, `identified`, `configuring`, `measuring`, redundant-ready, degraded C-only-ready, faulted, or recovering), the previous state, transition reason, transition count, and first/latest fault timing. |
| `spi ages` | Prints independent ages for C voltage, S voltage, temperature, status, configuration verification, and identity. A fresh status read cannot make an old cell sample fresh. |
| `spi authority` | Prints the C-authority bitmap: transport, PEC, counter, codes, range, slew, freshness, configuration, reference, identity, and topology. This is separate from full redundant voltage validity. |
| `spi events` | Filters the retained first-fault/event ring to ADBMS fault changes, state transitions, unexpected device resets, diagnostic failures, and software injections. |
| `spi lockdiag` | Reports SPI owner-mutex acquisitions, contention, maximum wait/hold time, and ownership violations. All periodic and CLI ADBMS traffic must use the same owner. |
| `balance shadow` | Runs the production balancing planner without writing DCC or PWM registers. It reports the cells that would be selected and always reports `applied:0`. |
| `spi tempemu 25` | Exercises the production thermistor forward/raw/inverse path, all 24 sensor-to-mux routes, and synthetic COMM ACK/address-NACK/data-NACK parsing. It also demonstrates one failed mux remaining non-publishable. It never issues `WRCOMM` or `STCOMM` and is not proof that the physical mux bus works. |

### Guided and streaming diagnostics

```text
spi mapcheck baseline
spi mapcheck verify <cell 1-15> [min_delta_mV]
spi mapcheck clear
spi cstream [samples 1-3600] [interval_ms 10-60000]
```

`mapcheck` is only for an approved isolated low-voltage cell simulator. Capture
a baseline, change exactly one simulator channel, then verify the expected cell.
It detects shifted, duplicated, or swapped logical mapping without enabling
balancing or BMS_OK.

`cstream` emits machine-readable C-channel samples while releasing the SPI
owner lock between conversions. Each individual conversion remains serialized;
the long stream does not hold the bus for its entire duration.

### Bench-only software fault injection

```text
spi inject status
spi inject pec
spi inject counter
spi inject stale
spi inject invalidc 5
spi inject reference
spi inject comm
spi inject reset
spi inject clear
```

Injection is compiled only in the bench profile and requires the service CLI.
It adds a fault to the firmware validation inputs and forces BMS_OK low; it does
**not** transmit malformed SPI traffic or alter the physical ADBMS register
image. HIL and vehicle profiles compile the feature out. Clearing an injection
removes only the synthetic input. It does not clear real faults, retained event
evidence, or safety latches.

### Unexpected ADBMS reset detection

A PEC-valid response whose command counter unexpectedly returns to zero is
classified separately from a normal mismatch. Firmware tracks the current and
sticky IC masks, a monotonic reset count, the last reset timestamp, and the
associated lifecycle transition. A commanded reset, normal wake/sleep flow,
and an uncommanded reset must remain distinguishable in the bench record.

### Configuration provenance

The `ver`, `spi snapshot`, and `spi authority` outputs identify the build
profile, voltage mode, topology, String A/B selection, diagnostic compile
switches, build commit, and desired/readback configuration fingerprints. Save
this output with every bench log so results cannot be attributed to the wrong
binary.

### Focused host regression target

```text
make -C host_tests adbms-resilience-test
```

The target runs the one-SMB suite in both redundant and explicit C-only modes.
It verifies S-validity masks, C-authority and age gates, lifecycle behavior,
shadow-planner purity, software-only injection behavior, and unexpected-reset
classification. This focused target does not replace the project-wide five-SMB
system SIL suite.


## Sense-path, OV/UV, connection, and fuse commands (2026-08-04)

```text
volt
spi owc
spi ows
spi faults
spi rawdump
fault fuse
fault fuse clear
status
```

`volt` prints software C-threshold masks, populated ADBMS Status-D OV/UV masks, hardware/software disagreement masks, active/sticky electrical sense-path masks, and the advisory parallel-connection observer state.

`spi owc` runs one complete C-path baseline/even/odd open-wire diagnostic while BMS output and balancing are inhibited. It then performs a mandatory normal C/S conversion. The command reports an **electrical sense-path open** and does not claim that the 1 A fuse specifically failed. A failed restore immediately invalidates C authority.

`spi ows` runs the S-path implementation for development only. Its result is not physically meaningful on Rev5 until the direct Cell1->S2N through Cell14->S15N routing defect is corrected.

`fault fuse` reports the main-fuse/HV-path plausibility monitor. Current hardware should report `unavailable`; that is the intended fail-closed state because authoritative AIR auxiliary contacts and independent load-side voltage are absent. `fault fuse clear` is a service-only controlled clear and requires BMS_OK inhibited/low, fresh OFF or SHUTDOWN command, a discharged load bus, and near-zero current.

The parallel-connection observer cannot prove an individual wire bond failed. It reports only repeated group-resistance outlier evidence and remains advisory unless a future target is separately characterized and build-authorized.


### ADG728 ACK-cycle requirement (v0.3.3)

The address and data slots use ADBMS6830 `FCOMM=0x8` so SDA is released on
the ninth clock and the ADG728 can pull it low for ACK. For address `0x4C`
and control byte `0x00`, the pre-STCOMM COMM payload must be:

```text
68 98 08 00 19 FF
```

The older `60 98 00 00 19 FF` pattern is invalid for slave discovery because
it makes the ADBMS6830 generate the ACK bits itself.

### ADG728 ACK readback decoding (v0.3.4)

After `STCOMM`, the COMM control nibbles are readback status codes rather than
a byte-for-byte copy of the written ICOMM values. A successful ADG728 write can
return:

```text
67 98 77 00 1F FF
```

`COMM0=0x67` means START plus slave address ACK. `COMM2=0x77` means blank
with SDA held high between bytes plus slave data ACK. `ICOMM1=0x0` (SDA held
low) and `ICOMM1=0x7` (SDA held high) are both valid blank readback states; the
ACK decision is carried by `FCOMM1=0x7`. Firmware v0.3.4 accepts both and no
longer falsely reports `PARTIAL_OR_DATA_NACK` for the observed `0x77` result.

### ADBMS2950B command and scaling hardening (v0.3.5)

The ADBMS2950B path now applies the same protocol lessons learned during the
ADBMS6830B temperature-bus bring-up:

- command PEC is generated at runtime;
- `STCOMM` clocks are generated from the requested COMM-byte count;
- slave ACK slots use `FCOMM=0x8`, and ACK/NACK is decoded from the post-command
  `FCOMM` readback nibble;
- the EVAL EEPROM pointer probe exposes raw pre/post COMM bytes;
- CFGA/CFGB verification masks read-only and reserved fields instead of using
  a full six-byte `memcmp`;
- sample freshness requires a completed `I1CNT` advance, not an `I1PHA`-only
  phase change;
- current and VBAT scaling use an explicit DER/EVAL/custom calibration record;
- a redundant I1/I2 and VB1/VB2 diagnostic is available through
  `apm redundant`;
- undocumented `CLRC` and daisy-chain-incompatible `RDALL*` entries are not
  exposed by the mixed-ring driver;
- SPI transactions include explicit CS setup/hold delays and always deassert
  CS on failure.


### ADBMS2950B standalone readiness and CLI expansion (v0.3.6)

v0.3.6 adds explicit standalone evaluation topology, RDSID identity proof,
REFUP gating, predicted six-bit command-counter checking on every read,
unexpected-reset detection, coherent raw CFGA/CFGB/STAT/FLAG/SID snapshots,
raw and decoded STAT/FLAG output, bounded topology-aware recovery, and
stage/reason diagnostics. `apm clear` also clears sticky CCNT/reset and recovery
counters without changing readiness.

Use `make apm-bench-gates` for the self-contained firmware-only gate. The full
repository `make ci` still requires sibling thermistor, SoP, fuse, and dashboard
oracle assets not included in every firmware ZIP.
