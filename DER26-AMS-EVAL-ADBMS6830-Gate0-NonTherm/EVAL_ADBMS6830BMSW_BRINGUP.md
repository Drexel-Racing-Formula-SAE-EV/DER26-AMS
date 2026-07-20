# AMS + EVAL-ADBMS6830BMSW SPI bring-up guide

This branch is a low-energy, monitor-only fixture for one Analog Devices
`EVAL-ADBMS6830BMSW` board connected to the DER26 AMS through its onboard
ADBMS6822. It carries the Gate-0 non-thermistor ADBMS fixes into the eval
profile without enabling any vehicle output.

It is not a vehicle image and it is not permission to energize the tractive
system. Keep the AIRs, charger, inverter, fan loads and other actuators
disconnected. Physically inhibit `BMS_OK`; the firmware lock is an additional
layer, not the only layer.

## 1. What this image does

```text
Build profile              bench / physical ADBMS
ADBMS topology             one ADBMS6830B, 16 cell channels
AMS isoSPI path            String B / CS_B
ADBMS startup              wake + SRST + PEC/counter/product-checked RDSID
CFGA/CFGB writes           disabled
ADBMS2950/APM              absent and disabled
cell balancing writes      disabled
open-wire stimulation      disabled
DER SMB thermistor mux     disabled
fan outputs                forced to 0%
estimator                  disabled
AMS state                  locked in START/monitor
BMS_OK                     compile-time locked low
CAN-fed ADBMS HIL          disabled and incompatible with this profile
```

Normal firmware behavior is still selected with
`AMS_EVAL_ADBMS6830_BMSW=0`; the special eval image must not be reused as the
five-SMB vehicle image.

## 2. Build selection: physical eval is not HIL

This package already defaults to the correct values in
`Core/Inc/ams_build_profile.h`:

```c
AMS_EVAL_ADBMS6830_BMSW=1
AMS_BUILD_PROFILE=1             /* AMS_PROFILE_BENCH */
```

For an auditable CubeIDE configuration, add those two symbols explicitly to
the Debug configuration under **MCU GCC Compiler > Preprocessor**. Confirm that
none of these incompatible symbols are enabled:

```c
AMS_HIL_REPLACE_ADBMS=1         /* must NOT be set */
AMS_ENABLE_HIL_CAN=1            /* must NOT be set */
AMS_ENABLE_APM_2950=1           /* must NOT be set */
AMS_BUILD_PROFILE=2             /* HIL profile; must NOT be used */
```

The HIL profile replaces physical ADBMS measurements with CAN-injected data.
It cannot test SPI or isoSPI and the compile-time gates intentionally reject
that combination. For the real eval board, use the bench profile above.

Expected UART banner after flashing:

```text
Build:eval-6830bmsw ... APM2950:0 inhibit:1
EVAL LOCK: 1x ADBMS6830B / 16 cells / String B; ...
```

If the banner says `bench`, `hil`, or `vehicle`, stop: the wrong image is
running.

## 3. AMS, debugger and UART connections

A Nucleo is not required. The intended controller path is:

```text
external ST-Link -> MCU-breakout SWD header -> STM32F767
PC USB           -> MCU-breakout J701 FT231X USB/UART -> USART3
STM32 SPI6       -> AMS ADBMS6822 channel B -> isolated twisted pair
isolated pair    -> EVAL-ADBMS6830BMSW J3 / isoSPI A
```

### Flashing

1. Initially leave the eval board and its stack supply disconnected.
2. Power the AMS/MCU breakout from its approved, current-limited low-voltage
   supply. Do not rely on debugger back-powering.
3. Connect ST-Link `SWDIO/TMS`, `SWCLK/TCK`, ground, target-voltage sense and
   optionally `NRST` to the labeled SWD header. Follow the current schematic
   and cable pinout; do not inject 5 V from the debugger into the target.
4. Build the eval Debug configuration and flash it.
5. Connect the MCU-breakout J701 Micro-USB UART bridge. It is an FT231X wired
   to USART3 (`PD8` TX, `PD9` RX).
6. Open the enumerated serial port at `115200 baud, 8 data bits, no parity,
   1 stop bit, no flow control`.

The firmware-side SPI6 mapping is authoritative for this branch:

```text
SCK     PG13
MOSI    PG14
MISO    PG12
CS_A    PE2
CS_B    PE4   <- active path for this fixture
```

Some old schematic notes label CS_B as PF4. That note is stale for this test;
the current Cube configuration, `main.h`, startup GPIO code and CLI all use
`PE4`.

## 4. SPI6 settings

The AMS ADBMS6822 `PHAPOL` straps require SPI mode 3. Before sending any ADBMS
traffic, the eval profile verifies:

```text
master, 2-line full duplex, 8-bit, MSB first
CPOL high, CPHA second edge (SPI mode 3)
software NSS, prescaler 256
TI mode disabled, hardware CRC disabled
```

The Cube project reports about `421.875 kbit/s`. A mismatch leaves
`smb_ready=false` before the first ADBMS transaction.

Each checked wake uses conservative 1 ms low/high intervals. A voltage scan
then uses a 3 ms reference interval and a 17 ms redundant-conversion wait.
The sequence is deliberately longer than the 6830 core/reference wake and
8-to-16 ms redundant conversion bounds.

## 5. Eval-board cell-input and isoSPI wiring

The isoSPI cable carries communication only; it does not power the eval board.
The ADBMS6830B eval board is powered and stimulated by the J1 cell-stack
connection.

### J1 resistor ladder

Use the detachable J1 screw-terminal block and follow the ADI board guide:

```text
J1 pin 1       C0 / stack negative
J1 pins 2-16   C1 through C15
J1 pin 17      C16 / stack positive
```

Install sixteen equal `100 ohm, 0.5 W or higher` resistors, one between every
adjacent C input. Connect the stack-equivalent supply only across J1 pin 17
(positive) and pin 1 (negative). ADI's example uses 52.8 V total to produce
about 3.3 V per simulated cell. That is a real multi-tens-of-volts source,
not a 3.3 V logic supply.

Before inserting J1 into the board:

1. With the supply off, confirm 16 resistor sections and no adjacent short.
2. Set an appropriate current limit under the team's approved lab procedure.
   The ideal 16 x 100 ohm ladder alone draws about 33 mA at 52.8 V; the eval
   board adds its own load, so do not choose a limit from that number alone.
3. Energize the detached ladder and measure every adjacent node
   `C1-C0` through `C16-C15`, plus total `C16-C0`.
4. Confirm correct monotonic polarity and record the values.
5. Turn the supply fully off and verify discharge before connecting J1.

Do not improvise a partial J1 hookup or assume a low total voltage will power
the board. Do not use optional supply/GPIO headers unless following their
specific ADI procedure.

### isoSPI cable

With both systems de-energized:

1. Connect the AMS **String B** isolated output (the String-B two-pin connector
   on the isolated-comms sheet) to eval-board **J3 / isoSPI A**.
2. Leave eval-board **J4 / isoSPI B** open for the one-board fixture.
3. Preserve the documented pair polarity and connector orientation. Verify
   pin numbers from the latest drawings; do not trust wire color alone.
4. Do not add a logic-ground wire across the isolation barrier.

For the combined boot, connect the isoSPI cable and J1 while supplies are off,
then energize the eval-board stack supply. After the eval board is powered,
reset or power-cycle the AMS so its one-shot startup SID check sees the 6830.
If the eval board was absent during AMS boot, manual probes may still help with
diagnosis, but reset the AMS before judging automatic-scan readiness.

## 6. First CLI session

The ADBMS task scans at the slower bench cadence. Manual commands are
serialized with the scan; if a command reports that a scan is active, wait for
the cycle to finish and retry. Do not repeatedly hammer commands around the
guard.

Run this sequence in order:

```text
ver
status
bringup board
bmsok status
fault

spi pins
spi status
spi clear
spi enable
spi cs b pulse 10
spi scope b read 20
spi probeb
spi sid
spi stat
spi cfgchk
spi status

volt
bringup adbms6830
bringup ready
bringup evidence
```

What each SPI command does:

| Command | Purpose | Expected result |
|---|---|---|
| `spi pins` | Prints the live handle/pin mapping | CS_B is PE4; SPI mode 3 |
| `spi status` | Prints counters, topology, last transfer, PEC/counter health | one logical/physical device, 16 cells, String B |
| `spi clear` | Clears local debug counters only | no remote flags or configuration are changed |
| `spi enable` | Enables detailed local transfer accounting | debug enabled |
| `spi cs b pulse 10` | Generates ten active-low CS_B pulses for a scope check | PE4 toggles and returns high |
| `spi scope b read 20` | Repeats read-only RDCFGA traffic | CS/SCK/MOSI and return traffic are visible |
| `spi probeb` | One explicit String-B RDCFGA integrity probe | `HAL_OK` |
| `spi sid` | Reads SID with PEC/counter/product-ID validation | exactly one valid ADBMS6830B |
| `spi stat` | Runs ADAX and reads Status A-E | valid status for mask `0x0001` |
| `spi cfgchk` | Reads RDCFGA in monitor-only mode | integrity pass; no expected-byte comparison |
| `volt` | Prints the automatic scan result | 16 usable/updated cells and mask `0xFFFF` |

Expected healthy summaries include:

```text
smb_ready:1 / init:HAL_OK / timer_ready:1
logical:1 physical:1 monitored_cells:16
SID valid mask:0x0001, identity mismatch:0x0000
PEC pass:0x0001, PEC fail:0x0000
command-counter mismatch:0x0000
updated/usable cell mask:0xFFFF
```

Compare all 16 CLI voltages to the recorded adjacent-node DMM measurements.
Use an agreed lab tolerance that includes the supply, resistor, board and DMM
uncertainties; do not infer calibration from a single close-looking channel.

`bringup ready` is expected to remain **not releasable**. Temperature, current,
IMD, charger and other vehicle gates are intentionally absent or invalid in
this fixture, and BMS_OK remains locked low even if every cell read succeeds.

## 7. Commands deliberately blocked

Do not use the eval fixture for actuator or fault-stimulation testing. The
firmware blocks:

```text
BMS_OK release
state changes out of START
balance release/clear/write traffic
DER thermistor-mux commands
open-wire even/odd/full stimulation
cell ADC self-test and injected status-error commands
CLRFLAG
raw command/pattern/toggle scope presets
ADBMS2950/APM commands
```

Use only `spi cs ... pulse`; do not leave a CS line forced low. A cold-wake
command exists for diagnosis but is not needed in the normal first sequence.

## 8. Scope and measurement safety

MCU-side SPI6 signals may be measured relative to AMS low-voltage ground:

```text
PE4 CS_B, PG13 SCK, PG14 MOSI, PG12 MISO
```

The isoSPI pair and J1 cell nodes are not MCU-ground-referenced. Use a suitable
differential/isolated probe. Never attach an ordinary earth-referenced scope
ground clip to an eval-board stack node; it can short the ladder or defeat the
isolation barrier.

Capture these waveforms:

1. `spi cs b pulse 10`: PE4 active-low pulses, then idle high.
2. `spi scope b read 20`: mode-3 SCK idle high, MOSI command and MISO return.
3. ADBMS6822 transformer/isoSPI differential activity corresponding to the
   same command window.

## 9. Troubleshooting decision tree

| Observation | Most likely boundary to inspect |
|---|---|
| No UART device/output | AMS power, J701 cable/driver, correct COM port, 115200 8-N-1 |
| Wrong build banner | Wrong Cube configuration or stale flash image |
| `smb_ready=0` immediately after boot | Eval board not powered during startup, SPI mode guard, failed wake/SRST/SID; power eval then reset AMS |
| PE4 does not pulse | Wrong image, scan guard, wrong code pin mapping, GPIO init |
| PE4 pulses but no PG13 activity during `scope ... read` | SPI6 init/clock or command did not run |
| MCU SPI works but ADBMS6822 transformer is quiet | 6822 power, CS_B routing, PHAPOL/SPI mode, transceiver/transformer path |
| Outbound isoSPI but no return | Eval-board power, J3 vs J4, pair polarity, cable/connector, transformer termination |
| RX is all `00` or all `FF` | Open/miswired return path, MISO/CS issue, unpowered remote device |
| Repeated PEC failures | Pair polarity, termination, cable integrity, ringing/noise, clock mode/rate, grounding mistake |
| Valid PEC but SID identity mismatch | Wrong remote device/order or incorrect transaction framing; do not continue |
| SID/status pass but cell values are wrong | J1 ordering, missing/incorrect resistor, supply polarity, adjacent-node voltage, channel mapping |
| Command-counter mismatches | Remote reset/power instability, unexpected traffic, interrupted command sequence; capture evidence before clearing |
| `bringup ready` remains false | Expected for this fixture unless the ADBMS-specific lines themselves fail |

`spi clear` erases local debug counters, so capture `spi status` before using it
during a failure investigation. Power down before reversing a pair, reseating
J1, or changing any harness connection.

## 10. Minimum evidence and exit criteria

Record:

- Git commit and the two build symbols.
- Full boot banner, `ver`, `status`, `fault` and `bringup board`.
- `spi status` before and after the test.
- `spi probeb`, `spi sid`, `spi stat`, `spi cfgchk` and
  `bringup adbms6830` output.
- DMM `C1-C0` through `C16-C15`, total `C16-C0`, and CLI cell deltas.
- AMS supply voltage/current and eval stack supply voltage/current limit.
- CS_B/SCK/MOSI/MISO and differential isoSPI captures.
- At least several minutes of the 1 Hz automatic scan with zero unexplained
  PEC or command-counter failures.

Stop immediately for overcurrent, heating, smell, rail collapse, unexpected
BMS_OK, fan movement, a wrong-polarity cell node, repeated integrity errors or
any unexplained output transition.

Only after the one-board fixture is repeatable should testing move to a real
SMB. That next stage still begins with BMS_OK physically inhibited and all
balancing disabled.

## Official references

- ADI EVAL-ADBMS6830BMSW user guide:
  <https://analogdevicesinc.github.io/documentation/solutions/reference-designs/eval-adbms6830bmsw/index.html>
- ADI EVAL-ADBMS6830BMSW product page:
  <https://www.analog.com/en/resources/evaluation-hardware-and-software/evaluation-boards-kits/eval-adbms6830bmsw.html>
- ADI ADBMS6830B product/data-sheet page:
  <https://www.analog.com/en/products/ADBMS6830B.html>
- ADI ADBMS6821/ADBMS6822 data sheet:
  <https://www.analog.com/media/en/technical-documentation/data-sheets/adbms6821-adbms6822.pdf>
