# DER26 AMS — Five-SMB / No-APM Accumulator Bring-Up

## Purpose

This is the initial accumulator instrumentation image for:

- AMS plus MCU breakout
- five production SMBs, each using one ADBMS6830
- 15 monitored cells per SMB (75 total)
- 24 SMB thermistors per SMB (120 total)
- the two-channel DHAB current sensor path on the AMS
- no ADBMS2950/APM in the physical isoSPI ring

It is not the one-board EVAL-ADBMS6830BMSW image and it is not a vehicle
release image.

The normal final-ring firmware assumes six physical ADBMS devices:

```text
String A -> 5 x ADBMS6830 SMB -> 1 x ADBMS2950 APM -> String B
```

Simply setting `AMS_ENABLE_APM_2950=0` in that image is insufficient because
the 6830 driver would still emit wake traffic for six physical devices. This
profile changes the physical wake count to five and disables all APM traffic.

## Two test images — do not mix them up

| Test | Active path | Monitors/cells | Required profile |
| --- | --- | --- | --- |
| ADI eval-board resistor test | String B | 1 ADBMS6830 / 16 cells | `AMS_EVAL_ADBMS6830_BMSW=1` in the separate eval branch |
| Accumulator test in this branch | String A | 5 SMBs / 15 cells each | `AMS_ACCUMULATOR_5SMB_NO_APM=1` |

Flash the five-SMB image only for the accumulator test.

## What this profile locks

The five-SMB/no-APM build is intentionally measurement-only:

- `BMS_OK` is compile-time locked low, even if the mutable runtime inhibit is
  accidentally cleared.
- Application state remains `STATE_START`; service state changes are blocked.
- Balancing activation is compiled out.
- An explicit balance **clear/off** write remains available.
- Automatic and manual open-wire stimulus are disabled.
- ADBMS2950 initialization, sampling, CLI probes, and String-B service traffic
  are disabled.
- Fan commands are forced to 0% so invalid thermistors at startup do not cause
  the normal fail-safe 100% fan command during wiring bring-up.
- DHAB zero-calibration mutation is blocked; raw evidence can still be read.
- Thermistor mux acquisition, ADBMS6830 voltage/status/config reads, DHAB ADC
  acquisition, telemetry, logging, and the CLI remain enabled.

Precharge is still hardware-controlled on the present car. This image neither
commands nor validates precharge.

## Target build configuration

Create a dedicated CubeIDE Debug configuration, for example
`5SMB_NoAPM_Bench`, and add these MCU GCC compiler preprocessor symbols:

```text
AMS_BUILD_PROFILE=1
AMS_ACCUMULATOR_5SMB_NO_APM=1
AMS_ENABLE_APM_2950=0
```

Do not add the host-only symbol `AMS_HOST_ONLY_5SMB_NO_APM_TEST` to the target
build.

Ensure no project-level setting overrides the fixture with any of these:

```text
AMS_HIL_REPLACE_ADBMS=1
AMS_ENABLE_HIL_CAN=1
AMS_ENABLE_APM_2950=1
AMS_HW_BRINGUP=0
```

The profile supplies the intended target defaults:

```text
AMS_HW_BRINGUP=1
AMS_HIL_REPLACE_ADBMS=0
AMS_ENABLE_HIL_CAN=0
AMS_ENABLE_SERVICE_CLI=1
AMS_ENABLE_IMD=0
AMS_ENABLE_IWDG=0
AMS_ADBMS_SCAN_HZ=1
```

The fixture enforces the 1 Hz scan rate at compile time; an inherited project
symbol that tries to select a different rate will stop the build.

Clean the CubeIDE configuration before building so objects from the normal
six-device image cannot be reused. Save the ELF, map file, compiler output,
commit ID, and the exact preprocessor-symbol list with the test record.

The expected SPI6 configuration is:

- master
- full-duplex, two-line
- 8-bit
- software NSS
- MSB first
- mode 3: CPOL high, CPHA second edge
- prescaler 256 for the initial test

UART3 is configured as 115200 baud, 8 data bits, no parity, one stop bit, no
flow control.

## Safety boundary

This procedure does not authorize energized HV or contactor operation.

Before connecting or powering:

- Keep AIRs/contactors, inverter, charger, and other traction loads out of the
  test path.
- Physically inhibit or disconnect `BMS_OK`; do not rely only on firmware.
- Use the team's accumulator lockout, PPE, insulated-tool, cell-tap connection,
  and power-up procedures.
- Never connect or disconnect cell taps, thermistor harnesses, isoSPI cables,
  or current-sensor wiring on an energized accumulator unless an approved
  written procedure specifically requires it.
- Use a current-limited LV supply for the AMS and begin at the lowest limit
  that reliably boots it.
- Confirm expected 5 V and 3.3 V rails and `BMS_OK` low before connecting the
  accumulator communication path.
- Disconnect fans for this measurement-only phase if their wiring has not
  already been validated. The fixture also commands them to 0%.
- Stop immediately for excess current, heating, odor, rail collapse, gate
  movement, unexpected contactor/fan/charger activity, or a disagreement with
  an independent cell-voltage measurement.

## Physical connections

Make all changes with power removed.

### AMS, breakout, debugger, and UART

1. Seat the MCU breakout on the AMS and inspect connector alignment.
2. Connect the ST-Link using the team's known-good SWD wiring: target reference,
   ground, SWDIO, SWCLK, and reset as required. Avoid powering the target from
   two sources.
3. Use the existing UART/USB interface for UART3. A Nucleo board is not
   required.
4. Power the AMS from the current-limited LV bench supply and verify rails and
   `BMS_OK` low before adding the accumulator link.

### Five-SMB isoSPI path

The firmware's authoritative scan direction is String A:

```text
AMS ADBMS6822 String A -> SMB 1 -> SMB 2 -> SMB 3 -> SMB 4 -> SMB 5
```

If the accumulator harness closes the physical ring, connect SMB 5's return
to AMS String B using the intended no-APM bypass/interconnect. Preserve the
schematic's connector orientation, transformer side, and differential-pair
polarity. Do not treat the isoSPI pair as ground-referenced SPI.

This build does not automatically read the chain backward, compare both
directions, remap reverse IC order, or fail over to String B. Do not use the
String-B path as evidence of redundant operation.

### SMB measurement harnesses

- All five SMBs must be powered and connected in the team's validated cell-tap
  sequence.
- Firmware expects 15 monitored cells and 24 thermistors on every SMB.
- Verify SMB identity/order against physical labels before interpreting
  `SMB 0` through `SMB 4` in the CLI.
- Initial SPI success does not prove that every cell or thermistor connection
  is correct.

### DHAB current sensor

The firmware's present design mapping is:

| Signal | MCU input | Interpreted range |
| --- | --- | --- |
| `C_SENSE_L_MCU` | PC0 / ADC2_IN10 | DHAB ±50 A channel |
| `C_SENSE_H_MCU` | PA3 / ADC1_IN3 | DHAB ±800 A channel |

Confirm DHAB 5 V supply, return, and both signal conductors before using the
readings. With zero primary current, the nominal sensor-side outputs are near
2.5 V. After the AMS 100 kΩ/150 kΩ divider, the MCU-side ADC inputs are near
1.5 V, approximately 1860 counts for an ideal 3.3 V, 12-bit ADC. Use these as
diagnostic expectations, not calibration limits.

Polarity, gain, offsets, supply accuracy, and assembled-board divider accuracy
remain physically unvalidated. Do not use these readings for a current limit or
regen decision in this test.

## CLI sequence

The ADBMS task scans at 1 Hz. A service command may occasionally report that a
scan is active; wait for the scan to finish and retry rather than repeatedly
issuing commands.

### Phase 0 — verify the flashed image before accumulator traffic

```text
ver
status
bringup board
bmsok status
balance status
state
fan
fault
spi pins
```

Expected evidence:

- profile is `bench-5smb-noapm`
- startup banner reports `APM2950:0` and `inhibit:1`
- `BMS_OK` is low and inhibited
- state is `START`
- fan command is 0%
- SPI6 reports mode 3, MSB first, prescaler 256
- voltage and temperature may be not ready before the SMB chain is connected
- IMD is disabled/fail-closed in this bench image; that is not an SPI failure

Confirm the compile-time locks once:

```text
bmsok release
balance release
state charge
current zero
apm sid
spi probeb
spi owcheck
```

Every command above must report blocked/absent and must not change state or
send APM/open-wire traffic.

### Phase 1 — verify MCU SPI pins and String-A activity

```text
spi clear
spi enable
spi cs a pulse 10
spi preset normal
spi scope a read 20
spi status
```

With a logic analyzer or oscilloscope, verify:

- CS_A on PE2
- SCK on PG13
- MOSI on PG14
- MISO on PG12
- mode-3 clocking
- corresponding activity at the ADBMS6822 and on the outbound isoSPI path

Do not move to another phase if CS_A, clock mode, or rail behavior is wrong.

### Phase 2 — establish five-device protocol integrity

```text
spi probea
spi sid
spi stat
spi cfgchk
spi status
bringup adbms6830
```

Healthy expectations:

- five logical monitors
- expected IC mask `0x001F`
- SID-valid mask `0x001F`
- Status C/D/E-valid mask `0x001F`
- last packet PEC pass mask `0x001F` and fail mask `0x0000`
- command-counter mismatch mask `0x0000`
- configuration mismatch mask `0x0000`
- no all-zero or all-`0xFF` response pattern

The last-operation mask depends on which read command ran most recently. Save
the output of every command rather than relying on only the final summary.

### Phase 3 — cell-voltage scan

Wait for at least two automatic 1 Hz scans, then run:

```text
volt
bringup adbms6830
```

For a fully connected five-SMB accumulator, expect:

- 75 usable cells
- 75 updated cells on a successful current scan
- per-SMB usable and updated masks `0x7FFF`
- per-SMB stale/PEC/jump masks `0x0000`
- each displayed voltage agrees with an independent trusted measurement and
  the intended SMB/cell location

ADBMS code zero is a valid 1.500 V cell measurement and reaches the severe
undervoltage policy. It is not treated as missing data. Stop if any channel is
unexpectedly near 1.500 V rather than dismissing it as a software sentinel.

### Phase 4 — thermistor/mux scan

At 1 Hz, each scan reads three thermistors per SMB (one from each mux group).
All five SMBs are scanned in parallel, so a full 24-sensor sweep takes about
eight successful scans. Wait at least 10 seconds after stable communication,
then run:

```text
temp
```

Healthy steady-state expectations:

- 120 usable thermistors total
- per-SMB usable mask `0xFFFFFF` after the complete sweep
- normally 15 sensors updated in the current scan (3 per SMB), not 120
- each per-SMB updated mask normally contains the current three channels,
  while the usable mask retains the complete fresh sweep
- no open, short, stale, invalid, implausible-jump, or excessive-rate masks

For a focused mux check, use zero-based indices:

```text
tempsns 0 0
tempsns 4 23
```

Compare several sensors with an independent temperature reference. A correct
SPI transaction is not proof of thermistor mounting, thermal contact, divider
tolerance, or harness accuracy.

### Phase 5 — DHAB current acquisition

With the primary conductor at confirmed zero current and no traction load:

```text
current
current zero status
```

Capture:

- validity and reason
- selected range
- raw H/L ADC counts
- MCU-side H/L voltages
- reconstructed DHAB H/L voltages
- raw ±50 A and ±800 A current estimates
- filtered/selected current
- calibration provenance

Do not run a zero calibration in this fixture; mutation is blocked so the raw
baseline is preserved for review. Do not infer charge/discharge sign until a
controlled, independently measured current test confirms polarity.

### Phase 6 — save the complete evidence bundle

```text
status
fault
spi status
bringup adbms6830
volt
temp
current
bringup ready
bringup evidence
```

`bringup ready` must report that release is not allowed. In this fixture it is
a health summary only; it cannot authorize `BMS_OK`.

Save:

- flashed ELF hash and source commit
- CubeIDE configuration and all preprocessor symbols
- AMS, breakout, SMB, and accumulator revision/serial identifiers
- bench-supply voltage/current limits and observed current
- full UART log from reset onward
- logic-analyzer captures at MCU SPI, ADBMS6822, and isoSPI endpoints
- five-device SID/status/config results
- cell-voltage comparison table
- thermistor comparison table
- DHAB raw/voltage/current evidence
- every stop/retry/recovery event

## Fault-isolation order

1. **AMS does not boot:** remove accumulator communication, verify LV rails,
   debugger connection, reset, and flashed image.
2. **CS_A or SCK absent:** verify profile/banner, SPI6 mode, PE2/PG13 routing,
   and CLI command acceptance.
3. **MCU SPI exists but no isoSPI:** verify ADBMS6822 supplies, CS, PICO/POCI,
   transformer network, connector orientation, and differential-pair polarity.
4. **All-zero/all-`0xFF` response:** inspect MISO and the complete return path;
   do not keep increasing retry rate.
5. **Only some ICs pass:** locate the first missing device/link from String A;
   record masks before moving cables.
6. **SID passes but cell reads fail:** inspect cell-side power, input harness,
   configuration readback, conversion timing, PEC, and counter diagnostics.
7. **Cells pass but thermistors fail:** inspect ADBMS COMM ACK/PEC diagnostics,
   mux supply/addressing, VREG, AUX wiring, and thermistor harnesses.
8. **DHAB invalid:** inspect 5 V sensor supply, return, output midpoint,
   100 kΩ/150 kΩ dividers, PC0/PA3 mapping, ADC reference, and raw counts.

Power down before changing wiring. Preserve the first failure evidence; it is
usually more useful than a trace collected after multiple reconnections.

## Known boundaries

- The image was host-compiled and tested, including focused SIL,
  AddressSanitizer, UndefinedBehaviorSanitizer, and GCC static analysis.
- The normal default build's unit and comprehensive SIL suites were rerun to
  confirm the six-device final-ring behavior remains unchanged.
- This environment did not contain `arm-none-eabi-gcc`, so the exact STM32 ELF
  must still be clean-built in CubeIDE and reviewed before flashing.
- No physical isoSPI, cell, thermistor, DHAB, timing, or EMC behavior was proven
  here.
- There is no automatic dual-direction isoSPI redundancy in this image.
- APM, balancing, open-wire stimulus, IMD, hardware watchdog validation,
  contactor feedback, and vehicle release remain outside this test.
- Current polarity/calibration and thermistor end-to-end accuracy remain target
  validation items.
