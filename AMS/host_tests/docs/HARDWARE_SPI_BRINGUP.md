# AMS Hardware SPI Bring-Up Guide


Authored by Mahad Faisal, 2026.
This guide is for first bench tests of the AMS board ADBMS SPI/isoSPI path.
It assumes the final reversible ring is wired as:

```text
AMS String A -> 5 x ADBMS6830 SMB -> 1 x ADBMS2950 APM -> AMS String B
```

The ADBMS2950 is initialized and sampled in the final firmware, but its values
remain advisory and cannot affect BMS_OK until final-board scaling, polarity,
divider accuracy and fault policy are physically validated.

## Safety Setup

Use a low-voltage bench setup first. Do not rely on BMS_OK, charger control, or
APM current data until the SPI path is proven.

Build first hardware firmware with:

```text
AMS_HW_BRINGUP=1
AMS_ENABLE_APM_2950=1
AMS_APM_ENABLE_HV_DIVIDERS=0
```

`AMS_HW_BRINGUP=1` keeps BMS_OK physically inhibited until the CLI command
`bmsok release` is run. This lets the firmware boot, scan, and report status
without accidentally enabling the downstream system.
It also defaults balancing to inhibited so resistor-ladder or low-energy
cell-simulator tests cannot accidentally turn on discharge PWM. Use
`balance status`, `balance inhibit`, `balance clear`, and `balance release` from
the UART CLI.

Keep `AMS_APM_ENABLE_HV_DIVIDERS=0` for initial current/SPI testing. This forces
the APM GPO1/GPO2 divider controls low; do not enable pack-voltage dividers until
their final-board wiring and scale have been verified.

## Expected SPI Configuration

The Drexel AMS board straps ADBMS6822 `PHAPOL` / `PHAPOL2` high, so SPI6 must
run in mode 3:

```text
CPOL = HIGH
CPHA = 2EDGE
First bit = MSB
NSS = software-controlled CS
Initial prescaler = 256
```

Do not change SPI6 to mode 0 based only on the ADI eval-board guide. The eval
board can be jumpered differently; this AMS board strapping selects mode 3.

## Logic Analyzer Channels

Start on the STM32-side SPI signals:

```text
SPI6_SCK
SPI6_MOSI
SPI6_MISO
CS_A / five-SMB path
CS_B / one-APM path
GND
```

If available, also observe the isoSPI transformer side:

```text
ADBMS6822 IP/IM pair
SMB-side isoSPI pair
```

The first pass only needs proof that CS, SCK, and MOSI are sane and MISO changes
when an ADBMS read is attempted.

## First Boot Checklist

1. Flash the `AMS_HW_BRINGUP=1` binary.
2. Open UART CLI.
3. Confirm boot banner prints:

```text
Build:hw-bringup
BMS_OK_inhibit:1
ADBMS6822 SPI6 expected: mode3 CPOL HIGH CPHA 2EDGE
```

4. Run:

```text
status
bringup board
bmsok status
```

Expected:

```text
BMS_OK:0
inhibit:1
SPI6 CPOL:HIGH CPHA:2EDGE
bringup board reports spi6=PASS
```

If CPOL/CPHA do not match, stop and fix firmware/config before chasing hardware.
In a board-only harness with no accumulator connected, voltage and temperature
may correctly report not-ready. If the DHAB current sensor is powered from LV,
`bringup board` should report `current_zero=PASS` near mid-scale; if the harness
does not power the DHAB, a current warning is expected and should be recorded.

Run `current` and confirm the printed ADC map matches the bench harness:

| DHAB path | AMS net into MCU board | Connector path | STM32/Cube path |
|---|---|---|---|
| 50 A channel | `C_SENSE_L_MCU` / `C_SNS_L` | AMS `J4 pin 6` to MCU breakout `J601 pin 6` / `ADC2` | `PC0`, `ADC2_IN10`, `ADC_CHANNEL_10` |
| 800 A channel | `C_SENSE_H_MCU` / `C_SNS_H` | AMS `J4 pin 2` to MCU breakout `J601 pin 2` / `ADC1` | `PA3`, `ADC1_IN3`, `ADC_CHANNEL_3` |

This mapping is based on the current-sense nets, connector pin numbers, Cube
ADC instances, and firmware symbols. Do not swap it based only on stale drawing
notes.

## ADBMS6830 / SMB SPI Test Order

Run commands in this order:

```text
spi clear
spi enable
spi preset normal
spi scope
spi coldwake
spi status
spi probe
spi probea
spi probeb
spi sid
spi stat
spi cfgchk
spi status
bringup adbms6830
volt
fault
status
```

Expected useful signs:

```text
spi status shows CPOL:HIGH CPHA:2EDGE
spi preset normal; spi scope produces repeated CS_A/SCK/MOSI/readback activity
spi probe returns OK or at least records a non-OK HAL status
spi probea/probeb explicitly exercise CS_A and CS_B so the AMS-side
chip-select and ADBMS6822 channel routing can be confirmed on the scope
spi sid shows valid 48-bit serial IDs for each responding IC
spi stat exposes SLEEP/SPIFLT/THSD/OSCCHK and OV/UV status flags
TX preview shows command bytes plus PEC
RX preview changes from all-zero/all-FF when the chain responds
PEC pass/fail masks are visible
command counters are visible when response frames are valid
command-counter mismatch mask remains 0 during a stable command sequence
diag sticky PEC/counter masks stay 0 during repeated stable reads
spi cfgchk reports OK and config mismatch masks remain 0
bringup adbms6830 reports mode=PASS, response=PASS changing, PEC/SID/STAT pass
volt shows usable/updated/stale/PEC cell counts
```

Bad but useful signs:

```text
HAL_TIMEOUT or HAL_ERROR increments error_count
RX preview all 00 or all FF
bringup adbms6830 reports response=FAIL all_zero or response=FAIL all_ff
PEC fail mask set for all ICs
updated cell count stays 0
voltage reason remains not_ready or stale_scan
SID validity stays false
status flags remain invalid
command-counter mismatch appears after missed/corrupt transactions
```

Those results mean the debug path is working, even if hardware communication is
not yet fixed.

## Scope / Logic Analyzer Mode

Use `spi scope` when you need a clean waveform before trusting the higher-level
diagnostics:

| Command | What it does | Use it for |
|---|---|---|
| `spi preset status` | Prints the selected scope preset. | Confirming what plain `spi scope` will do. |
| `spi preset normal` then `spi scope` | Repeats real RDCFGA command + readback clocks on CS_A. | First five-SMB-path capture: MCU SPI6, ADBMS6822, isoSPI transformer, and MISO response. |
| `spi preset cmd` then `spi scope` | Repeats valid RDCFGA command frames without readback clocks. | Proving CS/SCK/MOSI and ADBMS6822 IP/IM output when the SMB response path may be dead. |
| `spi preset pattern` then `spi scope` | Repeats `AA 55 FF 00 69 96 12 34` on MOSI. | Signal integrity / pin mapping only. Treat as invalid ADBMS traffic. |
| `spi preset a` then `spi scope` | Repeats the normal five-SMB RDCFGA traffic on CS_A. | Explicitly selecting the production SMB side. |
| `spi preset b` then `spi scope` | Repeats RDCFGA-shaped scope traffic on CS_B. | Electrical routing evidence only; use `apm sid` for real APM identity. |
| `spi toggle` | Cycles normal -> command-only -> pattern -> CS_B readback. | Quick scope comparisons while moving probes. |
| `spi scope a read 20` | Repeats real RDCFGA command + readback clocks on CS_A. | Five-SMB full-path capture. |
| `spi scope a cmd 50` | Repeats valid RDCFGA command frames without readback clocks. | Proving CS/SCK/MOSI and ADBMS6822 IP/IM output when the SMB response path may be dead. |
| `spi scope a pattern 20` | Repeats `AA 55 FF 00 69 96 12 34` on MOSI. | Signal integrity / pin mapping only. Treat as invalid ADBMS traffic. |
| `spi scope b wake 20` | Repeats wake pulses on CS_B, no SCK. | Confirming the one-APM route before an APM identity read. |

For the normal five-SMB subset, start with CS_A:

```text
spi clear
spi preset normal
spi scope
spi status
```

Expected analyzer setup:

```text
SCK  = PG13, idle high
MISO = PG12
MOSI = PG14
CS_A = PE2
CS_B = PE4
```

This mapping is based on net labels and connector pins, not the stale free-text
pin note on the AMS schematic:

```text
AMS J5 pin 21: STRINGB_CS -> MCU breakout J801 pin 21: GP_OUT1 -> STM32 PE4
AMS J5 pin 23: STRINGA_CS -> MCU breakout J801 pin 23: GP_OUT3 -> STM32 PE2
AMS J5 pin 26: MOSI       -> MCU breakout J801 pin 26: SPI6_MOSI -> STM32 PG14
AMS J5 pin 28: SCLK       -> MCU breakout J801 pin 28: SPI6_SCK  -> STM32 PG13
AMS J5 pin 30: MISO       -> MCU breakout J801 pin 30: SPI6_MISO -> STM32 PG12
```

Cube/VSCode sanity: `DER26-AMS.ioc`, `Core/Inc/main.h`, and `Core/Src/main.c`
must all agree on this pin map before flashing. If you build from VSCode/GCC,
do not hand-move generated pin macros without also updating the `.ioc`, because
the next CubeMX regeneration can silently undo or contradict the manual change.
Because the AMS schematic has a conflicting free-text `CSB - PF4` note, run
`spi cspins both 10` during first bench bringup and scope both PE4 and PF4. That
command only pulses candidate pins for evidence; it does not change the normal
ADBMS runtime `CS_B` mapping.

On `spi preset normal; spi scope`, MOSI should begin each transaction with:

```text
00 02 2B 0A
```

That is RDCFGA plus command PEC. CS_A should stay low for the command and dummy
readback clocks. If the MCU-side signals are correct but ADBMS6822 IP/IM or the
SMB transformer pins do not move, focus on 6822 power/straps, transformer
orientation/part/install, isolation passives, and harness continuity before
chasing firmware.

## What To Check On The Logic Analyzer

For `spi probe`, verify:

```text
CS_A goes low once for the full command + dummy readback transaction
SCK idles high
data changes on MOSI with MSB-first command bytes
dummy bytes continue clocking during the readback phase
MISO is not stuck low/high if the ADBMS chain responds
CS_A returns high after the transaction
```

Use `spi probea` and `spi probeb` when validating hardware routing:

```text
spi probea  -> only CS_A / STRINGA_CS should pulse
spi probeb  -> only CS_B / STRINGB_CS should pulse
```

The normal SMB/ADBMS6830 subset is read from CS_A. The APM/ADBMS2950 is the
nearest device to CS_B and is read as a one-device subset. Do not interchange
these defaults: the ADBMS6830 driver intentionally clocks five response packets,
while the APM driver clocks one.

The driver uses one full-duplex transfer for reads:

```text
TX phase: command + PEC
readback phase: dummy 0xFF bytes clocked out while MISO is sampled
```

If SCK idles low, the flashed binary or generated CubeMX init is wrong.

For `spi sid`, verify:

```text
RDSID command bytes appear on MOSI
each SMB IC returns a stable nonzero 48-bit SID
the SID order remains stable across repeated reads
PEC pass mask covers every expected IC
```

For `spi stat`, verify:

```text
RDSTATC, RDSTATD, and RDSTATE reads occur as separate full-duplex transfers
SLEEP/SPIFLT/THSD/OSCCHK bits are visible in CLI output
cell OV/UV status masks are visible
GPIO/revision status is visible
```

Use `spi staterr` only as a diagnostic injection check. The ADBMS6830 datasheet
defines RDSTATC with ERR set as a way to verify that the SPIFLT diagnostic path
can assert; it is not a normal health-read command.

Use `spi clrflag` after recording the first status output if the chain reports
expected startup flags such as SLEEP. Then rerun `spi stat` and verify which
flags clear and which remain.

## ADBMS6830 Diagnostic Hooks

After basic SID/status reads are stable, use the extra diagnostic hooks:

```text
spi cfgchk
spi cellst
spi owcheck
spi oweven
spi owodd
spi auxdiag
spi status
```

Interpretation:

| Command | Purpose | Good sign |
|---|---|---|
| `spi cfgchk` | Read CFGA/CFGB and compare against packed TX config | status OK, cfgA/cfgB/cfg masks are 0 |
| `spi cellst` | Exercise cell ADC diagnostic conversion/poll/status path | status OK and no new PEC/counter errors |
| `spi owcheck` | Run the complete even/odd S-cell open-wire sequence, wait for each conversion, read all five groups, validate PEC/counters, and publish per-cell masks | status OK, both valid-IC masks cover every SMB, incomplete/fault masks are zero |
| `spi oweven` | Run and capture the even-channel diagnostic phase only | command/read status OK and even valid-IC mask covers every SMB |
| `spi owodd` | Run and capture the odd-channel diagnostic phase only | command/read status OK and odd valid-IC mask covers every SMB |
| `spi auxdiag` | Exercise AUX/GPIO ADC read/status path | status OK and temp/AUX path still reports plausible data |
| `spi diagclear` | Clear accumulated diagnostic health counters | sticky masks and counts reset to 0 |

Normal firmware also runs the full open-wire diagnostic every 30 seconds in
charge, discharge, and balance states. A transport, PEC, counter, incomplete
phase, or detected-cell result latches the ADBMS diagnostic fault and keeps
`BMS_OK` low. The implementation monitors 15 populated cells per SMB; it does
not interpret the unused sixteenth through eighteenth channels as pack leads.

These hooks prove firmware command flow and the implemented 2.0 V diagnostic
threshold. They do not by themselves prove the threshold against the assembled
SMB, harness, cell simulator, or all physical open locations. Before HV use,
inject known opens one lead at a time at low energy, capture both parity
measurements, and confirm the reported physical-cell masks against the released
ADBMS6830 procedure and actual SMB wiring.

## Fault Isolation Matrix

| Symptom | Likely area | Next check |
|---|---|---|
| No UART banner | MCU boot/clock/UART | Check power, reset, flash, UART pins |
| CLI works, no SCK | SPI init or command path | Run `spi coldwake`/`spi probe`; confirm hspi6 handle |
| SCK/MOSI works, CS_A never toggles during SMB test | CS pin mapping | Check `CS_A_GPIO_Port` / `CS_A_Pin` wiring |
| SMB tests pass but APM commands never toggle CS_B | APM-side CS mapping | Check `CS_B_GPIO_Port` / `CS_B_Pin` wiring |
| CS/SCK/MOSI works, MISO stuck | ADBMS6822/isoSPI/SMB path | Check ADBMS6822 power, isoSPI pair, transformer, SMB power |
| RX changes but PEC always fails | SPI mode, bit alignment, chain order, noise | Confirm mode 3, sample point, SMB count, isoSPI wiring |
| SID reads but order changes | Daisy-chain instability or mixed string routing | Repeat `spi sid`; check SMB order, harnessing, and CS string selection |
| Status shows SLEEP after clear | Device reset/sleeping between commands | Check wake timing, REFON/config writes, power stability |
| Command-counter mismatch | Missed command, reset/sleep, or interleaved transaction | Check SPI lock, task timing, CS noise, and PEC failures |
| First probe works, later reads corrupt | task/CLI collision or wake timing | Confirm shared SPI lock, increase delay, avoid repeated CLI spam |
| BMS_OK goes high during bring-up | build flag or inhibit bug | Confirm `AMS_HW_BRINGUP=1`, run `bmsok inhibit` |

## ADBMS2950 / APM Test Order

Only after SMB/ADBMS6830 communication is understood, keep the final build at:

```text
AMS_HW_BRINGUP=1
AMS_ENABLE_APM_2950=1
AMS_APM_ENABLE_HV_DIVIDERS=0
```

Then run:

```text
apm clear
apm enable
apm sid
apm config
apm sample
apm scope 20
apm status
bringup apm2950
```

Expected:

```text
APM SID identifies the expected device
CFGA/CFGB read back byte-for-byte
RDSTAT/RDIVB1 PEC and command-counter checks pass
APM SPI/health counters update
APM TX/RX previews are visible
bringup apm2950 says FINAL_RING and ADVISORY_NON_GATING
divider controls remain OFF
APM failures do not change BMS_OK
the production DHAB path remains the authoritative current input
```

Do not use ADBMS2950/APM for safety gating until command behavior, scaling,
current sign, shunt value, VBAT divider values and fault thresholds are confirmed
on the final board. The CLI and SIL tests validate software behavior, not those
electrical assumptions.

## Releasing BMS_OK

Only release BMS_OK after all of these are true:

```text
status shows current valid
status shows voltage valid
volt shows full usable cell coverage
fault shows no hard fault
temp data is valid or fan fail-safe behavior is understood
bringup ready reports release_allowed=YES
BMS_OK output has been observed low while inhibited
```

Then:

```text
bmsok release
status
```

To force it low again:

```text
bmsok inhibit
```

## What SIL Already Proves

The host SIL tests prove the firmware logic fails closed under bad, stale,
partial, contradictory, and out-of-order simulated inputs.

They do not prove:

```text
real SPI electrical timing
ADBMS6822 physical behavior
isoSPI transformer polarity
SMB cable integrity
actual PEC behavior under noise
real cell voltage scaling
APM current/voltage scaling
ST-Link/debugger reliability
```

Any hardware-discovered bug should become a new host regression test once the
root cause is understood.
