# AMS Hardware SPI Bring-Up Guide

This guide is for first bench tests of the AMS board ADBMS SPI/isoSPI path.
It assumes the ADBMS6822 is on the AMS board, the ADBMS6830 devices are on the
SMBs, and the ADBMS2950/APM path is debug-only until verified.

## Safety Setup

Use a low-voltage bench setup first. Do not rely on BMS_OK, charger control, or
APM current data until the SPI path is proven.

Build first hardware firmware with:

```text
AMS_HW_BRINGUP=1
AMS_ENABLE_APM_2950_DEBUG=0
```

`AMS_HW_BRINGUP=1` keeps BMS_OK physically inhibited until the CLI command
`bmsok release` is run. This lets the firmware boot, scan, and report status
without accidentally enabling the downstream system.

Keep `AMS_ENABLE_APM_2950_DEBUG=0` for the first SMB/ADBMS6830 tests. Set it to
`1` only when intentionally probing the ADBMS2950/APM path.

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
CS_B / SMB chip-select path
CS_A / APM chip-select path, only when testing APM
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
bmsok status
```

Expected:

```text
BMS_OK:0
inhibit:1
SPI6 CPOL:HIGH CPHA:2EDGE
```

If CPOL/CPHA do not match, stop and fix firmware/config before chasing hardware.

## ADBMS6830 / SMB SPI Test Order

Run commands in this order:

```text
spi clear
spi enable
spi coldwake
spi status
spi probe
spi sid
spi stat
spi status
volt
fault
status
```

Expected useful signs:

```text
spi status shows CPOL:HIGH CPHA:2EDGE
spi probe returns OK or at least records a non-OK HAL status
spi sid shows valid 48-bit serial IDs for each responding IC
spi stat exposes SLEEP/SPIFLT/THSD/OSCCHK and OV/UV status flags
TX preview shows command bytes plus PEC
RX preview changes from all-zero/all-FF when the chain responds
PEC pass/fail masks are visible
command counters are visible when response frames are valid
command-counter mismatch mask remains 0 during a stable command sequence
volt shows usable/updated/stale/PEC cell counts
```

Bad but useful signs:

```text
HAL_TIMEOUT or HAL_ERROR increments error_count
RX preview all 00 or all FF
PEC fail mask set for all ICs
updated cell count stays 0
voltage reason remains not_ready or stale_scan
SID validity stays false
status flags remain invalid
command-counter mismatch appears after missed/corrupt transactions
```

Those results mean the debug path is working, even if hardware communication is
not yet fixed.

## What To Check On The Logic Analyzer

For `spi probe`, verify:

```text
CS_B goes low once for the full command + dummy readback transaction
SCK idles high
data changes on MOSI with MSB-first command bytes
dummy bytes continue clocking during the readback phase
MISO is not stuck low/high if the ADBMS chain responds
CS_B returns high after the transaction
```

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

## Fault Isolation Matrix

| Symptom | Likely area | Next check |
|---|---|---|
| No UART banner | MCU boot/clock/UART | Check power, reset, flash, UART pins |
| CLI works, no SCK | SPI init or command path | Run `spi coldwake`/`spi probe`; confirm hspi6 handle |
| SCK/MOSI works, CS_B never toggles | CS pin mapping | Check `CS_B_GPIO_Port` / `CS_B_Pin` wiring |
| CS/SCK/MOSI works, MISO stuck | ADBMS6822/isoSPI/SMB path | Check ADBMS6822 power, isoSPI pair, transformer, SMB power |
| RX changes but PEC always fails | SPI mode, bit alignment, chain order, noise | Confirm mode 3, sample point, SMB count, isoSPI wiring |
| SID reads but order changes | Daisy-chain instability or mixed string routing | Repeat `spi sid`; check SMB order, harnessing, and CS string selection |
| Status shows SLEEP after clear | Device reset/sleeping between commands | Check wake timing, REFON/config writes, power stability |
| Command-counter mismatch | Missed command, reset/sleep, or interleaved transaction | Check SPI lock, task timing, CS noise, and PEC failures |
| First probe works, later reads corrupt | task/CLI collision or wake timing | Confirm shared SPI lock, increase delay, avoid repeated CLI spam |
| BMS_OK goes high during bring-up | build flag or inhibit bug | Confirm `AMS_HW_BRINGUP=1`, run `bmsok inhibit` |

## ADBMS2950 / APM Test Order

Only after SMB/ADBMS6830 communication is understood, build with:

```text
AMS_HW_BRINGUP=1
AMS_ENABLE_APM_2950_DEBUG=1
```

Then run:

```text
apm clear
apm enable
apm status
apm probe
apm status
```

Expected:

```text
APM debug counters update
APM TX/RX previews are visible
APM failures do not change BMS_OK
normal current/voltage/fault logic still ignores APM
```

Do not use ADBMS2950/APM for safety gating until command behavior, PEC, scaling,
current sign, shunt value, and VBAT divider values are confirmed on the bench.

## Releasing BMS_OK

Only release BMS_OK after all of these are true:

```text
status shows current valid
status shows voltage valid
volt shows full usable cell coverage
fault shows no hard fault
temp data is valid or fan fail-safe behavior is understood
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
