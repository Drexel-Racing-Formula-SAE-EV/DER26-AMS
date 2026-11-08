# New AMS: onboard ADBMS6822 bring-up

General guide for AMS bundle v2.6.30; firmware/CLI v0.5.33, 2026-10-05.
For the current one-SMB bench build v0.5.41, start with
[`ONE_SMB_BENCH.md`](ONE_SMB_BENCH.md). Its periodic cell and temperature scans run.
Based on the supplied ECU v2.10.7 / AMS v2.6.28 source-fix archive.

## What to build

Import `AMS` as an existing STM32CubeIDE project. Clean-build one configuration:

| CubeIDE configuration | Connect | Expected devices | Selection |
|---|---|---|---|
| BENCH Validation 5-SMB Debug or Release | AMS J1 to SMB0 port A, SMB0 B to SMB1 A, continuing through SMB4 | 5 SMBs, 5 physical devices, no APM | String A / PE2 |
| BENCH Validation 1-SMB Debug or Release | AMS J2 to one SMB port A | 1 SMB, 1 physical device, no APM | String B / PE4 |

Start with the five-SMB build if using the previously tested five-SMB chain. The
one-SMB build is useful for isolating the new board's second channel. To prove
both channels, power down between wiring changes and use the matching build.
The default five-SMB configuration is **not** a six-device SMB+APM ring build.
Do not insert an APM into this five-device topology or interpret `spi probeb`
as a second five-SMB read. A full six-device ring needs its explicit topology
configuration and separate validation.

The supplied firmware already has the correct onboard Mode 3 and MCU pin map;
there is no EVAL-driver-to-onboard-driver replacement. This update adds
`spi onboard`, release identity and this bring-up guide. The CubeIDE build
paths remain `Bench5_Debug`, `Bench5_Release`, `Bench1_Debug`, `Bench1_Release`.
Configuration descriptions now identify onboard U1. Flash the ELF produced by
your clean build, not an older workspace image. `ver` must report **0.5.33**.

This is a source release. No ARM target binary was built here: the ARM GCC /
CubeIDE toolchain is unavailable in this environment. Host validation is
recorded in `VALIDATION_ONBOARD_6822.txt` at the package root. ECU files are
byte-for-byte unchanged from the supplied archive.

## Physical transition from the old workaround

1. Power down. Remove the complete AMS J15-to-EVAL jumper harness and external
   EVAL isoSPI connection. Do not parallel a populated onboard U1 with the
   EVAL board on the same chip select/MISO bus.
2. Use the new board's onboard U1 and transformer/filter path. J15 is a test
   access header, not the onboard isoSPI connector. Do not carry over the old
   board's U1 isolation/depopulation rework to the new board.
3. With power off, check continuity from MCU pins to U1 and that CS_A/CS_B are
   not shorted. Inspect U1 orientation, soldering/EP ground, T1/T2, and the
   connector path. Check the actual board against supplied AMS Rev3.1 sheet 2;
   a newer PCB revision was not supplied.
4. Power the LV board with the traction/contactors path disabled for this
   bring-up. Measure U1 supplies and strap levels before communicating. USB
   power reaching the MCU does not by itself prove the AMS/U1 rails are alive.

## Verified signal mapping

From AMS Rev3.1 `COMMS.SchDoc` (sheet 2), firmware `main.h`, `main.c`, and
`stm32f7xx_hal_msp.c`:

| Function | STM32 | U1 channel A pin | U1 channel B pin |
|---|---|---:|---:|
| SCK | PG13, SPI6 AF5 | 4 | 28 |
| MOSI / PICO | PG14, SPI6 AF5 | 2 | 26 |
| MISO / POCI | PG12, SPI6 AF5 | 3 | 27 |
| Active-low CS | PE2 for A; PE4 for B | 5 | 29 |
| PHAPOL | Board-strapped 3V3 | 14 | 22 |
| MSTR | Board-strapped 3V3 | 1 | 25 |
| XCVRMD | Board-strapped GND | 16 | 24 |
| RTO | Board-strapped GND | 15 | 23 |
| VDDS | 3V3 | 6 | 30 |
| VDD | 3V3 | 12 | 20 |
| VP | 3V3 | 13 | 21 |
| GND | GND, plus exposed pad 33 | 9 | 17 |
| isoSPI IP / IM | Differential signal | 11 / 10 | 19 / 18 |
| Onboard path | Transformer then connector | T1 then J1 | T2 then J2 |

PE4 is the firmware CS_B pin; do not adopt the stale PF4 schematic annotation.
Verify PE4-to-U1 pin 29 continuity on the new assembly. Do not use the old
`spi cspins both` PE4/PF4 experiment as a routine new-board test.

J15 remains useful as a scope point: 1=3V3, 2=GND, 3=SCK, 4=MOSI,
5=MISO, 6=STRINGB_CS. It does not expose CS_A.

The drawing explicitly crosses transformer nets for layout convenience.
Do not infer connector polarity merely from a symbol's top/bottom position.
Verify actual cable pin numbers/continuity and remote IPA/IMA orientation.
Use a differential probe for isolated isoSPI; do not bridge the isolation
barrier with an earth-referenced scope ground.

## SPI and wake nuances

- Mode **3**, CPOL=1, CPHA=1 (`SPI_POLARITY_HIGH`, `SPI_PHASE_2EDGE`). SCK idles
  high; data is sampled on the rising second edge. PHAPOL is tied to VDDS on
  both onboard channels. The `.ioc` also retains these settings.
- STM32 master, full-duplex two-line, 8-bit, MSB-first, software active-low CS,
  NSS pulse disabled, TI mode disabled. HAL hardware CRC is disabled; ADBMS
  command/data PEC is handled by the protocol driver.
- Default /256 of 108 MHz PCLK2 = **421.875 kHz**. Keep that speed for first
  bring-up; do not start with the optional /128 or /64 experiments.
- MSTR high puts U1 on the controller side. XCVRMD grounded selects standard
  bidirectional isoSPI. RTO is grounded; this build does not depend on LPCM
  timeout-monitor operation. WAKE/INTR pins have no MCU connection in this
  sheet; normal waking is through the SPI/isoSPI path.
- PHAPOL selection is latched following VDDS power-up and wake into READY;
  changing straps while running does not change the SPI mode. Power-cycle
  after any hardware strap changes. The driver already performs wake pulses;
  `spi coldwake` is the explicit diagnostic recovery command.
- CS_A and CS_B must not be asserted together. The shared POCI/MISO line is
  released by an unselected controller-side channel. A high idle MISO is
  therefore not proof of a working link.
- ADBMS6822 is a transparent transceiver, not an addressable register bank:
  there is no 6822 SID/config register to read. `spi sid` reads downstream
  ADBMS6830 identities. Successful HAL transfer alone proves neither the
  transceiver nor the remote device responded; require valid PEC and IDs.

ADI reference: ADBMS6821/ADBMS6822 Rev. B, Tables 20/21 and SPI configuration /
power / wake sections:
https://www.analog.com/media/en/technical-documentation/data-sheets/adbms6821-adbms6822.pdf

## CLI: initial board and transport check

USART3, **115200 8N1**, no hardware flow control. MCU TX=PD8, RX=PD9;
use a 3.3 V UART interface with common LV ground. Capture the entire session.
Commands below are separate lines:

```text
ver
status
bmsok status
balance status
spi onboard
spi status
bringup board
spi clear
spi enable
spi coldwake
spi probe
spi sid
spi stat
spi cfgchk
spi status
bringup adbms6830
volt
fault
```

`spi onboard` reports the schematic expectation and selected software topology;
it cannot detect whether U1 or an EVAL is physically connected. Its live
mode3 result describes the HAL configuration, not an oscilloscope measurement.
If a diagnostic reports `scan active`, retry it after the current scan. Do not
remove the scan guard or suspend the measurement task. `spi enable/disable`
controls debug instrumentation, not acquisition.

For five SMBs on J1, expect selected A, logical/physical 5, expected/SID mask
`0x001F`, device ID `0x03` for each SMB, valid distinct serial IDs, PEC failure
mask zero and command-counter mismatch mask zero. For one SMB on J2, expect
selected B, logical/physical 1 and mask `0x0001`. Cell data must match measured
inputs; a packet filled with 00 or FF is not a pass.

`spi probe` always uses the selected SMB path. `spi probea` explicitly probes
SMBs on A. **`spi probeb` is an ADBMS2950/APM identity probe in this source.**
It is not appropriate for testing an SMB on J2: use the one-SMB build and
`spi probe`/`spi sid`. An absent APM is expected in both supplied configurations.

## Scope and repeatability checks

With the matching chain attached, use one of:

```text
spi scope a read 20
```

or, for the one-SMB J2 build:

```text
spi scope b read 20
```

Check the intended CS only, SCK idle high and approximately 421.875 kHz while
active, MOSI activity, and a valid returning response. Trace activity from
MCU to U1 pins, then U1 IP/IM, transformer, connector and the SMB. Scope commands
are diagnostic traffic; resume normal observation afterward.

After initial SID/configuration checks pass:

```text
spi csoak 100
spi timing
spi faults
spi snapshot
volt
temp
current
can diag
rtos
bringup snapshot
```

Save the complete results; require stable IDs, zero new PEC/counter errors,
consistent voltages and no new communication fault through repeated reads.
Power-cycle the complete LV setup and repeat the basic sequence to prove cold
startup. Keep onboard A/B results separate: proving one channel does not prove
the other transformer/connector. These checks establish new-board transport;
they do not substitute for the remaining system validation.

## Other new-board checks and retained limits

Check `current` for both DHAB paths: PC0/ADC2_IN10 low-range, PA3/ADC1_IN3
high-range. Check the actual sensor supply, zero-current level and prior
calibration before interpreting its validity flags. Current readings from the
old AMS are not measurements of this new ADC assembly.

Automatic thermistor mux scanning remains **off** in the supplied profile.
For explicit measurements use `tempsns <ic> <sensor 0-23>`; `tempbus idle` is a
bus-level electrical diagnostic, while `tempbus scan` actively exercises the mux
bus. Retain the established SMB pull-up validation; replacing the AMS does not
by itself resolve an SMB routing issue. `temp` may show missing/stale values
until valid measurements exist. Do not claim full temperature coverage from
cell SPI success.

`bringup board`, `can diag`, `current`, `rtos` and `bringup snapshot` provide the
rest of the new-board evidence. CAN remains 1 Mbit/s in both IDE variants.
BENCH_VALIDATION keeps BMS_OK and balancing compile-time inhibited, C-only SMB
voltage mode, and existing safety fixes. The passive five-SMB estimator assumes
an open/no-current bench; it is not a loaded-pack validation configuration.
Do not use `bmsok release` or `balance release` to turn this into a driving build.
