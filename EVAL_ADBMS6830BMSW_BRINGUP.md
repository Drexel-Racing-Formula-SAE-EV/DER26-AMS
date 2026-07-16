# DER26 AMS + EVAL-ADBMS6830BMSW monitor-only branch

This package is a branch-ready firmware variant for the Analog Devices
`EVAL-ADBMS6830BMSW` evaluation board. It is intended only for staged,
low-energy communication and cell-voltage bring-up through the ADBMS6822
already present on the AMS.

It is **not** permission to energize the vehicle, close AIRs, connect a
charger, enable balancing, or treat the AMS safety output as validated.

## What changed

The package defaults to:

```text
AMS_EVAL_ADBMS6830_BMSW = 1
ADBMS monitors           = 1
cell channels            = 16
active AMS isoSPI path   = String B / CS_B
ADBMS init               = SRST + SID integrity read; no CFGA/CFGB write
BMS_OK                    = compile-time locked low
balancing                 = compile-time disabled
DER SMB thermistor mux    = disabled
open-wire stimulation     = disabled
fan outputs               = forced to 0%
75-series EKF             = disabled
AMS state changes         = blocked; remains START
CAN HIL injection         = rejected at compile time
```

The normal five-SMB/15-cell source behavior is still selected with
`-DAMS_EVAL_ADBMS6830_BMSW=0` and is exercised independently by the production
host tests.

## Electrical connection boundary

The firmware adaptation does not make the hardware literally connector-only.
Before power, verify all of the following against the current ADI board user
guide and your AMS schematic:

1. Keep HV vehicle wiring, AIRs, charger, fan loads and actuators disconnected.
2. Physically isolate or inhibit `BMS_OK`; do not rely only on firmware.
3. Power the EVAL board from its J1 cell-stack/resistor-ladder interface using
   the ADI-approved setup. isoSPI does not power the monitor board.
4. Use a current-limited source and an approved lab procedure. The official
   16-channel resistor-ladder example is a multi-tens-of-volts setup, not a
   casual 3.3 V logic test.
5. Connect the AMS **String B** isolated pair to EVAL-board **Port A / J3** for
   the default firmware path. Preserve pair polarity and the documented
   termination/jumper configuration.
6. Do not connect a logic ground between the isolated isoSPI cable ends.
7. Leave EVAL Port B / J4 in the user-guide-prescribed end-of-chain state.

If you intentionally wire the AMS String A output instead, use `spi probea`
for the first manual probe and change the selected runtime string only after
confirming the physical mapping. The automatic scan in this branch remains on
String B.

## Build and host verification

The eval profile is already the default in this package; no CubeIDE symbol is
required. Import/open `AMS` in STM32CubeIDE and build the desired Debug or
Release configuration.

Before flashing, run:

```sh
cd AMS/host_tests
make eval-adbms6830-test
make eval-adbms6830-sanitize
make eval-adbms6830-analyze
make unit
make test
make production-gates
make asan
make ubsan
make analyze
```

The host checks cannot prove STM32 linking, isoSPI signal integrity, board
power sequencing, channel scaling, connector polarity, or real hardware
timing. A target build and current-limited physical test are still required.

## First powered test

Use a physically inhibited BMS output and stop immediately for unexpected
current, heating, rail collapse, MOSFET-gate movement, fan movement, or any
unexpected output transition.

Expected UART banner:

```text
Build:eval-6830bmsw ... BMS_OK_inhibit:1
EVAL LOCK: 1x ADBMS6830B / 16 cells / String B; ...
```

Recommended command order:

```text
ver
status
spi pins
spi probeb
spi sid
spi stat
volt
bringup adbms6830
```

Expected behavior:

- `spi probeb`, `spi sid` and `spi stat` must return valid HAL/PEC/counter
  results for exactly one IC.
- `volt` reports all 16 channels. Confirm each channel against an independent
  DMM before trusting scaling or ordering.
- Temperature, IMD, current, charger and vehicle-readiness faults may remain
  asserted because those systems are outside this isolated eval setup.
- `BMS_OK` remains low even if the mutable runtime inhibit is corrupted or a
  CLI release is attempted.
- `balance release`, balance writes, `tempsns`, open-wire commands, ADC
  self-test/error-injection commands, and raw command/pattern scope modes are
  blocked.
- The legacy ECU voltage frame layout still represents the 15-cell vehicle
  segment contract. Use the CLI or logger detail frames to inspect channel 16
  during this eval-board test.

## Exit criteria before using an SMB

Do not call the link validated until you have captured:

- AMS SPI6 SCK/MOSI/MISO and CS_B timing.
- ADBMS6822 isoSPI transformer activity and correct pair polarity.
- Stable SID/status/config reads with no PEC or command-counter errors.
- All 16 ladder voltages compared with a calibrated DMM.
- Disconnect/reconnect and corrupted-link tests that fail closed.
- A target map/size report and stack high-water measurements.

Only then move to a real SMB, still with balancing and BMS_OK physically
inhibited for the first stage.

## Official board references

- ADI EVAL-ADBMS6830BMSW product page:
  <https://www.analog.com/en/resources/evaluation-hardware-and-software/evaluation-boards-kits/eval-adbms6830bmsw.html>
- ADI evaluation-board guide:
  <https://analogdevicesinc.github.io/documentation/solutions/reference-designs/eval-adbms6830bmsw/index.html>
