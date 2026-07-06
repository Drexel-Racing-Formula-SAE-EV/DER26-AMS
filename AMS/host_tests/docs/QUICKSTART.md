# Quickstart


Authored by Mahad Faisal, 2026.
## 1. Put the harness in the AMS repo

```bash
cd DER26-AMS-feature-canbus_charger
cp -r /path/to/DER26-AMS-host-test-harness host_tests
```

## 2. Run normal tests

```bash
cd host_tests
make test
```

## 3. Run sanitizer tests

```bash
make asan
```

## 4. Run static analyzer

```bash
make analyze
```

## 5. Expected pass line

```text
ALL COMPREHENSIVE HOST INJECTION TESTS PASSED
```

## 6. Interpreting results

Pass means host-exercised firmware logic passed fake-signal and fault-injection checks.

Pass does not mean hardware is validated. You still need CubeIDE build, board flashing, ADBMS chain testing, charger CAN sniffing, ECU/dashboard test, and safe low-voltage bench validation.

## Optional Renode pre-hardware smoke

The repo includes an experimental Renode harness in `../renode/` for checking
that the STM32 image boots far enough to print the UART CLI banner and accept
safe bring-up commands before hardware is available.

From the repository root:

```bash
AMS_RENODE=1 ./ci/stm32/build_ams_headless_gcc.sh
./AMS/renode/scripts/run_renode_smoke.sh
```

Renode is useful for boot/CLI/task smoke and modeled ADBMS6830, SMB
temperature, DHAB current sensor, charger CAN, and CAN-TX packetization
software-path checks. It does not prove
ADBMS6822/isoSPI electrical behavior, ADBMS2950/APM, real charger CAN
transceiver behavior, ADC noise, or board-level electrical behavior. By
default, `AMS_RENODE=1` enables firmware-side fake backends for ADBMS6830,
temperature, current, charger, and CAN capture paths.

The Renode fake chain can be driven from the UART CLI before hardware exists:

```text
adbmsfake status
adbmsfake reset
adbmsfake ov 0 3
volt
fault
adbmsfake pec 0x0004
spi stat
bringup adbms6830
adbmsfake missing 0x0002
spi probe
adbmsfake reset
```

These commands validate firmware parsing and safety logic against modeled
ADBMS6830 data. They do not validate physical SPI, isoSPI, or analog behavior.

The Renode current and charger fakes can drive the next layer of safety logic:

```text
currentfake reset
current
currentfake amps 70
current
currentfake charge 10
current
currentfake mismatch 20 0
current
currentfake fail on
current
currentfake reset

chargerfake reset
state charge
charger
chargerfake flags 0x02
charger
chargerfake txfail on
charger
chargerfake txfail off
chargerfake timeout on
charger
state start
```

Use `currentfake status` and `chargerfake status` to see the fake model state,
then use the normal `current`, `charger`, `fault`, and `status` commands to see
what firmware accepted and how it faulted.

Temperature and CAN packetization fakes cover the next board-only layer:

```text
tempfake reset
temp
tempfake hot 0 0
temp
fault
fault reset-temp
tempfake missing 2 0x000003
temp
tempfake reset

canlog clear
state discharge
canlog
state charge
chargerfake rxgood
charger
canlog
```

For a repeatable manual Renode pass, open the scenario console and paste one
block at a time from the included command sheet:

```bash
./AMS/renode/scripts/run_renode_scenario_console.sh
```

```text
AMS/renode/scripts/cli_scenario_commands.txt
```

The scenario CLI sets modeled inputs for common cases:

```text
scenario healthy
scenario ov
scenario uv
scenario hot
scenario cold
scenario charge-ready
scenario charger-timeout
scenario charger-txfail
scenario current-trip
fault reset-all
```

`scenario current-trip` intentionally uses precharge/start state, so 70 A should
trip the precharge overcurrent policy. Clear latches with `fault reset-current`
or `fault reset-all` before moving to the next case.

## Hardware bring-up CLI order

See `docs/HARDWARE_SPI_BRINGUP.md` for the full hardware SPI/isoSPI test plan
and `docs/BRINGUP_CLI_REFERENCE.md` for the staged CLI summary commands.

For first board tests, build with `AMS_HW_BRINGUP=1` so BMS_OK stays inhibited until manually released:

```text
status
bringup board
bmsok status
spi clear
spi enable
spi coldwake
spi probea
spi probeb
spi probe
spi sid
spi stat
spi status
bringup adbms6830
volt
current
temp
bringup ready
bmsok release
```

Use `bmsok inhibit` to force BMS_OK low again. Keep `AMS_ENABLE_APM_2950_DEBUG=0` until ADBMS2950/APM probing is intentional; set it to `1` only for CLI-only APM bring-up.
