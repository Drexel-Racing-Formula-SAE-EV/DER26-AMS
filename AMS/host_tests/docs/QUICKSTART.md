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
spi preset normal
spi scope
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

Use `bmsok inhibit` to force BMS_OK low again. The final ring builds with
`AMS_ENABLE_APM_2950=1`, while `AMS_APM_ENABLE_HV_DIVIDERS=0` keeps the APM
divider controls off for initial SPI/current testing. APM measurements remain
advisory and non-gating.
