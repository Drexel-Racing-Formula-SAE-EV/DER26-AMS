# DER26 AMS ADBMS SPI Read Hardening — Validation Report

**Date:** 2026-07-26
**Target:** DER26 AMS v0.3.6 ADBMS path
**Environment:** host GCC; no STM32 ARM toolchain or hardware connected

## Source changes validated

- removed cell-read mutation of `last_temp_updated_mask[]`;
- removed dead file-scope `rx_pec_error`;
- made `adbms6830_rd48_checked()` fail on PEC/counter integrity faults;
- added categorized `last_read_result` diagnostics;
- added explicit 1 microsecond CS setup/hold timing to ADBMS6830 and ADBMS2950 transports;
- added fail-safe CS cleanup for delay and HAL failures;
- documented the future 10 Hz no-rewake architecture and hardware gates.

## Commands completed

```bash
make -C AMS/host_tests clean unit
make -C AMS/host_tests test
make -C AMS/host_tests analyze whole-source-analyze
make -C AMS/host_tests asan ubsan
```

## Results

- focused AMS unit suite: PASS;
- thermistor unit suite: PASS;
- comprehensive host injection/SIL suite: PASS;
- GCC analyzer for the host and unit translation units: PASS;
- GCC whole-source analyzer in bench and fully acknowledged vehicle profiles: PASS;
- AddressSanitizer and UndefinedBehaviorSanitizer suites: PASS;
- no sanitizer finding was reported.

The strengthened ADBMS unit suite specifically passed:

```text
PASS ADBMS topology/delay guards
PASS ADBMS SPI debug write/full-duplex
PASS ADBMS SPI checked-read integrity
PASS ADBMS SPI SID/status/counter diagnostics
PASS ADBMS command counter rejects stale data
PASS ADBMS2950 SPI write/full-duplex
PASS ADBMS2950 SID/sample integrity
```

## Not completed

- `arm-none-eabi-gcc` target compile/link/map: toolchain unavailable;
- STM32F767 flash/run;
- logic-analyzer capture of CS/SCK/MOSI/MISO;
- confirmation that the configured delay timer is 1 MHz on target;
- five-SMB plus APM physical-ring communication;
- cold/warm/long-idle wake campaign;
- target scan-duration and deadline-miss measurements;
- 10 Hz scan qualification.

## Release disposition

- **1 Hz inhibited hardware bring-up:** software-ready, physical evidence still required.
- **10 Hz normal scan:** not released; timing/wake evidence remains open.
- **Vehicle/BMS authority:** unchanged and still controlled by existing release gates.
