# AMS v0.5.57 vehicle 1 Mbps — host validation

Validation timestamp: 2026-11-08. All commands run from `host_tests`.
A native compile/test harness is not an ARM compile, hardware test or driving release qualification.

## Commands

```sh
make -j2 vehicle-variant-test cli-async-test can-tx-scheduler-test vehicle-project-check adbms-wake-policy-test measurement-integrity-test apm-hardening-test apm-driver-regression
```

Exit: 0.

## Pass records

- PASS real CLI IT TX wrap, ordering, CRLF and cooperative backpressure
- PASS bounded RX, CRLF, burst rejection, overflow rejection and stop bypass
- PASS cancellation and stalled TX have finite cooperative waits/recovery
- PASS failed IT start recovery and exact stop recognition
- PASS public temperature wrappers, stale boundary, zero and wrap
- PASS rolling sweep, duplicate rejection, failed visit, recovery, timestamp provenance and time filter
- PASS capture age, wrap, scheduled visits and transport rejection
- PASS nested/burst wake checks do not emit additional wake trains
- PASS production 6822/6830 wake/idle/integrity/timing regressions
- PASS four CubeIDE configurations, identity, 1-Mbit/s runtime/project/Cube timing; no fabricated release evidence
- PASS drive release rejects missing validation evidence
- PASS native drive-profile sources and full-75-cell/no-CLI/five-device contract with synthetic test-only evidence
- PASS late boundary rejection and clean recovery
- PASS carried uncertainty, extrema, sticky mixed range, next-window recovery
- PASS unknown uncertainty and late sample rejection
- PASS tick wrap and excessive gap rejection
- PASS rotation cannot hide a real-sample gap
- PASS stale rotation cannot charge a new window for an old interval
- PASS CAN calibration quality, immutable sequence/age, expiry and fallback
- PASS CAN per-reading expiry, aggregates, overflow and tick wrap
- PASS SoP and SoH reject unknown/zero uncertainty as calibrated
- PASS SoP/SoH effective cell/temperature freshness, mux cadence and tick wrap
- PASS ADBMS2950 calibration/phase freshness
- PASS ADBMS2950 masked config readback
- PASS ADBMS2950 COMM ACK/EEPROM probe
- PASS ADBMS2950 redundant diagnostic/CS timing
- PASS ADBMS2950 REFUP/snapshot/flags/CCNT reset
- ALL AMS UNIT TESTS PASSED
- PASS ADBMS2950 single-shot/continuous initialization
- PASS ADBMS2950 calibration/phase freshness
- PASS ADBMS2950 masked config readback
- PASS ADBMS2950 COMM ACK/EEPROM probe
- PASS ADBMS2950 redundant diagnostic/CS timing
- PASS ADBMS2950 REFUP/snapshot/flags/CCNT reset
- ALL AMS UNIT TESTS PASSED
- PASS DER26 CAN V4 TX scheduler unit/stress tests
- PASS actual CAN task stages vehicle protected telemetry and loads hardware mailbox fixture
- PASS 1-Mbit/s vehicle variant, five-SMB/no-APM/no-fan topology, drive transitions, STOP, ECU RX, charge rejection and final-load disable

## Excluded

The DIAG5 APM standalone ADC-probe fixture targets a separate evaluation-board mode and was not ported into this installed five-SMB vehicle variant.
The 250 kbps charging-control gate and charger-enable logic are NOT part of this vehicle release.
Real profile-3 compilation remains blocked until source-required validation evidence is supplied.
No cross ARM toolchain, flash/target validation, or measured 1Mbps ECU interoperability was available in this environment.
