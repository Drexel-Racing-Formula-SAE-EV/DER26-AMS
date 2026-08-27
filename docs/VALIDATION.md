# Validation Strategy

No single verification layer is sufficient for safety-relevant embedded firmware. The repository therefore keeps host logic tests, deterministic stress, source-contract gates, target-build checks, HIL assets, and explicit hardware qualification boundaries.

## Host/SIL

Run the self-contained firmware CI from:

```bash
cd AMS/host_tests
make firmware-ci
```

This covers the current unit/system suites and architecture/profile/static-analysis gates that do not require external repositories.

For additional sanitizer/stress qualification:

```bash
make firmware-asan
make ubsan
make stress
```

Focused targets are documented in [`../AMS/host_tests/README.md`](../AMS/host_tests/README.md) and [`../AMS/host_tests/docs/TEST_MATRIX.md`](../AMS/host_tests/docs/TEST_MATRIX.md).

## Repository/target-build checks

`ci/scripts/` verifies the expected project structure and rejects generated artifacts. `ci/stm32/build_ams_headless_gcc.sh` provides an ARM-GCC target build independent of an IDE workspace.

## Target bench qualification

Target hardware remains mandatory for properties host tests cannot measure, especially:

- task-stack high-water marks;
- ISR timing and scheduling jitter;
- CAN interrupt/error behavior;
- ADBMS/isoSPI wake/read timing;
- physical sensor polarity/calibration;
- watchdog and reset behavior;
- BMS_OK electrical safe-state behavior.

## HIL

`HiL/` contains the plant/support side used to exercise firmware behavior beyond pure host logic. HIL qualification does not replace final electrical/vehicle validation.
