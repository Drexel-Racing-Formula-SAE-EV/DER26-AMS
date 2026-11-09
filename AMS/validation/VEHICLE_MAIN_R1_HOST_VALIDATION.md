# AMS v0.5.57-r1 — 1 Mbps vehicle main-branch validation

Test date: 2026-11-08 (container/native GCC). This is source + host-test validation, not ARM target compilation.

## Supported vehicle CI

```sh
cd host_tests
make -j2 vehicle-ci
```

**PASS** (exit code 0). Coverage includes:
- Four vehicle CubeIDE configuration/profile bindings and unique output directories
- Runtime + source + CubeMX 1 Mbps CAN timing and HSE PLL contract
- Both vehicle profile-4 and test-evidenced profile-3 estimator initialization at 27% (five segment EKFs and coulomb counter)
- Fail-closed profile-3 compile check without validation acknowledgements
- Synthetic test-only profile-3 source syntax checking; no release evidence claimed
- Five SMBs/no installed APM/fan topology and no charger-only configuration
- Vehicle BMS_OK/state/CAN handling, scheduler, CLI, ADBMS acquisition timing and measurement integrity
- Standalone portable ADBMS2950B driver tests (driver is not installed or active on the vehicle)

Raw output: `VEHICLE_MAIN_R1_HOST_TEST_OUTPUT.txt`.

## Legacy mixed-topology unit suite

```sh
cd host_tests
make unit
```

**FAIL** after successful compilation/link: 68 legacy assertion failures in voltage fault, wake/timing, and ADBMS2950 sample expectations. Before this revision, `make unit` failed to link with missing HAL/RTOS tick functions; that link issue is corrected using host-only shims. The failing legacy assertions are not silently disabled or treated as passing. They are not part of the vehicle-only CI profile.

## Not validated

No ARM toolchain available; no ELF/HEX, flash, hardware timing, CAN ACK, or ECU interoperability test. Profile-3 production build still requires its real validation evidence.
