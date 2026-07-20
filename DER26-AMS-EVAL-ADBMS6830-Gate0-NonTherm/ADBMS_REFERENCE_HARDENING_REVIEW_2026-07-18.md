# AMS ADBMS Reference-Aligned Hardening Review

Date: 2026-07-18  
Scope: production AMS firmware, ADBMS6830 SMB monitoring and advisory ADBMS2950 APM integration  
Release: `DER26-AMS-C-Hardened-ADBMS-Mixed-Ring-Crosschecked-2026-07-18`

## Outcome

The ADBMS paths were reviewed against the supplied device/reference material and then hardened in the existing C firmware. The final source passes the complete host verification matrix described below.

This release is **host-verified, not target- or hardware-validated**. It is not approval for HV operation. Initial physical testing must remain current-limited and low voltage, with `BMS_OK` physically inhibited and balancing disabled.

## Implemented changes

### ADBMS6830 SMB path

- Separates the five logical SMBs from the six physical devices expected on the complete ring.
- Bounds the configured topology before packing, parsing, SPI, or GPIO operations.
- Validates PEC and command counters before publishing cell, status, temperature, or diagnostic data.
- Reads and validates Status A through E, including supply/reference plausibility and oscillator-count limits.
- Treats the redundant-conversion `COMP` indication as expected conversion activity rather than a fault.
- Clears OV/UV flags with the required addressed write payload instead of issuing an incomplete command-only transaction.
- Performs startup conversion and status checks before allowing ADBMS readiness.
- Uses bounded conversion and wake delays and propagates wake/transport failures.
- Runs a fresh S-ADC open-wire sequence using baseline/even/odd measurements and rejects stale or incomplete images.
- Schedules status, configuration, and open-wire diagnostics at distinct bounded intervals.
- Replaces the misleading CLI “self-test” behavior with a full-channel conversion-path integrity diagnostic.
- Saturates diagnostic counters and exposes status through the CLI.

### ADBMS2950 APM path

- Adds the ADBMS2950 as an advisory sixth device on String B without making unvalidated current data a `BMS_OK` input.
- Initializes the current ADC through a bounded first-calibration conversion before entering continuous operation.
- Uses `UNSNAP`/`SNAP` around coherent status, current, voltage, and flag reads.
- Validates PEC, a coherent command counter, reset-clear state, conversion completion, and sample freshness.
- Preserves the last numeric sample for diagnosis while immediately clearing validity metadata after any failed or incomplete coordinated scan.
- Keeps HV divider-enable outputs off until divider scaling, polarity, and hardware behavior are validated.
- Adds CLI diagnostics while quarantining legacy unsafe helper paths.

### Mixed-ring coordination

- Assumes the physical order `String A -> five ADBMS6830 SMBs -> one ADBMS2950 APM -> String B`.
- Requires all five String-A devices to identify as ADBMS6830B before the first SMB configuration write.
- Fixes the SMB write owner to String A and the APM write owner to String B; an opposite-end subset write is rejected before GPIO or SPI activity.
- Validates exact counts, embedded-array pointers, shared SPI/timer/CS bindings, active ends, and write owners before physical measurement or balancing operations.
- Locks the five-device SMB frame to 44 bytes and the one-device APM frame to 12 bytes at device boundaries.
- Accounts for APM transactions when predicting the SMB command counter.
- Resynchronizes counter tracking after partial, failed, initialization, or standalone service transactions.
- Issues an APM read only after a successful full SMB scan has established a one-shot full-ring-awake token.
- Consumes that token before the APM transaction and invalidates APM freshness if the coordinated scan cannot complete.
- Prevents a previous valid APM current sample from remaining marked valid after a later failed SMB/APM epoch.

## Files changed

- `Core/Inc/ext_drivers/accumulator.h`
- `Core/Inc/ext_drivers/adbms2950.h`
- `Core/Inc/ext_drivers/adbms6830_data.h`
- `Core/Inc/ext_drivers/adbms6830_functions.h`
- `Core/Inc/ext_drivers/adbms_shared.h`
- `Core/Src/ext_drivers/accumulator.c`
- `Core/Src/ext_drivers/adbms2950.c`
- `Core/Src/ext_drivers/adbms6830.c`
- `Core/Src/tasks/adbms_task.c`
- `Core/Src/tasks/cli_task.c`
- `host_tests/src/ams_host_test_runner.c`
- `host_tests/unit/ams_unit_test_runner.c`

## Verification completed

| Check | Result |
| --- | --- |
| Isolated unit suite | Passed, 43 tests |
| Comprehensive host injection/SIL suite | Passed, 96 tests |
| ADBMS2950 APM SIL profile | Passed |
| Production safety gates | Passed |
| BMS output ownership and heartbeat gates | Passed |
| Static-allocation gate | Passed |
| Hardware-bring-up build/profile | Passed |
| CAN-fed ADBMS HIL-replacement profile | Passed |
| IMD-enabled profile | Passed |
| AIR-feedback-stub profile | Passed |
| IWDG-enabled safety profile | Passed |
| AddressSanitizer + UndefinedBehaviorSanitizer | Passed |
| Standalone UndefinedBehaviorSanitizer | Passed |
| GCC static analyzer | Passed |
| Deterministic fuzz | Passed, 50,000 cycles |
| Concurrent stress | Passed, 12,000 cycles |
| Separate-translation-unit strict GCC compile | Passed with warnings treated as errors |
| Fresh extraction: unit, system SIL, APM SIL, and production gates | Passed |

The environment did not contain `arm-none-eabi-gcc`, Clang, or cppcheck. No STM32 target link, map/stack analysis, flash, timing capture, or physical fault injection was performed.

## Required hardware validation

1. Confirm the exact six-device physical order from both ends using SID/configuration reads before enabling any write-capable operation.
2. Capture SPI and isoSPI traffic with a logic analyzer and verify wake timing, packet count, PEC, and command-counter progression.
3. Verify the behavior of partial-ring writes, an open link at each boundary, a missing device, and recovery after wake/sleep transitions.
4. Validate ADBMS2950 shunt value, gain, current polarity, current scaling, VB1 divider ratio, and divider-control polarity against the actual APM hardware.
5. Inject bad PEC, wrong command counters, SPI timeouts, stale conversions, and broken-ring cases while confirming `BMS_OK` remains low.
6. Validate open-wire behavior with known resistive fixtures before relying on it as a pack-level diagnostic.
7. Cross-build and link with the actual STM32 toolchain, inspect flash/RAM/map output, and measure task stack and timing margins.

## Deliberate limitations

- ADBMS2950 data remains advisory and does not gate `BMS_OK`.
- ADBMS2950 fault-register bits are reported but are not yet comprehensively classified into safety policy.
- Automatic dual-direction redundant reads and failover are not implemented.
- ADBMS2950 divider enables remain off.
- The cell conversion CLI diagnostic is not a complete silicon latent-fault self-test.
- Existing board-level limitations—including unvalidated IMD hardware and optional AIR auxiliary feedback—remain unchanged.

## Low-energy first-test boundary

- Physically inhibit `BMS_OK`; do not depend only on a software flag.
- Disconnect HV, AIRs/contactors, charger, and actuator loads.
- Keep balancing and APM divider enables off.
- Power from a current-limited low-voltage supply and begin with read-only SID, configuration, status, and cell-conversion diagnostics.
- Stop on unexpected rail current, heating, output movement, counter/PEC instability, or a topology mismatch.

The supplied private reference material was used only for local semantic cross-checking and is not included, quoted, or redistributed in this release.
