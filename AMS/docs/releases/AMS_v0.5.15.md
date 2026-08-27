# AMS v0.5.15 Safety Review Updates

Date: 2026-08-26
Baseline: ECU v2.10.6 / AMS v0.5.14 BENCH hotfix

## Changes

- Kept AMS CAN task at 1536 StackType_t words (6144 bytes).
- Replaced the single fixed 96-word low-stack warning threshold with a per-task margin policy:
  - warning = max(96 words, 25% of configured stack)
  - critical = max(64 words, 15% of configured stack)
- Added `rtos_stack_critical_mask` / `rtos_stack_critical` diagnostics and a dedicated RTOS fault flag.
- A critical stack margin is now a hard fault and therefore forces BMS authority fail-low before a FreeRTOS overflow occurs.
- Exposed warning and critical thresholds per task through the `rtos` CLI and the RTOS CAN logger status byte.
- Added a host regression proving a critical CAN stack margin drops BMS_OK before an actual overflow.
- Added four explicit CubeIDE bench configurations:
  - BENCH Validation 5-SMB Debug
  - BENCH Validation 5-SMB Release
  - BENCH Validation 1-SMB Debug
  - BENCH Validation 1-SMB Release
- The 1-SMB configurations compile `AMS_BENCH_VALIDATION_SINGLE_SMB=1`; the 5-SMB configurations do not.
- Gave each CubeIDE bench variant a separate build output directory so one variant cannot silently overwrite another.
- Unified source provenance under `AMS_SOURCE_REVISION="DER26-AMS-v0.5.15-20260826"`; TESTDAY compiled from this tree no longer advertises the older v0.5.12 source revision.
- Updated the target-project host gate to require both 1-SMB and 5-SMB configurations and their exact profile/bitrate defines.

## Validation run

Passed on the v0.5.15 tree:

- Comprehensive AMS host injection/unit/SIL test suite
- RTOS stack/heap diagnostics regression, including new critical-margin fail-low case
- Profile gates (BENCH, HIL, TESTDAY, BENCH_VALIDATION, VEHICLE evidence gate)
- Estimator topology gates
- Production safety gates
- Static-allocation gate
- State/BMS ownership gate
- CubeIDE target-project/configuration gate
- DER26-CAN-V4 load-budget gate
- DER26-CAN-V4 contract gate
- Whole-source GCC `-fanalyzer`/syntax analysis in bench and vehicle profiles

## Still requires target evidence

This release does not replace hardware qualification. Before vehicle authority is enabled, record real target stack high-water marks for every AMS task under worst-case CAN/CLI/ADBMS/estimator load, plus target WCET/ISR, linker-map/RAM, watchdog and physical sensor/interface validation.
