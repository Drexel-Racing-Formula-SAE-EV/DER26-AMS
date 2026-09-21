# DER26 AMS follow-up review — package v2.6.20

Date: 2026-09-05. Firmware identity: 0.5.23.
Baseline: supplied v2.6.19 measurement-integrity archive (firmware 0.5.22).

## Disposition of the four quoted defects

All four already have fixes in the supplied baseline; this patch preserves them.

| Prior defect | Verified implementation |
|---|---|
| Timestamp race | ADBMS takes the current-window mutex before capturing the boundary. Current sampling captures completion while holding the same mutex. The accumulator rejects backward timestamps without resetting its integration cursor. |
| Carried uncertainty lost | Rotation preserves last-sample uncertainty, range, extrema and calibration provenance. |
| Mixed range overwritten | Separate metadata-initialized state prevents mixed/unknown zero from being mistaken for an empty window. |
| CAN 0x68B overclaims calibration | Quality uses the snapshot current's record confidence, nonzero calibration ID and known nonzero uncertainty. Sequence and age describe that same current window. |

The existing arithmetic regressions pass as part of the expanded focused executable. No measurement arithmetic was changed in this follow-up.

## New finding 1 — Medium: CAN does not expire individual readings

**Fixed.** `AMS/Core/Src/tasks/canbus_task.c`, `can_measurement_view_build()` and its ECU/logger detail consumers.

The outer CAN loop already rejects snapshots older than 500 ms. That does not protect individual readings approaching the existing 2500 ms cell or 12000 ms temperature age limits. A fresh publication may contain a still-usable reading only 10 ms from expiry. The baseline keeps its valid flag, usable mask and numeric detail value after another 11 ms, until a subsequent publication changes them or the whole snapshot expires.

Reproduction using the production CAN view builder:

- Publication and voltage-completion ticks: 1000 ms.
- One cell age: 2490 ms; one temperature age: 11990 ms. Their mask bits are set. A second fresh cell and temperature remain available.
- At tick 1010, the readings are at their respective age limits and remain usable.
- At tick 1011, baseline still reports both voltage and temperature valid. The new regression fails on that assertion before the fix.
- After the fix, the expired readings are removed from aggregates and usable masks; global voltage/temperature validity clears. Detail accessors return the existing invalid encodings while preserving the fresh readings.

The fix uses ages relative to the correct timestamps: cell ages are stored at voltage completion, temperature ages at publication. Limit subtraction avoids unsigned age-addition overflow; ordinary tick wrap is covered. A compact per-view mask keeps summary and detail decisions consistent without modifying the immutable snapshot. CAN identifiers and wire layouts do not change.

Impact is telemetry correctness and receiver interpretation. This reproduction does not establish a torque-authority bypass; the supervisor and SoP have independent gates. It also does not claim that queued frames are re-aged continuously after encoding.

## New finding 2 — Low / defensive contract fix: unknown uncertainty accepted as calibration evidence

**Fixed.** `AMS/Core/Src/sop/ams_power_state.c`, `build_sop_input()` and `build_soh_input()`.

Both adapters checked uncertainty for nonzero but not for `UINT16_MAX`, which means unknown. Given a confident calibration flag and a nonzero record ID, the baseline converts that sentinel to 65.535 A and sets `current_calibrated` true. CAN and estimator acquisition already reject this sentinel.

A focused test calls the production adapters with otherwise identical metadata and uncertainties 500, 0 and 65535 mA. The baseline fails the SoP assertion for the unknown value. Both adapters now accept 500 and reject zero/unknown as calibration evidence; the expanded test passes.

**Reachability limit:** normal DHAB calibration confidence already bounds its calibrated uncertainties. This review has not demonstrated a normal live publisher producing a confident record plus unknown uncertainty in a valid window. This is a reproduced adapter-contract defect, not a proven currently reachable unsafe power authorization. The fix is two explicit sentinel checks, with no model retuning.

## Other review results and scope

Inspected current-window arithmetic, metadata carry and publication pinning; current-task sampling/publication; ADBMS boundary locking; estimator consumption and cumulative-charge continuity; SoP/SoH adapters; CAN snapshot selection, encoding and scheduler generation/abort handling; fan control and IMD capture/read handling. This was a focused continuation review, not a claim of exhaustive verification of all source files.

Checks that did not become new findings:

- Whole-snapshot CAN expiry and estimator stopped-publisher expiry already exist.
- Snapshot readers pin buffers while copying; the writer checks pin ownership.
- Scheduler required delivery distinguishes completion from discard, preserves reserved frames, and handles stale critical/protected generations.
- Current-window stale-tail and out-of-order recovery have existing focused coverage.

The condensed changelog still advertised firmware 0.5.20 as the current source revision. Updated it alongside the canonical version header to 0.5.23. Package version and firmware version remain separate numbering schemes.

## Validation and limits

Command from `AMS/host_tests`:

```sh
make measurement-integrity-test
```

Result: PASS, nine named groups, compiled with project warning flags and `-Werror`. New checks cover per-reading expiry at and after the exact limit, remaining-fresh readings, aggregates, detail invalid encodings, usable masks, unknown ages, tick wrap, and known/zero/unknown current uncertainty through both real adapters. Existing current-window/CAN calibration checks also pass in that executable. Mutable-driver fallback stubs fail if accidentally reached by the new snapshot detail tests.

Pre-fix failures were observed first for CAN age validity, then for the unchanged SoP uncertainty adapter after the CAN fix. No full MiL, broad SIL, hardware reproduction, or target firmware build is claimed. `arm-none-eabi-gcc` is unavailable in this environment. No target binary is included.

Five-SMB temperature validation is not revoked or rerun. Current H-path hardware/calibration, remaining vehicle release evidence, and the existing watchdog/bus-off policy decisions remain as documented in the prior review. No Zephyr code, gate overrides, balancing authority, or BMS authority changes were made.

## Changed files

- `AMS/Core/Src/tasks/canbus_task.c`
- `AMS/Core/Src/sop/ams_power_state.c`
- `AMS/Core/Inc/ams_version.h`
- `AMS/host_tests/unit/measurement_integrity_test.c`
- `docs/CHANGELOG.md`
- This review and the included patch against v2.6.19.
