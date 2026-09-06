# DER26 Accumulator Management System Firmware

Firmware, validation infrastructure, HIL support, and engineering tools for the DER26 Accumulator Management System (AMS).

Current package: **v2.6.27**, AMS source **v0.5.30**. See the
[final software-freeze candidate](AMS/docs/PRE_ZEPHYR_FINAL_SOFTWARE_FREEZE_CANDIDATE_v2.6.27.md), the
[final repeated-bus-off closeout](AMS/docs/PRE_ZEPHYR_FINAL_BUSOFF_CLOSEOUT_v2.6.26.md), the
[pre-Zephyr freeze hardening](AMS/docs/PRE_ZEPHYR_FREEZE_HARDENING_v2.6.25.md), the
[final pre-Zephyr code fixes](AMS/docs/PRE_ZEPHYR_FINAL_CODE_FIXES_v2.6.24.md), the
[safety-policy closeout](AMS/docs/SAFETY_POLICY_CLOSEOUT_v2.6.23.md), the
[post-closeout regression fixes](AMS/docs/POST_CLOSEOUT_REVIEW_FIXES_v2.6.22.md),
and the [v2.6.21 migration-readiness code closeout](AMS/docs/OPEN_FINDINGS_CODE_CLOSEOUT_v2.6.21.md).
The v2.6.19-v2.6.22 measurement, CAN, build/provenance, and freshness fixes are
retained. v2.6.23 closes the watchdog and state-dependent CAN bus-off policies.
v2.6.24 fixes physical bus-off event timing, 499/500/501 ms on-wire authority
timing, service recovery state separation, and tuning-CAN producer feature gating.
v2.6.25 additionally requires transport-epoch settlement before a protected
completion can satisfy the discharge recovery deadline, unifies physical and bench
bus-off event handling, refreshes repeated identical CAN soft-error age, and makes
failed service recovery leave repeated-bus-off state intact. CAN authority is
credited only after the required protected `0x680`–`0x687` set completes on wire,
not when software merely queues it; battery models are not retuned.

v2.6.26 closes the final repeated-CAN-bus-off event-loss defect: CAN1 SCE now owns physical BOFF identity before HAL sticky error accumulation, every event is sequenced ISR-side, the exact three-event/10-second sliding window is maintained at the event boundary, and the third genuine event hard-latches TX/BMS low without waiting for the 10 Hz CAN task. Task-side recovery consumes sequence deltas, so clustered events and a later event during pending recovery cannot be collapsed.

v2.6.27 hardens the remaining ISR/task ownership edges: charger ENABLE is revalidated at the bxCAN hardware-load boundary, shared CAN/charger completion scalars have explicit ISR/task visibility, recovery settlement is tied to an unchanged BOFF sequence, and manual CAN recovery commits transport plus application safety state atomically so a new BOFF cannot be erased by post-return CLI cleanup. The focused CAN regression source now contains 19 cases. Per user request, the final v2.6.27 suites are intentionally left for the recipient to rerun before freezing the baseline.

The target application runs on an STM32F767 with FreeRTOS and interfaces with the accumulator monitoring chain, pack-current sensing, charger/IMD/AIR-related inputs, and the vehicle CAN network. The repository also contains host-side verification, HIL assets, and reference tools used to develop and qualify the firmware.

> **Safety note:** a successful build or host-test run does not authorize HV or vehicle operation. Hardware validation, calibration evidence, and build-profile gates remain part of the release process.

## Repository map

```text
AMS/                 STM32CubeIDE firmware project
  Core/              project-authored application code
  Drivers/           STM32 HAL/CMSIS vendor code
  Middlewares/       FreeRTOS/CMSIS-RTOS middleware
  docs/              firmware contracts and module documentation
  host_tests/        host unit/SIL/stress/static-contract tests
HiL/                 hardware-in-the-loop plant/support assets
Tools/               reference models, replay tools, and utilities
ci/                  repository and headless target-build scripts
.github/workflows/    continuous-integration workflow
docs/                repository-level architecture, safety, status, and navigation
```

Start with [`docs/README.md`](docs/README.md) for the documentation index or [`AMS/README.md`](AMS/README.md) for the firmware itself.

## Quick start

### Host validation

```bash
cd AMS/host_tests
make firmware-ci
```

For sanitizer/stress qualification:

```bash
make firmware-asan
make ubsan
make stress
```

See [`AMS/host_tests/README.md`](AMS/host_tests/README.md) and the test matrix under `AMS/host_tests/docs/` for focused targets.

### STM32 target project

Import `AMS/` into STM32CubeIDE. The repository keeps the `.ioc`, `.project`, `.cproject`, linker scripts, startup code, HAL, and FreeRTOS middleware required by the native project.

A headless ARM-GCC build is also provided under `ci/stm32/` for CI/reproducibility checks.

## Main firmware areas

- `AMS/Core/Src/tasks/` — RTOS task ownership and scheduling.
- `AMS/Core/Src/ext_drivers/` — ADBMS, CAN, current, IMD, AIR, fan, charger, CLI, and safety-facing services.
- `AMS/Core/Src/measurement/` — canonical measurement interface.
- `AMS/Core/Src/estimator/` — battery state estimation.
- `AMS/Core/Src/sop/` — State-of-Power, power strategy, and fuse observer.
- `AMS/Core/Src/soh/` — State-of-Health logic.

The historical `ext_drivers` name is retained because CubeIDE, tests, and CI reference the existing source paths. See [`docs/CODE_ORGANIZATION.md`](docs/CODE_ORGANIZATION.md) for a maintainer-oriented map.

## Current development status

The repository has been synchronized from the latest complete AMS source snapshot available for this cleanup and old generated/stale repository artifacts have been removed. Current open review items are tracked in [`docs/STATUS.md`](docs/STATUS.md); do not infer vehicle-release readiness from the repository version alone.

## Documentation policy

The active documentation describes the current architecture and supported workflows. One-off patch reports, obsolete release notes, old debug transcripts, and generated build outputs are intentionally not kept in the active tree. Git history or formal release archives should be used for forensic history.

## Attribution

Team and vendor attribution is summarized in [`docs/ATTRIBUTION.md`](docs/ATTRIBUTION.md). Vendor code retains its upstream licenses and notices.
