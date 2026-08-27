# DER26 Accumulator Management System Firmware

Detailed docs on notion [https://verdant-newt-bdb.notion.site/4870013cb18983eeae808152ffa32e9d?v=0330013cb189835c97e5881a239e0dac]([url](https://verdant-newt-bdb.notion.site/4870013cb18983eeae808152ffa32e9d?v=0330013cb189835c97e5881a239e0dac)))
Firmware, validation infrastructure, HIL support, and engineering tools for the DER26 Accumulator Management System (AMS).

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
