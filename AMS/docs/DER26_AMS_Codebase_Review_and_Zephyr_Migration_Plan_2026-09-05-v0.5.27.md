# DER26 AMS codebase review and Zephyr RTOS migration plan

**Review date:** 2026-09-04; migration-plan baseline updated 2026-09-05 after follow-up code review
**Reviewed software baseline:** DER26 AMS firmware `0.5.27` / package `v2.6.24`, incorporating the v2.6.17 CAN scheduler corrections, v2.6.19 current-window/measurement-integrity fixes, v2.6.20 follow-up fixes, v2.6.21 migration-readiness closeout, v2.6.22 post-closeout regression fixes, the v2.6.23 watchdog/CAN safety-policy closeout, and the v2.6.24 final pre-Zephyr event-time/telemetry/service-recovery fixes
**Target MCU:** STM32F767ZIT6, Cortex-M7, 216 MHz, 2 MiB flash, 512 KiB physical SRAM
**Current kernel:** STM32Cube-generated FreeRTOS/CMSIS-RTOS2 integration
**Candidate kernel:** Zephyr RTOS
**Document purpose:** static codebase review plus a detailed, behavior-preserving migration plan
**Explicit non-action:** this document update does not start the Zephyr port. The v2.6.24 FreeRTOS baseline keeps the v2.6.23 watchdog/CAN safety policy and closes the final code-review regressions in its event-time implementation, exact discharge recovery deadline handling, service-recovery state separation, and tuning-CAN feature configuration. It does not retune battery-model parameters or estimator equations, change CAN IDs/encoding, or change the approved SPI String-B PE4 mapping. No full MiL rerun, ARM target build, flash, physical CAN bus-off test, watchdog-reset timing test, or new hardware validation is claimed for v2.6.24.

## 1. Executive decision

The AMS can be migrated to Zephyr on the existing STM32F767 hardware, but it should not be treated as a kernel-call translation exercise. The current application contains mature battery algorithms, important fail-low mechanisms, hardware-specific timing, and a substantial verification harness. Those assets should be preserved. The risky work is reproducing the safety, concurrency, timing, startup, fault-recovery, and physical-I/O behavior under a new kernel and driver stack.

The recommended approach is:

1. Freeze and fully identify firmware 0.5.27 / package v2.6.24 as the current FreeRTOS comparison baseline.
2. Close the remaining hardware-validation, evidence-reconciliation, target-timing, and target-build-provenance gates; do not reopen issues already fixed and regression-covered in 0.5.27.
3. Introduce narrow platform interfaces while the FreeRTOS product remains functional.
4. Build a separate Zephyr **no-authority** image on a custom DER26 board definition.
5. Port one hardware boundary at a time, with differential replay and target measurements at every step.
6. Reach bench parity before any vehicle-authority build is possible.
7. Qualify a pinned long-term Zephyr baseline and retain the FreeRTOS image as the rollback release until vehicle evidence is complete.

The suggested version policy is to use **Zephyr 4.4.0 for the immediate prototype**, because it is the current stable release as of this review, but not to lock the production qualification baseline to it automatically: 4.4 reaches end of life in April 2027. Put an explicit checkpoint in the program to rebase onto **Zephyr 4.6 LTS** when it is released, if the schedule permits, before vehicle qualification. If the migration begins after 4.6 LTS is available, start directly on that release. Zephyr 3.7 LTS is a defensible conservative alternative, but it begins the migration on older APIs and tooling.

### Overall judgement

| Area | Judgement | Meaning |
|---|---|---|
| MCU and peripheral support | Feasible | Zephyr supports the STM32F767 family and the required general peripheral classes. |
| Algorithm portability | Strong | EKF, SoP, SoH, fuse, and most fault logic are already pure C with little or no RTOS/HAL coupling. |
| Safety architecture | Worth preserving | Compile-time authority profiles, sole-owner BMS assertion, direct fail-low path, watchdog, and retained fault logging are good foundations. |
| Driver portability | Moderate work | ADBMS uses HAL status types broadly and depends on exact SPI/GPIO/microsecond timing; IMD capture may need a small STM32-specific adapter. |
| Concurrency portability | Moderate-to-high risk | The measurement store is sound, but the root application object, broad critical sections, mixed CMSIS/native FreeRTOS calls, ISR contracts, and shared driver buffers require deliberate treatment. |
| CAN migration | Feasible with a spike | Application scheduling policy should be retained; the HAL mailbox implementation should be replaced by Zephyr CAN callbacks/state handling, not copied. |
| Physical readiness | Vehicle readiness not established by this review | Latest supplied H-channel measurements show a physical fault. Other release gates require evidence reconciliation; a default validation macro or old comment does not prove testing was never completed. |
| Migration authorization | Code-cleared for Phase 1 | Known code-fixable blockers and safety-policy ambiguities are closed in 0.5.27. A no-authority Zephyr skeleton may start; hardware/timing/evidence gates still block parity and vehicle authority. |

### Effort range

The planning estimate is **31.5–54 engineer-weeks** of serial work from baseline closure through vehicle release. With two experienced embedded engineers and dependable hardware/test support, the likely elapsed time is **18–28 weeks**, assuming hardware issues do not block progress. A no-authority Zephyr prototype with safe startup, basic I/O, current measurement, and initial ADBMS communications is approximately **8–13 engineer-weeks**. Functional bench parity is approximately **22–36 engineer-weeks**.

These are planning ranges, not commitments. The principal schedule variable is hardware availability and validation—not C syntax conversion.

**Estimate scope:** these ranges include preparatory refactoring, new test infrastructure, hardware qualification, and vehicle release—not just the RTOS port. They are not a measured minimum. Reuse existing accepted evidence and subtract already-completed work after Phase 0 reconciliation. The two-engineer elapsed estimate assumes some independent driver/test work overlaps; hardware waits are outside the engineer-week total.

### Is migration justified?

Zephyr adoption is optional. The current application does not require it to fix the known CAN defects or to run the battery algorithms. Remaining on FreeRTOS has the lowest immediate integration risk. Zephyr offers a maintained driver/build ecosystem, first-class board/configuration descriptions, standardized diagnostics, and a stronger path for future upstream driver work. Those are maintenance and development benefits, not automatic improvements in safety or numerical accuracy.

Use the smallest migration scope that achieves those benefits: retain the existing task/state model and algorithm source initially, replace the RTOS/platform boundaries, and measure parity. The subsystem-state decomposition and owner-thread request interfaces described later are recommendations, not prerequisites for a first working port. Do not create abstractions for their own sake or redesign working modules simply because Zephyr is available.

## 2. Scope, method, and confidence

### 2.1 Reviewed material

The review covered:

- all authored AMS `Core/Src` and `Core/Inc` modules;
- task creation, scheduling, priorities, stacks, mutexes, critical sections, ISR use, and watchdog behavior;
- measurement publication, estimator consumption, fault aggregation, BMS_OK ownership, and panic logging;
- ADBMS6830, ADBMS2950/APM, current sensor, CAN, fan, IMD, AIR-sense, UART/CLI, and board adapters;
- EKF, SoP, SoH, fuse, voltage/current/temperature fault, and observer modules;
- build profiles and authority gates;
- STM32Cube `.ioc`, generated initialization, linker scripts, host-test infrastructure, build scripts, and engineering documentation;
- the supplied AMS Rev 3.1, MCU-breakout, APM, and related vehicle schematics/BOMs where they define AMS interfaces;
- official Zephyr release, board, driver, kernel, testing, logging, retention, and build documentation current at the review date.

This is a whole-codebase architectural and static review, with deeper tracing of safety-, concurrency-, timing-, and migration-critical paths. It is not a formal line-by-line proof, functional-safety certification, compiler qualification exercise, or substitute for on-target testing.

### 2.2 Evidence labels

Findings use four confidence labels:

- **Confirmed source defect:** directly reproducible from the reviewed source/configuration.
- **Confirmed inconsistency:** two authoritative-looking artifacts disagree; the operational choice may still be correct.
- **Policy decision required:** implementation is coherent, but the intended safety behavior is not sufficiently specified.
- **Unmeasured risk:** no source defect is claimed; target evidence is required before acceptance.

### 2.3 Actions deliberately not taken

- No Zephyr repository, west workspace, board definition, Devicetree, or Kconfig was created.
- This plan update incorporates the v2.6.21 migration-readiness closeout, the v2.6.22 post-closeout fixes described in Section 5.2a, the v2.6.23 policy closeout described in Section 5.2b, and the v2.6.24 final code fixes described in Section 5.2c. v2.6.24 preserves the agreed watchdog/CAN policy while making the CAN decision points event-time deterministic and restoring the intended tuning-CAN producer configuration.
- No full MiL campaign was rerun. v2.6.24 changes supervisory/CAN event-time bookkeeping, service-recovery state, and a telemetry compile gate, not the estimator/SoP/SoH numerical models or calibration parameters; focused host safety/CAN/HIL/tuning regressions cover the changed behavior.
- No ARM target binary was built or flashed for v2.6.24 because `arm-none-eabi-gcc` is not available in the review environment.
- No physical CAN bus-off injection, IWDG reset-timing run, or new hardware reproduction is claimed for v2.6.24.
- Host validation passed the self-contained `firmware-ci` target, comprehensive injection and IWDG-enabled safety suites, CAN scheduler/transport regressions, CAN-backed HIL replacement test, profile gates, measurement-integrity tests, all 15 review-fix gates, release/contract gates, and whole-source GCC analysis in bench and vehicle profiles with `-Wundef -Werror=undef`. Focused Clang analysis of the changed CAN/safety/estimator sources was also clean.
- No claim is made that the present source is vehicle-qualified.

### 2.4 Evidence reconciliation

The user has reported temperature scanning enabled and validated across all five SMBs. This report does not revoke that result. Historical source comments and default-false vehicle-validation macros show requirements and default configuration, not the complete bench history. Phase 0 should link existing temperature evidence to the exact board, firmware, scan mode, and validation gate; only genuinely missing portions should require additional testing. The same rule applies to all prior accepted tests. The H-channel fault is based on the latest supplied ADC/TP4 observations, not a new physical measurement during this review. Absence of AIR auxiliary contacts is a capability limitation, not automatically a release blocker unless the approved safety requirements demand that feedback.

### 2.5 Baseline corrections incorporated before migration

The current migration baseline is no longer the original v0.5.20 review snapshot. The following FreeRTOS-side defects were corrected before the Zephyr port begins and are now **behavioral parity requirements**, not migration tasks to redesign:

1. **Current-window boundary serialization.** ADBMS now acquires the current-window mutex before capturing the voltage boundary timestamp, and current publication uses the same serialized ordering. A sample that crosses a voltage boundary must not be integrated into the wrong window or cause a later window to be marked valid with biased charge/current.
2. **Carried current metadata preservation.** Opening the next current window preserves the carried sample's uncertainty, range state, extrema, and calibration provenance because that sample still participates in the next integration interval.
3. **Mixed-range latching.** Current-range state distinguishes uninitialized from mixed/unknown; once a window observes mixed ranges it cannot silently return to a single-range label before rotation.
4. **CAN current calibration provenance.** ECU current diagnostic quality is derived from the same measurement snapshot's calibration record, calibration ID, and known nonzero uncertainty. Electrical plausibility alone does not justify the "calibrated primary" quality code.
5. **CAN per-reading aging.** Freshness of the enclosing snapshot is insufficient. Individual cell and temperature ages are advanced to CAN encode time; expired readings are removed from valid aggregates, usable masks, and ECU/logger detail values while still-fresh readings remain available.
6. **SoP/SoH unknown-uncertainty rejection.** `UINT16_MAX` remains the unknown-current-uncertainty sentinel and cannot be accepted as calibration evidence by SoP or SoH adapters; zero uncertainty is also not accepted as calibration evidence.

The v2.6.20 follow-up regression covers exact age-limit behavior, post-limit expiry, remaining-fresh readings, aggregate/mask/detail consistency, unknown ages, tick wrap, and known/zero/unknown uncertainty through the production adapters. These tests should move with the common core and be run against both FreeRTOS and Zephyr builds.

## 3. Current AMS baseline

### 3.1 Size and composition

The updated authored AMS `Core` remains approximately **59.2k lines** across **49 C files and 53 headers**. The v2.6.19/v2.6.20 measurement fixes, v2.6.21 migration-readiness guards, and v2.6.22 freshness/scheduling/build fixes change only a small number of production lines, so the planning-scale size is unchanged. The complete project remains approximately **114.6k lines** of application code, tests, documentation tools, and support code excluding bundled vendor/middleware/build output.

Largest or most consequential implementation units include:

| Module | Approximate size | Review significance |
|---|---:|---|
| `ext_drivers/adbms6830.c` | 8,100 lines | Primary cell/temperature monitor protocol and diagnostics; largest hardware-coupled unit. |
| `tasks/cli_task.c` | 7,718 lines | Extensive bring-up/service shell; blocking UART output and substantial maintenance/attack surface. |
| `ext_drivers/adbms2950.c` | 3,096 lines | APM transport and measurements; currently advisory. |
| `tasks/canbus_task.c` | 2,991 lines | Application CAN publication, charger handling, detail/tuning generation, and deadlines. |
| `ext_drivers/canbus.c` | 2,119 lines | bxCAN filters, ISR queues, TX completion, errors, and recovery. |
| `ext_drivers/accumulator.c` | 2,098 lines | ADBMS chain topology, startup, transport ownership, and measurements. |
| `estimator/ams_soc_ekf.c` | 2,091 lines | Production SoC/R0 estimator; largely portable pure C. |
| `tasks/adbms_task.c` | 1,806 lines | Periodic acquisition, balancing recovery, fault/state publication. |
| `sop/ams_sop.c` | 1,327 lines | State-of-power constraints and authority computation. |
| `tasks/estimator_task.c` | 1,173 lines | Measurement-to-estimator integration and publication. |
| `ext_drivers/ams_safety.c` | 1,096 lines | panic, fail-low GPIO, retained fault log, watchdog, and diagnostics. |
| `ext_drivers/air_monitor.c` | 1,018 lines | Future physical contactor-feedback monitor; not active on current hardware. |

The size distribution is relevant: the migration should extract stable platform boundaries rather than rewrite large drivers and algorithms simultaneously.

### 3.2 Current task model

FreeRTOS uses a 1 kHz tick, preemption, 56 priority levels, static and dynamic allocation, stack-overflow checking, newlib per-task reentrancy, and a 32 KiB heap (`Core/Inc/FreeRTOSConfig.h:55-95`). Application tasks themselves are created statically.

| Current task | Nominal rate | FreeRTOS priority | Configured stack | Role |
|---|---:|---:|---:|---|
| Error/safety supervisor | 20 Hz | 17 | 256 words / 1 KiB | Aggregates safety state, owns normal BMS_OK assertion, updates watchdog. |
| Current | 50 Hz | 12 | 256 words / 1 KiB | Samples dual-range DHAB channels, validates/selects current, manages zero calibration. |
| ADBMS | 10 Hz normal; 1 Hz bring-up | 11 | 1,536 words / 6 KiB | Cell/temp acquisition, diagnostics, balancing, APM interaction, coherent publication. |
| CAN | 10 Hz scheduler; 2 Hz detail | 10 | 1,536 words / 6 KiB | Charger commands, protected status/power frames, diagnostics, base detail and tuning. |
| Estimator | 10 Hz | 8 | 1,536 words / 6 KiB | EKF, SoP, SoH, fuse and derived-state publication. |
| Fan | 5 Hz | 7 | 192 words / 768 B | Open-loop fan PWM command. |
| AIR monitor | 2 Hz legacy/future | 7 | 192 words / 768 B | Disabled unless true auxiliary feedback is configured. |
| IMD | 10 Hz | 6 | 192 words / 768 B | Interprets TIM2 frequency/duty capture. |
| CLI | 20 Hz | 4 | 512 words / 2 KiB | UART command shell and bench evidence. |

Static application stacks total approximately **24.8 KiB** with all optional tasks, **23.3 KiB** in the common bench configuration, or **24.1 KiB** for a likely vehicle configuration. This excludes TCBs, idle/timer stacks, newlib state, globals, heap, interrupt stack, and library use. The monolithic global `app_data_t` is also material; a host-linked image reports 50,616 bytes for `app`, which is an indication only because the target ABI/layout may differ.

### 3.3 Hardware/peripheral contract

| Function | Current STM32 mapping/configuration | Migration constraint |
|---|---|---|
| BMS_OK | PE0, active-high output; direct-register fail-low path | Must be low before ordinary drivers/kernel are trusted and from fatal context. |
| Current H / ±800 A | PA3, ADC1_IN3, 480-cycle sample | Preserve sample time and scaling; current physical path is presently reading 0 V and must be repaired. |
| Current L / ±50 A | PC0, ADC2_IN10, 480-cycle sample | Preserve dual-range acquisition and validation behavior. |
| CAN1 | PD0 RX, PD1 TX, 1 Mbit/s | Preserve exact nominal timing: prescaler 3, SJW 2 TQ, BS1 15 TQ, BS2 2 TQ, approximately 88.9% sample point. |
| CAN transceiver | SN65HVD230QDR on MCU breakout, 3.3 V, board termination | Confirm actual termination/topology on vehicle harness. |
| ADBMS SPI | SPI6 mode 3; PG13 SCK, PG14 SDO/MOSI, PG12 SDI/MISO; /256 from 108 MHz = 421.875 kbit/s | Preserve CS/wakeup/microsecond timing and initially retain conservative clock. |
| ADBMS CS A | PE2, active-low | Explicit Devicetree GPIO plus adapter-owned sequencing. |
| ADBMS CS B | PE4 in firmware and `.ioc` | PE4 is the frozen board contract. The PF4 schematic annotation was a labeling error and is not a migration blocker. |
| CLI | USART3, PD8/PD9, 115200 8N1 | Bench/HIL/test-day diagnostic shell. Vehicle profile sets `AMS_ENABLE_CLI=0`; preserve that no-shell default. |
| Microsecond timebase | TIM1 | Replace with a proven monotonic microsecond implementation, counter, or cycle-based adapter. |
| IMD capture | TIM2 dual-edge/reset-mode capture | Prototype early; generic counter API may not directly express the exact topology. |
| Fans | TIM3/TIM4/TIM5, six PWM channels, about 32.13 kHz | Preserve frequency, active polarity, safe startup state, and duty mapping. |
| Watchdog | STM32 IWDG, vehicle profile only | Preserve reset timing and settle the feed-policy question before porting. |
| System clock | HSE 8 MHz, PLL to 216 MHz | Stock Zephyr Nucleo board defaults to 72 MHz; custom board must set and verify 216 MHz deliberately. |

The AMS Rev 3.1 hardware routes two ADBMS6822 isoSPI interfaces to the SMB/APM ring, a LEM DHAB S/155 dual-range current sensor, IMD input, TSAL-related interfaces, six fan outputs, charger and vehicle CAN, and BMS_OK. The MCU breakout contains the STM32F767 and CAN transceiver. AIR_CONTROL is a coil-command voltage observation, not physical AIR/precharge auxiliary feedback; the software correctly avoids treating it as proof of contactor position by default.

### 3.4 Build profiles and authority

The build-profile system is one of the strongest parts of the current design. BENCH, HIL, TESTDAY, BENCH_VALIDATION, and VEHICLE are distinct compile-time intents. TESTDAY and BENCH_VALIDATION are compiled with immutable no-authority barriers. VEHICLE cannot compile until explicit physical and contract-validation macros are supplied (`Core/Inc/ams_build_profile.h:444-555`).

Vehicle gates cover, among other items:

- IMD and IWDG target validation;
- current polarity and calibration;
- balancing current/time/thermal evidence;
- full vehicle CAN timing/staleness behavior;
- SoP model/calibration/CAN contracts;
- fuse calibration and low-current extrapolation;
- cell open-wire and temperature pull-up validation;
- temperature thresholds;
- immutable source and model/contract revision identifiers.

These concepts must survive the Zephyr migration. They can move to Kconfig fragments and C compile-time assertions, but they must not become mutable runtime settings.

## 4. Architecture assessment

### 4.1 What should be preserved

1. **Fail-low startup and panic behavior.** `main.c:109-115` forces BMS_OK low immediately after HAL reset initialization, before normal clock/peripheral initialization. `ams_safety.c` also contains a direct-register GPIO fallback and a retained panic/fault record.
2. **Sole normal assertion owner.** `set_bms()` accepts assertion only from the error/safety task and converts unauthorized assertions to fail-low behavior (`app.c:832-896`). Other contexts may deassert.
3. **Compile-time no-authority profiles.** TESTDAY and BENCH_VALIDATION cannot gain BMS or balancing authority through a command. Do not generalize this to every existing BENCH/HIL configuration: the final source macros explicitly protect those two profiles, while other profiles require their own policy review.
4. **Coherent measurement publication and current-window integrity.** The ADBMS measurement store uses double buffers, reader pinning, sequence identifiers, and copy-outside-lock semantics. The estimator consumes each sequence once. The v2.6.19 fixes also make current-window boundary ordering, carried uncertainty/range/calibration metadata, and mixed-range latching part of this contract.
5. **Fixed-storage CAN scheduler and measurement-quality semantics.** The corrected v2.6.17 scheduler separates critical, protected-required, protected-advisory, and detail traffic, with generation/freshness tracking rather than an unbounded FIFO. The v2.6.19/v2.6.20 baseline additionally requires same-snapshot calibration provenance and per-reading cell/temperature aging at encode time.
6. **Pure computational cores.** EKF, SoP, SoH, fuse, fault thresholds, and most observers can remain common source compiled under both systems.
7. **Retained fault log semantics.** The no-init ring uses CRC and commit-last behavior, which is appropriate for abrupt reset/fault evidence.
8. **Bounded waits and fail-low timeout handling.** ADBMS/current locks have bounded waits; failures flow toward inhibition/panic rather than indefinite blocking.
9. **Host verification assets.** The repository already has unit, contract, randomized, metamorphic, sanitizer, profile, and MiL tooling. A migration should add cross-platform evidence, not discard these tests.

### 4.2 Structural liabilities to address before or during migration

1. `app_data_t` is simultaneously a composition root, cross-subsystem state database, diagnostics record, and supervisor input. This causes broad ownership and many critical sections.
2. The code mixes CMSIS-RTOS2 APIs, native FreeRTOS APIs, STM32 HAL, and direct peripheral/register access. There is no single portability boundary.
3. ADBMS drivers expose `HAL_StatusTypeDef` through large portions of their internal logic even though only a small number of calls actually touch SPI/GPIO/timers.
4. ADBMS6830 and ADBMS2950 use module-global transmit/receive buffers and rely on serialization through one recursive operation mutex. This is non-reentrant hidden state.
5. The safety task makes a coherent BMS decision inside a broad scheduler critical section. The intent is valid—prevent stale reassertion after an immediate deassert—but the duration is unmeasured.
6. Several state publications copy many fields in critical sections. Source length does not prove a timing violation, but target interrupt-off duration must be measured.
7. Vehicle CLI policy is now explicit: `AMS_ENABLE_CLI=0` in the vehicle profile prevents UART3 initialization, RX arming/callback processing, CLI board initialization, and CLI task creation. `AMS_ENABLE_SERVICE_CLI` remains a separate mutation gate and cannot be enabled when the diagnostic CLI transport is compiled out.
8. Dynamic allocation and a 32 KiB FreeRTOS heap are enabled although authored application tasks/mutexes are static and no direct application `malloc`/`pvPortMalloc` use was found.
9. The qualified headless **Release** build now explicitly selects an AMS authority profile (default profile 5 / BENCH_VALIDATION to match the checked-in CubeIDE configurations), uses the same `-Os -g0` Release optimization, refuses to run without `SOURCE_DATE_EPOCH`, records profile/mode/toolchain/dependency/linker/CubeMX provenance plus an immutable authored-source-input-tree SHA-256, and emits artifact hashes. Uncontrolled IDE/Debug builds remain outside the release-reproducibility path.
10. The generated default Cube task remains as disabled scaffold. CubeMX project identity is corrected to DER26 and is source-gated; the remaining scaffold is cleanup debt, not an identity contradiction.

## 5. Findings register

### 5.1 Open findings that should be closed before the Zephyr branch becomes authoritative

| ID | Severity | Confidence | Finding | Required disposition |
|---|---|---|---|---|
| F-07 | High | Unmeasured risk | Critical-section/IRQ-off duration and task WCET/jitter are not proven on target. | Instrument DWT cycle counts, release jitter, and interrupt masking under normal and injected-fault workloads. |
| F-08 | High | Hardware blocker | The H/±800 A current path currently reads zero at TP4/ADC, while L is healthy. | Repair/diagnose the physical path and validate signed calibration before ADC equivalence or vehicle current authority is claimed. |
| F-09 | High where authority depends on it | Evidence reconciliation / hardware limitations | Source retains S-path/pull-up warnings and physical validation gates; the user reports five-SMB temperature validation completed. Balance/IMD/IWDG release evidence was not established here. AIR auxiliary feedback is absent. | Reconcile accepted bench evidence before declaring gates open. Treat absent auxiliary feedback as a limitation unless a requirement makes it mandatory. Track genuine remaining hardware work separately. |
| F-11 | Moderate | Confirmed source/configuration debt | Static app objects coexist with enabled heap, software timer task, deletion APIs, and newlib reentrancy; no authored dynamic allocation was found. | Keep this as a Zephyr vehicle-configuration requirement: allocation-free after initialization, preferably heap-free entirely, with a configuration/build proof. Do not rewrite the validated FreeRTOS baseline solely to satisfy the future-kernel policy. |
| F-12 | Moderate | Partially closed provenance debt | The exact bundled FreeRTOS source is now pinned by SHA-256 and the `task.h` V10.2.0/source-banner V10.2.1 mismatch is recorded rather than rewritten. The remaining gap is target artifact/toolchain evidence: no current ARM Debug/Release ELF/MAP/HEX/BIN was produced in this review environment. | Run the headless Debug and deterministic Release builds on the approved ARM toolchain, archive `BUILD_PROVENANCE.txt`, artifact hashes, ELF/MAP/HEX/BIN and stack-usage output, and record the toolchain version. |
| F-14 | Moderate | Unmeasured risk | Linker scripts present SRAM as one 512 KiB region despite STM32F7 bank/bus differences. Zephyr's stock Nucleo target exposes 384 KiB. | Use the conservative 384 KiB region initially; place DTCM deliberately only after DMA/cache/linker behavior is tested. |

### 5.2 Code-review findings closed in v2.6.21

| Closed item | Status in firmware 0.5.24 / package v2.6.21 | Migration requirement |
|---|---|---|
| F-02 CubeMX DER25 identity | **Closed.** `DER26-AMS.ioc` already names `DER26-AMS` for both project and project file, and the release-identity gate rejects regression. The earlier plan entry was stale relative to the supplied source. | Preserve the exact board/project identity when creating the Zephyr board and Devicetree. |
| F-03 SPI CS_B PE4/PF4 inconsistency | **Closed as a documentation-only schematic label error.** Firmware pin mapping remains PE4; no SPI pin remap was made. The release-identity gate now also rejects PE4/CS_B drift between `main.h` and the `.ioc`. | Freeze PE4 in the board contract/Devicetree and do not carry the mislabeled PF4 annotation into Zephyr. |
| F-04 CAN ISR concurrency documentation | **Closed.** `CONCURRENCY_OWNERSHIP.md` states that CAN RX uses FreeRTOS ISR APIs, and `check_can_irq_contract.py` ties CAN RX0/TX/SCE priorities to the configured FreeRTOS syscall ceiling and CubeMX values. | Recreate the ISR-callability/priority contract explicitly under Zephyr. |
| F-10 vehicle CLI/UART surface | **Closed.** New `AMS_ENABLE_CLI` is enabled for bench/HIL/test-day images and is compile-time zero for vehicle. Vehicle startup no longer initializes UART3, arms CLI RX, initializes the CLI board endpoint, starts the CLI task, or executes CLI UART callbacks. Service mutation cannot be enabled without the diagnostic CLI transport. | Keep the no-shell vehicle default unless a later product requirement explicitly reintroduces one. |
| F-13 wall-clock Release metadata | **Closed and strengthened in v2.6.22 for the qualified headless Release path.** Release requires `SOURCE_DATE_EPOCH`, explicitly selects/records the AMS profile, matches CubeIDE Release optimization, records the authored source-input-tree hash plus toolchain/FreeRTOS/linker/CubeMX/build-configuration provenance, and emits artifact SHA-256 hashes. | Use the deterministic Release path for the FreeRTOS baseline and apply the same controlled-build/profile principle to Zephyr. |
| FreeRTOS provenance ambiguity portion of F-12 | **Closed in source.** The exact bundled FreeRTOS tree is pinned by SHA-256; the conflicting upstream labels are retained and documented instead of being falsified. | Preserve exact dependency revision/hash provenance when selecting the Zephyr baseline. |

### 5.2a Post-closeout defects fixed in v2.6.22

| Closed item | Status in firmware 0.5.25 / package v2.6.22 | Migration requirement |
|---|---|---|
| Headless Release/profile mismatch | **Fixed.** The script no longer relies on the header fallback to BENCH. It explicitly validates/selects `AMS_BUILD_PROFILE`, defaults to profile 5 to match checked-in CubeIDE configurations, passes the single-SMB validation selector explicitly, and uses `-Os -g0` for Release. Provenance records the profile and immutable authored source-input-tree hash. | Zephyr qualification builds must make authority/profile selection explicit and artifact-visible; a build-type name alone must never imply safety authority. |
| AUX2 missed-deadline replay burst | **Fixed.** The ADBMS AUX2 redundancy cadence has an explicit initialized epoch, re-arms after transport/topology loss, and advances to the first future deadline instead of replaying every missed slot. | Preserve non-backlogging diagnostic scheduling across boot, suspend, bus loss, and recovery. Missed diagnostic periods are skipped, not burst-replayed. |
| SoP/SoH constituent freshness omission | **Fixed.** Stored cell and temperature ages are advanced to solve time with wrap-safe/unknown propagation. Cell freshness retains the 250 ms bound; temperature has a separate 1000 ms bound to accommodate the normal ~0.8 s eight-position healthy thermistor scan while still failing closed when thermal data ages out. | Carry constituent acquisition age/timestamp semantics through the Zephyr measurement view. Snapshot publication freshness alone is insufficient for power/health authority. |
| Negative profile-gate false-pass hole | **Fixed.** Three expected-failure authority tests now define the expected profile and assert the exact intended compiler diagnostic after failure. | Preserve purpose-specific build-policy tests; unrelated compile failure must not satisfy an authority/profile gate. |

Focused validation after these fixes passed `measurement-integrity-test`, `profile-gates`, `review-fixes-test` (15/15), `adbms-v05-contract-gate`, `whole-source-analyze`, `power-core`, and `unit`. No ARM target build or new physical validation is claimed.

### 5.2b Safety-policy findings closed in v2.6.23

| Closed item | Status in firmware 0.5.26 / package v2.6.23 | Migration requirement |
|---|---|---|
| F-05 IWDG scope | **Closed.** IWDG is explicitly a software-liveness/integrity watchdog. External/process faults continue to fail BMS_OK low but do not stop watchdog feeding while critical software heartbeats remain healthy. Panic, heartbeat/safety-heartbeat loss, RTOS integrity faults/critical stack margin, explicit stop-feed injection, and watchdog-start failure remain reset-worthy. | Preserve the distinction under Zephyr: battery/process faults fail low without creating reset loops; task/kernel integrity failure stops watchdog service. |
| F-06 CAN bus-off policy | **Closed.** START requires a fresh required protected `0x680`–`0x687` generation to complete on the wire before CAN authority is established; software queue/commit success is insufficient. CHARGE, BALANCE, and HIL where CAN replaces ADBMS fail low immediately on bus-off. DISCHARGE receives one continuous recovery epoch: ECU torque authority must expire by 300 ms without a fresh changing `0x680`; AMS hard-fails BMS_OK at 500 ms if a fresh required protected generation has not completed on the wire. Three bus-offs in 10 s or the application TX inhibit latch hard-latch immediately. ABOM/manual recovery alone never grants authority. | Preserve these exact state/timing/latch semantics in the Zephyr CAN state adapter and differential replay; verify the 300 ms ECU side and 500 ms AMS side physically before authority qualification. |

Focused regression covers the exact 500 ms boundary and tick wrap, on-wire required-generation authority establishment, software-acceptance rejection, late pre-recovery completion isolation, ABOM-only recovery, service recovery, repeated bus-off, charge/balance behavior, CAN-backed HIL loss, external-fault watchdog feeding, heartbeat watchdog block, and RTOS-integrity watchdog block. No target CAN/IWDG timing evidence is claimed.

### 5.2c Final pre-Zephyr code-review regressions fixed in v2.6.24

| Closed item | Status in firmware 0.5.27 / package v2.6.24 | Migration requirement |
|---|---|---|
| CAN bus-off event-time semantics | **Fixed.** The first physical BOFF ISR now captures the event tick and AMS state. CHARGE, BALANCE, and CAN-backed HIL perform the ISR-safe physical BMS_OK fail-low immediately. The later 10 Hz task consumes the captured event for counters, diagnostics, recovery, and latching instead of redefining the event at poll time. | Zephyr CAN state callbacks must preserve the hardware-event timestamp/state and must not shift safety deadlines to a lower-rate worker thread. Immediate fail-low states must use an ISR/callback-safe direct path. |
| DISCHARGE 500 ms completion/order race | **Fixed.** The CAN scheduler now records the actual tick at which the required protected `0x680`–`0x687` generation completes on wire. Recovery succeeds only when a fresh post-bus-off generation completed **strictly before** 500 ms. Exact 500 ms and later fail. The supervisor and CAN task use the same event-time evidence, so 499/500/501 ms outcomes are independent of task ordering and are wrap-safe. | Preserve event-time, not observation-time, semantics in Zephyr. Differential tests must cover 499/500/501 ms, task-order reversal, stale pre-recovery completions, and tick wrap. |
| Service `can recover` state alias | **Fixed.** A service request waiting for fresh on-wire authority proof is represented by a separate authority-refresh state and no longer masquerades as an unresolved physical bus-off epoch. CHARGE/BALANCE therefore cannot immediately re-latch merely because service recovery is waiting for a fresh generation. | Keep physical fault epochs separate from service/administrative authority-refresh states. A successful driver recovery action alone must never grant authority. |
| Tuning-CAN producer feature gate | **Fixed.** `AMS_ENABLE_TUNING_CAN` is defined centrally in common application configuration, so estimator producer and CAN consumer compile from the same feature decision. The 250 kbit/s prohibition remains compile-time enforced. Authored-source analysis now uses `-Wundef -Werror=undef`, and a runtime host test proves the estimator publishes the tuning snapshot at supported default bitrates. | Keep telemetry feature gates single-source and compile-visible to all producers/consumers. Preserve undefined-macro-as-error analysis in the Zephyr application build. |

Focused regression now covers physical ISR timestamp/state capture, immediate CHARGE/BALANCE fail-low before task polling, delayed-poll invariance, exact 499/500/501 ms on-wire completion semantics, tick wrap, service authority refresh, and the estimator tuning producer. `firmware-ci`, `unit`, comprehensive safety/injection tests, CAN scheduler/transport tests, profile and measurement-integrity gates, all review-fix gates, and whole-source GCC analysis pass. Focused Clang analysis of every changed production CAN/safety/estimator file is clean. No ARM target build, flash, physical bus-off injection, or target timing evidence is claimed.

### 5.3 Closed baseline defects that become migration parity requirements

| Closed item | Baseline status | Migration requirement |
|---|---|---|
| Former F-01 runtime/source version mismatch | **Closed before v2.6.19.** `app.h` aliases `VER_*` to the canonical `ams_version.h`; the behavior was introduced by firmware 0.5.23 and is preserved in the 0.5.27 comparison baseline. | Preserve a single canonical version/build-manifest path and verify shell/CAN/retained/artifact identity in Zephyr. |
| Current-window timestamp race | **Fixed in v2.6.19.** Boundary timestamp acquisition and current publication are serialized through the current-window mutex. | Preserve ordering across the voltage boundary; differential tests must include a higher-priority current publication crossing the boundary. |
| Carried current uncertainty/range loss | **Fixed in v2.6.19.** Carried sample metadata persists into the next integration window. | Zephyr window rotation must carry uncertainty, range state, extrema, and calibration provenance with the carried current value. |
| Mixed range becoming single-range again | **Fixed in v2.6.19.** Initialization and mixed states are distinct. | Once mixed during a window, remain mixed until rotation. |
| CAN 0x68B calibration overclaim | **Fixed in v2.6.19.** Quality uses calibration provenance from the same current snapshot. | No telemetry layer may infer calibrated quality from `valid` alone. |
| CAN individual cell/temperature stale-data leak | **Fixed in v2.6.20.** Encode-time aging removes expired readings from aggregate values, masks, and detail output. | Re-age every individual reading against the correct acquisition/publication timestamp; snapshot freshness alone is insufficient. |
| SoP/SoH constituent-age freshness | **Fixed in v2.6.22.** Effective cell/temperature ages are advanced to solve time, unknown/future references fail stale, and temperature uses an explicit slower bound compatible with the healthy multiplexed scan cadence. | Preserve the reference timestamp associated with stored ages and compose it with elapsed solve time; use separate freshness policy where sensor acquisition cadence differs materially. |
| SoP/SoH unknown-uncertainty calibration acceptance | **Fixed defensively in v2.6.20.** Zero and `UINT16_MAX` are rejected as calibration evidence. | Common-core adapters must preserve the sentinel contract under both kernels. |

### 5.4 Existing CAN scheduler corrections

The v2.6.17 CAN scheduler work addressed six confirmed problems in the preceding baseline:

- a detail-generation local object larger than the CAN task stack;
- mailbox completion/reuse bookkeeping race;
- recovery logic that trusted HAL software state rather than hardware bus-off state;
- incomparable normal/shutdown charger generation counters;
- required-frame completion based on array position rather than frame class;
- tuning replacement of a pending base detail snapshot.

The fixed design is suitable to carry forward at the **application scheduler** layer. Target timing, real bxCAN callback ordering, bus-off recovery, and bus load still need hardware evidence; host regression success is not target qualification.

## 6. Module-by-module review

### 6.1 Startup, board, and generated platform code

Relevant files:

- `Core/Src/main.c`
- `Core/Src/board.c`
- `Core/Src/stm32f767z.c`
- `Core/Src/stm32f7xx_hal_msp.c`
- `Core/Src/stm32f7xx_it.c`
- `Core/Inc/main.h`
- `DER26-AMS.ioc`
- `STM32F767ZITX_FLASH.ld`

Strengths:

- Startup enters a fail-low condition before normal peripheral initialization.
- Board construction keeps most handle-to-driver mapping in one place.
- Current ADC channels, ADBMS SPI, CAN timing, UART, timers, and BMS_OK mappings are explicit and compile-time visible.
- Scheduler-return and task-create failures enter the panic path.

Concerns:

- The generated application layer and hand-authored layer are interleaved. Cube regeneration remains a source-drift risk.
- The historical STM32F407 file-name provenance comment in `stm32f767z.c` was already corrected in the supplied source; keep the source-identity gate because generated/manual platform files can drift again.
- The disabled generated `defaultTask` remains declared/scaffolded, so the source visually suggests two task-construction models.
- The `.ioc` identity is now DER26 and the PE4 String-B chip-select contract is machine-checked by the release-identity gate. The prior PF4 schematic label is not a firmware exception.
- The linker treats the MCU's SRAM as homogeneous. That is acceptable only while all users are CPU-only and placement is known; future DMA can make placement safety-relevant.

Migration treatment:

- Replace Cube-generated initialization with a reviewed custom Zephyr board and Devicetree overlay.
- Keep a small `ams_board_contract.h` containing safety-relevant electrical semantics that Devicetree alone cannot express: active level, permitted authority profiles, expected CAN timing, ADBMS wake timing, and safe output state.
- Generate a pin/peripheral contract report in CI from Devicetree and compare it to an approved manifest.
- Preserve a minimal STM32-specific early/fatal module for BMS_OK, reset-cause capture, retained memory, and any IMD timer mode not expressible through generic APIs.

### 6.2 Application composition and shared state

Relevant files:

- `Core/Src/app.c`
- `Core/Inc/app.h`

`app_data_t` is an understandable product of incremental development: it makes bench inspection and CLI diagnostics easy, but it is too broad to become the Zephyr architecture. It contains driver objects, task handles, sensor states, fault states, CAN diagnostics, estimator outputs, calibration information, heartbeats, build-profile state, and supervisor inputs. Some fields are authoritative, while others are flattened copies for display or publication.

Risks:

- ownership is implicit rather than type-enforced;
- writers and readers depend on critical-section discipline spread across modules;
- coherent measurement-store semantics do not automatically make every supervisor input coherent;
- a migration could accidentally change which copy is considered authoritative;
- the large root object makes unit isolation and stack/global budgeting harder.

Recommended decomposition, performed incrementally while preserving behavior:

| New domain object | Sole writer | Readers | Publication method |
|---|---|---|---|
| `ams_measurement_snapshot` | ADBMS/current integration publisher | safety, estimator, CAN | Existing sequence/pinned double-buffer semantics. |
| `ams_estimator_snapshot` | estimator thread | safety, CAN, CLI | Immutable versioned snapshot. |
| `ams_fault_snapshot` | safety supervisor | CAN, CLI, retained log | One coherent supervisor cycle result. |
| `ams_can_state` | CAN thread/driver adapter | safety and diagnostics | Small atomic status snapshot plus thread-owned queues. |
| `ams_actuator_state` | safety/fan/balance owners | diagnostics | Explicit owner and command/result fields. |
| `ams_calibration_store` | calibration service | current/estimator | Versioned immutable record with CRC/revision. |
| `ams_diag_counters` | subsystem-local writers | CLI/telemetry | Atomic or snapshot publication; never an authority source. |

Do not perform this as a simultaneous rewrite. First add accessors/snapshots around the existing object, then move one ownership domain at a time with byte-for-byte output checks.

### 6.3 Measurement publication

Relevant files:

- `Core/Src/measurement/ams_measurement.c`
- `Core/Inc/measurement/ams_measurement.h`
- publication call sites in `tasks/adbms_task.c`, `tasks/current_task.c`, and `tasks/estimator_task.c`

The double-buffered measurement design is a sound concurrency mechanism:

- a writer fills an inactive slot;
- metadata changes are protected briefly;
- readers pin a slot and copy outside the critical section;
- generation/sequence values prevent duplicate estimator consumption;
- store and snapshot sizes are statically constrained.

Migration treatment:

- keep the data layout and update protocol unchanged for the first Zephyr parity release;
- preserve the v2.6.19 current-window mutex ordering: the voltage boundary timestamp and sample publication cannot be observed in an order that assigns a crossing sample to the wrong integration window;
- preserve all carried-sample metadata used by the following window: uncertainty, selected/mixed range state, extrema, and calibration provenance;
- preserve mixed-range latching until rotation; do not overload a zero/unknown range value as both "not initialized" and "mixed";
- replace FreeRTOS metadata critical sections with a short `k_spinlock` or narrowly scoped interrupt lock;
- never copy a full snapshot while interrupts are locked;
- retain sequence wrap tests, reader-pin exhaustion tests, writer/reader stress tests, boundary-crossing current-window tests, stale-tail tests, and out-of-order timestamp recovery tests;
- expose a pure-C implementation where lock/unlock functions are injected, allowing the same tests under both kernels.

The migration must not revert to independent cell/current/temperature reads or weaken current-window epoch integrity. Coherent publication, serialized boundary ordering, and carried metadata are product requirements now.

### 6.4 Safety supervisor, BMS_OK, panic, and retained evidence

Relevant files:

- `Core/Src/tasks/error_task.c`
- `Core/Src/ext_drivers/ams_safety.c`
- `Core/Src/app.c:set_bms()`

The current safety concept is internally strong:

- startup and panic force BMS_OK low;
- only the supervisor task may assert it in ordinary operation;
- unauthorized assertions fail low;
- a fail-low request also schedules an urgent ADBMS balance mute without blocking the higher-priority safety context on SPI;
- task heartbeats, data freshness, sensor validity, model validity, and profile inhibits are aggregated;
- fault/panic evidence is retained with CRC/commit semantics.

The important concurrency invariant is subtle: a high-priority/ISR fail-low action must not be followed immediately by a supervisor reassertion based on a stale snapshot. Today, the supervisor protects its final decision and GPIO update inside a FreeRTOS critical section. Under Zephyr, preserve the invariant rather than copying the exact mechanism.

Recommended Zephyr pattern:

1. Build the supervisor input snapshot outside the final lock.
2. Compute the decision outside the lock.
3. Enter a very short `k_spinlock` or IRQ lock.
4. Recheck a monotonically increasing `fail_low_epoch` and any direct panic flag.
5. Only assert if the epoch is unchanged and all compile-time/runtime authority gates pass.
6. Write BMS_OK and update the output state.
7. Unlock immediately.

Every immediate deassert path increments `fail_low_epoch` before or atomically with the hardware deassert. This makes stale reassertion prevention explicit and measurable.

Zephyr implementation requirements:

- a `PRE_KERNEL_1` initialization hook or equivalent performs the earliest possible BMS_OK low configuration;
- a fatal-error hook uses direct STM32 register access, not a possibly compromised kernel object or generic GPIO path;
- normal assertion goes through a single wrapper that checks thread identity/authority configuration;
- retained reset/fault records use a linker-retained/no-init region and preserve schema, CRC, and commit-last rules;
- reset-cause registers are sampled before ordinary initialization clears them;
- stack overflow, kernel oops/panic, assertion, MPU fault, hard fault, bus fault, memory fault, usage fault, scheduler/start failure, and watchdog reset are mapped to deterministic AMS panic evidence.

### 6.5 Watchdog

The vehicle profile enables IWDG, although physical reset timing and persistence evidence remain target-validation gates. Firmware 0.5.27 preserves the closed F-05 policy with an explicit **software-liveness/integrity** policy.

IWDG feeding continues while the supervisor and critical task heartbeat set are healthy even when an external/process condition has already forced BMS_OK low. This includes voltage, temperature, current, charger, CAN, ADBMS, and fuse faults. Those conditions remain safety faults; they simply do not create watchdog reset loops that cannot repair the physical cause.

IWDG feeding stops for reset-worthy software conditions:

- panic/fatal exception paths;
- missed critical-task heartbeat or nonzero safety-heartbeat stale mask;
- RTOS integrity/resource failure (`rtos_fault`) or critical task-stack margin;
- explicit bench stop-feed fault injection;
- watchdog hardware-start failure.

The existing startup-grace behavior is retained: IWDG is started/fed during grace while BMS authority remains independently fail-low. Fatal RTOS hooks still execute the direct BMS fail-low action before best-effort bookkeeping.

Zephyr must preserve **intent**, not the FreeRTOS helper names. Use a hardware watchdog whose feed is owned by the safety supervisor only after the required Zephyr thread/task-watchdog liveness checks have passed. External battery/process faults must remain observable while software is healthy rather than being converted into reboot loops. Target qualification must measure feed cadence, actual reset latency, startup behavior, retained reset evidence, and persistent external-fault behavior.

### 6.6 Current sensing

Relevant files:

- `Core/Src/tasks/current_task.c`
- `Core/Src/ext_drivers/current_sensor.c`
- `Core/Src/ext_drivers/current_fault.c`
- `Core/Src/board.c`

The dual-range selection and plausibility logic is product-specific and should remain in portable application code. The present driver performs blocking software-triggered ADC conversions with a bounded 5 ms HAL timeout and 480-cycle acquisition time. It returns on the first failed conversion, so the timeout path is bounded more tightly than two independent full timeouts; ordinary conversions are much faster.

Migration treatment:

- create an `ams_current_adc_ops` interface that returns both channel counts, acquisition timestamp(s), reference/config identity, and an explicit status;
- reproduce 12-bit scaling and 480-cycle sampling on ADC1 channel 3 and ADC2 channel 10;
- preserve the fixed rule that zero calibration requires a successful, valid current conversion, not merely finite previous values;
- initially use synchronous Zephyr ADC reads from the 50 Hz current thread;
- consider DMA/asynchronous acquisition only after parity, because it changes timing, memory placement, cache maintenance, and failure modes;
- validate the H hardware path before differential testing;
- preserve the 0.5.27 uncertainty contract: a calibrated sample has known, nonzero uncertainty; zero and `UINT16_MAX` remain non-calibrated/unknown for estimator, SoP, SoH, and CAN quality decisions;
- preserve range-window semantics, including mixed-range latching and metadata carry across rotation;
- perform physical polarity, offset, gain, range-transition, saturation, uncertainty, temperature, and supply-ratio tests.

Do not move calibration defaults into Devicetree. Pins/reference topology belong there; signed calibration data belongs in a versioned calibration record.

### 6.7 ADBMS6830, ADBMS6822 transport, APM/ADBMS2950, and balancing

Relevant files:

- `Core/Src/ext_drivers/adbms6830.c`
- `Core/Src/ext_drivers/adbms2950.c`
- `Core/Src/ext_drivers/adbms_shared.c`
- `Core/Src/ext_drivers/accumulator.c`
- `Core/Src/tasks/adbms_task.c`

This is the largest and most timing-sensitive porting block. The protocol logic, PEC, command construction, decoding, status diagnostics, chain topology, and most policy can remain. The hardware operations are narrower than the files' sizes imply, but HAL status values are threaded through the code and shared buffers assume serialized use.

Recommended preparatory interface:

```c
typedef enum {
    AMS_IO_OK = 0,
    AMS_IO_TIMEOUT,
    AMS_IO_BUSY,
    AMS_IO_CRC,
    AMS_IO_NOT_READY,
    AMS_IO_HW_ERROR
} ams_io_status_t;

typedef struct {
    ams_io_status_t (*spi_transceive)(void *ctx,
                                      const uint8_t *tx,
                                      uint8_t *rx,
                                      size_t len,
                                      uint32_t timeout_us);
    void (*cs_set)(void *ctx, unsigned string_index, bool asserted);
    void (*delay_us)(void *ctx, uint32_t us);
    uint64_t (*now_us)(void *ctx);
} ams_adbms_bus_ops_t;
```

The exact API may differ, but its intent should not:

- platform-neutral status types above the adapter;
- explicit bus/context ownership;
- explicit deadlines/timeouts;
- no hidden global buffers;
- injectable time and transport for host differential tests.

Initial Zephyr execution model:

- one ADBMS owner thread remains responsible for the SPI/isoSPI chain;
- other threads send bounded requests or flags through static queues/atomics rather than taking a recursive bus mutex directly;
- urgent balance mute remains a nonblocking request observed at every interruptible wait boundary;
- CS and wake/cold-wake timing are owned by the adapter, because generic `cs-gpios` transactions may not express the required pulses/session behavior;
- SPI starts at the present 421.875 kbit/s, mode 3, then faster divisors are separately qualified;
- the 10 Hz acquisition, balance recovery, minimum balance-on time, APM cadence, open-wire cadence, and diagnostics retain current semantics.

The source's S-input and thermistor pull-up warnings require board-revision and bench-evidence reconciliation. In particular, retain the user's reported five-SMB temperature validation; do not order it repeated solely because an old warning remains. Zephyr must not be credited with fixing hardware. The C-only degraded mode remains an explicit compile-time observation profile and never becomes an automatic runtime fallback.

### 6.8 Estimator, SoP, SoH, fuse, and fault algorithms

Relevant directories:

- `Core/Src/estimator`
- `Core/Src/sop`
- `Core/Src/soh`
- pure fault/observer modules in `Core/Src/ext_drivers`

These modules are the lowest-risk part of the migration. They use standard C/math/string interfaces and already have substantial host and MiL coverage. Their numerical behavior should not be retuned while the RTOS is changing.

Rules for the first Zephyr release:

- same source files are compiled into FreeRTOS host/target and Zephyr builds;
- same floating-point ABI and explicit compiler flags are documented;
- no model constants, covariance values, thresholds, LUTs, state topology, or SoH episode logic changes are mixed into the port;
- replay identical timestamped measurements through both builds and compare every published field, validity bit, counter, and state transition;
- tolerances must be field-specific and justified; safety booleans and fixed-point CAN bytes should normally be byte-exact;
- FPU context switching and exception behavior are target-tested under preemption;
- `isfinite`/NaN/Inf, saturation, signed-zero, and range-boundary tests remain.

The earlier covariance-correlation calibration question remains a research/calibration topic, not a Zephyr task and not a reason to change the estimator during migration.

### 6.9 CAN and charger communications

Relevant files:

- `Core/Src/ext_drivers/canbus.c`
- `Core/Src/ext_drivers/can_tx_scheduler.c`
- `Core/Src/tasks/canbus_task.c`
- `Core/Src/ext_drivers/charger.c`

The application-level scheduling policy is more sophisticated than a generic transmit FIFO and should be retained:

- charger shutdown/command traffic has the highest urgency;
- protected required frames have deadlines and completion accounting;
- protected advisory frames are replaceable without weakening required completion;
- cell/temperature base snapshots are distinct from tuning pages;
- generation numbers prevent stale completion from being credited;
- recovery creates a new controller/data epoch.

Firmware 0.5.27 freezes the corrected event-time implementation of the bus-off authority policy that Zephyr must reproduce:

- **START:** `can_authority_ready` begins false; startup cannot enter an authority-capable state until every required frame in a fresh protected `0x680`–`0x687` generation completes on the wire.
- **CHARGE:** bus-off is an immediate sticky hard fault and BMS fail-low.
- **BALANCE:** bus-off is an immediate sticky hard fault and BMS fail-low.
- **HIL with CAN replacing ADBMS:** bus-off is immediate fail-low because the measurement source is lost.
- **DISCHARGE:** a transient bus-off starts one continuous recovery epoch. The ECU contract removes torque/inverter authority at 300 ms without a fresh changing `0x680`; AMS converts the condition to a sticky hard fault at 500 ms if a fresh required protected `0x680`–`0x687` generation has not completed on the wire and restored CAN authority.
- **Repeated failure:** three bus-offs within 10 s, or the application TX inhibit latch, are immediate sticky hard faults.
- Electrical ABOM/controller recovery, explicit bench service recovery, and software queue/commit success do not grant authority by themselves. A subsequent fresh required protected generation must complete on the wire to close the recovery epoch.

Zephyr port strategy:

1. Retain `can_tx_scheduler.c` as a portable policy module where possible.
2. Replace HAL mailbox ownership with a small `ams_can_zephyr.c` adapter using `can_send()` callbacks.
3. Install exact ID/mask filters for charger/ECU/HIL traffic; use a statically sized RX message queue or compact ISR callback that only timestamps/copies.
4. Treat Zephyr RX/state callbacks as ISR context: no blocking, no heap, no shell/log formatting, and only documented ISR-safe primitives.
5. Call `can_get_state()` for actual controller state and use the state-change callback for bus-off notification.
6. Prototype recovery semantics explicitly. Zephyr bxCAN `can_stop()` aborts outstanding transmissions and invokes callbacks; there is no generic per-frame cancel operation. On bus-off, stop/reset the controller epoch, discard stale scheduler generations, publish fresh critical/protected data, then restart/recover according to the approved policy.
7. If manual recovery is enabled, invoke `can_recover()` only from the CAN thread with a bounded timeout, never from an ISR or safety supervisor.
8. Configure the current nominal timing explicitly. Zephyr's bitrate-derived default sample point for rates above 800 kbit/s is not the present 88.9% timing.
9. Preserve measurement-view semantics from 0.5.27: CAN current quality uses same-snapshot calibration provenance, and each cell/temperature reading is re-aged at encode time against its own stored age plus elapsed time. Expired readings must be removed consistently from aggregate minima/maxima, usable masks, ECU detail, and logger detail.

The current AMS-only planning load is about 9.9% at 1 Mbit/s, with a planned whole-vehicle value around 21.2%. That indicates bandwidth headroom, but it is a model. Measure arbitration, error frames, burst behavior, deadlines, bus-off, and ECU staleness on the real bus.

### 6.10 IMD, AIR sense, fans, and peripheral outputs

**IMD:** TIM2 uses a specific edge-capture/reset configuration. The first porting spike should test whether Zephyr's STM32 counter/capture support can reproduce both frequency and duty measurement with the required timeout/overflow semantics. If not, implement a narrowly reviewed STM32 LL driver. Do not recast IMD as GPIO polling.

**AIR:** The current board observes a common coil-control signal, not auxiliary contact feedback. Keep the future AIR monitor compiled out unless actual auxiliary contacts and a reviewed adapter exist. The RTOS migration must not relabel the existing signal as physical state feedback.

**Fans:** The system is open-loop; there is no tachometer feedback. Map each PWM channel and safe inactive state explicitly. Verify frequency, polarity, duty, all-off on reset/panic, and interaction with timer clock changes. A change from 216 MHz to the stock Zephyr 72 MHz would silently alter timer behavior unless the custom clock is correct.

### 6.11 CLI, logging, and diagnostics

The CLI is valuable during bring-up but too large to port wholesale in the first hardware milestone. It contains substantial product-specific diagnostics and uses bounded blocking UART writes from a low-priority thread. The 0.5.27 vehicle profile sets `AMS_ENABLE_CLI=0`, so the vehicle startup/ISR path does not initialize UART3, arm RX, or create the CLI task. Preserve that no-shell vehicle default; target link-map evidence should still confirm unreachable CLI sections are eliminated from the qualified image.

Recommended split:

- **Stage 1:** minimal Zephyr shell commands for build identity, reset cause, BMS authority status, task/stack health, GPIO-safe state, current raw values, SPI probe, CAN state, and fault log.
- **Stage 2:** add ADBMS/APM evidence commands required for parity testing.
- **Stage 3:** selectively port other read-only diagnostics.
- **Never in vehicle:** mutation/fault-injection/service commands unless a separately reviewed service image is built with immutable BMS/balance inhibits.

Use deferred logging only after measuring its thread, buffers, stack, interrupt impact, and panic behavior. Vehicle builds should compile out or minimize verbose logging. The retained AMS safety ring remains the primary safety fault record; Zephyr coredump is optional bench/debug evidence, not a replacement.

### 6.12 Build, tests, and provenance

The host test suite and MiL environment are strong assets. The v2.6.24 baseline retains the v2.6.22 headless build with explicit profile selection, CubeIDE-parity Release optimization, controlled Release epoch metadata, dependency/configuration hashes, and an immutable authored-source-input-tree hash. The remaining weakness is **target evidence**, not the source script: no current approved ARM ELF/MAP/HEX/BIN was produced in this review environment.

Before the migration branch:

- record the approved GCC/STM32Cube/HAL/FreeRTOS revisions and preserve the v2.6.24 source-input/provenance manifest;
- produce clean DEBUG and deterministic RELEASE baseline builds with map files using explicit recorded profiles;
- enable warnings-as-errors for authored project code, with vendor code handled separately;
- archive flash/RAM section sizes, symbols, stacks, startup timing, WCET, and CAN/SPI/ADC traces;
- make build identity one generated source consumed by CLI, CAN telemetry, retained records, and artifact metadata;
- retain SHA-256 checksums and immutable model/calibration/contract revisions.

Zephyr should use a pinned west manifest, exact SDK/toolchain container, configuration fragments under review, deterministic build metadata, SBOM/license output, and an explicit vulnerability-maintenance process.

## 7. Zephyr feasibility and baseline decision

### 7.1 Version policy

Official Zephyr release information current on 2026-09-04 shows:

| Release | Status relevant to this program | Planning consequence |
|---|---|---|
| 4.4.0 | Current stable; released 2026-04-14; EOL 2027-04-12 | Appropriate for an immediate engineering prototype, not an automatic long-lived qualification lock. |
| 4.3.x | Stable line approaching EOL 2026-10-15 | Do not start a new migration here. |
| 3.7 LTS | Supported through 2029-07-27 | Conservative fallback if current LTS support matters more than newer APIs/tooling. |
| 4.6 LTS | Planned for April 2027 | Preferred production/qualification baseline if available before that phase. |

Decision rule:

- If Phase 1 begins now: prototype on a pinned 4.4.0 manifest, with a mandatory production-baseline review before Phase 6.
- If 4.6 LTS is available before the hardware/application integration branch stabilizes: rebase deliberately once, repeat the platform qualification matrix, then freeze.
- If schedule cannot tolerate a rebase and support horizon is paramount: evaluate 3.7 LTS during Phase 0 and accept the older-platform tradeoff explicitly.
- Never track Zephyr `main` for an AMS release.

### 7.2 STM32F767 support

Zephyr's maintained `nucleo_f767zi` board target supports the same MCU family and lists ADC, STM32 bxCAN, counter, GPIO, PWM, SPI, UART, watchdog, FPU, 2 MiB flash, and 384 KiB configured SRAM. This is enough to establish feasibility, but the AMS is not a Nucleo board and must not ship as an overlay pretending to be one.

Important differences:

- stock board clocking is 72 MHz, while AMS uses 216 MHz;
- AMS pin assignments and external circuitry differ;
- stock configured RAM is 384 KiB rather than the physical 512 KiB total;
- AMS requires early/fatal BMS_OK behavior and retained-memory policy;
- SPI CS/wakeup, IMD timer capture, and fan PWM need board-specific proof;
- a Nucleo console/debug configuration is not a vehicle configuration.

Create a first-class custom board derived from the SoC support, not a permanent ad hoc overlay.

### 7.3 API suitability

| Need | Zephyr facility | Suitability/qualification note |
|---|---|---|
| Preemptive periodic tasks | `k_thread`, absolute timeouts | Suitable; use preemptible priorities and absolute deadlines. |
| Static queues | `K_MSGQ_DEFINE` / statically allocated queues | Suitable, including no-wait ISR publication. |
| Short metadata locks | `k_spinlock` / IRQ locks | Suitable; measure IRQ-off time and copy data outside lock. |
| Thread-only resource lock | `k_mutex` | Recursive and priority-inheritance semantics available; prefer single-owner ADBMS design rather than broad recursive use. |
| CAN async TX/RX/state | CAN controller API, callbacks, filters, message queues | Suitable; callbacks run in ISR context and bxCAN recovery/cancel semantics need a spike. |
| ADC | ADC API / STM32 driver | Suitable in principle; exact 480-cycle sampling and dual-controller behavior must be verified in the pinned release. |
| SPI | SPI API plus GPIO/timing adapter | Suitable for transfers; custom CS/wake behavior likely remains application adapter logic. |
| PWM | PWM API | Suitable if exact timer frequency/polarity mappings are verified. |
| IMD timer input capture | Counter/capture API or STM32-specific adapter | Uncertain until prototype; this is an early feasibility spike. |
| IWDG | watchdog API | Suitable; feed policy and reset evidence remain application requirements. |
| Earliest fail-low | `SYS_INIT(..., PRE_KERNEL_1, ...)` plus direct-register fallback | Suitable if proved from reset and fault injection. |
| Retained ring | retained-memory/retention facilities or explicit no-init section | Suitable; reset survival and linker placement require target proof. |
| Bench CLI | Zephyr shell over UART | Suitable; begin minimal and keep vehicle policy separate. |
| Host/unit integration | ztest, Twister, native simulation | Suitable as additions; retain existing pure-C host/MiL suites. |

### 7.4 Go/no-go conclusion

There is no platform-support reason to reject Zephyr. The program should proceed to a no-authority prototype only after Phase 0 baseline gates. Vehicle migration remains a separate decision after ADBMS, CAN, ADC, IMD, timing, stack, watchdog, and fail-low evidence exists.

## 8. Proposed target architecture

### 8.1 Repository/workspace layout

One reasonable structure is:

```text
der26-ams-zephyr/
  west.yml
  app/
    CMakeLists.txt
    Kconfig
    prj.conf
    config/
      bench.conf
      bench_validation.conf
      hil.conf
      testday.conf
      vehicle.conf
    src/
      main.c
      ams_threads.c
      ams_supervisor.c
  boards/
    der/
      der26_ams/
        board.yml
        Kconfig.der26_ams
        Kconfig.defconfig
        der26_ams.dts
        der26_ams-pinctrl.dtsi
        board.cmake
  lib/
    ams_core/
      measurement/
      estimator/
      sop/
      soh/
      faults/
      can_policy/
      include/
  drivers/
    ams/
      adbms_bus_zephyr.c
      current_adc_zephyr.c
      can_zephyr.c
      imd_capture_stm32.c
      safety_gpio_stm32.c
      retained_fault_zephyr.c
  tests/
    unit/
    native_sim/
    hardware/
    differential/
  scripts/
    build_manifest.py
    compare_replay.py
    check_board_contract.py
  doc/
    architecture/
    safety/
    validation/
```

The exact repository can be separate or coexist with the current tree. A separate west topdir with the AMS core imported as a module is preferable during migration because it keeps the released FreeRTOS project buildable and gives both targets the same algorithm source.

### 8.2 Layer boundaries

| Layer | Responsibilities | Must not contain |
|---|---|---|
| Pure AMS core | measurement layouts, estimators, SoP/SoH/fuse, fault logic, scheduling policy, state machines, encoding/decoding | Zephyr, FreeRTOS, STM32 HAL, direct registers |
| Application services | thread loops, coherent snapshots, authority state machine, subsystem ownership, deadlines | vendor register knowledge except safety adapter calls |
| Platform interfaces | time, lock, queue, ADC, SPI, CAN, PWM, capture, watchdog, retained memory, UART | product thresholds/control-law decisions |
| Zephyr adapters | device handles, Kconfig/DTS access, driver calls, ISR callbacks | duplicate battery algorithms or silent fallback policy |
| STM32 safety micro-layer | earliest BMS low, fatal direct deassert, reset-cause, unavoidable timer details | ordinary business logic, heap, blocking calls |

### 8.3 Thread model

Zephyr uses smaller numeric values for higher preemptible priority. Preserve relative priority and rate first; optimize only from target evidence. Leave gaps for adjustment.

| Proposed thread | Initial Zephyr priority | Release model | Initial stack budget | Notes |
|---|---:|---|---:|---|
| Safety supervisor | 0 | absolute 50 ms | 2 KiB | Highest application thread; no blocking I/O. |
| Current acquisition | 2 | absolute 20 ms | 2 KiB | Synchronous ADC initially; publish timestamped result. |
| ADBMS owner | 3 | absolute 100 ms plus state-machine waits | 8 KiB | Sole SPI-chain owner; services urgent mute. |
| CAN service | 4 | absolute 100 ms plus callback events | 8 KiB | Owns scheduler and controller epoch; ISR callbacks only enqueue/update atomics. |
| Estimator | 6 | sequence-triggered or absolute 100 ms | 8 KiB | Consumes each measurement sequence once. |
| Fan service | 8 | absolute 200 ms | 1.5 KiB | Open-loop output, fail-safe defaults. |
| AIR auxiliary monitor | 8 | configured monitor period | 1.5 KiB | Absent until real aux-feedback hardware. |
| IMD service | 9 | absolute 100 ms/event-assisted | 1.5 KiB | Capture ISR publishes compact event/state. |
| Shell/diagnostics | 12 | event-driven / lowest | 4 KiB bench only | Not required in vehicle image unless explicitly approved. |

Initial all-feature stack allocation is approximately 36.5 KiB, intentionally conservative. Tune downward only after optimized target high-water and worst-case fault tests. Do not use Zephyr's system workqueue for safety, ADBMS, current, CAN deadline, or estimator operations; its priority and unrelated clients make it the wrong ownership boundary.

All application threads should be preemptible. Cooperative or metairq threads are unnecessary and would complicate timing assurance. Periodic threads should use absolute deadlines (`K_TIMEOUT_ABS_*` or equivalent) and record:

- scheduled release;
- actual start;
- completion;
- deadline miss/overrun;
- worst and rolling WCET;
- input sequence/age;
- stack high-water.

### 8.4 Communication model

Use static bounded communication only:

- pinned/double-buffered immutable measurement snapshots;
- fixed-depth message queues for ISR-to-thread events;
- atomic flags/counters for urgent idempotent requests such as fail-low/balance mute;
- explicit latest-value slots for replaceable CAN telemetry;
- no unbounded FIFO for sensor or safety data;
- no heap allocation in steady state or vehicle builds;
- clear overflow policy: a dropped diagnostic may increment a counter, but lost safety events must cause a deterministic conservative state.

Suggested primitives:

| Current concept | Initial Zephyr mapping |
|---|---|
| `taskENTER_CRITICAL` for small metadata | `k_spin_lock` or narrow IRQ lock |
| CMSIS recursive ADBMS mutex | Prefer single-owner thread plus static requests; temporary `k_mutex` only during staged port |
| Current-window mutex | `k_mutex` if thread-only; redesign as owner snapshot if practical |
| ISR RX ring | fixed `k_msgq` or retained custom ring with spin/atomic metadata |
| Task heartbeat fields | per-thread atomic sequence/deadline state sampled by supervisor |
| Delay-until loops | absolute kernel timeouts |
| Direct task-handle assertion check | supervisor capability token/thread identity plus compile-time authority |

### 8.5 Board and Devicetree design

The DER26 board definition should explicitly describe:

- STM32F767ZIT6 and 8 MHz HSE;
- 216 MHz clock tree, flash wait states, bus/timer clocks, and FPU;
- conservative 384 KiB SRAM region for the first port;
- USART3 PD8/PD9;
- CAN1 PD0/PD1 and transceiver/termination documentation;
- SPI6 PG12/PG13/PG14 and PE2/PE4 chip-select GPIOs;
- ADC1 PA3 and ADC2 PC0 with reference/scaling documentation;
- TIM3/4/5 PWM channels and safe states;
- TIM2 IMD capture resources;
- PE0 BMS_OK safe output;
- retained memory region;
- watchdog;
- any GPIOs used for AIR sense, charger interface, LEDs, and hardware revision identification.

Safety-relevant board details need machine checks beyond syntax:

1. Compare compiled DTS pin assignments to an approved pin manifest.
2. Verify clock rates used in CAN, SPI, PWM, ADC, and microsecond timing.
3. Fail the build if authority profiles use an unknown board revision.
4. Fail the build if BMS_OK active polarity/default state is not explicitly defined.
5. Emit a board-contract artifact alongside the firmware hash.

Use current Zephyr SDO/SDI terminology in new SPI files even though the legacy source and schematic use MOSI/MISO.

### 8.6 Kconfig/profile design

Create reviewed fragments, but keep safety rules in compile-time assertions too. Kconfig symbols can be overridden; the final compiled C contract is the last guard.

Common policy:

- `CONFIG_HEAP_MEM_POOL_SIZE=0` unless a measured, reviewed dependency requires otherwise;
- static thread, queue, and stack definitions;
- project code warnings as errors;
- assertions and stack diagnostics enabled in debug/HIL;
- exact logging level per profile;
- shell only in bench/service profiles;
- no runtime device power management until explicitly qualified;
- no userspace/MPU in the first parity port; assess it later as a separate change;
- no automatic optimization/control-law changes during kernel migration.

Profile intent:

| Profile | BMS authority | Balance authority | Injection/shell | Required hardware |
|---|---|---|---|---|
| Bench | inhibited | inhibited | diagnostics allowed | partial hardware acceptable with explicit invalidity |
| Bench validation | compile-time impossible | compile-time impossible | evidence tools allowed | defined five-SMB or one-SMB topology |
| HIL | compile-time impossible | compile-time impossible | HIL injection allowed | ADBMS replacement contract |
| Testday | compile-time impossible | compile-time impossible | selected read-only evidence | installed system under controlled observation |
| Vehicle | allowed only after all release macros/manifest gates | allowed after gates | no mutation; shell absent or strictly approved read-only | fully validated hardware and system contracts |

### 8.7 Memory and CPU budgets

Use the stock Zephyr board's conservative 384 KiB SRAM exposure as the initial budget, leaving DTCM unused until explicitly placed and tested. Provisional gates:

- total static/kernel RAM below 60% of 384 KiB before vehicle qualification;
- at least 25% unused stack and at least 512 bytes absolute headroom per critical thread under optimized worst-case stress;
- zero heap allocation after initialization, preferably no heap at all;
- nominal CPU below 50%, injected-fault/stress CPU below 70%;
- no missed safety/current/ADBMS/CAN deadlines under accepted load;
- flash below 50% of 2 MiB until logging/shell/debug sections are stripped and final update strategy is known;
- explicit accounting for retained data, interrupt stack, log buffers, libc, FPU contexts, and CAN/measurement queues.

These are engineering headroom gates, not derived safety requirements. Refine them after Phase 0 baseline measurements and Phase 1 Zephyr overhead measurements.

## 9. Safety invariants for the migration

The following are non-negotiable parity requirements. Each needs a requirement ID, implementation trace, and test evidence.

1. BMS_OK is electrically low from the earliest controllable point after reset.
2. BMS_OK goes low on kernel fatal error, CPU exception, stack failure, failed mandatory initialization, supervisor failure, and explicitly defined safety faults.
3. Only the safety supervisor may assert BMS_OK during normal operation.
4. Any other execution context may deassert but cannot assert.
5. A direct fail-low event cannot be undone by a stale supervisor computation.
6. Proposed Zephyr Bench, HIL, TESTDAY, and BENCH_VALIDATION images have final-writer compile-time barriers preventing BMS_OK assertion or balancing through supported application/command paths. This is a proposed tightening for some legacy profiles, not a claim that every current profile already has it. Compile-time checks do not protect against arbitrary code execution or arbitrary peripheral-register corruption; physical inhibition and the defined hardware fault model remain necessary.
7. BMS assertion requires one coherent, fresh, validated supervisor input snapshot and all required subsystem heartbeats.
8. Raw cell-voltage safety authority remains independent from filtered/estimator voltage products.
9. Degraded C-only voltage mode is compile-time explicit and cannot appear automatically after an S-path fault.
10. Current invalidity, range disagreement, ADC implausibility, stale data, and calibration invalidity retain current conservative behavior.
11. An urgent fail-low also requests balance mute without blocking the supervisor on SPI.
12. Charger shutdown is not replaced by older normal commands and is republished/recovered according to the CAN safety contract.
13. CAN required-frame completion is based on semantic frame class and actual completion in the current controller epoch.
14. Watchdog feed conditions exactly match the approved policy.
15. Retained records are either valid by schema/CRC/commit marker or ignored; partially written records are never reported as valid.
16. No safety decision depends on a diagnostic-only flattened copy or stale CAN/CLI representation.
17. No dynamic allocation is required in the steady-state vehicle path.
18. Timing overruns, queue overflow, driver-not-ready, and recovery exhaustion have explicit conservative outcomes.
19. Hardware outputs start in safe states and remain safe if the kernel never starts.
20. Build profile, board revision, source commit, calibration, thresholds, estimator, SoP, SoH, and CAN contract revisions are available in the running image and artifact manifest.
21. A current sample crossing an ADBMS voltage boundary cannot be assigned to the wrong integration window because boundary capture and current publication preserve one serialized ordering.
22. The current value carried into a new window carries its uncertainty, range/mixed state, extrema, and calibration provenance with it; mixed range remains latched until rotation.
23. Zero and `UINT16_MAX` current uncertainty are never accepted as calibrated-current evidence by estimator acquisition, SoP, SoH, or CAN quality encoding.
24. CAN cell/temperature validity is based on each reading's age at encode time, not merely the enclosing snapshot's age; expired readings cannot remain in aggregates, usable masks, or detail payloads.
25. CAN current quality and calibration fields are derived from the same coherent measurement snapshot and cannot be upgraded solely because the current conversion is electrically valid.
26. SoP/SoH constituent freshness is based on the age of the cell/temperature data actually used at solve time, not only the enclosing publication timestamp; unknown or implausible age references fail stale.
27. Periodic ADBMS diagnostics do not replay missed periods as a recovery burst after startup or transport/topology loss; the schedule resumes at a future slot.

## 10. Phased work breakdown

Do not run the phases as one long feature branch. Each phase produces a reviewable artifact and a go/no-go gate. Estimates assume an engineer already familiar with the current AMS and STM32 embedded development.

The hardware-dependent baseline measurements are required before claiming parity or vehicle readiness, not before writing any prototype code. Safe, no-authority board/driver work can proceed while the H-current path or evidence reconciliation is pending. Only that blocked subsystem's equivalence claim must wait. Automated tests should run when affected code or platform behavior changes; unchanged historical MiL campaigns need not be rerun at every phase. HIL below is an optional validation method, not a requirement to build a new HIL platform: existing replay, signal injection, safe loads, and target fault tests may provide equivalent requirement coverage.

### Phase 0 — baseline closure and migration requirements

**Estimate:** 1.5–2.5 engineer-weeks
**Authority:** current FreeRTOS images only; no Zephyr firmware yet

Tasks:

1. Freeze firmware 0.5.27 / package v2.6.24 as the FreeRTOS comparison baseline and archive the focused measurement-integrity, post-closeout, and migration-readiness regression results. Do not reopen F-01/F-02/F-03/F-04/F-10/F-13 or the v2.6.22 AUX2/freshness/profile-test defects as active blockers unless a later change regresses their gates.
2. Record the approved DER26 pin/net contract with SPI CS_B fixed at PE4; the prior PF4 schematic annotation is a known labeling error and is not a firmware-remap task.
3. Record the exact STM32F767, board revision, breakout revision, connector orientation, oscillator, transceiver, termination, ADBMS ring, APM, current sensor, IMD, fan, charger, and BMS_OK hardware contract.
4. Produce clean FreeRTOS DEBUG and deterministic RELEASE baseline ELF/MAP/HEX/BIN artifacts on the approved ARM toolchain using 0.5.27 / v2.6.24 with the AMS profile explicitly recorded. Archive `BUILD_PROVENANCE.txt`, `SHA256SUMS.txt`, `.su` files, and the exact toolchain version.
5. Capture flash/RAM/global/stack sizes.
6. Measure task execution time, release jitter, IRQ-off time, CAN load/deadlines, SPI waveform/timing, ADC acquisition timing, fan frequency, BMS fail-low latency, and watchdog timing.
7. Archive the approved v2.6.24 CAN bus-off state/event-time policy and include its exact 499/500/501 ms and immediate-fail-low vectors in the parity corpus.
8. Archive the approved software-liveness-only IWDG policy and its external-fault-versus-software-integrity vectors in the parity corpus.
9. Repair/diagnose the current H channel or explicitly mark it as a blocking hardware dependency.
10. Reconcile genuine remaining hardware/release evidence without repeating already accepted five-SMB temperature validation.
11. Freeze a list of byte-exact replay vectors and expected safety state transitions, including the v2.6.19/v2.6.20 measurement-integrity edge cases, v2.6.22 constituent-age/AUX2 recovery cases, and v2.6.23 watchdog-policy cases plus the v2.6.24 CAN event-time/service-recovery/tuning-feature cases.
12. Promote the focused current-window/CAN-aging/uncertainty regressions, v2.6.21 CLI gates, v2.6.22 profile/provenance/AUX2/effective-age gates, and v2.6.23 watchdog-policy and v2.6.24 CAN event-time/service-recovery/tuning gates into the dual-build parity suite where applicable.

Deliverables:

- approved current-system architecture and safety invariants;
- pin/peripheral/clock manifest;
- FreeRTOS baseline evidence bundle and artifact hashes;
- requirements-to-test matrix for migration parity;
- migration decision record selecting 4.4 prototype versus 3.7 LTS;
- hardware blocker register with owners and dates.

Exit gate:

- all provenance/pin contradictions closed or formally dispositioned;
- baseline can be rebuilt reproducibly enough to compare behavior;
- no-authority migration scope approved;
- watchdog and CAN recovery requirements approved;
- current physical blockers have owners and do not get silently deferred into software.

### Phase 1 — Zephyr workspace, custom board, and safe skeleton

**Estimate:** 1.5–2.5 engineer-weeks
**Authority:** BMS_OK and balancing physically and compile-time inhibited

Tasks:

1. Pin Zephyr/SDK/modules in `west.yml`.
2. Create custom `der26_ams` board, Kconfig, DTS, pinctrl, and configuration fragments.
3. Establish 8 MHz HSE to 216 MHz operation and verify derived bus/timer clocks.
4. Implement earliest BMS_OK low hook and direct-register fatal deassert.
5. Implement reset-cause capture and a minimal build manifest.
6. Bring up UART3 with only build/status/safety commands.
7. Enable kernel fatal/assert/stack diagnostics for bench.
8. Produce DEBUG and RELEASE builds with map/size/SBOM/hash output.
9. Add CI board-contract checks.

Tests:

- cold/warm/watchdog/software reset: BMS_OK never glitches high;
- deliberate fatal/assert/stack fault: output goes/remains low and cause is visible after reset;
- clock frequencies measured at observable peripherals;
- exact board/pin contract emitted from build;
- no-authority cannot be overridden at runtime;
- memory layout does not use DTCM unexpectedly.

Exit gate:

- safe skeleton boots repeatedly on the actual AMS board;
- BMS fail-low timing is at least as conservative as baseline;
- build identity is deterministic and matches artifacts;
- no peripheral beyond diagnostics can assert vehicle authority.

### Phase 2 — platform-neutral core extraction and dual build

**Estimate:** 3–5 engineer-weeks
**Authority:** none

Tasks:

1. Define neutral status, time, lock, bus, and snapshot interfaces.
2. Compile estimator, SoP, SoH, fuse, fault, observer, encoding, and scheduler policy under both environments from the same sources.
3. Remove `HAL_StatusTypeDef` from core-facing ADBMS interfaces incrementally.
4. Move shared buffers into explicit driver/context instances.
5. Add build-time type/layout assertions for replay structures and CAN payloads.
6. Establish a differential-replay tool producing machine-readable field deltas.

Tests:

- existing host/MiL suites remain unchanged and passing when relevant source moves occur;
- byte-exact CAN encoding and safety booleans;
- field-toleranced float outputs with documented compiler/FPU settings;
- malformed input, NaN/Inf, sequence wrap, timestamp wrap, saturation, and reset-state tests;
- current-window crossing/race replay, carried uncertainty/range/calibration metadata, mixed-range latching, and unknown-uncertainty sentinel tests;
- CAN per-reading age-limit, post-limit expiry, usable-mask/aggregate/detail consistency, and same-snapshot calibration-quality tests;
- no Zephyr header in pure-core dependency graph.

Exit gate:

- the core is built and tested in both systems;
- no algorithm/control-law change is mixed into the extraction;
- outputs match the frozen replay corpus.

### Phase 3 — current ADC, fans, IMD, and watchdog adapters

**Estimate:** 2.5–4 engineer-weeks
**Authority:** none

Tasks:

1. Port dual ADC acquisition with exact channels, resolution, sampling time, reference/scaling, and timestamps.
2. Port fan PWM safe-state and duty behavior.
3. Spike TIM2 IMD capture with generic API; write minimal STM32 adapter if required.
4. Port IWDG according to the approved policy, initially disabled in the ordinary bench fragment.
5. Port calibration record validation and diagnostics.

Tests:

- ADC raw-count comparison from known voltages and real DHAB outputs;
- open/short/rail/stale/timeout/range-disagreement tests;
- zero-calibration refusal on invalid samples;
- signed current, range transition, current step, temperature, and uncertainty testing;
- known/zero/`UINT16_MAX` uncertainty provenance tests and mixed-range metadata carry through current-window rotation;
- fan frequency/polarity/duty/all-off-on-reset across all six channels;
- IMD valid/invalid frequency/duty, missing signal, stuck level, overflow, and ISR load;
- watchdog block reason, feed cadence, reset timing, retained evidence, and persistent-fault behavior.

Exit gate:

- each peripheral meets baseline behavior and its physical validation requirements;
- current H hardware path works and both channels are calibrated before claiming current parity;
- no unexplained timing/stack regression.

### Phase 4 — ADBMS/isoSPI/APM transport and acquisition

**Estimate:** 4–6 engineer-weeks
**Authority:** none; balancing remains inhibited except dedicated safe-load tests

Tasks:

1. Implement Zephyr SPI6/time/CS adapter at 421.875 kbit/s mode 3.
2. Reuse PEC/protocol/decoder code through neutral interfaces.
3. Establish one ADBMS owner thread and static request/event paths.
4. Port awake-session, wake/cold-wake, delay, retry, transport-ready, and diagnostic behavior.
5. Port five-SMB and one-SMB validation topologies.
6. Port APM/ADBMS2950 as advisory first.
7. Port coherent measurement publication and urgent balance mute.
8. Port balance recovery and minimum-on timing, keeping balancing inhibited until dedicated validation.

Tests:

- logic-analyzer comparison of CS/SCK/SDO/SDI and wake timing;
- identity/config/status readback and PEC fault injection;
- open ring, reversed/absent SMB, missing APM, wrong chain length, sleeping transceiver, SPI timeout, and corrupted response;
- 10 Hz deadline and jitter under CAN/UART/fault activity;
- raw C/S/AUX/status/diagnostic byte and engineering-unit comparison;
- urgent balance-mute latency while transport is busy or waiting;
- no stale/partial measurement publication;
- reset and recovery at every acquisition state-machine point.

Exit gate:

- full intended ring is stable for an agreed soak period;
- measurement images and faults match the baseline/replay oracle;
- WCET, bus timing, stack, and recovery bounds pass;
- hardware routing defects remain explicitly gated rather than masked.

### Phase 5 — measurement integration, supervisor, and thread parity

**Estimate:** 3–5 engineer-weeks
**Authority:** none

Tasks:

1. Port the pinned double-buffer measurement store.
2. Implement thread rates/priorities/absolute releases and heartbeat/deadline telemetry.
3. Port supervisor snapshot and fail-low epoch mechanism.
4. Port estimator consumption/publishing without numerical changes.
5. Port fault log and retained memory fully.
6. Remove temporary broad root-state aliases where evidence permits.

Tests:

- producer/reader concurrency and sequence-wrap stress;
- force a higher-priority current publication across an ADBMS voltage-boundary capture and verify exact window current/charge, validity, uncertainty, range, and calibration provenance;
- injected ISR fail-low at every supervisor decision point to prove no stale reassertion;
- thread starvation, intentional overrun, queue full, mutex timeout, and priority inversion tests;
- CPU/FPU context stress while EKF is preempted;
- BMS output remains inhibited in all migration profiles;
- retained log consistency across reset during every write stage;
- end-to-end replay of coherent measurement through safety/estimator/telemetry.

Exit gate:

- all 27 migration safety invariants have implementation traces and bench evidence applicable to this phase;
- critical sections and timing meet approved bounds;
- no reliance on diagnostic flattened state for authority.

### Phase 6 — CAN scheduler, vehicle messages, and charger

**Estimate:** 3–5 engineer-weeks
**Authority:** none

Tasks:

1. Integrate the fixed application scheduler with Zephyr CAN callbacks.
2. Install exact filters and 1 Mbit/s timing.
3. Implement fixed RX event queues and overflow diagnostics.
4. Implement controller epoch, bus-off, stop/abort/recovery, and fresh-data barrier behavior.
5. Port charger command/shutdown and ECU mission input.
6. Port protected required/advisory, base detail, tuning, and logger streams in stages.
7. Update the whole-vehicle DBC/contract and timing evidence.

Tests:

- scope/CAN-analyzer proof of bit timing and sample-point configuration;
- exact CAN IDs, DLC, byte order, scale, validity, sequence, freshness, and cadence;
- current diagnostic quality with valid-but-uncalibrated, calibrated, zero-uncertainty, and unknown-uncertainty snapshots;
- individual cell/temperature aging at the exact stale limit and immediately after it, including aggregate minima/maxima, usable masks, ECU detail, logger detail, age overflow defense, and tick wrap;
- all three mailboxes busy, simultaneous completions, callback reorder, error passive, bus-off, unplug/replug, stuck dominant/recessive, and recovery timeout;
- shutdown while normal charger command pending;
- base detail during fast tuning congestion;
- required-frame completion by class;
- ECU stale-data reaction in charge and discharge;
- planned vehicle load, burst load, error frames, and diagnostic load;
- no callback executes blocking/application work in ISR context.

Exit gate:

- application behavior matches the corrected v2.6.17 scheduler policy and the v2.6.19/v2.6.20 measurement-quality semantics;
- system-level bus-off safety requirement passes with ECU/charger participants;
- measured utilization/deadlines have margin;
- no stale frame is credited after a controller epoch change.

### Phase 7 — shell, diagnostics, retained evidence, and release provenance

**Estimate:** 2–4 engineer-weeks
**Authority:** none

Tasks:

1. Port only required read-only diagnostic commands first.
2. Add separately gated bench/service mutation commands if still needed.
3. Establish profile-specific logging and panic behavior.
4. Finalize retained fault schema/version migration.
5. Unify build identity across shell, CAN, retained log, ELF note, and artifact manifest.
6. Add deterministic build, license, SBOM, and vulnerability records.

Exit gate:

- vehicle configuration has the approved shell/logging surface only;
- no diagnostic path can gain authority or materially starve a safety thread;
- running identity matches the signed artifacts.

### Phase 8 — automated parity and regression infrastructure

**Estimate:** 3–5 engineer-weeks
**Authority:** none

Tasks:

1. Keep existing host and MiL tests as the primary algorithm evidence.
2. Add ztest unit suites for Zephyr adapters and application-service behavior.
3. Add native simulation tests for scheduling policy, queues, state machines, recovery, and fault injection where hardware timing is not required.
4. Use Twister for board/profile/debug/release matrices.
5. Add FreeRTOS-versus-Zephyr replay comparison in CI.
6. Add static dependency, board-contract, Kconfig, stack-size, map, and forbidden-feature checks.
7. Run Zephyr hardening/configuration checks as advisory evidence.

Exit gate:

- every migrated requirement is covered at the cheapest applicable layer;
- target-only gaps are explicit;
- reproducible artifacts and evidence are generated from a clean workspace.

### Phase 9 — bench/HIL qualification

**Estimate:** 4–7 engineer-weeks
**Authority:** initially none; any authority test uses a safe simulator/load and a separately reviewed image

Campaigns:

- long-duration five-SMB/APM acquisition soak;
- HIL current/voltage/temperature/IMD/charger/CAN scenarios;
- fault at every initialization stage and periodic state;
- reset/brownout/noise/unplug/replug tests;
- timing, stack, CPU, IRQ latency, and bus-load stress;
- balancing only with measured load/current/thermal controls;
- FreeRTOS and Zephyr A/B replay and physical I/O comparison;
- power cycling during retained-log writes and configuration/calibration access;
- EMC/noise-representative disturbances where facilities permit.

Exit gate:

- bench parity report approved;
- no unexplained output, timing, state, or fault differences;
- all vehicle compile gates have actual evidence or remain false;
- independent code/safety review completed.

### Phase 10 — vehicle qualification and release

**Estimate:** 4–8 engineer-weeks
**Authority:** staged and controlled only after approval

Tasks:

1. Freeze the production Zephyr LTS/version and west manifest.
2. Re-run the complete platform matrix after the final version rebase.
3. Enable a vehicle profile only after every existing and new gate is satisfied.
4. Perform LV vehicle integration, charger integration, shutdown-loop/ECU staleness, and physical calibration.
5. Progress through HV simulator/pack, dyno, controlled vehicle, and endurance stages under the team's safety process.
6. Audit release artifacts, source, manifest, calibration, DBC, hashes, SBOM, known vulnerabilities, toolchain, and evidence.
7. Retain the approved FreeRTOS release and documented rollback/flashing procedure until Zephyr is formally accepted.

Exit gate:

- signed release decision with all evidence linked;
- no open high-risk finding;
- rollback image/procedure verified;
- installed firmware identity independently confirmed after flash.

## 11. Validation matrix

### 11.1 Evidence layering

| Layer | Purpose | Typical subjects | Not sufficient for |
|---|---|---|---|
| Pure host unit/oracle | Fast exhaustive logic and numerical tests | PEC, decoders, EKF, SoP, SoH, fuse, thresholds, CAN policy | ISR, timing, real driver behavior |
| Differential replay | Detect behavioral drift between baselines | snapshots, faults, estimator, CAN bytes, sequences | electrical behavior |
| Zephyr ztest/native simulation | Kernel adapter/state-machine integration | queues, locks, timeouts, recovery, profiles | STM32 peripheral quirks |
| Target low-voltage bench | Peripheral and scheduling proof | GPIO, ADC, SPI, CAN, PWM, capture, watchdog, retained memory | full pack/vehicle interactions |
| HIL | controlled scenario/fault coverage | sensor trajectories, staleness, ECU/charger contracts | all analog/EMC/physical failure modes |
| Pack/vehicle | final integration and physical evidence | polarity, calibration, loads, shutdown loop, timing | algorithm corner-case exhaustiveness |

### 11.2 Required parity classes

| Class | Comparison rule |
|---|---|
| Safety outputs/fault bits/profile authority | Exact state/event equivalence. Any difference requires documented requirement review. |
| CAN payloads and sequence/freshness metadata | Byte-exact for identical accepted inputs and timestamps, including calibration-quality provenance, per-reading stale exclusion, usable masks, and detail invalid encodings. |
| Integer counters/timestamps | Exact where the same logical clock is supplied; bounded relation where real schedulers differ. Current-window boundary ordering and per-reading age arithmetic must satisfy the same exact edge-case contract. |
| Floating estimator/SoP/SoH outputs | Field-specific numerical tolerance after identical compiler/FPU configuration; safety quantization must remain exact. |
| Deadline/recovery behavior | Requirement-equivalent, not necessarily implementation-identical. |
| Electrical timings | Within approved hardware ranges and no worse than baseline safety bounds. |
| Stack/CPU/memory | Meet new headroom gates; not merely equal to baseline. |

### 11.3 Mandatory target measurements

- reset-to-BMS-low and fatal-to-BMS-low latency;
- any BMS_OK glitch during reset, boot, flash, debugger attach, fatal, and watchdog reset;
- maximum interrupt-disabled interval;
- task start jitter and WCET distributions under normal, CLI, CAN saturation, SPI fault, and injected-fault loads;
- stack unused bytes for every thread and interrupt stack;
- CPU utilization and longest non-preemptible section;
- SPI clock, phase, polarity, CS pulse, wake, command-to-read, and idle/session timing;
- ADC sample/conversion timing and raw-count accuracy;
- PWM frequency/duty/polarity;
- IMD capture accuracy, timeout, and ISR rate;
- CAN sample point/bit timing, frame cadence, arbitration latency, completion, bus-off, recovery, and utilization;
- IWDG timeout/feed/reset and retained reset evidence;
- balance mute and restoration timing;
- coherent measurement age at supervisor and estimator consumption.

## 12. Risk register

| Risk | Likelihood | Impact | Mitigation | Trigger/owner action |
|---|---|---|---|---|
| Safety output glitches during Zephyr startup | Medium | Critical | PRE_KERNEL direct low, scope every reset mode, hardware pull-state review | Any high pulse stops migration. |
| Devicetree captures wrong board pin/revision | Medium | Critical | approved pin manifest, continuity evidence, compiled-DTS CI check | Resolve CS_B and connector revision before board freeze. |
| ADBMS timing changes under Zephyr SPI/scheduler | High | High | neutral bus adapter, conservative clock, logic-analyzer A/B, one owner thread | Any PEC/session/deadline regression blocks Phase 4. |
| CAN driver semantics invalidate scheduler assumptions | Medium | High | dedicated callback/abort/recovery spike, controller epochs, fault injection | Resolve before porting complete telemetry set. |
| IMD capture unsupported by generic API | Medium | High | early prototype, minimal STM32-specific driver | Decide generic versus LL adapter in Phase 3. |
| Current H hardware fault blocks parity | High now | High | repair/trace J11-R29-R30-TP4-J4-PA3; calibrate both channels | No current-authority gate without physical resolution. |
| Algorithm drift due compiler/FPU/optimization | Medium | High | same source, pinned flags, differential replay, FPU stress | Any safety/CAN difference requires root cause. |
| Hidden shared-state race emerges during decomposition | Medium | High | accessors first, one owner per state, stress/TSAN-capable host tests where possible | Never refactor and kernel-port a subsystem in one unreviewed step. |
| Zephyr 4.4 lifecycle expires mid-program | High if delayed | Medium/high | scheduled 4.6 LTS decision and one controlled rebase before qualification | Freeze no short-lived baseline for vehicle. |
| Memory/DTCM/DMA/cache error | Medium | High | use 384 KiB conservative memory first, no DMA initially, map/cache tests | Explicit review before enabling DMA/DTCM. |
| Stack estimate too small in fault/log paths | Medium | High | conservative initial stacks, sentinel/high-water, worst-case optimized stress | Less than gate margin blocks release. |
| Watchdog resets mask valid external faults | Medium | High | approve policy, test reset loop/evidence/charger behavior | Decision required in Phase 0. |
| CLI/logging causes timing or security surface | Medium | Medium/high | minimal shell, lowest priority, profile compile-out, rate limits | Vehicle shell requires explicit approval. |
| Existing hardware limitations get misattributed to RTOS | High | High | independent hardware blocker register and A/B tests | No workaround silently changes safety authority. |
| Test rewrite loses existing evidence | Medium | High | reuse core source/tests; add ztest rather than replace MiL/oracles | Existing campaign remains required until superseded formally. |
| Team attempts a flag-day cutover | Medium | Critical | dual-build phases, small PRs, exit gates, rollback branch | Reject broad rewrite PRs. |
| Zephyr dependency/vulnerability maintenance lapses | Medium | High | pinned manifest, vulnerability review, planned update cadence | Release checklist includes supported-version status. |

## 13. Program controls and review structure

### 13.1 Change rules

- One behavioral dimension per change: platform extraction, driver port, or algorithm calibration—not combinations.
- Every safety-relevant change states its old/new invariant and test evidence.
- Generated DTS/Kconfig/build manifests are archived with binaries.
- Vendor/Zephyr updates are separate from application changes.
- No migration PR enables BMS or balancing authority by default.
- Bench exceptions are separate named profiles, never mutable escapes in vehicle builds.
- Every timing assumption is either measured or marked provisional.
- Every hardware ambiguity has an owner and evidence link.

### 13.2 Suggested review ownership

| Review | Minimum reviewers |
|---|---|
| Board/DTS/clock/pin contract | firmware + electrical engineer |
| BMS_OK/panic/watchdog/supervisor | firmware + independent safety reviewer |
| ADBMS/isoSPI/balancing | firmware + battery/electrical engineer |
| CAN/charger/ECU contract | AMS firmware + ECU/vehicle-controls owner |
| Estimator/SoP/SoH equivalence | firmware + controls/model owner |
| Vehicle profile/release | firmware, electrical, controls, test/safety authority |

### 13.3 Definition of done for a migrated subsystem

A subsystem is not “ported” when it compiles. It is done only when:

1. ownership and interfaces are documented;
2. no forbidden platform dependency leaks into the core;
3. static memory and failure behavior are explicit;
4. old and new outputs are compared with approved rules;
5. target electrical/timing behavior is measured where applicable;
6. fault injection covers timeout, stale, invalid, reset, and recovery paths;
7. stack/CPU/deadline data passes gates;
8. build/profile/diagnostics identify the running implementation;
9. documentation and test traceability are updated;
10. authority remains disabled until the program-level gate.

## 14. Cutover and rollback strategy

### 14.1 Parallel baseline

Keep the current FreeRTOS release buildable throughout the project. The portable core is shared; platform/application integration remains separate. Every accepted Zephyr milestone records the FreeRTOS comparison artifact and replay corpus version.

### 14.2 Cutover ladder

1. Zephyr on spare MCU breakout, outputs disconnected.
2. Zephyr on AMS board, BMS_OK/balance physically inhibited.
3. Passive sensors and one-SMB topology.
4. Five-SMB/APM ring with authority inhibited.
5. HIL with CAN/current/temp/IMD/charger models.
6. Safe-load balancing tests.
7. LV vehicle network, BMS authority still inhibited.
8. HV simulator/pack under approved safety process.
9. Controlled authority test.
10. Vehicle/dyno/endurance progression.

### 14.3 Rollback

- preserve the last approved FreeRTOS binary, source commit, toolchain container, calibration, and DBC;
- verify the physical flashing/recovery path before first Zephyr authority test;
- define rollback triggers: unexplained BMS output, missed deadline, data incoherence, CAN safety mismatch, reset-loop, stack breach, or hardware disagreement;
- after rollback, collect retained/log/analyzer evidence before power cycles overwrite context;
- do not maintain two authority-capable implementations indefinitely; set a formal acceptance point after which Zephyr is primary and FreeRTOS becomes archived recovery evidence.

## 15. Detailed go/no-go checklist

### Before writing Zephyr product code

- [x] Canonical runtime source versioning is unified in the 0.5.27 baseline; the eventual archived target artifact must still be checked against its manifest.
- [x] CubeMX/project identity is DER26 and regression-gated.
- [x] SPI CS_B mapping is frozen at PE4; the PF4 schematic annotation is treated as a labeling error, not an unresolved firmware pin conflict.
- [x] Vehicle CLI policy is explicit: diagnostic CLI/UART path is compiled out in the vehicle profile.
- [x] Qualified headless Release path explicitly records the AMS profile, matches CubeIDE Release optimization, requires deterministic `SOURCE_DATE_EPOCH` metadata, records immutable source-input/dependency/configuration provenance, and emits artifact hashes.
- [x] Exact bundled FreeRTOS source is pinned by SHA-256 while the conflicting upstream version labels remain documented.
- [ ] FreeRTOS DEBUG and RELEASE target baselines are built on the approved ARM toolchain and archived.
- [ ] Current task/IRQ timing, stacks, RAM/flash, CAN load, SPI, ADC, PWM, watchdog, and BMS fail-low are measured.
- [x] CAN bus-off policy is approved and regression-covered for START, CHARGE, DISCHARGE, BALANCE, repeated failure, service recovery, and CAN-backed HIL.
- [x] Watchdog feed policy is approved: software liveness/integrity only; external/process faults fail low without stopping IWDG feed.
- [ ] All 27 migration safety invariants and replay corpus are frozen, including current-window boundary, metadata carry, CAN per-reading aging, calibration provenance, effective SoP/SoH constituent ages, and non-backlogging AUX2 recovery cases.

### Before full hardware integration

- [ ] Custom board clock is measured at 216 MHz and all derived clocks are correct.
- [ ] BMS_OK cannot glitch high in any tested reset/fatal condition.
- [ ] Authority is compile-time and physically inhibited.
- [ ] ADC 480-cycle sampling is proven in the pinned Zephyr release.
- [ ] IMD capture implementation is selected and tested.
- [ ] ADBMS bus adapter has exact timing evidence.
- [ ] No unexpected DTCM/DMA/cache dependency exists.
- [ ] Core differential replay passes.

### Before bench-parity declaration

- [ ] H and L current paths are physically healthy and calibrated.
- [ ] Five-SMB/APM acquisition and diagnostic soak passes.
- [ ] Measurement snapshots and current windows are coherent under stress, including a current publication crossing the ADBMS voltage boundary.
- [ ] All safety fault/invalid/stale/timeout paths pass, including per-reading CAN expiry and zero/unknown current-uncertainty handling.
- [ ] CAN completion, congestion, bus-off, and recovery pass on target.
- [ ] IMD, fan, watchdog, retained log, and reset tests pass.
- [ ] All thread stacks, CPU, WCET, jitter, and IRQ-off gates pass.
- [ ] No unexplained FreeRTOS/Zephyr difference remains.

### Before vehicle-authority build

- [ ] Production Zephyr version is supported and frozen, preferably 4.6 LTS if schedule permits.
- [ ] All existing `AMS_PROFILE_VEHICLE` validation gates have real evidence.
- [ ] S-path, thermistor pull-ups, temperature thresholds, balancing, current, IMD, IWDG, fuse, SoP, CAN, mission, and model gates are closed.
- [ ] Whole-vehicle ECU/charger staleness and shutdown behavior pass.
- [ ] Independent safety/code review is complete.
- [ ] Release build contains no mutation/fault-injection path.
- [ ] Heap, shell, logging, assertions, and debug facilities match approved vehicle policy.
- [ ] Signed artifacts, manifest, hashes, calibration, DBC, SBOM, and vulnerability review exist.
- [ ] FreeRTOS rollback image and flashing procedure are verified.

## 16. Out of scope for the migration itself

The following should remain separate workstreams unless a safety defect forces action:

- retuning EKF covariance, adaptive measurement noise, or SoC/R0 models;
- changing SoP/SoH/fuse algorithms or thresholds;
- redesigning vehicle torque allocation or ECU/CM200 control ownership;
- changing CAN IDs/encoding for aesthetic reasons;
- increasing ADBMS SPI speed;
- enabling DMA;
- adding Zephyr userspace/MPU partitioning;
- adding secure boot/firmware update infrastructure;
- working around the physical H-current, S-path, thermistor, AIR-feedback, or connector issues in software;
- claiming certification solely from Zephyr adoption.

Each may be valuable later, but combining it with the kernel migration destroys attribution and expands the qualification surface.

## 17. Recommended immediate next actions

In order:

1. Freeze firmware 0.5.27 / package v2.6.24 as the comparison baseline. Archive the v2.6.21/v2.6.22 regression records, the v2.6.23 watchdog/CAN policy closeout, and the v2.6.24 final pre-Zephyr code-fix record and host validation with that baseline.
2. Promote the v2.6.19/v2.6.20 current-window, calibration-provenance, CAN per-reading aging, and uncertainty-sentinel cases; the v2.6.22 effective-age/AUX2 cases; and the v2.6.23 watchdog-policy cases and the v2.6.24 CAN ISR/event-time/499-500-501/service-refresh cases into the migration replay/parity corpus.
3. Create the approved DER26 pin/clock/peripheral manifest with SPI CS_B fixed at PE4; do not spend further migration work on the mislabeled PF4 schematic note.
4. Run the headless Debug and deterministic Release builds on the approved ARM toolchain and archive map, artifact hashes, provenance, stack-usage files, WCET/jitter/IRQ-off traces, BMS fail-low timing, CAN timing/load, SPI timing, ADC timing, and watchdog behavior.
5. Verify the ECU-side 300 ms stale-heartbeat torque removal and AMS-side 500 ms discharge bus-off fail-low timing on the real CAN network; verify IWDG feed/reset behavior on target.
6. Repair the H-current hardware path and keep other genuine physical gates in a separate visible tracker.
7. Approve all migration safety invariants and the replay corpus.
8. Then authorize Phase 1: a Zephyr 4.4.0 custom-board, no-authority safe skeleton. Hardware/evidence work may continue in parallel, but no parity/authority claim is allowed until its gate closes.

## 18. Source observations supporting key findings

| Observation | Location |
|---|---|
| Canonical firmware version is 0.5.27 in the updated baseline | `Core/Inc/ams_version.h:5-14` |
| Legacy `VER_*` aliases now derive from the canonical version header | `Core/Inc/app.h:32-34` |
| Build source revision derives from `AMS_SOURCE_REVISION` | `Core/Inc/ams_build_profile.h` build-manifest definitions |
| Vehicle CLI/UART/parser path is compile-time disabled by `AMS_ENABLE_CLI=0` | `Core/Inc/ams_build_profile.h`; `Core/Src/app.c`; `Core/Src/main.c`; `Core/Src/board.c`; `Core/Src/stm32f7xx_it.c` |
| Release build explicitly selects/records the AMS profile, matches CubeIDE Release optimization, requires `SOURCE_DATE_EPOCH`, records source-input/config/dependency provenance, and emits artifact hashes | `ci/stm32/build_ams_headless_gcc.sh` |
| Exact bundled FreeRTOS source is hash-pinned while V10.2.0/V10.2.1 label disagreement is documented | `host_tests/tools/check_freertos_provenance.py`; `docs/FREERTOS_PROVENANCE.md` |
| Current-window ordering/metadata fixes are part of the migration baseline | `Core/Src/measurement/ams_measurement.c`; `Core/Src/tasks/adbms_task.c`; `Core/Src/tasks/current_task.c` |
| CAN calibration provenance and per-reading aging are part of the migration baseline | `Core/Src/tasks/canbus_task.c` |
| SoP/SoH reject unknown current uncertainty as calibration evidence | `Core/Src/sop/ams_power_state.c` |
| SoP/SoH consume effective constituent cell/temperature ages at solve time; temperature has a separate healthy-scan-compatible freshness bound | `Core/Src/sop/ams_power_state.c`; `Core/Src/sop/ams_sop.c`; `Core/Src/soh/ams_soh.c` |
| AUX2 redundancy scheduling re-arms after transport loss and skips missed slots instead of replaying backlog | `Core/Src/tasks/adbms_task.c`; `Core/Inc/app.h` |
| IWDG software-liveness/integrity policy separates external/process fail-low from reset-worthy heartbeat/RTOS failures | `Core/Src/ext_drivers/ams_safety.c`; `Core/Inc/ext_drivers/ams_safety.h` |
| State-dependent CAN bus-off policy with ISR-captured event tick/state, immediate fail-low states, on-wire completion tick, strict pre-500 ms discharge deadline, separate service refresh, sticky latch, and START CAN-authority gate | `Core/Src/ext_drivers/canbus.c`; `Core/Src/tasks/error_task.c`; `Core/Src/tasks/canbus_task.c`; `Core/Inc/app.h` |
| ECU 300 ms changing-`0x680` heartbeat/torque timeout contract | `docs/AMS_ECU_CAN_CONTRACT.md` |
| CubeMX project name and project filename are both DER26 and are regression-gated | `DER26-AMS.ioc:225-226`; `host_tests/tools/check_release_identity.py` |
| Current rates/priorities/stacks | `Core/Inc/app.h:175-266` |
| Dynamic/static allocation, tick, heap, interrupt threshold | `Core/Inc/FreeRTOSConfig.h:55-95,126-147` |
| Startup fail-low before ordinary peripheral setup | `Core/Src/main.c:109-135` |
| Generated default task disabled but retained | `Core/Src/main.c:157-166` |
| All operational tasks are static | task start functions and `Core/Src/app.c:774-808` |
| Vehicle CLI startup/IRQ path is disabled independently of the service-mutation gate | `Core/Inc/ams_build_profile.h`; `Core/Src/app.c`; `Core/Src/main.c`; `Core/Src/board.c`; `Core/Src/stm32f7xx_it.c` |
| Only supervisor may assert BMS_OK | `Core/Src/app.c:832-896` |
| Vehicle physical/contract compile gates | `Core/Inc/ams_build_profile.h:444-555` |
| CAN RX ISR uses FreeRTOS ISR APIs | `Core/Src/ext_drivers/canbus.c:25-27` |
| ISR-API numeric threshold is priority 5 | `Core/Inc/FreeRTOSConfig.h:126-141`; CAN IRQ generated at priority 5 |
| Watchdog feed is blocked by software heartbeat/RTOS-integrity failures, not external/process faults | `Core/Src/ext_drivers/ams_safety.c` |
| ADC mappings and 480-cycle sample | `DER26-AMS.ioc:2-14,131-145` |
| CAN exact timing | `DER26-AMS.ioc:20-27`; `Core/Inc/app.h:51-53` |
| SPI/pin/clock mapping | `DER26-AMS.ioc:58-77,180-198,250-319` |
| CS_B firmware mapping PE4 | `Core/Inc/main.h:64-65`; CLI diagnostics explicitly flag PF4 as stale/conflicting |

### Appendix A — authored C-module migration disposition

This table accounts for every authored/generated C compilation unit under `Core/Src`. “Preserve” means reuse the behavior/source with only neutral interfaces; “adapt” means retain product logic behind Zephyr APIs; “replace” means the Cube/FreeRTOS implementation should not be carried into the Zephyr target.

| Current compilation unit | Disposition | Main review note |
|---|---|---|
| `app.c` | Incrementally decompose/adapt | Keep composition and authority intent; replace the broad shared root with owned snapshots over multiple phases. |
| `board.c` | Replace with board contract/adapters | Handle wiring becomes DTS/device bindings plus reviewed electrical semantics. |
| `main.c` | Replace | Zephyr entry/init model; preserve earliest fail-low/reset-cause behavior. |
| `freertos.c` | Retire | FreeRTOS/CMSIS initialization is not used by Zephyr. |
| `stm32f7xx_hal_msp.c` | Replace | Pin/clock/IRQ setup moves to DTS/pinctrl/Zephyr drivers. |
| `stm32f7xx_hal_timebase_tim.c` | Replace | Use Zephyr timing; provide explicit microsecond adapter for ADBMS. |
| `stm32f7xx_it.c` | Replace | Zephyr owns vector/driver interrupts; keep only approved custom ISR adapter if necessary. |
| `syscalls.c` | Replace/retire | Use pinned Zephyr libc configuration; no implicit heap/I/O assumptions. |
| `sysmem.c` | Retire | Vehicle design should not depend on `_sbrk`/dynamic heap. |
| `system_stm32f7xx.c` | Replace | Zephyr SoC/clock support; custom 216 MHz board configuration. |
| `tasks/error_task.c` | Adapt | Preserve supervisor rate and fault semantics; implement coherent snapshot and fail-low epoch. |
| `tasks/current_task.c` | Adapt | Preserve 50 Hz policy and dual-range behavior; inject ADC/time interface. |
| `tasks/adbms_task.c` | Adapt carefully | Preserve acquisition/balance state machine; make it sole bus owner. |
| `tasks/canbus_task.c` | Adapt carefully | Retain scheduler/publication semantics, same-snapshot current calibration quality, and per-reading cell/temperature aging; separate formatting/policy from driver. |
| `tasks/estimator_task.c` | Adapt | Preserve sequence-once consumption and outputs; thread wrapper only. |
| `tasks/fan_task.c` | Adapt | Preserve command policy; map to PWM adapter and safe state. |
| `tasks/air_task.c` | Keep disabled/adapt later | Only activate with genuine auxiliary contact hardware and reviewed monitor contract. |
| `tasks/imd_task.c` | Adapt | Event/capture input plus 10 Hz policy; exact timer proof required. |
| `tasks/cli_task.c` | Selectively replace | Port minimal evidence commands, not 7.7k lines wholesale; define vehicle absence/read-only policy. |
| `measurement/ams_measurement.c` | Preserve | Reuse double-buffer/pinning protocol and 0.5.27 current-window ordering/metadata semantics with injected short lock operations. |
| `estimator/ams_estimator_lut.c` | Preserve | Pure model data/interpolation; verify identical build inputs and numeric outputs. |
| `estimator/ams_soc_ekf.c` | Preserve | No RTOS rewrite or retune; differential replay and FPU validation. |
| `soh/ams_soh.c` | Preserve | Retain corrected episode aggregation and advisory semantics. |
| `sop/ams_fuse_observer.c` | Preserve | Pure model; retain randomized/strict oracle evidence. |
| `sop/ams_power_can.c` | Preserve | Byte-exact encoding and validity/fail-zero contract. |
| `sop/ams_power_state.c` | Preserve | State/topology behavior and zero/unknown current-uncertainty calibration contract remain common core. |
| `sop/ams_power_strategy.c` | Preserve | Battery capability ownership remains AMS; no controls redesign. |
| `sop/ams_sop.c` | Preserve | No model/calibration changes during migration. |
| `ext_drivers/accumulator.c` | Split/adapt | Keep topology/policy; move SPI/time/status to neutral ADBMS adapter. |
| `ext_drivers/adbms6830.c` | Preserve protocol, replace transport | Remove HAL status leakage and global buffers incrementally; exhaustive replay/logic-analyzer comparison. |
| `ext_drivers/adbms2950.c` | Preserve protocol, replace transport | Advisory APM first; same bus/status/context treatment as 6830. |
| `ext_drivers/adbms_shared.c` | Preserve | Keep common protocol helpers after platform dependency audit. |
| `ext_drivers/air_monitor.c` | Preserve algorithm, gate hardware | Do not infer physical contactor state from current common command signal. |
| `ext_drivers/ams_rtos_diag.c` | Replace | Implement Zephyr thread/stack/deadline diagnostics with equivalent evidence fields. |
| `ext_drivers/ams_safety.c` | Split/adapt carefully | Pure retained-record logic can stay; early/fatal GPIO, reset cause, watchdog, and kernel fault hooks become Zephyr/STM32 adapters. |
| `ext_drivers/can_tx_scheduler.c` | Preserve | Retain corrected fixed-storage priority/freshness policy; Zephyr driver adapter below it. |
| `ext_drivers/canbus.c` | Replace driver portion | Do not port HAL mailbox internals; implement filters/callbacks/recovery through Zephyr CAN API. |
| `ext_drivers/charger.c` | Preserve/adapt | Retain command/shutdown semantics; inject CAN/time output. |
| `ext_drivers/cli.c` | Replace/adapt minimally | Use Zephyr shell backend; retain only necessary line/protocol helpers. |
| `ext_drivers/current_fault.c` | Preserve | Pure validation/fault policy with existing unit coverage. |
| `ext_drivers/current_sensor.c` | Split | Preserve conversion/range/calibration logic; replace HAL ADC acquisition. |
| `ext_drivers/fans.c` | Split | Preserve temperature-to-duty behavior; replace timer-register/HAL output. |
| `ext_drivers/imd.c` | Split/adapt | Preserve signal interpretation; replace TIM capture plumbing. |
| `ext_drivers/main_fuse_monitor.c` | Preserve | Common computational core; retain oracle/replay evidence. |
| `ext_drivers/parallel_connection_observer.c` | Preserve | Common observer; no RTOS change needed. |
| `ext_drivers/stm32f767z.c` | Replace | HAL ADC/SPI/GPIO/time helpers become Zephyr adapters; avoid a new miscellaneous platform grab bag. |
| `ext_drivers/temperature_fault.c` | Preserve | Pure fault policy; thresholds remain separately gated. |
| `ext_drivers/thermistor_model.c` | Preserve | Generated/model contract and hardware calibration remain explicit. |
| `ext_drivers/voltage_fault.c` | Preserve | Raw-voltage safety behavior remains exact. |

## 19. Official Zephyr references

- [Zephyr releases and support dates](https://docs.zephyrproject.org/latest/releases/index.html)
- [Nucleo-F767ZI board support](https://docs.zephyrproject.org/latest/boards/st/nucleo_f767zi/doc/index.html)
- [Board porting guide](https://docs.zephyrproject.org/latest/hardware/porting/board_porting.html)
- [CAN controller documentation](https://docs.zephyrproject.org/latest/hardware/peripherals/can/controller.html)
- [CAN controller API](https://docs.zephyrproject.org/latest/doxygen/html/group__can__controller.html)
- [STM32 bxCAN driver source](https://github.com/zephyrproject-rtos/zephyr/blob/main/drivers/can/can_stm32_bxcan.c)
- [SPI API and terminology](https://docs.zephyrproject.org/latest/hardware/peripherals/spi.html)
- [ADC API](https://docs.zephyrproject.org/latest/hardware/peripherals/adc.html)
- [STM32 ADC driver source](https://github.com/zephyrproject-rtos/zephyr/blob/main/drivers/adc/adc_stm32.c)
- [Watchdog API](https://docs.zephyrproject.org/latest/hardware/peripherals/watchdog.html)
- [Threads](https://docs.zephyrproject.org/latest/kernel/services/threads/index.html)
- [Scheduling](https://docs.zephyrproject.org/latest/kernel/services/scheduling/index.html)
- [Message queues](https://docs.zephyrproject.org/latest/doxygen/html/group__msgq__apis.html)
- [Mutexes](https://docs.zephyrproject.org/latest/doxygen/html/group__mutex__apis.html)
- [Workqueues](https://docs.zephyrproject.org/latest/kernel/services/threads/workqueue.html)
- [Kernel timing and absolute timeouts](https://docs.zephyrproject.org/latest/kernel/services/timing/clocks.html)
- [System initialization levels](https://docs.zephyrproject.org/latest/doxygen/html/group__sys__init.html)
- [Shell](https://docs.zephyrproject.org/latest/services/shell/index.html)
- [Logging](https://docs.zephyrproject.org/latest/services/logging/index.html)
- [Coredump](https://docs.zephyrproject.org/latest/services/debugging/coredump.html)
- [Retained memory](https://docs.zephyrproject.org/latest/hardware/peripherals/retained_mem.html)
- [Ztest](https://docs.zephyrproject.org/latest/develop/test/ztest.html)
- [Twister](https://docs.zephyrproject.org/latest/develop/twister/index.html)
- [Native simulation](https://docs.zephyrproject.org/latest/boards/native/native_sim/doc/index.html)
- [West manifests and pinning](https://docs.zephyrproject.org/latest/develop/west/manifest.html)
- [Kconfig fragments](https://docs.zephyrproject.org/latest/build/kconfig/setting.html)
- [Task watchdog](https://docs.zephyrproject.org/latest/services/task_wdt/index.html)
- [Zephyr security overview](https://docs.zephyrproject.org/latest/security/security-overview.html)
- [Zephyr vulnerabilities](https://docs.zephyrproject.org/latest/security/vulnerabilities.html)
- [Security hardening tool](https://docs.zephyrproject.org/latest/security/hardening-tool.html)

## 20. Final recommendation

Approve a staged Zephyr investigation, not a product migration commitment. The STM32F767 and required peripherals are supported, the pure battery algorithms are portable, and the present safety/profile architecture gives a good starting point. The runtime/source version mismatch, code-fixable migration-readiness findings F-02/F-03/F-04/F-10/F-13, the v2.6.22 Release-profile/AUX2/SoP-SoH-freshness/profile-test regressions, safety-policy findings F-05/F-06, and the v2.6.24 final CAN event-time/service-recovery/tuning-CAN regressions are closed in the 0.5.27 baseline. The remaining formal gates are hardware, target timing/evidence, evidence reconciliation, memory/linker qualification, and target-artifact provenance. A custom-board, no-authority Zephyr Phase 1 skeleton may begin while those physical evidence items are being closed, but no bench-parity or vehicle-authority claim should be made until the applicable gates pass.

Do not make Zephyr responsible for solving existing hardware or calibration gaps, and do not combine the RTOS change with algorithm retuning. The success criterion is not that the firmware boots or that its tests compile. It is that the new image demonstrably preserves fail-low authority, coherent measurement, battery-state outputs, CAN contracts, watchdog intent, recovery behavior, deadlines, and diagnosability under real faults—with enough resource and lifecycle margin to maintain it.
