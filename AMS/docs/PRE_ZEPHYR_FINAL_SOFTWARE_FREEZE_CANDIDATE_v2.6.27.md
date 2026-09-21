# DER26 AMS v2.6.27 final pre-Zephyr software-freeze candidate

**Firmware:** 0.5.30
**Package:** v2.6.27
**Date:** 2026-09-06

This candidate incorporates the final ISR/task concurrency and service-recovery hardening found after the v2.6.26 repeated-bus-off closeout. It does not change battery-model parameters, estimator equations, CAN IDs/encoding, or the approved SPI String-B PE4 mapping.

## Changes since v2.6.26

1. **CAN safety-state ISR/task visibility is explicit.** Scalar CAN/BMS/state fields written directly by SCE bus-off handling are volatile so task/supervisor reads cannot be optimized into stale observations. Compound mailbox/scheduler state remains protected by critical sections rather than being blanket-marked volatile.
2. **Charger completion ownership is explicit.** CAN TX-completion-owned charger status/counter fields are volatile for ISR/task visibility while shutdown request identity and compound state continue to use critical sections and request IDs.
3. **Charger ENABLE is revalidated at the actual bxCAN load boundary.** A normal charger frame built while CHARGE was valid is converted to zero-voltage/zero-current DISABLE if state/BMS/shutdown/failure authority changed before the mailbox TXRQ write. Task-context revalidation and mailbox load are one short critical section, closing the reproduced scheduler-commit-to-hardware-load stale-enable race.
4. **CAN diagnostic ISR/task counters are explicitly shared.** ISR-written scalar load/completion/abort/pump diagnostics are volatile, while safety-relevant scheduler/mailbox compound state retains critical-section ownership.
5. **Recovery settlement is sequence-transactional.** Task recovery settles a specific BOFF event sequence and refuses commit if a newer SCE event appears during HAL state/hardware checks.
6. **Manual/service recovery commits transport and application CAN-safety state atomically.** `canbus_recover()` now receives `app_data_t *` and performs latch/window clear, CAN fault state, authority baseline, recovery/refresh state, policy latch, and recover count inside the same final critical transaction that proves no newer BOFF occurred. The CLI performs no post-return fault/latch cleanup, eliminating the race where a BOFF arriving after successful transport recovery could be erased by delayed administrative cleanup.
7. **Focused regression corpus is extended to 19 CAN transport tests.** The added source regression models a new BOFF immediately after successful service-recovery commit and verifies that it survives because no later CLI cleanup exists.
8. **Documentation baseline is synchronized.** The stale STATUS reference that paired “v0.5.28” with the 0.5.29 revision string is removed; this candidate is consistently identified as firmware 0.5.30 / package v2.6.27.

## Validation status

Per user request, **the regression suites were not rerun after the final v2.6.27 source edits/package preparation**. The v2.6.26 interrupted working tree already contained prior passing evidence for the repeated-bus-off sequence fix, charger hardware-load revalidation, comprehensive host safety/injection, HIL/ADBMS resilience, profile/contract gates, sanitizer/stress runs, GCC whole-source analysis, and focused Clang analysis. That historical evidence is not promoted to v2.6.27 certification.

Before freezing this candidate as the Zephyr comparison baseline, rerun at minimum:

```bash
cd AMS/host_tests
make can-tx-regression-test
make can-tx-scheduler-test
make safety-test
make test
make measurement-integrity-test
make profile-gates
make review-fixes-test
make can-v4-contract-gate can-irq-contract-gate can-clock-contract-gate
make adbms-v05-contract-gate
make whole-source-analyze
```

Recommended final confidence runs are `make firmware-ci`, `make firmware-asan`, `make ubsan`, and `make stress` if time permits.

No ARM target build, flash, physical CAN injection, watchdog timing measurement, or hardware validation is claimed.
