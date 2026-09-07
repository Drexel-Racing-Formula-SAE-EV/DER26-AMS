# DER26 AMS v2.6.26 final repeated-bus-off closeout

**Firmware:** 0.5.29  
**Package:** v2.6.26  
**Date:** 2026-09-06

This closeout fixes the remaining repeated-CAN-bus-off event accounting defect found in the final pre-Zephyr review.

- Physical BOFF identity is captured from CAN1 SCE before the HAL handler accumulates sticky error state.
- Every genuine event increments an ISR-side event sequence. The CAN task consumes sequence deltas rather than one pending boolean.
- The repeat policy is an exact sliding window of the latest three events. The third event at or within 10,000 ms hard-latches application TX and BMS fail-low immediately.
- Additional BOFFs during a pending recovery are not discarded and do not restart the original 500 ms discharge deadline.
- HAL ErrorCallback may keep TX suspended for a sticky BOF code, but it does not create target-side physical event identity.
- The legacy host harness, which has no CAN register instance/SCE interrupt, treats its direct HAL callback as one physical-equivalent event for regression parity.

Validation completed in the review environment: `make test`, `make review-fixes-test`, `make can-tx-regression-test`, `make can-tx-scheduler-test`, `make profile-gates`, `make measurement-integrity-test`, and `make whole-source-analyze` all pass. No ARM target build, flash, or physical CAN injection is claimed.
