# Current Source Status

Updated: 2026-09-04

The current source revision is `DER26-AMS-v0.5.20-20260904` (package v2.6.17).
It includes the v2.6.16 five-SMB bench observer and the CAN scheduler fixes
documented in `AMS/docs/CAN_SCHEDULER_FIXES_2026-09-04.md`. This source package
requires a target rebuild and flash; no hardware CAN validation was performed
for the September 4 changes.

## Software review findings closed in the current tree

1. **CAN DETAIL stack usage** — encoded-frame staging is owned by the persistent
   `canbus_device_t.tx_builder`; measurement/tuning caches and task stacks use static
   storage. The scheduler now also writes directly into its owned generation
   slot, removing the separate oversized automatic generation temporary.
2. **CAN interrupt coverage** — CAN1 RX0, TX, and SCE NVIC vectors are enabled and
   dispatch through `canbus_irq_handler(&hcan1)` into HAL and refill only after
   completion dispatch, matching RX FIFO0, TX-mailbox, bus-off,
   and error notifications enabled by the driver. A host static gate checks this
   contract against the CubeMX `.ioc`, MSP, ISR declarations, and driver.
3. **Fatal RTOS fail-low ordering** — stack-overflow, malloc-failure, and assert paths
   execute the safety panic/fail-low action before best-effort diagnostic bookkeeping.
   A host static gate prevents regression.
4. **Runtime version/provenance mismatch** — the runtime/source revision is
   synchronized to v0.5.20 (`DER26-AMS-v0.5.20-20260904`).
5. **Estimator startup/covariance defects** — production now uses constrained/retryable
   acquisition, full symmetric 3x3 `[SoC,Vp1,Vp2]` covariance with Joseph update,
   covariance health guards, exact innovation variance, and fail-closed SoP authority
   while estimator acquisition remains unresolved.
6. **Live estimator observability gap** — passive logger protocol v4 carries the full
   covariance cross terms, covariance repairs, exact innovation sigma, and per-segment
   acquisition diagnostics.

## Evidence gates that cannot be closed by source edits alone

1. **Target stack qualification** — measure Cortex-M7 task stack high-water under
   worst-case CAN/ADBMS/estimator/CLI load. Host static allocation checks are not a
   substitute.
2. **Physical CAN qualification** — measure 1-Mbit/s utilization, protected response
   time, error margin/recovery, and all-node compatibility on the vehicle bus. Current
   source planning is conservative and remains below the AMS 10% planning gate.
3. **Estimator calibration** — freeze Q/R, acquisition current thresholds, confidence
   floors, and model mismatch only after licensed-MATLAB campaigns plus real DER26
   current/voltage/temperature logs.
4. **Fuse/thermal/aging correlation** — validate model parameters against installed
   component and aged-cell data; do not promote MiL agreement to physical
   qualification.
5. **Release artifacts** — a vehicle release still requires target ELF/HEX/MAP,
   toolchain/configuration manifest, hardware evidence, and the normal build-profile
   validation locks.
6. **ECU SD decoder parity** — the raw `CAN###.BIN` contract is external to this AMS
   repository. The ECU decoder must be updated/verified for passive logger protocol v4
   before derived CSVs are treated as complete, while immutable raw frames remain
   replayable.

See `MiL/WORKING_STATUS.md`, `MiL/TEST_EVIDENCE_2026-08-29.md`, and
`AMS/docs/AMS_TUNING_CAN_SD_CONTRACT.md` for the current qualification boundary.
