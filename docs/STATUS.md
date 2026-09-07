# Current Source Status

Updated: 2026-09-06

The current source revision is `DER26-AMS-v0.5.30-20260906` (package v2.6.27).
It retains the v2.6.17 CAN scheduler fixes, v2.6.19 current-window/measurement
integrity fixes, v2.6.20 follow-up aging/uncertainty fixes, v2.6.21
migration-readiness closeout, and v2.6.22 post-closeout regression fixes.
v2.6.23 closes the remaining watchdog and CAN bus-off safety-policy decisions.
v2.6.24 closes the final pre-migration review findings: physical BOFF is now
timestamped/classified in the CAN ISR; CHARGE/BALANCE/CAN-backed-HIL fail-low is
physical-event immediate; DISCHARGE recovery uses actual on-wire completion time
with exact 499/500/501 ms and tick-wrap semantics; service authority refresh is
separate from a physical bus-off epoch; and the tuning-CAN feature gate is common
to estimator producer and CAN consumer with `-Wundef` whole-source enforcement.
v2.6.25 hardens that policy by refusing to credit a protected completion until
transport recovery has settled the old controller epoch and advanced the authority
baseline, routes bench CAN-bus-off injection through the same event path as physical
BOFF, refreshes repeated identical soft-error age, and preserves repeated-bus-off
state when manual recovery fails. v2.6.26 adds SCE-owned physical BOFF identity,
ISR-side event sequencing, and the exact three-event/10-second sliding latch so
clustered/recovery-pending events cannot collapse. v2.6.27 adds final ISR/task
visibility hardening, charger ENABLE revalidation at the bxCAN hardware-load
boundary, sequence-stable recovery settlement, and atomic transport/application
service-recovery commit. Battery model parameters and estimator equations are not
retuned.

No ARM target binary was built or flashed for v2.6.27 in the review environment.
The final v2.6.27 regression suites were intentionally not rerun during packaging per
user request; this is therefore a software-freeze candidate until those host gates pass.
See `AMS/docs/PRE_ZEPHYR_FINAL_SOFTWARE_FREEZE_CANDIDATE_v2.6.27.md`.

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
4. **Runtime version/build provenance** — the runtime/source revision is
   synchronized to v0.5.30 (`DER26-AMS-v0.5.30-20260906`); qualified headless
   Release builds explicitly select profile 5 by default, match CubeIDE Release
   optimization (`-Os -g0`), require a controlled epoch, and emit profile,
   source-input-tree, dependency, toolchain, and artifact hashes.
5. **Estimator startup/covariance defects** — production now uses constrained/retryable
   acquisition, full symmetric 3x3 `[SoC,Vp1,Vp2]` covariance with Joseph update,
   covariance health guards, exact innovation variance, and fail-closed SoP authority
   while estimator acquisition remains unresolved.
6. **Live estimator observability gap** — passive logger protocol v4 carries the full
   covariance cross terms, covariance repairs, exact innovation sigma, and per-segment
   acquisition diagnostics.
7. **Migration-readiness source findings** — DER26 Cube identity and CAN ISR
   documentation/IRQ gates are correct; vehicle CLI/UART startup is independently
   source-gated off; exact bundled FreeRTOS bytes are provenance-pinned. The SPI
   String-B chip-select remains PE4; the PF4 schematic annotation was a label error.

8. **Post-closeout regression fixes** — the qualified headless build cannot
   silently fall back to BENCH; AUX2 diagnostic cadence re-arms after transport
   loss instead of replaying missed slots; SoP/SoH reject stale constituent data
   using effective cell/temperature ages at solve time; and negative authority
   profile tests prove they fail for the intended compile-time policy diagnostic.
9. **Safety-policy closeout** — IWDG feed is now tied to software liveness and
   RTOS integrity rather than external/process faults. CAN bus-off is state-dependent:
   START requires a fresh required protected `0x680`–`0x687` generation to complete
   on wire before authority, charge/balance and CAN-backed HIL fail low immediately,
   discharge gets a 500 ms bounded recovery
   epoch after the ECU 300 ms torque-staleness deadline, and repeated bus-off/TX
   inhibit hard-latches communication authority.
10. **Final freeze hardening** — discharge bus-off completion evidence is accepted
    only after the old CAN controller epoch has been settled and the authority baseline
    advanced; physical and bench bus-off injection share one event path; repeated
    identical soft CAN errors refresh their age; and failed manual recovery preserves
    the repeated-bus-off latch/window.
11. **Final ISR/task ownership hardening candidate** — SCE-written CAN/BMS safety
    scalars and CAN-completion charger status have explicit cross-context visibility;
    charger ENABLE is revalidated at the bxCAN load boundary; recovery settlement is
    tied to one BOFF sequence; and service recovery commits transport plus application
    CAN-safety state atomically so a post-commit BOFF cannot be administratively erased.

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
