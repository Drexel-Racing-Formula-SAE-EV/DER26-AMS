# Concurrency and publication contract

This firmware targets one STM32F767 Cortex-M7 core running preemptive
FreeRTOS. The rules below are part of the target contract; the CAN ring in
particular is not intended to be portable standard-C lock-free code.

## Safety-output ownership

- The error/safety supervisor is the only normal context permitted to assert
  `BMS_OK`.
- Any fault, panic, or early-start context may force `BMS_OK` low.
- A low-to-high transition is evaluated while the supervisor holds the short
  safety critical section. Measurement producers publish their related fault
  fields under short critical sections so the supervisor cannot see half of a
  producer update.
- Interrupts and tasks outside the supervisor must never call an assertion
  path or directly set the output GPIO high.

## ADBMS/isoSPI ownership

- Complete ADBMS operations are serialized by the statically backed recursive
  mutex in `app.c`; the transaction lock covers wake, command construction,
  shared buffers, chip select, transfer, parsing, and readback.
- The periodic ADBMS task is the production owner. Service CLI diagnostics may
  request operations only in an explicitly enabled bench profile and use the
  same operation lock.
- The owner holds that lock through immutable measurement-snapshot publication,
  not only through the physical transfer. In CAN-fed ADBMS HIL builds, each
  injected cell/temperature triplet uses the same lock, so a HIL frame cannot
  modify the accumulator image halfway through an epoch copy.
- Lock and timer waits are bounded. Failure is propagated into the fail-closed
  diagnostic path rather than continuing with a stale shared buffer.

## CAN receive SPSC ring

- Producer: only `HAL_CAN_RxFifo0MsgPendingCallback()` for the configured CAN1
  instance.
- Consumer: only the CAN task through `canbus_process_rx_queue()`.
- The ISR writes every frame field, executes a Cortex-M data-memory barrier,
  and then publishes `rx_queue_head`.
- The task samples the published head, executes a data-memory barrier before
  copying the frame, then advances `rx_queue_tail` after another barrier.
- Head is written only by the ISR and tail only by the task. Adding another
  producer or consumer invalidates this design and requires a FreeRTOS queue or
  another reviewed synchronization mechanism.
- The ISR does not call FreeRTOS and therefore does not depend on the
  `configMAX_SYSCALL_INTERRUPT_PRIORITY` API restriction. Its NVIC priority
  must nevertheless remain documented and must not permit two producers for
  the same ring.
- `volatile` prevents compiler elision of the indices; `__DMB()` supplies the
  target ordering. This is a single-core Cortex-M contract, not a claim of C11
  atomic portability.

## Capture and retained-state publication

- IMD capture uses an odd/even sequence counter. The ISR publishes total count,
  high count, and timestamp as one versioned sample; the task rejects an
  in-progress or changed sequence.
- The retained `.noinit` fault ring disables interrupts for normal writers.
  Each record is invalidated first, receives a monotonically ordered sequence
  and CRC, and publishes its commit word last. Boot/snapshot recovery discards
  uncommitted or corrupt entries and reconstructs header progress from valid
  records.
- Hard-fault/NMI logging does not depend on the scheduler, HAL, heap, UART, or
  mutexes after forcing `BMS_OK` low.

## Measurement-epoch publication

- The current task owns sample updates to
  `ams_current_window_accumulator_t`; the ADBMS task rotates the accumulator at
  the voltage boundary. Both operations hold the current-data mutex, so they
  cannot modify the window concurrently. Timestamped valid samples are
  integrated while gaps or invalid samples are recorded rather than extending
  stale current indefinitely. The same window latches the calibration-record
  ID and confidence used by those samples; a missing or mid-window changed
  record keeps current data usable but rejects resistance-SoH confidence for
  that epoch.
- DHAB conversion/calibration state, current-window update, and ADBMS-boundary
  rotation share a separate statically allocated, priority-inheriting task
  mutex. The current task owns normal sampling; bench-only zero commands use
  the same boundary and cannot alter offsets/references halfway through a
  sample. Its acquire is bounded and a lock failure takes the existing
  panic/fail-low path; ADC and floating-point work are never performed with
  interrupts disabled.
- Any authorized service calibration mutation invalidates both the active
  integration epoch and scalar current readiness. The current task must run a
  fresh conversion/fault-policy publication before readiness can return.
- The ADBMS task rotates that accumulator at the completed voltage boundary
  and builds one `ams_measurement_snapshot_t` containing the voltage epoch,
  current interval/integral, newest temperature values and ages, balance state,
  transport validity, and publication timestamp.
- Publication uses two static buffers. A reader pins the published buffer in a
  short critical section, copies outside the critical section, then releases
  the pin. The producer never overwrites a pinned buffer; it drops that epoch
  and increments `publication_drop_count` instead. Every attempted epoch
  consumes a sequence number, so a later successful publication exposes the
  gap to consumers.
- The estimator consumes each published sequence at most once and derives its
  propagation interval from consecutive voltage-completion timestamps. It
  also rejects an otherwise readable snapshot once its publication age
  exceeds the hardware-input timeout.
- Its large snapshot-copy buffer is static and task-owned; it is not allocated
  on the 4 KiB estimator task stack. The compile-time snapshot/store ceilings
  prevent silent RAM growth, while target map and stack high-water evidence
  remain release gates.
- HIL measurement, truth, and summary frames are first decoded into local
  values and then assigned under one short critical section. The HIL estimator
  similarly copies one complete measurement frame before testing freshness or
  executing a step; it does not combine members from two CAN epochs.

## CAN publication

- Charger shutdown and the periodic charge-state charger command are attempted
  before compact telemetry. Best-effort detail traffic is suppressed when the
  compact bundle fails.
- Slow logger/detail traffic is phased across fast CAN cycles rather than sent
  as one approximately 187-frame burst.
- Each compact bundle freezes the non-measurement application fields once;
  electrical/thermal data come from one immutable measurement snapshot.
- Before the first measurement epoch, or after its age exceeds the CAN input
  timeout, the task transmits an explicitly invalid zeroed measurement view;
  it never falls back to traversing live ADBMS/current storage.
- Per-class attempt/failure/suppression counters plus task duration and
  deadline-miss counters are saturating and visible through `can diag`.

## Remaining architectural limitation

`app_data_t` is still a broad composition root. The estimator and compact CAN
path no longer traverse live ADBMS/current storage, but IMD, AIR, charger,
heartbeat and all supervisor inputs are not yet composed into one versioned
system snapshot. Safety producers publish their related fields atomically, so
the current supervisor path remains fail-closed, but removing the duplicated
flattened fields and introducing one immutable supervisor input snapshot is
still the main architectural migration left after this revision.
