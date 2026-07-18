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

## Remaining architectural limitation

`app_data_t` is still a broad composition root. Related current, voltage,
temperature, IMD, and AIR fault publications are protected, but telemetry may
legitimately combine samples from different acquisition instants. A future
architecture should replace flattened cross-subsystem reads with immutable,
timestamped subsystem snapshots and one supervisor input snapshot. That is a
larger behavior-preserving migration, not a prerequisite for the targeted
hardening in this revision.
