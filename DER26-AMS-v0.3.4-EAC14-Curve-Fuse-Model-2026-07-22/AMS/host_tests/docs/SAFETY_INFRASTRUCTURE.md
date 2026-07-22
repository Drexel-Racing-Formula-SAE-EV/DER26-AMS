# AMS Safety Infrastructure Additions

Authored by Mahad Faisal, 2026.

This note documents the firmware safety-infrastructure layer added after the
bench/HIL reviewed baseline. It does not change voltage, temperature, or current
threshold policy.

## Included

| Area | Behavior |
|---|---|
| CPU panic path | NMI, HardFault, MemManage, BusFault, UsageFault, unexpected Default_Handler IRQs, Error_Handler, assert failure, scheduler-return, task-create failure, and libc `_exit()` force BMS_OK low through a direct GPIO reset path before entering the fault loop. |
| Early BMS_OK low | BMS_OK is forced low immediately after `HAL_Init()` using the same direct GPIO path, before the normal GPIO init sequence. |
| CPU fault enables | MemManage, BusFault, and UsageFault traps are enabled after HAL init. |
| Reset cause | RCC reset flags are recorded at boot and cleared after capture. |
| Panic record | A `.noinit` RAM record keeps the last panic reason plus CFSR/HFSR/MMFAR/BFAR and a panic counter. |
| Watchdog | Optional IWDG support is compile-time gated by `AMS_ENABLE_IWDG`; runtime feed is controlled by the error task only. |
| Watchdog gate | Feeding is blocked during startup grace, panic, hard faults, stale heartbeat, invalid/faulted voltage, invalid/faulted current, invalid/faulted temperature, charger fault, fuse fault, or ADBMS diagnostic fault. |
| CAN bus-off | CAN HAL errors are polled, bus-off is counted, recovery is delayed, failed recovery is throttled, and CAN is restarted through Stop/ResetError/Start/ActivateNotification. In ADBMS-image HIL mode, bus-off/recovery-pending also holds the ADBMS diagnostic fault so stale injected images cannot briefly reassert BMS_OK. |
| CAN soft-error hold | Non-bus-off CAN errors remain visible as a soft CAN fault for a short hold window instead of being overwritten immediately by a clean transmit pass. |
| CAN diagnostics and pacing | Logger frame `0x69C` exports CAN error/recovery state. Frames `0x69D..0x69F` export safety reset/panic state, watchdog feed-gate state, and ADBMS diagnostic counters/flags. Frames `0x6A6..0x6A7` add current ADC and charger-command detail. Slow detail traffic is phased, critical charger traffic precedes compact traffic, compact failure suppresses detail, and `can diag` reports per-class attempts/failures plus task deadline misses. |
| Coherent measurement epoch | The ADBMS owner publishes voltage, aligned current window/integral, temperature values/ages, balance recovery and transport health through a reader-pinned static double buffer. A consumer never copies the large object while interrupts are disabled, and a producer drops rather than overwrites a pinned buffer. |
| Current calibration record | A fixed-width versioned record carries signed offsets, reference rails, ID/time, calibration temperature, quantified uncertainty and a field-wise CRC. Restore requires a valid record, fresh plausible raw channels, live offsets consistent with the stored board-specific offsets, and an explicit zero-current proof. The estimator only treats hardware current calibration as SoH-confident when the restored record, physical calibration-procedure gate, and separate polarity gate are all valid. |
| State transition boundary | All runtime state writes pass through one guarded boundary. It records previous/current state, reason, tick and a saturating count; blocks BMS_OK across synchronous balance cleanup; and contains invalid state values as `STATE_ERROR`. CI rejects new direct state writers. |
| Charge exit | Leaving `STATE_CHARGE` requests three prioritized zero-demand charger-disable frames from the CAN owner. Failed queue operations are retried and remain a blocking charger fault. A queued frame is explicitly not treated as end-to-end charger acknowledgement. |
| Fault log | A 32-entry `.noinit` RAM ring records boot, reset cause, panic, BMS_OK and application-state transitions, voltage/temp/current latch transitions, ADBMS diagnostic failures, CAN bus-off/recovery, and watchdog feed-stop events. |
| Fault injection | Destructive test commands are present only when compiled with `AMS_FAULT_INJECTION_CLI=1`. |

## Intentionally Not Included Yet

| Area | Reason |
|---|---|
| Physical IMD enablement | The TIM2 capture/freshness path exists but remains disabled and fail-closed by default until the external PWM/status wiring, pull network, polarity, scaling and fault cases are bench-validated. |
| Fuse-fault producer | The supervisor gate exists, but no confirmed protected fuse-monitor hardware signal currently produces `fuse_fault`. |
| AIR auxiliary-contact enablement | The complete monitor/evaluator exists behind a fail-closed build gate, but the current PCB lacks the reviewed AIR+/AIR-/precharge auxiliary inputs and board adapter. |
| Formal service fault-clear policy | Deferred until the team decides the final inspection/service reset workflow. Existing threshold latch behavior is unchanged. |
| Flash fault logging | Deferred to avoid flash-wear/noise during bring-up. Current log is RAM-only. |
| Current-calibration storage adapter | Record creation, validation and fail-closed restore exist, but no present firmware path writes calibration to flash/EEPROM. Board-specific allocation, redundant/wear-managed writes, power-loss testing and the physical zero-current procedure remain release work. |

## CLI

```text
fault resetcause
fault panic
fault log
fault log clear
can diag
can recover
wdg status
wdg enable
```

`wdg enable` has no hardware effect unless the firmware is compiled with
`AMS_ENABLE_IWDG=1`. Keep this disabled during first ADBMS/SPI bench probing.

Fault-injection commands are available only when `AMS_FAULT_INJECTION_CLI=1`:

```text
fault inject hardfault
fault inject busfault
fault inject canbusoff
wdg stopfeed
wdg feedok
```

Do not ship competition firmware with fault injection enabled.
