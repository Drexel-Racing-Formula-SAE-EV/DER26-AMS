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
| CAN diagnostics | Logger frame `0x69C` exports last CAN error, bus-off count, error count, recovery count, and recovery flags. |
| Fault log | A 32-entry `.noinit` RAM ring records boot, reset cause, panic, BMS_OK transitions, voltage/temp/current latch transitions, ADBMS diagnostic failures, CAN bus-off/recovery, and watchdog feed-stop events. |
| Fault injection | Destructive test commands are present only when compiled with `AMS_FAULT_INJECTION_CLI=1`. |

## Intentionally Not Included Yet

| Area | Reason |
|---|---|
| IMD/BSPD/fuse software gating | Requires one more schematic and harness confirmation before firmware should gate BMS_OK on these signals. |
| Formal service fault-clear policy | Deferred until the team decides the final inspection/service reset workflow. Existing threshold latch behavior is unchanged. |
| Flash fault logging | Deferred to avoid flash-wear/noise during bring-up. Current log is RAM-only. |

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
