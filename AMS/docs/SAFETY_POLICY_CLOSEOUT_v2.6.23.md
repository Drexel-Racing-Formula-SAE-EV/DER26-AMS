# DER26 AMS v2.6.23 safety-policy closeout

**Firmware:** 0.5.26  
**Package:** v2.6.23  
**Date:** 2026-09-05

This release implements the two safety-policy decisions that remained open after the v2.6.22 code review. It does not retune the estimator, SoP, SoH, fuse model, current calibration, ADBMS measurement timing, or the approved SPI String-B PE4 mapping.

## F-05 — IWDG semantics closed

IWDG is now explicitly a **software-liveness/integrity watchdog**, not a generic battery/process-health timer.

The supervisor continues feeding IWDG while software execution is healthy even when an external/process fault has already forced BMS_OK low, including voltage, temperature, current, charger, CAN, ADBMS, and fuse faults. This avoids reset loops that cannot repair the physical condition and preserves diagnostic/fault communication.

IWDG feed stops for reset-worthy software failures: panic/fatal paths, missed critical-task heartbeat/safety-heartbeat staleness, explicit bench stop-feed injection, failed watchdog start, and RTOS integrity/resource failure (`rtos_fault` or critical stack margin). Existing fatal RTOS hooks still fail BMS low before bookkeeping.

## F-06 — CAN bus-off policy closed

The CAN policy is state dependent:

- **START:** CAN authority begins false. START cannot transition into an authority-capable operating state until the controller is clean and every required frame in a fresh protected `0x680`–`0x687` generation has completed on the wire.
- **CHARGE:** bus-off immediately latches a hard CAN policy fault and forces BMS_OK low.
- **BALANCE:** bus-off immediately latches a hard CAN policy fault and forces BMS_OK low.
- **HIL with CAN replacing ADBMS:** bus-off immediately latches a hard fault because the measurement source is lost.
- **DISCHARGE:** the first transient bus-off starts one continuous recovery epoch. The ECU contract requires torque/inverter authority to expire by **300 ms** without a fresh changing `0x680` heartbeat. If a fresh required protected `0x680`–`0x687` generation has not completed on the wire by **500 ms**, AMS latches a hard CAN fault and forces BMS_OK low.
- **Repeated bus-off:** three bus-off events inside the existing **10 s** window, or the application TX inhibit latch, immediately become a sticky hard CAN policy fault.

Electrical ABOM recovery alone does not restore CAN authority. Software acceptance/queueing of a protected generation is also insufficient. `can_authority_ready` is set only after mailbox completion accounting proves that every required frame in a fresh protected `0x680`–`0x687` generation completed on the wire in the new controller epoch.

The hard CAN policy latch does not auto-clear. In the vehicle profile the service CLI is compiled out, so recovery is effectively by approved reset/service procedure. In a service-capable bench image, `can recover` may clear the sticky latch, but it starts a new bounded recovery epoch and does **not** grant CAN authority; a subsequent fresh required protected generation must complete on the wire to re-establish it.

## Regression coverage

Host tests cover:

- external/process faults continue IWDG feeding while heartbeats are healthy;
- heartbeat and RTOS-integrity failures stop IWDG feeding;
- START remains fail-low until a fresh required protected CAN generation completes on the wire;
- transient DISCHARGE bus-off remains non-hard before 500 ms and becomes hard exactly at 500 ms;
- the 500 ms boundary remains correct across 32-bit tick wrap;
- software queue/commit acceptance without required on-wire completion does not grant authority;
- a late pre-recovery completion is absorbed into the recovery baseline and cannot masquerade as fresh post-recovery delivery;
- ABOM electrical recovery alone does not restore authority;
- service recovery does not grant authority before a new required protected generation completes on the wire;
- BALANCE and CHARGE bus-off fail low immediately;
- HIL ADBMS replacement bus-off fails low immediately;
- three bus-offs in 10 s produce the sticky repeated-bus-off latch.

Validated host targets after the change: `unit`, `test`, `safety-test`, `hil-adbms-test`, `can-tx-regression-test`, `can-tx-scheduler-test`, `profile-gates`, `measurement-integrity-test`, `review-fixes-test`, and `whole-source-analyze` (bench and vehicle profiles).

No ARM target build/flash, physical CAN fault injection, watchdog reset timing measurement, or new hardware validation is claimed by this source-only closeout.
