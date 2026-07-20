# Gate-0 re-review quick-fix

Baseline archive revision: `958bf892b21ebd209171db3f13100acdf75f8540`

This bounded patch implements the software-only corrections from
`DER26_AMS_Pre_SoP_SoH_Firmware_ReReview_958bf89_2026-07-20`. Thermistor
model work was explicitly excluded.

## Implemented

- Enabled bxCAN request-order transmit priority and added a target-project
  gate that rejects a regression to arbitration-order mailbox selection.
- Added acquisition start, end, midpoint, duration, and conservative
  high/low-channel skew-bound diagnostics for sequential DHAB sampling.
- Added a measurement-epoch timing-valid flag. R0/SoH observation acceptance
  now requires acquisition duration at or below 10 ms.
- Added every epoch-specific voltage and temperature diagnostic mask to the
  immutable measurement snapshot. Phased logger frames now use those masks
  instead of mixing snapshot values with live accumulator state.
- Temperature confirmation now advances from measured elapsed tick time,
  capped at 1000 ms for pathological gaps.
- Replaced scan-count diagnostic scheduling with wrap-safe tick deadlines for
  status, configuration, and open-wire diagnostics. Added execution-duration
  and lateness metrics.
- Added an ownership-checked measurement writer abort operation and recovery
  tests.
- Renamed the SoH observation-count percentage from `confidence` to
  `maturity` so it is not mistaken for a complete uncertainty metric.

## Deliberately deferred

- Calibration A/B flash persistence: requires an approved STM32 flash layout,
  endurance policy, board/revision identity contract, and power-loss tests.
- Simultaneous timer-triggered dual-ADC/DMA sampling: requires target/CubeMX
  and hardware validation. The new duration is an upper bound on channel skew,
  not proof of simultaneity.
- Signed discharge/charge/regen polarity: requires physical current-direction
  evidence and an agreed ECU/inverter contract.
- CAN on-wire priority and deadline proof under bus saturation.
- Thermal SoP hottest-location policy and all target WCET/stack/HIL evidence.

## Focused validation

- Comprehensive host injection/SIL suite: passed.
- Isolated AMS unit suite: passed.
- CubeIDE target-project/CAN-priority gate: passed.
- GCC static analyzer: passed.

These are host/source results. They do not replace ARM target compilation,
CAN-analyzer evidence, current-polarity validation, or physical fault testing.
