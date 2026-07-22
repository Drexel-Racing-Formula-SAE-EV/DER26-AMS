# SoP/SoH commissioning and release gates

## Scope

This procedure moves the source from output-inhibited development through
measured target evidence and finally to ECU torque limiting. No single test
authorizes vehicle torque. Record date, operator, hardware serials, firmware
commit, build manifest, calibration revisions, instruments, raw logs, and pass
or fail for every gate.

## Gate 0: source and build identity

- Build the exact source commit with no local modifications.
- Capture `ver`, manifest schema 4, build profile, feature flags, commit,
  current-calibration revision, CAN
  contract revision, threshold revision, estimator revision, SoP revision, and
  SoH revision from the running target.
- Confirm service CLI, HIL injection, ADBMS replacement, and hardware bring-up
  are disabled for the vehicle image.
- Confirm vehicle compilation fails unless all SoP/SoH evidence macros and
  revision strings are supplied.
- Confirm `AMS_MISSION_CAN_CONTRACT_VALIDATED` and
  `AMS_FUSE_MODEL_VALIDATED` remain zero until their dedicated tests pass.
- Archive ELF, map, HEX/BIN, compiler version, linker script, and build log.

Pass criterion: the runtime manifest exactly matches the reviewed release
record and the profile gates reject an unacknowledged vehicle build.

## Gate 1: target resource evidence

The estimator task has a 1536-word (6144-byte on Cortex-M7) static stack. Host
GCC reports large individual frames because the complete five-segment input and
solver state are intentionally stack-local; host measurements are not ARM
proof.

On an STM32F767 release build:

1. Enable DWT CYCCNT instrumentation around `estimator_update_power()` without
   changing task priorities or optimization.
2. Measure boot, nominal, weak-cell, hot-cell, high-SOC charge, low-SOC
   discharge, invalid input, all-current-ceiling, and worst full-30-second-model
   cases.
3. Repeat under peak CAN RX/TX, ADBMS, current, error-task, and interrupt load.
4. Capture minimum FreeRTOS stack high-water mark over at least the longest
   planned bench/HIL run.
5. Verify no malloc, stack-overflow, assert, watchdog block, missed estimator
   heartbeat, or task deadline.

Acceptance criteria:

- worst measured SoH+SoP execution less than 15 ms, including instrumentation
  uncertainty, with at least 2x margin to the 100 ms period;
- estimator stack high-water never enters the configured 96-word warning band
  and retains at least 25% of allocated stack;
- no timing-induced stale power snapshot or CAN bundle.

If these criteria fail, reduce verified model cost or increase static resources
after whole-system memory review. Do not skip prediction checks.

## Gate 2: current calibration and sign

Use an isolated bidirectional calibrated current reference through the installed
sensor and conductor path.

- Verify positive firmware current during discharge and negative during charge.
- Sweep both sensor ranges through zero, normal, warning, fast-trip, saturation,
  and range-transition regions.
- Repeat at expected LV supply and temperature corners.
- Characterize bias, gain, noise, drift, range disagreement, and latency.
- Set the persisted calibration ID and uncertainty to a conservative bound.
- Disconnect, freeze, saturate, reverse, and corrupt the signal; DCL and CCL
  must become zero within the 250 ms end-to-end age contract.

Pass criterion: the signed current error plus declared uncertainty bounds every
measured point, and no invalid provenance can produce nonzero authority.

## Gate 3: measurement/topology integration

- Verify mapping of all five physical segments to DADEKF instances 0-4.
- Inject a unique voltage pattern into each of the 75 series groups and confirm
  the reported limiting segment/cell.
- Exercise all 120 thermistor channels, including open, short, boundary, stale,
  and missing sensors.
- Confirm any incomplete cell/temperature mask, stale measurement epoch,
  estimator fault, or unrecovered balancing makes both limits zero.
- Verify the pack-only HIL profile remains non-authoritative.

The existing CAN HIL supplies only one pack estimator state and therefore
cannot serve as power-authority acceptance HIL. Extend it to five independent
segment states and 75 cell/120 temperature measurements before Gate 6.

## Gate 4: electrical predictor validation

Use a programmable cycler and chamber with a representative installed segment
or pack and calibrated per-cell logging.

Test SOC and temperature grid points including at least:

- SOC: 0.05, 0.10, 0.25, 0.50, 0.75, 0.90, 0.98;
- temperature: 3, 10, 25, 40, 42, 55 degC as direction permits;
- pulse durations: 0.1, 1, 10, and 30 seconds;
- new and aged/high-resistance cells;
- intentionally weak/offset cells in each segment.

For each case, log the pre-pulse DADEKF state/covariance, measured current,
predicted extrema/binding, and every measured cell voltage. Prediction including
margin must bound the adverse measured voltage and SOC behavior. Validate the
zero-current infeasible case near charge overvoltage and discharge undervoltage.

Pass criterion: no accepted case violates an operational constraint when
commanded at the published limit. Use the observed residual distribution to
increase, never opportunistically decrease, the uncertainty margin.

## Gate 5: thermal predictor validation

- Instrument representative core proxies, production surface thermistors,
  busbars, fuse, contactors, and ambient/airflow.
- Exercise cold charge, hot discharge, repeated pulses, sustained 30-second
  loads, fan transitions, blocked airflow, and hot soak.
- Validate the two-node state over each horizon and demonstrate that the
  hottest-surface ambient proxy is conservative for installed conditions.
- Verify loss or staleness of any required thermistor zeroes both directions.

Pass criterion: temperature prediction plus the 3 degC combined margin bounds
the adverse measured core/surface temperature in every accepted test. Otherwise
refit parameters or increase margins.

### Main-fuse observer

- Instrument the installed EAC14-80 fuse body, holder terminals, adjacent
  busbars, and ambient; use the exact production fuse lot and mechanical stack.
- Exercise the 0.1/1/10/30-second current matrix, repeated pulses, hot starts,
  interrupted runs, 300-second quiescent initialization, cooling, and MCU
  resets without intentionally approaching destructive clearing conditions.
- Demonstrate that the configured 25% typical-I2t budget and temperature proxy
  are conservative over the accepted domain, or reduce the budget/increase the
  margin.
- Verify observer failure or an unknown startup state cannot raise any static
  118/80/70/70 A ceiling and that budget exhaustion reduces DCL to zero.

Pass criterion: dynamic fuse authority is purely subtractive, measured
temperature/current behavior is bounded, reset cannot manufacture capability,
and independent current-fault protection remains effective. Busbars,
connectors, cables, and contactors require separate component evidence.

## Gate 6: CAN and ECU HIL

Use the exact ECU firmware intended for the vehicle and the full five-segment
HIL. Exercise:

- normal counter rollover 15 to 0;
- every single-bit corruption in each power frame;
- wrong ID, extended ID, remote frame, DLC 0-7, wrong version;
- duplicate, missing, reordered, delayed, mixed-counter, and partial bundles;
- solver snapshot age crossing 250 ms and bundle skew crossing 50 ms;
- AMS reset, ECU reset, CAN bus-off/recovery, and sender task stall;
- independent discharge, regen, and charger authorization changes;
- capacity/resistance priors and valid learned SoH;
- an older queued positive-torque command concurrent with a new zero DCL.
- valid, stale, corrupted, discontinuous, and wrong-DLC `0x688` mission
  requests; two-message recovery; Qualify entry with/without stationary
  confirmation; automatic 30% weakest-segment Limp latch;
- optional `0x689` loss/corruption without invalidating the primary four-frame
  bundle;
- Endurance-to-Qualify and Qualify-to-Endurance transitions while confirming
  no profile exceeds any simultaneously calculated hard horizon.

Pass criteria:

- ECU exposes zero until two consecutive complete bundles are accepted;
- any integrity/freshness failure immediately removes dynamic power authority;
- final torque transmission revalidates the newest limits;
- torque-derived DC current and power never exceed both relevant limits;
- independent inverter enable and shutdown paths remove torque even during CAN
  failure or a transmit race.

## Gate 7: SoH persistence and observability

Provide a board-specific two-slot storage adapter. Do not assign flash pages
until the linker map, bootloader, wear budget, and erase granularity are known.

- Use schema-3 `ams_soh_persist_record_t` without rewriting fields.
- Increment generation, write an inactive slot, read it back, validate CRC and
  schema, then make it the selected newest record.
- Interrupt power at every write boundary and verify the previous or new record
  is selected, never a hybrid.
- Corrupt every field and CRC; invalid records must not load.
- Verify an incomplete capacity or resistance estimate remains invalid after
  restart.
- Verify an aged segment's retained resistance upper bound cannot decrease
  after a transient unobservable interval or reboot; battery/segment replacement
  must use an explicit service reset and new calibration record.
- Compare learned capacity and R0 growth to calibrated independent tests.

Pass criterion: persistence cannot manufacture confidence, revert to an older
record when a newer valid record exists, or destroy the last valid record during
an interrupted write.

## Gate 8: low-energy vehicle and dyno progression

1. Start with wheels off ground or a controlled dyno and a low torque cap.
2. Log requested torque, motor speed, inverter DC bus/current, pack current,
   measured cells/temperatures, raw/published DCL/CCL, binding, age, and ECU
   accepted bundle state.
3. Confirm current/power conversion for motoring and regen over speed and DC-bus
   voltage; use conservative inverter efficiency.
4. Increase torque only after reviewing each run.
5. Perform low-SOC, high-SOC, hot, cold, weak-cell, CAN interruption, AMS reset,
   ECU reset, and shutdown tests.
6. Keep regen locked out until the full charge-direction matrix passes.

Pass criterion: observed pack current and power remain inside the published
envelope with documented control/measurement latency margin, and all failures
reduce authority within the specified deadline.

## Release evidence checklist

- [ ] Reviewed source commit and clean ARM build artifacts.
- [ ] DWT WCET and task-stack high-water evidence.
- [ ] Installed current sign/calibration/uncertainty report.
- [ ] Five-segment/75-cell/120-thermistor mapping evidence.
- [ ] Electrical and thermal model validation report.
- [ ] Installed EAC14-80 observer and reset/initialization validation.
- [ ] Fuse, contactor, busbar, cable, inverter, charger, and regen limit review.
- [ ] Mission request/status CAN and ECU profile-transition validation.
- [ ] Full CAN/ECU HIL fault matrix.
- [ ] SoH two-slot persistence and power-cut evidence.
- [ ] Dyno/low-energy staged logs and review approval.
- [ ] Updated immutable model, calibration, CAN, and threshold revisions.

Until every applicable item is complete, keep the vehicle build gate false and
do not grant torque authority from SoP/SoH frames.
