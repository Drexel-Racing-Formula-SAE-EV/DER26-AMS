# Measurement integrity fixes and follow-up review

Package v2.6.19; production AMS v0.5.22 (20260905). Baseline: v2.6.18 / v0.5.21.

The four defects from the preceding measurement/CAN review are fixed. The follow-up pass found and fixed four further defects in the same data path. One separate timing finding remains open below. This was a focused review of current acquisition, window integration, metadata publication and CAN consumption, not a claim that every module in the repository has been re-audited or is defect-free.

## Reported defects fixed

| Defect | Production change | Evidence |
|---|---|---|
| Old window boundary can arrive after a newer current sample and allow a later incomplete window to appear valid | Capture current sample completion and logical window closure under the same current-window mutex. Reject backwards sample/boundary timestamps without rewinding/resetting the active interval. Keep that interval invalid until a correctly ordered rotation. | Real accumulator regression reproduces sample90/sample110/rotate100 ordering; validates invalidity through the next rotation and subsequent clean recovery. Mutex placement was reviewed and compiled, not tested with real RTOS preemption. |
| Rotation drops carried uncertainty and range | Retain metadata belonging to the last accepted sample separately from metadata pending for the next sample. Seed each new window from the carried sample. Unknown uncertainty is UINT16_MAX, not zero. | A 500 mA carried uncertainty remains 500 mA after fresh 100 mA samples; unknown remains unknown. Subsequent all-100 mA window recovers to 100 mA. |
| Mixed range becomes single-range again | Separate metadata initialization state from range value zero. Mixed range remains zero for that entire window. | 1 -> 2 -> 2 stays mixed; subsequent all-range-2 window reports 2. |
| CAN 0x68B falsely labels all valid DHAB current calibrated | Quality 2 requires confident nonzero calibration identity and a known nonzero uncertainty from the canonical window. Quality 1 represents valid current without that proof. | Real encoder regression covers uncalibrated, calibrated, unknown uncertainty, zero uncertainty, stale and live fallback cases. |

## Additional defects found and fixed

1. **Carried extrema overwritten.** The first fresh sample in a rotated window replaced min/max even though the carried value contributed to its integral. Min/max now include the carried value. Regression: 30 A carry followed by 10 A retains max 30 A and a 0.4 As integral across a 30 ms window.
2. **CAN data and metadata describe different epochs; stale current survives publication caching.** The electrical frame used canonical window current while 0x68B used the latest live sample's age and sequence. Both now describe the immutable current window. A current sample older than 100 ms expires independently of the 500 ms publication-cache lifetime. The live fallback also checks current age. Regression deliberately gives the live sample a newer tick and different sequence and verifies that it cannot refresh old canonical data.
3. **Rotation hides a real sample gap.** An intermediate rotation advances the integration cursor, so checking only that cursor permits interpolation between actual samples more than 100 ms apart. Updates now check both the integration cursor and the previous real sample timestamp. Regression: sample10, rotate90, sample150, rotate160 remains invalid.
4. **Stale tail crosses into a later window.** When rotation rejected a stale tail it retained a valid carry and an integration cursor preceding the next boundary. A later fresh sample could integrate that old interval into the new window. Stale rotation now clears the carry. Regression: sample60, rotate150, rotate170, sample180, rotate190 yields a valid recovered 20 ms window with 0.2 As at 10 A, not 0.4 As. Recovery uses the existing bounded first-sample hold policy; it does not reconstruct missing historical charge.

## Remaining finding: fault confirmation counts nominal task periods

**Priority: P2; established by source tracing, not target timing measurements.**

`Core/Src/tasks/current_task.c`, in `current_task()`, still passes `CURRENT_TASK_PERIOD_MS` (20 ms) to `current_fault_update()`. In `Core/Src/ext_drivers/current_fault.c`, that argument is added to `pending_ms` and `sensor_invalid_ms`. A 100 ms confirmation therefore takes five qualifying executions regardless of how long those executions are spaced apart. At a sustained 50 ms sampling interval, those five executions span 200 ms from the first observation to confirmation, rather than the nominal 80 ms between five 20 ms observations. Severe instantaneous thresholds and independent supervisor/heartbeat protection still exist; this finding alone does not prove an unprotected vehicle condition.

Follow-up implementation should define wall-clock confirmation semantics, the first observation, state/threshold changes and missing-sample behavior together; then use wrap-safe elapsed timestamps and saturating counters. Add delayed-task/fault-transition tests and measure target task jitter. This release leaves the existing protection timing policy unchanged rather than presenting a new timing policy as already validated.

## Scope and behavior

Changed production paths:

- `Core/Inc/measurement/ams_measurement.h`
- `Core/Src/measurement/ams_measurement.c`
- `Core/Src/tasks/current_task.c`
- `Core/Src/tasks/adbms_task.c`
- `Core/Src/tasks/canbus_task.c`
- Canonical version and CAN-contract definitions in `Core/Inc/ams_version.h` and `Core/Inc/ams_build_profile.h`

The ADBMS boundary is a logical publication/integration boundary captured after acquiring the mutex. It is not a claim that the ADCs sample simultaneously. Existing per-cell physical timestamps/ages remain available. Target measurements must bound the extra mutex wait and acquisition skew.

CAN contract suffix is now CURRENT2. IDs, DLC and byte order stay the same. Quality 1 is an explicit valid/unproven-calibration value; canonical DHAB bytes 4-5 identify the current window, not a newer live sample. Decoder consumers must follow the updated `AMS_ECU_CAN_CONTRACT.md`. APM standalone diagnostics retain their existing source policy.

Previous CAN scheduler and release-identity fixes are retained. No Zephyr migration, estimator/SoH/SoP algorithm tuning, fault-threshold changes or flashing was performed.

## Validation performed

- `make -C AMS/host_tests measurement-integrity-test`: PASS, seven regression groups using the production accumulator and CAN encoding source.
- Same regressions compiled at O2 with AddressSanitizer and UndefinedBehaviorSanitizer: PASS. Leak detection was disabled because LeakSanitizer cannot inspect processes in this environment; the initial leak-enabled run ended in a tooling error, not a reported application memory defect.
- Changed measurement/current/ADBMS/CAN translation units: PASS host GCC syntax checks with warnings treated as errors in both repository bench-exercise and vehicle-validation profiles.
- Canonical firmware/CLI identity gate: PASS.
- Baseline comparison: the new regression compiled against unchanged v2.6.18 and failed the old-boundary preservation assertion, confirming that the test detects the reviewed defect.

The focused target is included in `firmware-ci` for subsequent runs. Existing full MiL campaigns and unrelated suites were not rerun. ARM target linking, target RAM/stack high-water measurements, real preemption/ADC timing and physical CAN traces were not performed here. The updated production source needs a target rebuild before flashing.
