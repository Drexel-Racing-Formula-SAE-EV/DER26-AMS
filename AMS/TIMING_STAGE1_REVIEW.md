# Timing stage 1 — source candidate, not a hardware-qualified release

Base: supplied v0.5.57-r1 VEHICLE main 1M fixed source.

Implemented:
- Shared SPI TX preparation, HAL transfer, RX copy, debug previews and activity/error bookkeeping remain under recursive SPI ownership.
- 32-entry binary per-driver frame ring records start, software-observed end, byte count, HAL result and coherent flag. Read session.frame_trace and frame_trace_sequence through SWD with target halted. 384 bytes plus sequence per driver; wraps overwrite oldest entries. No output or allocation in acquisition. This is wall time, not CPU time. Not every transport path is covered (notably ADC polling/wake edges); existing wake counters remain.
- Cell open-wire and AUX2 return after their intrusive post-acquisition work, preventing subsequent diagnostic stacking in that pass. Existing status/config early returns remain. This is a collision reduction, not a measured-cost admission scheduler or a guarantee of coverage deadlines.
- AUX2 and thermistor positions retain separate attempt/success timestamps and completed masks. HAL_OK is recorded as successful API completion; HAL_ERROR must still be decoded to distinguish detected diagnostic fault from failed transport. No new retry loop or protection authority.
- Optional I-cache benchmark flag AMS_BENCHMARK_ICACHE=1, default 0; existing SPI prescaler benchmark remains default /256.

Reviewed effects:
- CS is deasserted before trace recording; trace does not deliberately stretch CS low.
- Leaf locks now extend through bounded memcpy/bookkeeping. Outer acquisition ownership is unchanged; CLI lock contention is not claimed fixed.
- No new wakes or conversions. Wake/session guard, voltage/temperature thresholds, excluded channels, balancing timing, CAN rate and current path unchanged.
- Diagnostic deferral can increase coverage age. The new success timestamps expose it through SWD; maximum age and overload policy still need implementation/qualification.
- Conservative start-based awake proof intentionally retained. Software completion time can be arbitrarily late after preemption, and is not sufficient authority to relax the guard.

Validation:
- make -C host_tests adbms-wake-policy-test vehicle-variant-test: PASS, including real-driver trace wrap/length/duration tests and production app/task syntax checks.
- Generic make test: FAILS on existing vehicle profile guards (generic harness requests excluded APM/profile combinations). No claim of full-suite success.
- No ARM compiler available; no target build/link, flash or hardware timing measurement.

Remaining from approved plan:
1. Full trace including wake/retry/diagnostic events, ISR-aware CPU accounting, lock and CAN latency.
2. Proven physical-activity timestamp semantics and conditional reference readiness.
3. Cost-based diagnostic admission, completion classifications, bounded fair retries and overdue CLI reporting.
4. Optional product decimation after consumer/freshness audit.
5. Independent hardware clock/cache benchmarks.
DMA and temperature pipelining remain later work. This package is explicitly the reviewed first implementation stage, not completion of the entire optimization plan.
