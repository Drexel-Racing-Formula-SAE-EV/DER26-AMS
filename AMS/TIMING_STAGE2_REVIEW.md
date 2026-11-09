# Timing stage 2 — reviewed source candidate

Base: supplied vehicle main v0.5.57-r1 plus timing stage 1. Source revision now identifies TIMING2. TESTDAY inhibition and VEHICLE release gates remain intact.

## Changes and intended effects

1. Transport ownership from stage 1 encloses TX preparation, HAL transfer, RX copying and bookkeeping. New host regression poisons the shared RX buffer at ownership release: caller RX must already be copied. HAL fixtures assert ownership.
2. SPI6/8-bit frames use start + floor(bytes*8*divider/108) microseconds as a lower bound on physical completion. APB2 cannot exceed 108 MHz within STM32F767 specifications. Unsupported SPI/data size retains start. If observed duration is too short, start is retained. The original coarse millisecond age stays conservative. No late completion timestamp authorizes wake suppression. Tests cover delayed observation and wrap. Hardware must use the supported clock tree and timer calibration.
3. Reference preconversion wait increased from 3000 to 4500 us, covering documented 4400 us maximum plus 100 us margin. This adds 1.5 ms when paid; the 17 ms redundant conversion and temperature conversion waits remain unchanged. No extra keepalive or polling traffic.
4. RAW-source acquisition reads AVG8/IIR every fifth voltage call (first call includes them). AMS_ADBMS_OPTIONAL_PRODUCT_DIVISOR=1 restores original cadence; range 1..10. Selected AVG8/IIR estimators always acquire full-rate. Driver clears product-updated masks each epoch, and accumulator clears product usability masks before normalization; skipped products are unavailable for that epoch, not fresh cached values. Filter settling/count behavior remains conservative. Normal 20 read epochs become 8 reads on four of five RAW acquisitions: average 10.4 reads/epoch, roughly 8 ms average wire saving at /256, excluding all other work. First/fifth epochs still have original cost; no worst-case deadline claim.
5. Existing status/config/cell open-wire ordering retained. AUX2 and thermistor tests use observed high-water wall costs, floored at startup estimates 30/60 ms, against remaining nominal scan period. These estimates are provisional admission inputs, not WCET. AUX2 defers at most 500 ms after its due time; thermistor work at most 2000 ms after due, except mandatory higher-priority diagnostics can still delay either. Forced admissions are counted and may exceed the scan period; one diagnostic per post-acquisition pass, no catch-up burst. Overdue thermistor work takes precedence over AUX2 to prevent faster-period starvation.
6. One delayed retry for incomplete positions, then fair rotation. No immediate retry or held-bus loop. Coverage completion uses full IC valid_mask, so completed disagreement/suspect results count as tested coverage while remaining faults. Transport-incomplete results do not count.
7. Existing SPI status summary prints coverage masks and advisory overdue masks plus cost/deferred/forced counters. Coverage age alarms are advisory only: AUX2 12 seconds, thermistor 144 seconds. These are NOT newly established safety FTTIs. Disabled diagnostics report no overdue masks. Success timestamps are actually completion timestamps, not proof of healthy sensors. Detailed per-position ticks and frame ring remain available through halted SWD inspection.

## Review and limits

- No changes to topology, cell masks, thresholds, BMS_OK logic, CAN speed, current path, balancing duration/recovery, or DMA. No D-cache enable. SPI /256 and I-cache-off remain defaults; /128, /64 and I-cache remain independent benchmark options.
- Admission uses scan entry and nominal period, conservatively ignoring extra balancing-on slack. High-water includes preemption and can cause more future deferral; forced/overdue reporting makes this visible.
- Mandatory diagnostics and intrinsically long tests can still overrun. Existing freshness/fault behavior remains; this is overload visibility and bounded optional work, not a guarantee that all coverage/100 ms deadlines are feasible.
- Frame trace is software wall timing, not true CPU/ISR accounting. Full task-switch/ISR accounting, CAN latency tracing and hardware benchmarks are still outstanding. Conversion polling and temperature pipelining are deferred. No ARM compiler/build/link or hardware validation was available.

## Validation

See validation/TIMING_STAGE2_TESTS.txt, TIMING_STAGE2_VARIANTS.txt and TIMING_STAGE2_SOURCES.txt. Production driver wake/integrity tests, RX ownership-poison regression, frame trace wrap, conservative activity timestamps, RAW decimation and task scheduling pass. Diagnostic budget unit tests cover exact fit, deferral, forced admission, wrap, missing coverage and all-failing retry fairness. Syntax checks cover all four AUX2/therm feature combinations. Full-rate RAW, AVG8 and IIR configurations pass driver tests. Native tests do not establish target timing or production diagnostic hardware response.

Generic make test remains incompatible with this vehicle-only project's excluded APM/profile combinations; stage 1 review records that limitation. This is source for review and bench qualification, not a prebuilt flash image or drive signoff.
