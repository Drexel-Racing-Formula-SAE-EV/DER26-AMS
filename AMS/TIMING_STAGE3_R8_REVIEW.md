# v0.5.57-r8: adaptive diagnostics, temperature pipeline and profiling

Source revision TIMING7-PHASED. Incremental to r7, retaining the SPI/DMA and /256 STCOMM policy. Source only: no ARM build/link, flash, electrical qualification or real-RTOS timing measurement performed.

## Implemented

### Adaptive thermistor diagnostic scheduling

The scheduler prefers the existing compact down/up diagnostic when the available scan budget admits its conservative cost estimate. When only a smaller budget is available it runs one stimulus phase per visit, prioritizing the pending second phase over AUX2. Separate high-water costs track compact and phase operations.

Every phase obtains an unstimulated baseline, applies one stimulus, restores and checks production CFGA, and obtains an unstimulated recovery sample before returning. No active stimulus or modified configuration is retained across scans. Each direction uses its own nearby baseline; the health report now exposes both baselines. Recovery qualification retains the worse of the two recovery deltas, and a fault from either direction remains a fault after merging.

A half-test has no completed-valid mask and cannot earn successful diagnostic coverage. The pair must complete within one second with matching topology, configuration fingerprint and transport-integrity epoch. Invalid, stale or interrupted pairs are rejected. Pending pairs also expire through a metadata-only scheduler path when no further work can be admitted, preventing an over-budget high-water estimate from leaving a pending operation stuck indefinitely.

The shared measurement implementation also supports the original compact pair, avoiding duplicated register logic. The public phase API owns the recursive SPI lock through stimulus, restoration and qualification. Explicit synchronous diagnostics continue to use the compact pair.

### Partial-write restoration defect fixed

The previous code restored CFGA only if the stimulus write returned success. A failed transaction may still reach some chips. Restoration is now required after every attempted stimulus write. A restoration failure immediately sets the application's configuration fault; it does not wait for the next periodic configuration check. Stimulated samples never enter the primary temperature image.

### Strict advisory admission

Overdue AUX2/thermistor diagnostics no longer override the remaining scan budget. Missing coverage remains visible through existing age/mask telemetry. The legacy `forced` counters now count late-but-admitted work; CLI labels them `late_admitted`.

This is estimate-based admission, not a hard WCET guarantee: once a stimulus begins, restoring hardware takes precedence over abandoning it at a time budget boundary. Actual overruns update measured high-water costs. Mandatory status/configuration/C-path open-wire behavior is retained; long C-path open-wire jobs are not split by this release. Sustained overload may leave advisory coverage overdue; it is never reported as a pass.

### Temperature pipeline

The task selects and ACK-checks all three temperature muxes before normal cell conversion. Their analog nodes settle while cell conversion runs. The later temperature capture waits only the unelapsed portion of the existing 3 ms settling requirement.

Preparation is single-use and carries sensor selection, timestamps and an integrity epoch. Stale preparation (over 200 ms), changed selection, missing ACK validity, invalid topology or an intervening transport-integrity failure rejects capture. A failed preparation is not immediately retried again in the same visit. Standalone temperature acquisition still prepares its own muxes. Coarse-clock-only callers include an extra millisecond of settling margin; normal task operation uses the existing fine clock hook.

Normal scan preparation/capture remains under the task's existing image/bus ownership. Cell conversion and balance-off timing are not shortened. Primary voltage/temperature freshness and protection thresholds are unchanged.

### Passive task and IRQ profiling

New `perf [clear]` reports task-context cycles, number of running slices, longest active slice, maximum observed unblock-to-run latency, instrumented IRQ cycles and fixed-slot overflow. Sixteen static records hold names/handles and counters; no heap allocation, SPI traffic or ISR formatting is added.

FreeRTOS switch/ready trace hooks and CAN, UART, DMA, timer and SysTick instrumentation supply the observations. The SysTick instrumentation is a small modification to the pinned FreeRTOS CM7 port and must be retained/reviewed on vendor updates. PendSV/SVC boundary overhead, profiler overhead and uninstrumented exception boundaries are not fully separated: these are useful task-context measurements, not certified WCET or a complete scheduler trace.

Initialization enables the shared DWT clock without resetting an already running counter. The ADBMS timing initializer was changed to preserve that same monotonic clock. `perf clear` resets measurements after warm-up without resetting DWT or other timing epochs. Names/slots are retained; overflow is explicit. The profiler is intended for the current mostly static task set, not arbitrary task churn. Periodic IRQ/checkpoint activity is required to resolve the 32-bit cycle-counter wrap unambiguously.

### Cache benchmark candidates

Three inhibited profile-4 configurations are added, bringing the exact configuration count to twelve:

| New Debug configuration suffix | Diagnostics | I-cache | D-cache |
|---|---|---:|---:|
| SPI64 DMA DIAG | AUX2 + thermistor OW | 0 | 0 |
| SPI64 DMA DIAG ICACHE | Same | 1 | 0 |
| SPI64 DMA DIAG ICACHE DCACHE | Same | 1 | 1 |

All use /64 normal SPI and /256 STCOMM. Standard configurations retain their previous /256 polling default and caches remain default-off. Correction to the earlier discussion: I-cache already had an opt-in switch; it was not enabled by default. D-cache is a new bench-only option, not a production default. Configuration fingerprints include transport, cache and advisory-diagnostic feature selections. `perf` reports actual cache-enable bits on target.

SPI DMA buffers retain their aligned, dedicated SRAM2 bank and cache maintenance. The DMA test now checks the clean/clean-invalidate/invalidate calls, alignment and full-buffer ranges with D-cache indicated. This validates the software call contract, not hardware cache coherency or target speedup.

## Test results and measured model effects

Full `make -C host_tests vehicle-ci`: PASS. Added gates include actual-driver phase and pipeline fixtures, actual diagnostic scheduler admission with mixed loads, profiler arithmetic, twelve exact CubeIDE configurations and target-branch native parsing of the kernel hooks/cache candidate. The latter uses native GCC and host newlib declarations; it is not an ARM compile/link.

ASan/UBSan: PASS for phase, pipeline, profiler, scheduler and DMA fixtures; leak detection disabled in this environment. Tests cover normal restoration between phases, incomplete coverage, expiry, fingerprint/integrity cancellation, independent baselines, preserved down-direction faults, failed stimulus-write restoration, unchanged primary samples, residual settling, one-shot preparation, stale/mux/integrity rejection, clock wrap, nested IRQ exclusion, profiler reset, bounded slots, strict admission and recovery after deferred work.

Actual driver/accumulator synthetic wire-time SIL, 200 scans per scenario, includes the r7 slow-mux policy and new preparation order:

| Normal SPI divider | r7 nominal mean | r8 nominal mean / max | Nominal accepted | Nominal warm wakes, r7 -> r8 |
|---|---:|---|---:|---|
| /256 | 42.732 ms | 39.232 / 62.507 ms | 200/200 | 600 -> 400 |
| /128 | 34.412 ms | 30.912 / 49.769 ms | 200/200 | 600 -> 400 |
| /64 | 30.243 ms | 26.743 / 43.385 ms | 200/200 | 600 -> 400 |

Thus nominal modeled /64 acquisition improves another 11.6%, with approximately two warm wakes per ordinary scan. Intrusive diagnostics, startup and faults have additional necessary transactions/wakes. Persistent PEC failures still reject acquisition. In the injected 2.5 ms preemption scenario, /128 and /64 have no over-100 ms modeled scans; /256 still has one, with a 112.507 ms maximum. No whole-system 100 ms guarantee follows from this limited SIL.

At /256 the compact thermistor diagnostic retains the r7 modeled 32.566 ms total and five warm wakes. Separate restored phases model about 25.155 ms each. Splitting therefore reduces each admitted job's duration but increases total diagnostic configuration/reference/wake work. Adaptive admission pays that cost only when the compact pair cannot fit. These figures are synthetic packet/wire results, not analogue response, target CPU time or electrical qualification.

Evidence is in `validation/STAGE3_*` and `SPI_RATE_SIL_DIV*.csv`. Historical r1-r7 reports/logs remain historical; this report describes the current behavior.

## Target checks and remaining work

Start with the inhibited baseline, then /128 and /64 transport candidates. Compare identical scan/diagnostic loads. Use `ver`, `spi status`, `spi timing`, `spi lockdiag`, `can diag`, `rtos`, and `perf clear` after warm-up, followed by `perf` after a fixed measurement interval. Compare the three DIAG cache configurations independently. Scope CS/SCK and mux SCL/SDA, verify normal cell voltages while mux settling overlaps cell conversion, and verify normal CFGA after each diagnostic visit.

Check stack/heap headroom and the linker map: new static profiler/phase state consumes RAM. Verify DWT wrap, nested IRQ operation, DMA completion/abort/cache coherency, actual task/unblock latency, primary sample freshness, successful diagnostic coverage and ADC noise under the new sequence. Target data must establish performance; native tests cannot qualify the harness, thermistor stimulus response, cache behavior or all RTOS collisions.

Unimplemented items remain: UART circular RX/TX DMA, timer-triggered/current ADC redesign, broader conversion-completion polling, and decomposition of the mandatory C open-wire diagnostic. UART and ADC changes need their own ownership/overrun/sample-alignment tests. ADC-completion polling must prove which C/S/AUX conversions are complete; a faster poll must not weaken redundant-image readiness. The new profiler provides evidence to prioritize these changes. No current-sense calibration, estimator model, CAN bitrate, protection threshold, channel requirement or authority gate was relaxed.
