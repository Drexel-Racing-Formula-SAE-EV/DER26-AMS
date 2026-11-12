# v0.5.57-r9 — Stage 4 I/O candidates

Source revision: TIMING8-IO. Incremental base: v0.5.57-r8. Source-only project.

## Result and configuration

Three further optimizations are implemented behind independent, bench-only flags. All default to zero. Existing standard /256 polling builds and gated vehicle configuration are retained. Five SMBs, 75-cell vehicle observation, fan disabled, no installed APM, and 1 Mbps CAN remain unchanged. No qualification/evidence gate or protection threshold was relaxed.

Sixteen CubeIDE configurations are present. New inhibited profile-4 presets:

| Configuration suffix after `VEHICLE COMMS 1M SPI64 DMA DIAG` | UART DMA | ADC paired start | AUX polling |
|---|---:|---:|---:|
| `UARTDMA Debug` | 1 | 0 | 0 |
| `ADCPAIR Debug` | 0 | 1 | 0 |
| `AUXPOLL Debug` | 0 | 0 | 1 |
| `ALLIO Debug` | 1 | 1 | 1 |

These new presets retain SPI6 /64 DMA and periodic AUX2/thermistor diagnostics from the r8 DIAG preset; I-cache and D-cache are OFF to isolate the I/O changes. Existing r8 cache presets remain. Flags can be combined with those cache experiments manually in profile 4; target-branch syntax checks exercise all flags plus both caches. The fingerprints distinguish all three new selections.

Use the existing `VEHICLE COMMS 1M SPI64 DMA DIAG Debug` as the common baseline; compare one new preset at a time before ALLIO. These are inhibited bench experiments, not a qualified driving release.

## UART3 DMA

Implementation: `Core/Src/ext_drivers/cli.c`, its private `cli_uart_dma.inc`, and IRQ routing in `stm32f7xx_it.c`.

- DMA1 stream 1/channel 4: USART3 RX, circular 256-byte buffer, HT/TC and UART IDLE events.
- DMA1 stream 3/channel 4: USART3 TX, normal DMA, existing 64-byte maximum chunk and 4096-byte queue retained.
- USART3 and both DMA IRQs use preemption priority 5; they cannot re-enter the shared RX parser. Buffer positions use live NDTR, not potentially stale callback lengths.
- HT and TC both pending means service was delayed enough that a wrap may be ambiguous. Both UART IDLE and DMA entry points detect this BEFORE parsing. Drop the partial line and discard until a terminator; retain complete commands already queued. DMA errors similarly forbid consumption and defer recovery to the CLI task.
- Stop commands retain their queue bypass and output cancellation. Under continuous RX, a stop is recognized at the next half-buffer event, up to about 11.1 ms at 115200 8N1, plus IRQ latency. An idle gap normally delivers the shorter line sooner. This is a changed latency tradeoff, not a claim of identical byte-interrupt timing.
- Separate 32-byte-aligned RX/TX bounce buffers use the existing dedicated `.spi_dma` SRAM bank. RX is never CPU-written after start; invalidate before CPU consumption. TX is cleaned before DMA. Cache operations cover whole exclusive lines. SPI DMA and UART DMA together reserve 1344 bytes in the 4096-byte bank.
- Before HAL initialization or abort, stream shutdown has a finite 1024-check bound. Failure quarantines UART DMA buffers until MCU reset. This avoids entering HAL's tick-dependent abort wait with the tick masked. DMA error IRQs also pre-stop streams before entering HAL's error path.
- HAL's normal UART TC callback advances TX only after the last byte finishes. Active bytes cannot be overwritten by queue producers. The original enqueue, stall timeout, cancellation, and damaged-line policies remain.

Expected effect: fewer interrupts for sustained diagnostic traffic, not higher baud rate. Continuous RX produces roughly 90 HT/TC events/s instead of roughly 11520 RX byte events/s; IDLE events are additional and sparse individual bytes may still cause an event per byte. TX uses DMA completion plus UART TC rather than one TXE interrupt per byte. These are event-count calculations, not measured CPU gains.

## Current ADC paired software start

Flag: `AMS_CURRENT_PAIRED_START`. Actual driver: `current_sensor_read_adc`.

Both channels are configured, both independent ADCs are started back-to-back, and only then are results polled. One shared 5 ms conversion-wait window replaces two independent 5 ms waits. Cleanup stops every ADC whose start was attempted, including a failed/partial start. Configuration/start/poll/stop failure rejects the entire pair; neither count is published as fresh. Previous accepted counts remain diagnostic history. Existing conversion/calibration, range switching, disagreement, faults, task ownership mutex, 50 Hz release, and completion-time publication are retained.

This reduces software skew; it is NOT a simultaneous timer-triggered acquisition, ADC DMA, or a changed sample rate. A higher-priority interrupt can still separate the two starts. HAL overhead/ADC stop time and task preemption are outside the shared poll-window calculation, so 5 ms is not a whole-function WCET claim.

## Single-shot AUX/AUX2 readiness

Flag: `AMS_ADBMS_READY_POLL`. The normal single-GPIO AUX and AUX2 read paths now optionally use bounded, cooperative readiness polling:

- AUX: first poll after 1000 us; AUX2: first poll after 7000 us. Subsequent busy probes are separated by up to 250 us. Maximum 32 probes.
- PLAUX/PLAUX2 remain CS-low through command and at least 2*N response clocks. The terminal response bit represents the complete physical chain. No repeated conversion command is issued.
- Initial activity-aware bridge readiness is retained after the long AUX2 interval; subsequent probes reuse activity. No wake-before-every-probe policy. A stale activity gap, changed integrity epoch, SPI failure, or bounded timeout rejects the operation and withdraws link proof.
- Windows remain 1250 us/10000 us, with an explicit 500 us probe/rounding allowance. Preemption beyond that bound rejects the attempt; it is not silently accepted late. Individual HAL transfers retain their existing finite transport timeout.
- Readiness has no PEC. It only permits a subsequent register read; existing PEC, command-counter, selection, reference, and sample qualification still determine validity. Diagnostics count polls, busy polls, and failures in `spi timing`/status output.
- Polling is refused during a coherent SNAP/freeze epoch.

Important exclusion: the primary C/S acquisition is continuous in this hardware configuration. PLADC cannot be used to wait for completion of continuously running ADCs. The existing primary wait, freshness, coherence, and diagnostic stimulus/recovery delays therefore remain. This change does not shorten the 3 ms physical mux-settling requirement or the 6 ms thermistor stimulus/recovery periods.

The synthetic five-device AUX2 case, with readiness at 8 ms, completed its wait in 8342 us at /256, including one warm readiness operation. The fixed wait was 10000 us. This is a model result, not a measured device conversion time. Polling adds SPI traffic and may offer little improvement for normal AUX; bench comparison may favor retaining its fixed 1250 us wait.

## Verification and review corrections

- Full `make -C host_tests vehicle-ci`: PASS, including all retained timing/session/wake, scheduler, DMA, measurement integrity, profile, CAN, acquisition SIL and project gates.
- New `stage4-test`: PASS against actual CLI/current/ADBMS source with synthetic HAL/MMIO. Includes 32 KB TX wrap/backpressure, RX wrap and duplicate callback suppression, IDLE-before-DMA-overrun ordering, damaged-line rejection, stop bypass, stopped-stream recovery, finite-shutdown quarantine, ADC partial failures/shared timeout/tick wrap, AUX chain propagation/busy/ready/timeout/SPI failure/freeze rejection, and normal temperature-pipeline integration.
- New Stage 4 tests under AddressSanitizer + UndefinedBehaviorSanitizer: PASS; leak detection disabled. Fake-MMIO cache checks verify alignment/range/call sites, not physical Cortex-M7 cache behavior.
- Native target-branch syntax: PASS with all three flags, SPI DMA, both caches, IRQ routing, kernel profiling and pinned HAL UART/UARTEx/DMA sources enabled. This is not ARM code generation or a target link.
- During review, fixed IDLE-before-DMA overrun ordering, protected against HAL tick-dependent abort waits, and ensured failed ADC start attempts are stopped.
- Sixteen exact build configurations and distinct builder paths checked. New flags reject non-profile-4 builds; no fabricated release evidence added.

Evidence: `validation/STAGE4_VEHICLE_CI.txt`, `validation/STAGE4_SANITIZERS.txt`, and `validation/STAGE3_TARGET_SYNTAX.txt` (expanded in r9). Incremental source changes: `TIMING_STAGE4_R9.patch`.

No ARM toolchain build/link, flash, oscilloscope check, real interrupt-load test or complete-chain hardware qualification was performed. Host tests use synthetic hardware events; they cannot prove real power integrity, isoSPI margin, DMA/cache correctness, sample skew or deadlines.

## Hardware comparison and remaining work

Capture `ver`, `uart status`, `spi timing`, `current`, `perf`, and `rtos` for baseline and each isolated preset under sustained CLI traffic and simultaneous CAN/temperature acquisition. Check received command ordering, stop-to-supervisor latency, DMA faults/recovery, current-pair error rate, conversion wait/probe counts, PEC/counter failures, diagnostic successful coverage and maximum task gaps. Do not accept an average-time improvement that worsens deadline misses or measurement validity. Test UART DMA with the same cache mode on both sides of an A/B comparison.

Still outstanding: true timer-triggered ADC synchronization/ADC DMA; decomposition of the mandatory C open-wire diagnostic; target map/headroom and ARM link; measured /128 and /64 isoSPI margin; physical DMA/cache and ISR-latency qualification. The firmware does not yet establish a guaranteed 100 ms worst-case whole-system acquisition deadline.

## Primary references

- ST RM0410, DMA1 request mapping and ADC trigger/control descriptions: https://www.st.com/resource/en/reference_manual/rm0410-stm32f76xxx-and-stm32f77xxx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
- Analog Devices ADBMS6830B Rev.0, polling methods pp.54–55 and command table: https://www.analog.com/media/en/technical-documentation/data-sheets/adbms6830b.pdf
- Pinned STM32F7 HAL sources included in this project: `stm32f7xx_hal_uart.c`, `stm32f7xx_hal_uart_ex.c`, `stm32f7xx_hal_dma.c`.
