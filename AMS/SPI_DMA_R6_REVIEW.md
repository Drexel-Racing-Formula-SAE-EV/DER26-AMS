# v0.5.57-r6 SPI rate and DMA implementation

Historical r6 review. Superseded by MUX_CLOCK_R7_REVIEW.md for STCOMM clock policy, build matrix and timing results. The original all-fast mux transactions exceeded the installed ADG728 rating; r7 corrects that dependency.

Source revision TIMING5-SPI-DMA. Incremental to r5. No target flash, ARM build/link or hardware qualification performed. No ELF included.

## Implementation plan and result

1. Keep the /256 polling baseline available unchanged in transport selection.
2. Isolate /128 rate testing from DMA testing with separate CubeIDE configurations.
3. Implement bounded full-duplex DMA under the existing recursive SPI mutex and CS lifetime.
4. Test completion, timeout, error, stale interrupt, recovery and buffer publication; expose passive CLI counters.
5. Retain vehicle release gates until target evidence exists. Hardware rate/error and scheduling measurements are the remaining step.

All software steps above are implemented; target qualification remains pending.

## Build matrix

| CubeIDE configuration | SPI divider / nominal rate | Transport |
|---|---|---|
| VEHICLE COMMS 1M Debug / Release | /256, 421.875 kbit/s | Polling baseline |
| VEHICLE COMMS 1M SPI128 POLL Debug | /128, 843.750 kbit/s | Polling rate candidate |
| VEHICLE COMMS 1M SPI256 DMA Debug | /256, 421.875 kbit/s | DMA candidate |
| VEHICLE COMMS 1M SPI128 DMA Debug | /128, 843.750 kbit/s | Combined candidate |
| VEHICLE RELEASE GATED 1M Debug / Release | /256 | Existing gated polling release |

Rates assume the reviewed 108 MHz SPI6 peripheral clock. /64 (1.6875 Mbit/s) remains supported as an explicit build define and is tested in native SIL; it has no new CubeIDE preset. No rate is switched automatically at runtime. Transport-specific build fingerprints distinguish the candidates. CAN remains 1 Mbit/s, five SMBs, fan disabled, APM excluded. DMA candidates are compile-time restricted to inhibited profile 4; their BMS_OK/balance restrictions are unchanged.

The CubeMX .ioc remains the baseline /256 polling configuration. DMA is intentionally owned by user code, not HAL SPI DMA handles/MSP generation. Do not regenerate the project and assume generated code preserves the source/build options; review the diff. DMA IRQ handlers and initialization hooks are in user-code regions.

## Transport design

- New `ams_spi6_transport.c/.h` wraps all ADBMS6830 SPI transactions, including ADC polling. The existing leaf transport keeps the recursive bus mutex and CS asserted until the wrapper returns and existing integrity checks complete.
- Frames of at least 16 bytes use full-duplex DMA in the enabled candidate. Small commands and pre-scheduler calls retain polling. Writes also drain RX through DMA, preventing receive overrun.
- DMA2 stream 5 TX and stream 6 RX both use channel 1. Interrupt priority 6 is below the FreeRTOS syscall ceiling (5). No stream is shared with another configured peripheral.
- The transport deliberately uses direct stream registers rather than HAL SPI DMA callbacks. The checked-in HAL completion implementation can perform polling waits in IRQ context. Here the IRQ records completion/error flags and gives a static semaphore; the calling task blocks while the wire transfer runs.
- Both TX and RX completion and zero remaining transfer counts are required. A short bounded task-side BSY check guards final wire completion before CS release. DMA errors, timeout, BSY/overrun/mode faults reject the transfer without copying RX data to its caller.
- Cleanup disables peripheral DMA requests, disables both streams, clears peripheral/pending IRQ flags and marks the operation inactive. A following transaction drains any semaphore token before arming. This quiescence prevents an old completion from authorizing a later transfer. There is no asynchronous buffer handoff or deferred HAL callback.
- Failure resets SPI6 and restores its saved mode/rate registers. Stream disable is bounded to 1024 checks. If a stream cannot be stopped, the transport quarantines SPI6 until MCU reset and forbids polling fallback as well as DMA; it never reuses a potentially active buffer. DMA2 is not globally reset.
- Existing frame-specific timeouts are retained; semaphore waiting has at most one tick of quantization overhead, plus scheduling latency. Cleanup loops have fixed iteration bounds, not 100 ms HAL abort waits. This is not a proof of whole-system WCET.
- Two 512-byte bounce buffers occupy an aligned, explicit SRAM2 region at 0x2007F000. Both linker scripts reserve the final 4 KiB, reducing general RAM to 508 KiB for every configuration. All DMA buffer accesses are within this bank. Cache clean/invalidate operations are present if D-cache is subsequently enabled; this update does not enable D-cache.

## Diagnostics

`spi status` now includes `SPI DMA en`, started/completed, errors/timeouts, polling count and quarantine state. Counters wrap naturally as unsigned 32-bit diagnostics. They do not change protection authority. Existing SPI prescaler, frame/session trace, PEC, command counter, scan and lock diagnostics remain available. Source inspection is insufficient to establish reduced CPU occupancy; target measurements are required.

## Validation

- Full `make -C host_tests vehicle-ci`: PASS, including all seven CubeIDE configuration contracts, release identity and existing protection/profile checks.
- Executable real DMA transport with synthetic MMIO/RTOS: PASS for success, task suspension, separate TX/RX IRQs, timeout, DMA error, late IRQ, subsequent reuse, writes, incomplete NDTR, stuck BSY, pre-kernel polling, diagnostic counters and quarantined refusal.
- ASan/UBSan DMA transport and existing production wake tests: PASS. Leak detection disabled in this environment.
- Real DMA-enabled target source, main/IRQ/CLI and 6830 driver native syntax: PASS at /256, /128 and /64. This is not ARM compilation or linking.
- Both linker memory layouts: native linker smoke PASS with empty C/math/compiler archive stubs. DMA buffer and stack boundary resolve to 0x2007F000. No target headroom conclusion follows from this smoke test.
- Actual driver/accumulator wire-time SIL: 200 scans per scenario/rate. No analogue, isoSPI electrical or complete RTOS model. This SIL tests rates with polling; DMA engine behavior is exercised separately by the MMIO tests above.

| Rate | Nominal mean / maximum | Accepted nominal | Over 100 ms with injected 2.5 ms preemption |
|---|---|---:|---:|
| /256 | 42.732 / 66.007 ms | 200/200 | 40/200 |
| /128 | 34.043 / 52.900 ms | 200/200 | 1/200 |
| /64 | 29.688 / 46.330 ms | 200/200 | 0/200 |

All three modeled rates retain rejection of persistent PEC failures. /128 nominal mean improves 20.3% over r5 /256. No DMA speedup percentage is claimed: DMA reduces CPU polling, while the faster clock reduces wire time. Long diagnostic work and RTOS interference are not eliminated by either change.

Evidence: `validation/SPI_DMA_VEHICLE_CI.txt`, `SPI_DMA_TEST.txt`, `SPI_DMA_SANITIZERS.txt`, `SPI_DMA_LINK_LAYOUT.txt` and `SPI_RATE_SIL_DIV*.csv`. Older r1-r5 documents/logs are retained as historical evidence. The r6 patch is incremental from r5.

## Target acceptance sequence

Build and flash each inhibited communications candidate independently, in this order: baseline /256 polling; /128 polling; /256 DMA; /128 DMA. Preserve the same harness, five-SMB topology, power, CLI workload and diagnostics settings.

For each, collect `ver`, `spi status`, `spi timing`, `spi lockdiag`, `can diag`, `rtos` and `uart status`; scope CS/SCK plus relevant task markers. Confirm measured clock rate and that DMA started/completed counters increase only for enabled builds. Measure task active time and CAN service latency rather than interpreting non-yield wall time as CPU utilization.

Exercise normal scan, diagnostic collisions, reference/configuration recovery, communication interruption/reconnect, DMA timeout/error and repeated commands. Require fresh coherent raw/AUX results, no unexplained PEC/counter-error increase, no ownership leak or wedged CS, bounded failure response, maintained watchdog behavior and no new task/freshness regression. Compare worst-case latency and diagnostic successful coverage, not only averages. Abort-time/quarantine behavior needs target fault injection or debugger-controlled DMA stalls. Perform no drivetrain/charging validation with these inhibited profiles.

Once /128 polling is electrically proven, it is the first candidate to promote. Promote DMA independently only after target IRQ, memory-map/cache, abort and scheduling evidence passes. No unsupported validation macro was set by this change.

## Primary reference

ST RM0410, DMA2 request mapping (Table 28), stream/flag programming and SPI master DMA sequence:
https://www.st.com/resource/en/reference_manual/rm0410-stm32f76xxx-and-stm32f77xxx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf

Also reviewed the project-pinned `Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_spi.c` completion/abort implementation and the existing ADBMS6830B timing basis in WAITOPT_R5_REVIEW.md.
