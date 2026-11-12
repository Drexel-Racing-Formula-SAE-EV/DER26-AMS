# v0.5.57-r5: verified reference reuse and fewer idle gaps

Source revision: TIMING4-WAITOPT. Incremental change from r4; source-only release.

## What changed

- Reference readiness is now cached only after an integrity-checked CFGA read proves REFON on for every IC. Newly established evidence still requires 4.5 ms maturation. Valid whole-chain replies maintain the proof with a conservative one-second expiry; unrelated reads cannot revive expired proof.
- Reset, CFGA writes, link/integrity failures and topology changes invalidate that proof. Failed readiness never authorizes the short conversion path.
- Normal voltage scans reuse a mature reference instead of paying the reference-startup delay every scan.
- Single-GPIO primary AUX conversion waits are 1.25 ms instead of 4 ms, only with verified mature REFON and SOAKON disabled. This avoids an otherwise unnecessary link wake before AUX readback.
- Removed the 1 ms delay after each successful, integrity-checked temperature read. Retained the 3 ms physical mux-settling interval. Removed a wake immediately before retry mux settling; the following checked transaction owns readiness.
- Corrected AUX2 wait from 9 ms to 10 ms: the minimum 110 Hz update rate requires more than 9 ms. This intentionally increases that diagnostic wait.
- CLI status reports reference probe/wait/reuse and short AUX-wait counters without additional bus transactions.

The existing r4 physical-activity/session guards and complete transport buffer ownership remain. No SPI clock increase, DMA, safety-mask relaxation, CAN bitrate change, or profile gate bypass is included.

## Timing basis

ADBMS6830B datasheet: https://www.analog.com/media/en/technical-documentation/data-sheets/adbms6830b.pdf

Reference startup maximum is 4.4 ms; this implementation retains 4.5 ms. The primary AUX minimum update rate is 0.9 kHz (about 1.112 ms); 1.25 ms includes margin when no programmed soak is active. AUX2 minimum is 110 Hz (about 9.091 ms), hence 10 ms. CFGA REFON and SOAKON are checked rather than inferred from desired configuration. The reference proof expires before the minimum watchdog interval.

Retained waits include cold/warm interface readiness, redundant C/S conversion, mux settling, and diagnostic stimulation/restoration. This is a reduction of demonstrated redundant work, not a claim that every possible wait has been minimized.

## Validation and effects

Full `make -C host_tests vehicle-ci`: PASS, including release identity, supported vehicle profiles, diagnostic budget, production wake/session tests and acquisition SIL. ASan/UBSan production and SIL fixtures: PASS; leak detection disabled. Exact logs are under validation/WAITOPT_*.

Paired deterministic SIL uses actual driver/accumulator code and the same fixture for r4 and r5 (200 scans each):

| Metric | r4 | r5 |
|---|---:|---:|
| Nominal mean voltage + temperature | 60.548 ms | 42.732 ms |
| Nominal maximum | 96.922 ms | 66.007 ms |
| Nominal warm wakes | 1397 | 600 |
| Nominal accepted scans | 200 | 200 |
| 2.5 ms preemption mean | 86.448 ms | 68.632 ms |
| 2.5 ms preemption maximum | 146.922 ms | 116.007 ms |
| 2.5 ms preemption scans over 100 ms | 40 | 40 |

Nominal modeled duration falls 29.4%; warm wakes fall 57.1%, about seven to three per scan. These are modeled results, not oscilloscope measurements or CPU utilization. The extra initial CFGA verification is intentional. All scans recover in the tested transient PEC/timeout scenarios; persistent PEC failure rejects all voltage and temperature acquisitions.

Production tests cover cold maturation, repeated reference reuse, expired evidence, bad PEC, REFON off, CFGA writes, reset invalidation and refusal of the short path with SOAKON set. SIL checks AUX readiness, mux settling, counter wrap, lock/CS cleanup and nominal wake/wait budgets.

## Remaining limits and next bench checks

The model does not emulate a full RTOS, electrical isoSPI behavior, analogue settling/noise or all diagnostic collisions. Long diagnostic jobs and preemption can still overrun 100 ms; phasing/admission of those jobs remains separate work. No ARM build/link, target flash or hardware timing qualification was performed.

Build the existing VEHICLE COMMS 1M Debug/Release configuration for bench evidence; it retains propulsion/charge output inhibition. The gated vehicle release configuration remains gated. No prebuilt ELF is included.

On the five-SMB bench capture normal operation, startup, idle/reconnect and simultaneous diagnostics. Compare scan duration/misses, wakes, reference reuse counters, successful diagnostic coverage, PEC/counter errors and CAN latency. Confirm each selected AUX result is fresh at 1.25 ms and retained temperature accuracy. Reject the optimization if stale conversions, extra transport errors or freshness violations appear. Measure physical CS/SCK and relevant task markers; SIL alone cannot approve vehicle operation.

Historical r1-r4 reports describe earlier revisions. This report and WAITOPT_R5.patch describe the incremental r5 behavior.
