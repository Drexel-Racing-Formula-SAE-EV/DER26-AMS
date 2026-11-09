# v0.5.57-r4 — wake/session correction

Source-only successor to r3 Timing Stage 2 and its acquisition SIL. Native verification is complete; no ARM build/link or hardware measurements were performed. This is the five-SMB, 1 Mbps vehicle project, not charge-only or standalone APM firmware.

## Defect and correction

The previous fine-clock proof used frame start plus a conservative minimum wire duration, but wakeup_checked independently required HAL milliseconds since frame start <3. A 44-byte /256 transfer takes approximately 835 us; adding a 2500 us post-transfer interruption can trip the coarse check even though the fine-clock idle proof remains below 3000 us. With repeated interruptions, r3 rejected 200/200 voltage scans.

A single link_recent predicate now governs coherent and standalone reuse. It requires matching topology, valid activity, fine-clock age strictly below the unchanged 3000 us guard, and independent coarse-clock age below one second. The latter is ONLY a wrap-alias rejection check: it cannot authorize a fine-clock age of 3000 us or greater. Frame-start coarse timestamps intentionally remain conservative. No delayed software completion is promoted to proof of recent physical traffic.

Session opening cannot renew the physical proof. Session guard decisions use wake.activity_us rather than a logical session-start timestamp. Rewake cleanup no longer overwrites physical activity with current software time. Gap telemetry is sampled at the next guard check; successful-frame bookkeeping no longer overwrites it with frame-to-frame duration that includes wire time.

Checked command, write and read operations acquire the existing recursive, priority-inheritance bus mutex before wake decisions or shared command preparation. Ownership extends through transfer, PEC/counter bookkeeping, proof updates and all error returns. Existing outer coherent-acquisition ownership remains. Leaf buffers stay protected through copy-out. No interrupt masking was added. A preemption between the final guard and physical clocks is still possible; integrity rejection and bounded epoch retry remain necessary.

Canonical AMS_SOURCE_REVISION construction was restored without changing the release checker. Project revision identifiers now contain TIMING3-WAKEFIX. vehicle-ci now includes diagnostic-budget tests and the acquisition SIL.

## Verification

- Full `make -C host_tests vehicle-ci`: PASS, including canonical identity, project/configuration, measurement, CLI, CAN, wake, diagnostic budget, acquisition SIL and existing APM host regressions.
- Exhaustive 1000 microsecond phases relative to HAL's millisecond tick: 2999 us accepts, 3000 us rejects; neither inserts a wake into a coherent snapshot.
- Full microsecond-wrap alias rejects stale proof; legitimate short intervals across both clock rollovers pass; session reopen cannot refresh an expired proof.
- Existing topology changes, failed PEC/counters, failed SPI, stopped timer, stretched wake pulse, reset, runtime-clock replacement, RX ownership and capture-age tests pass.
- Acquisition SIL: seven scenarios, 200 scans each, actual driver and accumulator; synthetic HAL/time/packets. Persistent corrupted measurements remain rejected. Intermittent 8 ms pauses and injected transport failures retain bounded retry and recovery.
- AddressSanitizer/UndefinedBehaviorSanitizer: see WAKEFIX_SANITIZERS.txt. Leak detection disabled because the execution environment runs under ptrace; this is not a leak-check result.

## Paired r3/r4 acquisition result

| Scenario | r3 accepted | r4 accepted | r3 restarts | r4 restarts |
|---|---:|---:|---:|---:|
| Nominal | 200/200 | 200/200 | 0 | 0 |
| 2.5 ms interruption after every coherent read | 0/200 | 200/200 | 200 | 0 |
| Intermittent 8 ms interruption | 200/200 | 200/200 | 59 | 59 |
| Persistent PEC failure | 0/200 | 0/200 | 200 | 200 |

Nominal modeled voltage+temperature mean remains 60.548 ms, maximum 96.922 ms. The 2.5 ms interruption scenario now completes actual acquisitions, averaging 86.448 ms with maximum 146.922 ms and 40/200 scans over 100 ms. This is not a speed regression hidden by the report: previously those scans aborted. Recovering valid acquisition does not remove scheduling interference. Warm wakes in that case decrease from 1797 to 1357; genuine conversion/idle gaps still require wake trains.

## Limits and next qualification

This is NOT the complete scheduler timing fix. The SIL has no full RTOS, ISR execution, CAN/CLI interference, electrical idle model or analogue plant. It does not execute the production diagnostic scheduler. The existing forced long thermistor diagnostic can still exceed the nominal 100 ms budget. No DMA, SPI prescaler, conversion wait, protection threshold or sensor requirement was changed in this update.

Next hardware check: build VEHICLE COMMS 1M Debug/Release (profile 4), confirm TIMING3-WAKEFIX identity with `ver`, capture `spi timing` and the existing 32-frame binary ring over several full diagnostic rotations. Compare genuine guard gaps/restarts, raw and temperature ages, diagnostic completion/overdue masks and CAN task gaps. Repeat with controlled CLI traffic. Target SWD/scope traces must establish actual inter-frame gaps and preemption margin before changing SPI speed or guard values. Profile 3 remains gated on its existing evidence requirements.

Next code work: an integration harness for the actual diagnostic scheduler, followed by splitting over-budget diagnostics into bounded phases with verified restoration/abort behavior. Do not simply raise the scan deadline, disable diagnostics or increase the wake guard to hide overload.

## Reproduce

```
make -C host_tests vehicle-ci
python3 host_tests/sil/run_acquisition_sil.py --baseline /path/to/r3/project
```

The ZIP includes WAKEFIX_R4.patch (incremental from r3), full source, test source, logs and hashes. Older stage reports are historical; this file supersedes their coarse-age-veto and CI status.

Reference: Analog Devices ADBMS6821/6822 Rev. B, Table 7 lists minimum idle timeout 4.3 ms and maximum readiness 10 us. The existing 3 ms guard remains conservative and unchanged; these bridge values alone do not qualify the full SMB chain or ISR margin.
https://www.analog.com/media/en/technical-documentation/data-sheets/adbms6821-adbms6822.pdf
