> Historical r3 findings. The coarse-age failure is corrected in r4; see WAKEFIX_R4_REVIEW.md and current validation logs.

# Acquisition SIL findings

The existing host suite was not full-system SIL. Added a reproducible acquisition SIL using actual production driver and accumulator code, not a copied driver algorithm. No production firmware changes in this deliverable.

Compared supplied v0.5.57-r1 against timing-stage2 (r3), 200 scans/scenario. Virtual time excludes CPU execution and diagnostics.

| Scenario | Original | Timing stage 2 |
|---|---:|---:|
| Nominal accepted | 200/200 | 200/200 |
| Nominal mean acquisition | 67.064 ms | 60.548 ms |
| Nominal maximum | 95.422 ms | 96.922 ms |
| Nominal total transfers including setup | 7601 | 5681 |
| Repeated 2.5 ms coherent-read pause accepted | 0/200 | 0/200 |
| Intermittent 8 ms pause accepted | 200/200 | 200/200 |
| Intermittent PEC / timeout accepted | 200/200 | 200/200 |
| Persistent PEC accepted | 0/200 | 0/200 |

Mean nominal time drops about 9.7%, and total transfer count about 25.3%. Maximum nominal time increases by 1.5 ms, consistent with the longer reference wait. Optional decimation improves average workload, not the longest full-product epoch. Both temporary fault scenarios demonstrate recovery; persistent corruption is rejected. Detailed CSV includes temperature failures and retries.

NEW SOURCE-PROVEN LIMITATION: frame_activity_lower_bound improves microsecond accounting but activity_ms still stores frame-start ms and wakeup_checked independently demands age <3 ms. The coarse-age test therefore vetoes some genuinely sub-3-ms microsecond proofs. The rig includes an isolated regression reproducing this exact condition, plus a sustained scenario with all voltage scans rejected. It does not justify raising the coherent guard generally. Next fix should reconcile the coarse wrap-sanity check with the conservative activity lower bound while keeping the microsecond guard authoritative. Test rounding phases, transfer lengths, time wrap and stale cache rejection before applying.

The 32-event frame ring and per-position diagnostic tests still do not establish full-task schedulability. Next SIL expansion should execute the actual periodic diagnostic scheduler and publication path, with reproducible task/ISR interference, complete fault/coverage outputs and CAN deadlines. This version deliberately does not pretend that the existing helper unit tests exercise that integration. Hardware still establishes clock/isoSPI margins and actual execution time.
