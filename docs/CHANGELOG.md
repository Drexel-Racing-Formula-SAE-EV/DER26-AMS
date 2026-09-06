# Condensed Changelog

The active repository keeps a condensed technical history. Detailed forensic history belongs in Git commits/tags and formal release archives rather than dozens of dated patch files in the project root.

## 0.5.x — DER26 safety, ADBMS, CAN, estimation, and observability work

Major work represented in the current source line includes:

- ADBMS6830 acquisition, diagnostics, freshness, and failure hardening;
- DER26 CAN V4 compact/status/power/logger/tuning telemetry;
- segment/pack estimator, SoP, SoH, fuse observation, and passive tuning data;
- build-profile authority restrictions and validation gates;
- expanded host unit/SIL/stress/static-contract verification;
- explicit bench-validation and single-SMB/five-SMB configurations;
- RTOS/CAN robustness review and target-validation hooks.

Current synchronized source revision marker: `DER26-AMS-v0.5.30-20260906`.

For exact historical changes, use repository history and release tags.


## Final pre-Zephyr software-freeze candidate — 2026-09-06 (package v2.6.27, AMS v0.5.30)

- Explicit ISR/task visibility for SCE-written CAN/BMS safety scalars and CAN-completion charger status/counters.
- Final charger ENABLE authority revalidation at the bxCAN hardware-load boundary, converting stale enable to zero-demand disable.
- BOFF-sequence-stable recovery settlement so a new SCE event cannot be committed away.
- Atomic service-recovery commit across transport latch/window, application CAN fault state, authority baseline/refresh state, policy latch, and recovery accounting; CLI no longer clears CAN safety state after return.
- Focused CAN regression source expanded to 19 cases, including a new post-success-recovery BOFF-survival case.
- No v2.6.27 regression-suite rerun is claimed; final host validation is intentionally delegated to the recipient before baseline freeze.


## Final repeated bus-off closeout — 2026-09-06 (package v2.6.26, AMS v0.5.29)

- CAN1 SCE is the target-side source of physical BOFF event identity; sticky HAL error state no longer creates event identity.
- BOFF transitions use an ISR-side monotonic sequence so multiple events cannot collapse before the 10 Hz CAN task.
- The three-events-in-10-seconds rule is an exact sliding window and latches TX/BMS low on the third event boundary.
- Later BOFF events during pending recovery are retained and do not move the first event's 500 ms discharge deadline.
- Task recovery consumes the event-sequence delta and preserves host fault-injection compatibility.
- Comprehensive host injection, focused CAN scheduler/transport, profile gates, review-fix gates, measurement-integrity tests, and whole-source GCC analysis pass.

## Pre-Zephyr freeze hardening — 2026-09-06 (package v2.6.25, AMS v0.5.28)

Closed the final clean-pass CAN edge cases before freezing the FreeRTOS comparison
baseline. DISCHARGE recovery now accepts on-wire completion evidence only after the
old controller epoch has been fully settled and the post-recovery authority baseline
has advanced, preventing a delayed pre-bus-off callback from satisfying the 500 ms
rule when the CAN task is delayed. Bench `fault inject canbusoff` now shares the same
event-state path as physical BOFF. Repeated identical soft CAN errors refresh their
fault age, and failed manual recovery no longer partially clears the repeated-bus-off
TX latch/window.

Focused host regression covers delayed settlement, stale old-epoch completion, exact
499/500/501 ms behavior, tick wrap, physical-equivalent fault injection, repeated ACK
error aging, and failed service recovery. Comprehensive test/safety-test, review/profile/
CAN-contract gates, measurement-integrity tests, and whole-source GCC analysis pass.
No ARM target build, flash, or physical timing validation is claimed. See
`AMS/docs/PRE_ZEPHYR_FREEZE_HARDENING_v2.6.25.md`.


## Safety-policy closeout — 2026-09-05 (package v2.6.23, AMS v0.5.26)

Closed F-05 and F-06 before the Zephyr migration baseline is frozen. IWDG is now
a software-liveness/integrity watchdog: external battery/sensor/charger/CAN/ADBMS/
fuse faults fail BMS_OK low without creating a watchdog reset loop, while panic,
critical heartbeat loss, RTOS fault, critical stack margin, and explicit stop-feed
conditions remain reset-worthy.

CAN bus-off policy is explicit by state. START requires every required frame in
a fresh protected `0x680`–`0x687` generation to complete on wire before transition;
CHARGE, BALANCE, and CAN-backed HIL fail low immediately; DISCHARGE permits one
continuous transient recovery epoch, with ECU torque authority required stale by
300 ms and AMS fail-low at 500 ms if a fresh required protected generation has not
completed on wire and restored authority. Three bus-offs inside 10 s or the
application TX latch become sticky hard faults. Electrical ABOM recovery, manual
service recovery, and software queue/commit acceptance never grant authority by
themselves.

Host regression includes exact 500 ms and tick-wrap boundaries, service recovery,
HIL replacement, repeated bus-off, watchdog external-fault survival, RTOS-integrity
watchdog blocking, and bench/vehicle whole-source analysis. No ARM target build,
flash, or new physical CAN/watchdog evidence is claimed. See
`AMS/docs/SAFETY_POLICY_CLOSEOUT_v2.6.23.md`.

## CAN scheduler fixes — 2026-09-04 (package v2.6.17, AMS v0.5.20)

Removed the full-generation detail stack temporary. Required completion now
uses frame classes and validates the required count. Fast tuning yields to a
pending base snapshot. Fixed mailbox selection respects both hardware flags and
software ownership; all CAN vectors defer refill until HAL dispatch finishes.
Bus-off recovery waits for hardware BOFF and outstanding requests to settle,
then waits for fresh charger/protected publication. Charger TX chronology and
shutdown acknowledgment identity are separate, and obsolete loaded critical
commands receive abort requests. Repeated-bus-off recovery is counted once.

See `AMS/docs/CAN_SCHEDULER_FIXES_2026-09-04.md` for focused verification and
target-test limits. Battery estimator, SoH, SoP, fuse and MiL logic are unchanged
from package v2.6.16.


## Production acquisition candidate — 2026-08-29

Without changing the synchronized `DER26-AMS-v0.5.15-20260826` release marker, the
current development tree adds a production-C candidate for robust estimator startup:
constrained dynamic SoC correction while acquisition is unresolved, retryable 20 s
fixed-basis relaxation acquisition, cross-segment consensus, current-confidence /
uncertainty gating, conservative covariance and adaptive-R reset on anchor, acquisition
telemetry, SoH advisory gating, and segment-local estimator validity. A C regression
also drove a float32-stable centered sufficient-statistics implementation for the
fixed-basis fit. The same development line now retains the full 3x3
`[SoC,Vp1,Vp2]` covariance with production NIS/NEES telemetry and covariance
fail-closed guards, and production SoP has an explicit fatal unacquired-estimator
authority reason. The synchronized release marker remains v0.5.15.


## Full covariance observability / runtime safety close-out — 2026-08-29

The development candidate completes live observability for the full estimator
covariance by bumping the passive logger schema to protocol v4, adding signed
cross-covariance and acquisition pages, exact innovation sigma, and covariance-repair
telemetry. CAN1 TX and SCE interrupt handlers/NVIC configuration now match the
notifications enabled by the asynchronous transport, and fatal RTOS hooks make the
BMS fail-low panic precede diagnostic bookkeeping. Static gates preserve these
contracts. The synchronized firmware revision marker remains v0.5.15.

## Final pre-Zephyr code fixes — 2026-09-05 (package v2.6.24, AMS v0.5.27)

Closed four defects from the final clean FreeRTOS pass. CAN bus-off safety timing now
starts from the physical BOFF ISR event rather than the later 10 Hz CAN task poll; the
ISR also records the operating-state classification and performs immediate fail-low for
CHARGE, BALANCE, and CAN-backed HIL. The protected CAN scheduler now records the exact
on-wire completion tick for the required `0x680`–`0x687` generation, so DISCHARGE
recovery is decided from event time with exact 499/500/501 ms and tick-wrap semantics,
independent of CAN-task/error-task phasing. Manual service recovery now uses a separate
`can_authority_refresh_pending` state instead of pretending a fresh physical bus-off
epoch occurred, avoiding CHARGE/BALANCE re-latch races. Finally,
`AMS_ENABLE_TUNING_CAN` moved to common configuration so the estimator producer and CAN
consumer compile the same telemetry path; whole-source analysis now treats undefined
preprocessor identifiers as errors.

Host verification passed `firmware-ci`, comprehensive `test`/`safety-test`, CAN TX
scheduler/regression tests, measurement-integrity tests, profile/review/contract gates,
whole-source GCC analysis with `-Wundef -Werror=undef`, and focused Clang analysis of
the changed CAN/safety/estimator files. No ARM target build, flash, or physical timing
validation is claimed. Battery models and estimator equations were not retuned. See
`AMS/docs/PRE_ZEPHYR_FINAL_CODE_FIXES_v2.6.24.md`.

## Post-closeout regression fixes — 2026-09-05 (package v2.6.22, AMS v0.5.25)

Closed four defects found by the follow-up review of v2.6.21. The headless
Release path now explicitly selects an AMS build profile (default profile 5 /
BENCH_VALIDATION), matches the checked-in CubeIDE Release optimization
(`-Os -g0`), and records the profile plus an immutable source-input-tree hash in
build provenance. ADBMS AUX2 redundancy scheduling now re-arms from the current
epoch after startup or transport loss and skips missed slots instead of replaying
a backlog. SoP and SoH now advance stored cell and temperature ages to solve time,
with a separate 1 s thermal freshness bound matching the healthy eight-position
thermistor scan cadence. Three negative authority-profile compile tests now define
the expected profile and verify the exact intended compile-time diagnostic, so an
unrelated compiler failure cannot produce a false pass.

Host validation passed `measurement-integrity-test`, `profile-gates`,
`review-fixes-test` (15/15), `adbms-v05-contract-gate`, `whole-source-analyze`,
`power-core`, and `unit`. No ARM target build, flash, or new hardware validation
is claimed. No battery-model parameters or estimator equations were retuned. See
`AMS/docs/POST_CLOSEOUT_REVIEW_FIXES_v2.6.22.md`.

## Migration-readiness code closeout — 2026-09-05 (package v2.6.21, AMS v0.5.24)

Reviewed the Zephyr migration-plan open findings against the v2.6.20 source and
closed the source-level items that can be corrected without changing battery or
safety-policy behavior. Vehicle now has an independent compile-time diagnostic
CLI/UART gate (`AMS_ENABLE_CLI=0`), while bench/HIL/test-day images retain the
shell. Qualified headless Release builds require `SOURCE_DATE_EPOCH`, emit build
provenance plus artifact hashes, and gate the exact bundled FreeRTOS source tree
by deterministic SHA-256. CubeMX DER26 identity and CAN ISR documentation/IRQ
gates were already correct in the supplied source; the migration plan was stale
on those two items. The String-B SPI chip-select remains PE4; the PF4 schematic
annotation is treated as a labeling error.

No watchdog policy, CAN bus-off policy, current calibration, BMS authority,
balancing, estimator, SoP/SoH/fuse, CAN scheduler, or hardware pin behavior was
changed. See `AMS/docs/OPEN_FINDINGS_CODE_CLOSEOUT_v2.6.21.md`.

## Follow-up review — 2026-09-05 (package v2.6.20, AMS v0.5.23)

Confirmed the four v2.6.18 current-window/CAN defects remain fixed. CAN now
expires individual cell/temperature readings at encoding time and uses the
resulting masks for aggregates, ECU/logger detail values and usable masks.
SoP/SoH reject the UINT16_MAX unknown-current-uncertainty sentinel as
calibration evidence. No RTOS migration or validation-gate changes.

See `AMS/docs/FOLLOWUP_REVIEW_v2.6.20.md` for reproductions, scope and limits.
