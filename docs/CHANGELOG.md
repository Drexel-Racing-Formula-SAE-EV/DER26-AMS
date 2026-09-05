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

Current synchronized source revision marker: `DER26-AMS-v0.5.23-20260905`.

For exact historical changes, use repository history and release tags.

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

## Follow-up review — 2026-09-05 (package v2.6.20, AMS v0.5.23)

Confirmed the four v2.6.18 current-window/CAN defects remain fixed. CAN now
expires individual cell/temperature readings at encoding time and uses the
resulting masks for aggregates, ECU/logger detail values and usable masks.
SoP/SoH reject the UINT16_MAX unknown-current-uncertainty sentinel as
calibration evidence. No RTOS migration or validation-gate changes.

See `AMS/docs/FOLLOWUP_REVIEW_v2.6.20.md` for reproductions, scope and limits.
