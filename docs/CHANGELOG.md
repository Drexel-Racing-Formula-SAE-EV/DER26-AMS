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

Current synchronized source revision marker: `DER26-AMS-v0.5.15-20260826`.

For exact historical changes, use repository history and release tags.


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
