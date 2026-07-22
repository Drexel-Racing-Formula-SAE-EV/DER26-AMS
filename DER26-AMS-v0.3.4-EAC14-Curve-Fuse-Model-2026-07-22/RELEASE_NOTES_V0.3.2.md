# DER26 AMS v0.3.2 — independent fuse oracle and replay tooling

**Date:** 2026-07-22

## Added

- Independent `long double` exact-ZOH fuse observer oracle.
- Independent Heun/trapezoidal self-check of the reference integration.
- 50,000-update randomized production/reference CI comparison.
- Strict CSV replay with production/reference state and all horizon caps.
- Synthetic pulse, corner-exit, autocross, endurance, reset, and invalid-input traces.
- Startup/reset/calibration policy sweep.
- `make fuse-oracle` CI gate.
- `make fuse-replay` non-gating characterization target.
- ASan/UBSan coverage for the fuse oracle.

## Validation

- 50,000 randomized updates: pass.
- 142 replay/policy cases: pass.
- Production underestimation violations: zero.
- Nonconservative cap violations: zero.
- Nonconservative latch transitions: zero.
- Production firmware logic changes: none.

## Important result

The current 25%-of-typical-I²t commissioning budget exhausts in approximately
0.5 seconds for the included 100 A / 0.5 A uncertainty / 45°C estimated-fuse
case and requires minutes to clear its 50% hysteresis. The model remains gated
from vehicle authority pending installed-fuse and real-current calibration.

A fixed 50–80% warm-reset seed is not safe for every reset because the actual
pre-reset modeled state may already exceed 100%. Trusted state restore or a
fail-zero/exhausted fallback is required.
