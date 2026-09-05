# AMS v0.5.16 Resistance-SoH Transient Hardening

Date: 2026-09-03
Source revision: `DER26-AMS-v0.5.16-20260903`

This candidate fixes a production false-ageing failure exposed by the licensed
v2.6.8 MiL C5 run. The five segment EKFs had acceptable R0 p95 error (~5.8-7.5%),
but one qualified high R0 observation could be retained forever by `ams_soh.c`,
causing nominal resistance growth to latch at 1.161 versus a 1.0096 plant target.

Changes:

- Resistance SoH now consumes only **fresh R0 observations**: the upstream
  estimator record must have both `LAST_OBSERVABLE` and `ADVISORY_VALID`.
- Slow ageing retention requires the median of **9 distinct qualified R0
  observations** before a segment is first accepted or its monotonic retained
  upper bound is increased.
- Confirmed ageing remains monotonic/persistent: later healthy observations do
  not erase a previously confirmed high-resistance state.
- Pending confirmation windows are intentionally transient and are not written to
  persistence records; reboot therefore cannot promote unconfirmed ageing.
- Added host regression proving one 60% R0 spike among eight healthy qualified
  observations does not create permanent ageing, while nine sustained 40% ageing
  observations do increase the retained state.

This does not relax SoP authority, fuse limits, current calibration/polarity gates,
capacity-SoH confidence gates, cell-voltage limits, or the underlying EKF R0 update
criteria. Instantaneous estimator/SoP behavior remains immediate; only the slow
retained ageing state gets confirmation filtering.

Verification in the packaging environment:

- MiL `make check`: PASS
- SoP/SoH production host suite: PASS
- new resistance transient-spike regression: PASS
- SoP/SoH ASan+UBSan suite: PASS
- whole-source GCC analyzer, bench + vehicle profiles: PASS

Licensed MATLAB C5/C0-C8 rerun remains required.
