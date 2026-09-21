# AMS v0.5.17 Resistance-SoH Correlated-Episode Hardening

Date: 2026-09-03
Source revision: `DER26-AMS-v0.5.17-20260903`

Licensed v2.6.11 C5 showed that raw production R0 was accurate (roughly
5.8-7.5% p95) while aggregate resistance SoH still false-aged to 1.1393 versus
a 1.0096 plant target.  Retained-result diagnostics proved the failure shape:
32 fresh R0 observations arrived as one correlated excitation episode.  The
first nine-sample block median was 1.114-1.139 across segments, then successive
blocks decayed to roughly 1.084-1.108 and 1.061-1.082.  v0.5.16 committed the
first block before the episode had ended.

Changes:

- resistance SoH now treats contiguous fresh R0 updates as one correlated
  observation episode rather than independent ageing samples;
- an episode closes only after 2.5 s without a qualified fresh R0 update;
- at least 9 observations are required before an episode can update SoH;
- up to the latest 33 qualified ratios are retained in a fixed static ring;
- the robust episode median is computed only after the episode closes, preventing
  the leading polarization transient from becoming a permanent monotonic ageing
  state;
- confirmed ageing remains monotonic and persistence behavior remains unchanged;
- pending episode samples are transient and are not written to persistence;
- new host regression reproduces a 32-s 1.14->1.06 correlated decay and proves it
  is not latched at the leading-edge value, while a sustained 1.40 episode still
  confirms genuine resistance growth.

This change does not alter EKF Q/R, covariance floors, SoP authority, fuse limits,
cell-voltage limits, capacity-SoH gates, or R0 update qualification.

Packaging-environment verification:

- `make -C MiL check`: PASS
- SoP/SoH production host core: PASS
- correlated-episode regression: PASS
- SoP/SoH ASan+UBSan: PASS
- whole-source GCC analyzer, bench + vehicle profiles: PASS

Licensed MATLAB C5/core rerun remains required.
