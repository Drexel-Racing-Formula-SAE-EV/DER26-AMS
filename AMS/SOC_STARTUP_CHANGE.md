# Temporary startup SoC prior — profile 6 only

- `AMS_EKF_DEFAULT_SOC_INIT` is `0.27f` **only** for `AMS_PROFILE_CHARGE_REVIEW` (profile 6). Other profiles retain `1.0f`.
- Seeds all configured segment EKFs and fallback coulomb-counted SoC consistently. No estimator logic, charging authority, voltage thresholds, or CAN command behavior was changed.
- Derived from the roughly 3.55 V/cell Molicel P42A readings in an earlier v0.5.48 log, assuming a rested pack near 25 C. **Not a current measured SoC**; treat as provisional and potentially stale if the pack was charged/discharged after that log.
- Future implementation: persist last validated SoC with CRC/version/pack ID, acquisition timestamp and age checks, then use rested OCV if available; otherwise fall back to a clearly marked estimate. No persistent storage or restore was added in this revision.
- Status: profile-6 SoC initialization and charge-review host tests pass, profile-4 regression startup check passes; not an ARM compiled/flash-ready image. The prior hardware/charging-safety limitations remain unchanged.
