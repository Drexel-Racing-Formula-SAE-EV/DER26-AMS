# DER26 fuse oracle, replay, and policy characterization

> **Superseded calibration notice — v0.3.4:** This report records the v0.3.2
> fixed-I²t placeholder study. Its numerical exhaustion predictions must not be
> used for the current model. The independent-oracle infrastructure remains
> relevant, but the production observer now uses the EAC14-80 time-current curve.
> See `EAC14_80_CURVE_MODEL_UPDATE_2026-07-22.md`.


**Date:** 2026-07-22  
**Scope:** host tooling and CI around `ams_fuse_observer.c`  
**Production firmware behavior changed:** no

## 1. Work completed

This update adds two independent layers:

1. A standalone, high-precision fuse reference model that does not call or
   include the production observer.
2. A CSV replay and policy-characterization tool that runs every sample through
   both the production observer and the reference model.

The reference uses the exact zero-order-hold solution of the leaky excess-I²t
model. A separate high-resolution Heun/trapezoidal integrator checks the exact
reference itself. This prevents the replay/calibration tool from simply
reproducing an error in the production observer.

Permanent CI coverage was added as `make fuse-oracle`. The test includes 50,000
randomized updates plus directed initialization, cooling, invalid-input,
derating, cap, and latch checks. ASan and UBSan pass.

## 2. Independent-oracle result

The production observer uses `q*dt` for new heat input while applying the exact
exponential cooling factor to the old state. The independent oracle uses the
exact convolution `q*tau*(1-exp(-dt/tau))`.

The production update is therefore slightly conservative for the same sampled
current. Across the completed validation:

- 50,000 randomized CI updates passed.
- 142 replay/policy cases passed strict comparison.
- No production thermal-state underestimate occurred.
- No production cap exceeded the exact-reference cap beyond tolerance.
- No nonconservative latch transition occurred.
- No state, validity, or authority mismatch occurred.
- Maximum state relative difference across the 142 cases was approximately
  `0.0004965` (0.04965%).
- Maximum same-latch current-cap difference was approximately `0.155 A`.
- Twenty-three samples had conservative one-sample latch skew; none were
  nonconservative.

This validates the implemented integration direction and numerical behavior.
It does not validate the installed-fuse calibration.

## 3. Default-model pulse results

The default synthetic pulse uses:

- Current command: 100 A.
- Current uncertainty: 0.5 A.
- Temperature proxy: 30°C.
- Unmeasured-fuse margin: +15°C.
- Estimated fuse temperature: 45°C.
- Temperature-derated continuous current: approximately 77.09 A.
- Usable budget: `8020 * 0.25 = 2005 A²s`.

The resulting excess rate is approximately `4157.24 A²`. Ignoring the small
cooling term, the 2005 A²s budget is consumed in about 0.482 s. The sampled
replay exhausts at 0.5 s, as expected.

| 100 A pulse | Peak production utilization | Exhausted | Recovery to 50% latch clear |
|---:|---:|---:|---:|
| 0.5 s | 1.036 | Yes, at 0.5 s | about 218.5 s after pulse end |
| 1.0 s | 2.070 | Yes, at 0.5 s | about 426.2 s after pulse end |
| 1.5 s | 3.103 | Yes, at 0.5 s | about 547.6 s after pulse end |
| 3.0 s | 4.000 state clamp | Yes, at 0.5 s | about 623.8 s after pulse end |

For the 1.5-second pulse at a 300-second cooling constant:

| Usable fraction | Budget | First exhaustion | Peak utilization | Recovery time |
|---:|---:|---:|---:|---:|
| 25% | 2005 A²s | 0.5 s into pulse | 3.103 | 547.6 s after pulse end |
| 50% | 4010 A²s | 1.0 s into pulse | 1.551 | 339.7 s after pulse end |
| 75% | 6015 A²s | 1.5 s into pulse | 1.034 | 218.0 s after pulse end |

All three tested fractions still exhaust on this 1.5-second pulse. The result
shows that the existing constants are commissioning placeholders and are not
ready for authority. It does not establish what the correct fraction should
be; that requires the installed fuse's guaranteed time-current behavior and
real vehicle traces.

The repeated-corner and synthetic-endurance traces reached the configured
`maximum_state_multiple = 4` under every tested 25%, 50%, and 75% budget and
120 s, 300 s, and 600 s cooling constant. These traces are intentionally harsh
and synthetic, so this is a model-characterization result rather than vehicle
evidence.

## 4. Startup-policy result

With the existing 300-second continuous low-current requirement:

- Cold-soak authority became valid at 300 seconds in a trace containing 305
  seconds of idle.
- Any current excursion above 5 A before completion resets the soak timer.
- Known-cold and seeded startup policies became authoritative on the first
  100 ms update.
- A seeded state preserves less initial headroom, as intended.

No production startup behavior was changed.

## 5. Warm-reset result

Two reset traces were evaluated.

### Reset before exhaustion

The modeled utilization immediately before reset was 0.2606.

| Reset policy | Seed after reset | Post-reset authority | Post-reset exhaustion |
|---|---:|---:|---:|
| Unknown/current behavior | 0.000 | Never recovered in the remaining high-current trace | 0.7 s after reset, but non-authoritative |
| Known cold | 0.000 | Immediate | 0.7 s after reset |
| Fixed warm seed | 0.500 | Immediate | 0.3 s after reset |
| Fixed warm seed | 0.800 | Immediate | 0.1 s after reset |
| Restore | 0.2606 | Immediate | 0.5 s after reset |

### Reset after exhaustion

The modeled utilization immediately before reset was 3.9339.

| Reset policy | Seed after reset | Post-reset authority | Post-reset exhaustion |
|---|---:|---:|---:|
| Unknown/current behavior | 0.000 | Never recovered in the remaining high-current trace | 2.7 s after reset, but non-authoritative |
| Known cold | 0.000 | Immediate | 2.7 s after reset |
| Fixed warm seed | 0.500 | Immediate | 2.3 s after reset |
| Fixed warm seed | 0.800 | Immediate | 2.1 s after reset |
| Restore | 3.9339 | Immediate | Already exhausted at reset |

The fixed 50–80% warm seeds are not universally conservative. If the actual
pre-reset state can already exceed 100%, those seeds erase exhaustion and
briefly restore nonzero capability. Therefore:

1. A trusted persisted restore is the preferred continuity mechanism.
2. Persistence must have CRC, version, bounds, age, and reset-reason checks.
3. When no trusted state exists after a warm/on-track reset, fail-zero unknown
   state is safe but may remain non-authoritative indefinitely.
4. A fixed fallback intended to remain authoritative must seed at or above the
   exhausted threshold, not 50–80%, unless another mechanism proves a tighter
   upper bound on pre-reset utilization.
5. `known-cold` is acceptable only when the fuse is genuinely known cold; it is
   not a valid generic MCU-reset policy.

These are design conclusions only. No persistence or reset policy was added to
production firmware.

## 6. CI and tooling added

New files:

```text
Tools/fuse_replay/fuse_reference_oracle.[ch]
Tools/fuse_replay/fuse_replay.[ch]
Tools/fuse_replay/fuse_replay_main.c
Tools/fuse_replay/generate_synthetic_traces.py
Tools/fuse_replay/run_policy_sweep.py
Tools/fuse_replay/Makefile
Tools/fuse_replay/README.md
AMS/host_tests/fuse/fuse_oracle_test.c
AMS/host_tests/fuse/Makefile
```

New host targets:

```bash
make fuse-oracle   # CI gate: independent oracle + replay smoke tests
make fuse-replay   # non-gating policy characterization
```

`make ci` now includes `fuse-oracle`. `make asan` includes the fuse-oracle
sanitizer target.

## 7. What remains before fuse authority

- Obtain guaranteed fuse time-current and minimum-energy data, not only a
  typical melting-I²t number.
- Confirm the exact installed fuse, holder, busbar, enclosure, and cooling.
- Replay logged acceleration, autocross, and endurance pack-current traces.
- Measure fuse/holder temperatures during representative repeated pulses.
- Select and document the thermal initialization/reset/persistence policy.
- Confirm authority, recovery, and torque behavior in ECU/vehicle SIL.
- Keep `AMS_FUSE_MODEL_VALIDATED=0` until those evidence gates are closed.
