# DER26 EAC14-80 curve-model update and validation

**Date:** 2026-07-22  
**Repository target:** v0.3.4  
**Scope:** Main-fuse observer, independent oracle, trace replay, documentation

## Executive result

The previous fixed `8020 A²s × 25%` accumulator has been replaced by a
preliminary **time/current-dependent EAC14-80 curve observer**.

The implementation now uses:

- The 80 A current-versus-time centerline digitized from Eaton ELX1308 page 4.
- The page-2 8020 A²s typical melting value only as an exact 800 A anchor:
  `8020 / 800² = 0.01253125 s`.
- The page-4 ambient-temperature derating curve.
- A normalized thermal/resource state with first-order cooling.
- An independent `long double` exact-state oracle and separate numerical
  integration check.

The change fixes the model-form error of treating one high-current typical I²t
number as a universal pulse budget. It does **not** close the final calibration
gap: the published curves are typical centerlines, not guaranteed-minimum
boundaries.

`AMS_FUSE_MODEL_VALIDATED` remains zero by default and no vehicle authority was
enabled.

## Source interpretation

The EAC14-80 data sheet states:

- Rated current: 80 A.
- Typical melting I²t: 8020 A²s.
- The 8020 A²s value is measured at 10 In, or 800 A.
- A current-versus-time curve and temperature derating curve are provided.
- At 3 In, the stated electrical characteristic spans 0.1 to 15 seconds.

The broad 3 In range and absence of plotted tolerance bands prevent the
centerline from being treated as a guaranteed minimum.

## Curve representation

The production and independent reference each contain separately transcribed
rounded points from the 80 A current-versus-time curve over approximately
0.01–100 seconds. Log-log interpolation is used between points.

- Below the 100-second chart boundary, a fitted low-current asymptote is used:

```text
t = 75.6 * (I / 80 - 1)^(-3.93)
```

- Above the highest point, the final log-log slope is extended.
- Any extrapolated result sets `AMS_FUSE_REASON_CURVE_EXTRAPOLATED`.
- The configuration is locked to the EAC14-80 rating; it cannot silently reuse
  the curve for a different fuse rating.

The digitized inputs are committed as:

```text
Docs/eac14_80_typical_time_current_digitized.csv
Docs/eac14_temperature_derating_digitized.csv
```

## State model

Let `x` be dimensionless fuse utilization. The curve provides a typical melt
time for the temperature-equivalent 25 °C current. The commissioning-usable
curve time is:

```text
t_usable = curve_time_fraction * t_typical
```

Default `curve_time_fraction = 0.25`.

The source rate is selected so the exact first-order model reaches `x=1` at
`t_usable` from a cold state under constant current:

```text
x_dot = -x / tau + r(I)
r(I) = 1 / [tau * (1 - exp(-t_usable / tau))]
```

Production uses the conservative discrete update:

```text
x[k+1] = x[k] exp(-dt/tau) + r(I[k]) dt
```

The independent oracle uses the exact zero-order-hold convolution:

```text
x[k+1] = x[k] exp(-dt/tau)
       + r(I[k]) tau (1 - exp(-dt/tau))
```

Horizon current caps are solved by bisection against `x_predicted <= 1` and
then intersected with the static SoP current ceilings.

## Validation completed

### Permanent oracle tests

Passed:

- Exact curve-state oracle versus high-resolution numerical integration.
- EAC14-80 curve anchors and temperature-derating anchors.
- Directed production/reference comparison.
- 50,000 randomized production/reference updates.
- Invalid-input fail-closed behavior.

### Production SoP integration

The complete production SoP/SoH core suite passed, including:

- Conservative fuse observer behavior.
- Randomized strategy/fuse invariants.
- Measurement-to-estimator-to-power integration.

### Replay sweep

The policy sweep executed 144 strict cases.

| Metric | Result |
|---|---:|
| Strict passes | 144 / 144 |
| Production state-underestimate violations | 0 |
| Nonconservative cap violations | 0 |
| Nonconservative latch violations | 0 |
| Result mismatches | 0 |
| Maximum utilization absolute delta | 0.00025855 |
| Maximum utilization relative delta | 0.00020766 |
| Conservative one-sample latch skews | 2 total |

The production approximation remained slightly conservative relative to the
exact oracle.

## Key synthetic characterization

These numbers describe the **preliminary model**, not guaranteed real-fuse
behavior. The traces include 300 seconds of low-current initialization, 0.5 A
current uncertainty, and a temperature proxy that produces approximately
45 °C estimated fuse temperature during the pulse.

| Trace | Peak utilization | Exhaustion | Recovery |
|---|---:|---:|---:|
| 100 A for 0.5 s | 0.001668 | No | N/A |
| 100 A for 1.5 s | 0.004997 | No | N/A |
| 100 A for 3.0 s | 0.009968 | No | N/A |
| Repeated corner-exit profile | 0.189232 | No | N/A |
| Synthetic autocross | 0.053413 | No | N/A |
| Synthetic endurance | 0.199549 | No | N/A |
| 160 A for 20 s | 1.521932 | 17.9 s into pulse | ~341 s after pulse start |
| 200 A for 5 s | 1.536114 | 3.2 s into pulse | ~341.7 s after pulse start |

The 160 A and 200 A traces deliberately exceed normal vehicle static current
ceilings. They exist to force curve-state exhaustion and recovery paths.

For the 100 A pulse, the model reports an effective temperature-equivalent
current of approximately 104.69 A, a typical extrapolated melt time near
7678 seconds, and a 25%-fraction usable time near 1919 seconds. This replaces
the earlier incorrect prediction of near-instant exhaustion from a fixed 2005
A²s bucket.

## What remains unvalidated

1. The time-current points are approximate digitizations of a typical curve.
2. The 25% time fraction is not derived from guaranteed production tolerance.
3. The low-current region relevant to 100–120 A is extrapolated beyond the
   plotted 100-second boundary.
4. The 300-second cooling constant is still a commissioning assumption.
5. Fuse temperature is proxied rather than measured directly.
6. Repeated-pulse aging and installed holder/busbar thermal effects are unknown.
7. Real acceleration, autocross, and endurance current traces have not yet been
   replayed.
8. Warm-reset persistence/initialization policy remains unresolved.

## Release rule

Do not enable fuse-model authority until:

- Eaton provides guaranteed-minimum/tolerance information or equivalent
  characterized evidence;
- the installed fuse and thermal path are tested;
- real vehicle traces are replayed;
- reset-state handling is validated; and
- the evidence revision is frozen in the vehicle build.

The exact manufacturer request is in
`Docs/EAC14_80_MANUFACTURER_DATA_REQUEST.md`.
