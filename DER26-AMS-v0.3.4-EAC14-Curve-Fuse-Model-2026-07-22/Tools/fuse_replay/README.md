# DER26 EAC14-80 curve observer, independent oracle, and trace replay

This host-only tool validates and characterizes the preliminary Eaton
EAC14-80 main-fuse observer in `ams_fuse_observer.c`. It does not enable fuse
model authority and it does not replace installed-vehicle testing.

## Physical input used

The model uses the January 2026 Eaton ELX1308 data for the EAC14-80:

- Rounded points digitized from the **80 A current-versus-time centerline** on
  page 4, over the charted range of approximately 0.01 to 100 seconds.
- The tabulated **8020 A²s typical melting I²t at 10 In** on page 2, used only
  to anchor the 800 A point at `8020 / 800² = 0.01253125 s`.
- Rounded points from the page-4 ambient-temperature derating curve.

The curves are typical centerlines, not guaranteed-minimum limits. The default
`curve_time_fraction = 0.25` is therefore a commissioning-only margin, not a
validated safety factor. Currents below the lowest digitized point and above
the highest point are explicitly flagged as extrapolated.

The implementation is locked to the EAC14-80 rating. It does not silently
rescale this curve for another fuse rating.

## State model

The observer carries a dimensionless thermal/damage utilization state `x`.
`x = 1` is the preliminary curve-derived exhaustion boundary.

For an effective current, the temperature-derated equivalent current at 25 °C
is found first. The digitized curve supplies a typical melt time `t_typ`. The
usable commissioning time is:

```text
t_usable = curve_time_fraction * t_typ
```

A source rate is selected so the exact first-order thermal model reaches
`x = 1` after `t_usable` from a cold state under constant current:

```text
x_dot = -x / tau + source_rate(Ieq25)
source_rate = 1 / [tau * (1 - exp(-t_usable / tau))]
```

The production observer deliberately uses a conservative discrete input term:

```text
x[k+1] = x[k] exp(-dt/tau) + source_rate[k] dt
```

The independent reference uses `long double` and the exact zero-order-hold
convolution:

```text
x[k+1] = x[k] exp(-dt/tau)
       + source_rate[k] tau (1 - exp(-dt/tau))
```

For positive `dt`, the production input term is slightly larger. Expected
safety direction:

```text
production utilization >= exact-reference utilization
production current cap  <= exact-reference current cap
```

The exact reference is separately checked against a high-resolution
Heun/trapezoidal numerical integration. It does not include or call the
production observer.

## Important limitations

This model is preliminary because the published data does not provide:

- Guaranteed-minimum pre-arcing time-current or I²t curves.
- Production-tolerance bands around the plotted centerline.
- Repetitive-pulse aging or damage data for this installation.
- Holder, busbar, enclosure, and airflow thermal behavior.
- Direct fuse-element temperature measurement in the current vehicle.

The current low-current extension below the 100-second chart boundary is an
explicit power-law extrapolation. It is useful for replay and sensitivity work,
but it must not be interpreted as a manufacturer guarantee.

Keep `AMS_FUSE_MODEL_VALIDATED=0` until the evidence requirements in
`Docs/EAC14_80_MANUFACTURER_DATA_REQUEST.md` and the vehicle validation plan
are closed.

## Build and validate

```bash
cd Tools/fuse_replay
make
make traces
make smoke-ci
make policy-sweep
make asan
```

The permanent CI oracle is run from:

```bash
cd AMS/host_tests
make fuse-oracle
```

It covers:

- Curve and temperature anchors.
- Exact solution versus independent numerical integration.
- Directed production/reference comparisons.
- 50,000 randomized production/reference updates.
- Invalid-input fail-closed behavior.
- Nonconservative state, cap, and latch detection.

## Replay a trace

```bash
./fuse_replay \
  --trace traces/single_100a_1p5s.csv \
  --output results/single_100a_1p5s_replay.csv \
  --summary results/single_100a_1p5s_summary.csv \
  --strict
```

The detailed output contains:

- Production and reference utilization.
- Remaining normalized headroom.
- Temperature-equivalent 25 °C current.
- Typical and commissioning-usable melt times.
- 0.1, 1, 10, and 30-second caps.
- Latch and authority state.
- Extrapolation and reason flags.
- Per-sample production/reference differences.

## Accepted trace columns

Required:

```text
timestamp_ms,current_a
```

`time_s` may replace `timestamp_ms`. Optional columns:

```text
current_uncertainty_a
temperature_proxy_c
measurement_valid
current_calibrated
current_polarity_validated
temperature_measured_at_fuse
model_validated
event
```

`ambient_temp_c` is also accepted as the temperature proxy. `event=reset`
applies the selected reset policy before processing that row.

The first row establishes the initial timestamp and is not integrated. Each
later row is treated as the measurement for the preceding interval, matching
the production task call pattern.

## Characterization controls

```text
--curve-time-fraction VALUE   Fraction of the typical centerline time
--cooling-tau-s VALUE         Thermal-memory time constant
--startup cold-soak|known-cold|seeded:UTIL
--reset unknown|known-cold|restore|seeded:UTIL
```

The fuse rating is intentionally not configurable: this implementation is for
the installed EAC14-80 curve only.

Startup and reset policies are replay experiments. They are not implemented as
new vehicle behavior by this update.

## Synthetic traces

`generate_synthetic_traces.py` creates deterministic traces for:

- 100 A pulses of 0.5, 1.0, 1.5, and 3.0 seconds.
- Repeated corner-exit pulses and regenerative intervals.
- Synthetic autocross and 20-minute endurance profiles.
- 160 A / 20 s and 200 A / 5 s curve-stress cases. These exceed normal static
  vehicle current ceilings and exist only to exercise the fuse model.
- Reset and invalid-input cases.

`run_policy_sweep.py` compares startup, reset, curve-time-fraction, and cooling
policies. Its output is characterization evidence, not final calibration.
