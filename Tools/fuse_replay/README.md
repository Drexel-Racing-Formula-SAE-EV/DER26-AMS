# DER26 main-fuse oracle and trace replay

This tool validates and characterizes the production
`ams_fuse_observer.c` without using that implementation as its own oracle.
It is host-only engineering tooling; it does not alter vehicle firmware
behavior or enable fuse-model authority.

## Why two implementations exist

The production observer advances the dimensionless thermal-utilization state with the
conservative discrete update

```text
x[k+1] = x[k] exp(-dt/tau) + q[k] dt
```

The independent reference uses `long double` and the exact zero-order-hold
solution of the continuous model:

```text
x[k+1] = x[k] exp(-dt/tau)
       + q[k] tau (1 - exp(-dt/tau))
```

The reference is implemented from scratch in
`fuse_reference_oracle.c`. It does not include or call the production fuse
observer. The exact reference is also checked against an independently
substepped Heun/trapezoidal integration in the CI test.

For positive `dt`, the production `q*dt` input term is slightly larger than
the exact convolution term. The expected relationship is therefore:

```text
production thermal state >= exact reference thermal state
production current cap    <= exact reference current cap
```

The strict replay checks those safety directions as well as bounded numeric
agreement. Conservative one-sample latch skew is reported separately and is
allowed; a nonconservative production latch transition fails.

## Build and validate

```bash
cd Tools/fuse_replay
make
make traces
make smoke
make policy-sweep
```

The permanent CI oracle is run from:

```bash
cd AMS/host_tests
make fuse-oracle
```

That target runs:

- Directed exact-reference checks.
- Exact solution versus high-resolution trapezoidal integration.
- Temperature-derating checkpoints.
- 50,000 randomized production-versus-reference updates.
- Invalid-input fail-closed checks.
- CSV replay smoke tests including invalid samples and a reset event.

## Replay a trace

```bash
./fuse_replay \
  --trace traces/single_100a_1p5s.csv \
  --output results/single_100a_1p5s_replay.csv \
  --summary results/single_100a_1p5s_summary.csv \
  --strict
```

The detailed output contains production and reference state, utilization,
remaining budget, all four discharge and charge horizon caps, latch state,
authority state, reason flags, and numeric deltas for every sample.

## Accepted trace columns

Required:

```text
timestamp_ms,current_a
```

`time_s` may replace `timestamp_ms`. Optional columns are:

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

`event=reset` applies the selected reset policy before processing that row.
A three-column trace such as
`timestamp_ms,current_a,ambient_temp_c` is accepted because
`ambient_temp_c` is recognized as the temperature proxy.

The first row establishes the initial timestamp and is not integrated. Each
later row is treated as the measurement for the interval since the previous
row, matching the production task call pattern.

## Initialization policies

Startup:

```text
conservative    Production behavior: seed maximum utilization and exhausted
cold-soak       Characterization-only low-current soak from a zero-state prior
known-cold      Initialized immediately at zero utilization
seeded:0.50     Initialized immediately at a specified utilization
```

Reset:

```text
unknown         Production behavior: reset to conservative maximum/exhausted
known-cold      Reset to zero utilization and initialized
seeded:0.80     Reset to a specified utilization and initialized
restore         Restore the pre-reset modeled state
```

The conservative startup and unknown-reset policies mirror production. The
other policies remain characterization options and require external evidence
before they can represent a vehicle startup.

## Synthetic traces

`generate_synthetic_traces.py` creates deterministic traces for:

- 100 A pulses of 0.5, 1.0, 1.5, and 3.0 seconds.
- Repeated corner-exit pulses with regenerative intervals.
- A synthetic autocross profile.
- A 20-minute synthetic endurance profile with rising temperature.
- Resets before and after budget exhaustion.
- Invalid measurement/calibration/polarity samples.

`run_policy_sweep.py` compares startup, reset, usable-budget, and cooling-time
policies. Its output is characterization evidence, not a final calibration.
Real fuse, holder, busbar, enclosure, and vehicle-current evidence is still
required before setting `AMS_FUSE_MODEL_VALIDATED`.
