# Reusable accumulator model framework

## Quick start

From MATLAB R2025b or a compatible release:

```matlab
cd HiL/simulink/scripts
setup_hil
run_all_tests
```

Build a configured P42A/75s6p model:

```matlab
build_p42a_75s6p
```

Generate a staged ESP32 component:

```matlab
generate_esp32_plant
```

The script prints the parity command for the new artifact. Direct installation
is blocked: `qualify_generated_plant.py` strictly compiles the staged source,
compares it with the configured-Simulink reference, verifies the complete
manifest/hash chain, and leaves promotion to the ordered toolchain driver after
a staged ESP-IDF build succeeds.

`HIL_DATA_ROOT` must contain external raw datasets when refitting. If omitted,
the default is `simulink/data`, which is intentionally not populated.
`HIL_OUTPUT_ROOT` optionally redirects generated models, reports, and code.
Fresh release code generation also requires the configured independent
electrical holdout to execute and pass; an unavailable dataset cannot be
promoted as an implicitly validated plant.

## Configuration composition

```matlab
cell_cfg = hil.config.cell('p42a');
pack_cfg = hil.config.pack('der26_75s6p');
sim_cfg = hil.config.simulation('hppc_validation');
[params, build] = hil.build_parameters(cell_cfg, 'Save', false);
result = hil.run(cell_cfg, pack_cfg, sim_cfg, params);
files = hil.export_report(result);
```

The configuration hash includes all four layers and the parameter hash. Changing
topology, mappings, sample time, profile semantics, or parameters therefore
produces a different artifact identity.

`us06_25c`, `udds_25c`, and `la92_25c` expose the checked-in drive traces.
Synthetic HPPC, constant-current, constant-power, CSV, and MAT profiles use the
same profile loader and explicit scaling policy.

## Physics

The electrical model is OCV minus ohmic, fast-polarization, and
slow-polarization voltage:

```text
Vcell = OCV(SoC,T) - Icell*R0(SoC,T) - Vp1 - Vp2
Icell = Ipack / Np
Vpack = sum(Vgroup)
```

The thermal model is a two-node core/surface network using `Cc`, `Cs`, `Rcs`,
and `Rsa`. LUT temperature lookup is clamped to its characterized domain; state
temperature itself is not silently clamped.

The checked-in P42A source snapshot exactly reflects generated model 1.67:

- `Q_nom = 4.2 Ah`
- `R2 = 0.004 ohm`
- `C2 = 12000 F`
- `Cc = 55 J/K`
- `Cs = 15 J/K`
- `Rcs = 1.5 K/W`
- `Rsa = 8 K/W`
- temperature LUT breakpoints: 5, 25, and 40 degC

## Pack variation modes

| Mode | MATLAB reference | Generated model |
|---|---|---|
| `uniform` | Uniform group parameters/states | Uniform output image |
| `deterministic_spread` | Legacy regression image | Legacy-compatible image |
| `parameter_distributed` | Independent bounded group parameters/states | Code generation blocked |

The generated-model limitation is recorded in every model manifest. It is not
equivalent to the independently evolved reference.

## Data contract

Every adapter normalizes to:

- time in seconds with the first retained sample shifted to exactly `t=0`;
- cell current in amperes, positive for discharge;
- cell voltage in volts;
- surface and ambient temperature in degC;
- capacity in Ah;
- SoC in 0–1;
- explicit initial SoC, initial OCV, pulse SoC, and SoC provenance when
  supplied;
- an explicit surface-temperature evidence source;
- cell/test/source identity.

Normalization reports missing fields, rejected records, removed rows, duplicate
timestamps, sign handling, resampling, temperature availability, pulse count,
SoC coverage, and the declared source units. Generic adapters apply validated
unit conversion into seconds, amperes, volts, degrees Celsius, and amp-hours;
unsupported unit declarations fail instead of being silently treated as SI.
Ambient or chamber temperature is never substituted for a measured
cell-surface trace. With no explicit measured surface channel, thermal
accuracy is `NOT_RUN`.

Generic CSV/MAT adapters keep `fit_tests` and `validation_tests` separate.
Candidate configurations should provide independent holdout files; fitting
functions never consume that holdout field. `build_candidate_cell` rejects a
missing holdout partition, any fit/holdout source-file overlap, a template
acceptance configuration, or any numerical holdout failure.

The P42A adapter freezes a cell-level partition: A2/A4/A6/A8/A9/A11/A12/A13
fit the model, while A15/A19/A21/A37 are holdout cells. HCGT records are
windowed per pulse and linked to published OCV/SoC metadata.

## Numerical acceptance

`configs/acceptances/` contains temperature-aware, test-specific gates.
Qualification checks comparable-sample count plus RMS, maximum, p95, mean
bias, endpoint voltage, endpoint SoC, surface-temperature RMS/maximum, and
HPPC loaded/relaxation errors. A run with enough samples but unacceptable
accuracy cannot report PASS. Candidate limits must be copied, justified, and
frozen before inspecting the independent holdout.

## Validation

`hil.validate_all` performs:

- parameter dimensions/sign/domain checks;
- 75s6p pack-image invariants and scaling guards;
- 12s2p structural topology and scaling guards;
- optional thermal sensitivities;
- numerical-acceptance PASS/FAIL self-tests;
- P42A electrical holdout validation automatically when raw data is present;
- optional Simulink/reference parity when Simulink is available;
- parameter and validation report export.

Every report has separate `PASS`, `FAIL`, or `NOT_RUN` gates for static
configuration, model invariants, P42A data, Simulink execution, generated-C
parity, ESP-IDF, and hardware. `framework_passed` is not a claim that
`qualification_complete` is true.

Host-C regression is separate:

```bash
cd ../esp32_plant/tests
make test
make baseline > candidate.csv
python3 ../../simulink/tools/compare_baseline.py \
  ../../simulink/validation/baselines/p42a_75s6p_codegen_oracle.csv \
  candidate.csv
```

## Candidate cells

Do not rename P42A values and call them a candidate model. Copy
`configs/cells/candidate_template.m` and
`configs/acceptances/candidate_template.m`, supply a dataset adapter/config,
replace all limits and fit priors, then use `scripts/build_candidate_cell.m`.
The candidate phase is intentionally blocked until raw data, identity, and
predeclared accuracy gates are available.
