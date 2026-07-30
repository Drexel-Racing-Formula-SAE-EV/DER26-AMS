# Configuration contracts

- `cells/`: identity, ratings, LUT axes, fitting rules, thermal prior, provenance.
- `packs/`: Ns/Np, segment and sensor mappings, initial state, variation,
  thermal layout, cooling boundary.
- `simulations/`: sample/stop time, profile semantics, ambient, initial SoC,
  measurement effects, output location, execution engine.
- `datasets/`: root resolution, adapter, units/sign, source/version metadata.
- `acceptances/`: frozen temperature/test-specific numerical qualification
  limits and rationale.

Factories return scalar structs and are loaded through `hil.config.*`. The
validator rejects inconsistent segment counts, invalid indices, non-monotonic
axes, invalid signs, and build attempts using candidate templates.

`candidate_csv_template.m` documents the generic CSV dataset layer. Copy it and
`acceptances/candidate_template.m` to new names and replace every placeholder;
do not build either template directly or set limits after viewing holdout
results.
