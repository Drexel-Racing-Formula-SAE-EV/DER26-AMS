# DER26 SoP host tests

This directory contains three separate forms of SoP verification:

1. `sop_test.c` — directed production tests for specific requirements and edge
   cases.
2. `sop_metamorphic_oracle.c` — deterministic black-box CI properties across
   large seeded state sets.
3. `sop_sensitivity_probe.c` — non-gating calibration characterization.

Run from this directory:

```bash
make test
make metamorphic
make sensitivity
make asan
```

The metamorphic state count is configurable:

```bash
make metamorphic SOP_METAMORPHIC_STATES=100000
```

The default is 20,000 states per direction (40,000 total). The test is
reproducible: failures report the first seed. The sanitizer target runs a
smaller metamorphic campaign by default; override it with
`SOP_METAMORPHIC_ASAN_STATES`.

The metamorphic test is not an independent electrothermal model. It verifies
necessary public-API properties and fail-zero behavior. Numeric calibration
still requires the HIL/segment/vehicle validation path.

## Fuse observer companion validation

The SoP directory's directed fuse tests are supplemented by an independent
fuse oracle in `../fuse` and the replay tooling in `Tools/fuse_replay`. Run
`make -C .. fuse-oracle` from this directory's parent. The independent oracle
does not call production fuse integration code.
