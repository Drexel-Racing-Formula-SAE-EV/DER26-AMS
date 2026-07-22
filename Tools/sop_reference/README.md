# DER26 independent SoP oracle

This directory is the numerical verification path for the embedded robust
finite-horizon SoP solver. It is not a second production implementation and it
has no vehicle authority.

The independence boundary is intentional:

- Embedded C reads `AMS/Core/Src/estimator/ams_estimator_lut.c`.
- Python parses the separately generated HIL constants in
  `HiL/esp32_plant/components/plant_model/const_params.c`.
- The electrical, two-node thermal, uncertainty, voltage-anchor, and bisection
  equations are independently implemented in Python.
- Tests compile the target C solver as a temporary shared library, pass the
  same input structures to both implementations, and compare all four
  discharge/charge horizons and predicted power.

The HIL tables include 303 OCV values plus 36-point R0, R1, inverse-C1, and
inverse-time-constant calibrations across SOC and temperature. The test also
checks that the independently stored direct-R1 table is consistent with the
inverse parameterization used by the estimator.

Run the nominal example:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 Tools/sop_reference/demo.py
```

Run differential verification:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -v \
  -s Tools/sop_reference/tests
```

or from `AMS/host_tests`:

```bash
make power-oracle
```

Coverage includes nominal, low-voltage, hot, high-SOC, aged-pack, seeded
random, covariance/SoH conservatism, horizon nesting, and bisection-boundary
cases. Temporary shared objects are created outside the repository and removed
after the test.
