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

## Portable ECU CAN consumer

`ecu_power_consumer.c/.h` is the fail-zero reference implementation for the
AMS-to-ECU power contract. It intentionally does **not** expose one flat struct
containing scalar authority and raw horizon arrays. The API is split by safety
role:

```c
der26_power_consumer_get_immediate_authority(...);
der26_power_consumer_get_feasibility_envelope(...);
der26_power_consumer_get_resource_state(...);
der26_power_consumer_get_soh(...);
```

The immediate-authority getter is the only interface intended for the final
torque transmit clamp. The feasibility getter contains exactly three wire
horizons (`0.1 s`, `10 s`, `30 s`) explicitly documented as constant-current
feasibility, not a control schedule. There is no phantom fourth/1-second array
slot. The active mission-selected scalar is carried by `0x684`/`0x685`.

The optional `0x689` resource frame and `0x68A` per-horizon binding frame are
accepted only as counter-synchronized metadata. Their loss or corruption does
not invalidate an otherwise coherent four-frame scalar-authority bundle. A
resource getter fails and zeroes its output unless `0x689` is fresh, matches
the active authority counter, and arrived within the same-cycle 50 ms
synchronization window. A malformed advisory invalidates only its own cached
metadata; it never revokes a valid scalar authority bundle.

Run the consumer contract test:

```bash
cd AMS/host_tests
make power-consumer
```
