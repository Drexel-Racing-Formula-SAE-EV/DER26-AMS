# ESP32 accumulator HIL node

The ESP32 runs the checked-in generated battery plant every 100 ms and publishes
pack truth and an AMS cell/temperature image over an MCP2515 at 250 kbit/s.

## Stable plant API

Application code includes `plant_model_adapter.h`, never a generated Simulink
header. The stable API is:

```c
bool plant_init(const plant_configuration_t *configuration);
bool plant_reset(float initial_soc, float initial_temperature_C);
bool plant_step(float pack_current_A, float ambient_temperature_C);
bool plant_get_outputs(plant_output_t *output);
bool plant_get_state(plant_state_t *state);
void plant_terminate(void);
```

`plant_model_binding.h` maps this API to the current generated identifiers.
`plant_model_manifest.h` owns array sizes, sample time, segment offsets/counts,
and flattened topology indices. Regeneration changes those two generated
headers, not `main.c`.

## CAN interface

| ID | Frame | Encoding |
|---:|---|---|
| `0x200` | Measurement | pack voltage 10 mV/count, current 10 mA/count, surface temperature 0.01 C/count, counter |
| `0x201` | Truth | SoC 0.01%/count, core temperature 0.01 C/count, counter, 24-bit step |
| `0x202` | Summary | minimum/maximum group voltage 1 mV/count, maximum/average sensor temperature 0.01 C/count |
| `0x210` | Cell image data | generation, packed segment/local index, three group voltages |
| `0x211` | Temperature image data | generation, packed segment/local index, three sensor temperatures |
| `0x212` | Image control | START/topology or COMMIT/CRC32 |
| `0x300` | Control | reset when bytes begin `A5 5A 52` |

Positive current means discharge. Pack voltage uses 10 mV/count because a 75s
pack cannot fit in an unsigned 16-bit field at 1 mV/count.

The shared image is invalid until the first complete model calculation. Startup
and commanded soft reset both execute an immediate zero-current model step
before CAN publication. A soft reset preserves and advances the transport
counter, so it cannot emit a coherent zero-voltage image or unnecessarily
restart the generation epoch.

For firmware built with `AMS_HIL_REPLACE_ADBMS=1`, the AMS stages one
generation behind received bitmaps and publishes it only after every channel
and the canonical CRC32 pass. Identical duplicates and arbitrary data-frame
order are allowed; missing, conflicting, timed-out, replayed, and corrupt
generations leave the prior complete image untouched. Loss of image freshness
fails closed through normal AMS logic. See
`../ATOMIC_CAN_IMAGE_PROTOCOL.md`.

## Host tests

No ESP-IDF installation is required:

```bash
cd tests
make test
./build/baseline_runner > candidate.csv
python3 ../../simulink/tools/compare_baseline.py \
  ../../simulink/validation/baselines/p42a_75s6p_codegen_oracle.csv \
  candidate.csv
```

For a newly generated component, `hil.generate_code` also creates a parity
case. Build all host runners, execute the exported input, and compare both CSV
files in MATLAB:

```bash
make all
./build/parity_runner /path/to/input_profile.csv 1.0 25.0 \
  > generated_output.csv
```

```matlab
report = hil.validate_codegen_parity( ...
    "/path/to/reference_output.csv", ...
    "/path/to/generated_output.csv");
assert(report.passed)
```

## Target build

With ESP-IDF activated:

```bash
idf.py set-target esp32
idf.py build
idf.py -p <PORT> flash monitor
```

For a new generated candidate, use
`python3 ../tools/run_toolchain_qualification.py --promote`; it builds a
temporary ESP-IDF project before changing the frozen component. The serial
diagnostics report plant-step execution time, 67-frame image time, 70-frame
burst time, deadline misses, and distinct MCP2515 success, retry, arbitration,
TX-error, abort, bus-off, timeout, SPI, warning/passive, and overflow counts.
Target acceptance still requires bitrate/termination, scaling, freshness,
reset, current polarity, queue high-water, bus-off, and fault-response
verification on the bench. See `../HARDWARE_QUALIFICATION_PLAN.md`.
