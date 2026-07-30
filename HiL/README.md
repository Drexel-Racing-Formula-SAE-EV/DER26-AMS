# DER accumulator HIL

This directory contains a reusable battery-plant framework plus the frozen,
host-regressed P42A/75s6p generated-C snapshot used by the current ESP32 HIL
node. The snapshot is not hardware-qualified.

The refactor separates four concerns:

- cell identity, limits, LUT axes, and fitting policy;
- pack topology, segment mapping, thermistor mapping, and variation policy;
- simulation profile, sample time, ambient conditions, and scaling;
- raw-dataset location and adapter behavior.

No MATLAB source depends on a developer-specific absolute path. External data
is located through `HIL_DATA_ROOT`; generated artifacts may be redirected with
`HIL_OUTPUT_ROOT`.

## Layout

```text
HiL/
  simulink/
    +hil/                  reusable MATLAB package
    adapters/              raw-data adapters
    configs/               cell, pack, simulation, and dataset configs
    models/                clean source template and frozen legacy models
    parameters/            reviewed source artifacts and manifests
    profiles/              profile loaders and synthetic cases
    scripts/               thin operator entry points
    tools/                 migration and regression utilities
    validation/            frozen baseline and review artifacts
  esp32_plant/
    components/plant_model stable adapter plus generated model
    main/                   FreeRTOS/CAN plant node
    tests/                  host-C adapter and baseline tests
  common/                   shared ESP32/AMS CAN image protocol
  SOURCE_MANIFEST.json      reviewed source/archive/tool-status provenance
```

## Current frozen configuration

- Cell: Molicel INR-21700-P42A.
- Pack: 75 series groups, six cells in parallel.
- Segments: five groups of 15 series groups.
- Temperature image: 120 configured sensor locations.
- Plant step: 0.1 s.
- Current sign: positive means discharge.
- Generated physics: one representative 2RC cell and two-node thermal state,
  scaled to the pack and expanded to fixed-size AMS outputs.

The checked-in generated model is still the behavior oracle. Its constants were
reconstructed into a reviewed MAT artifact and its host-C behavior is frozen in
`simulink/validation/baselines/`.

## Scope boundary

The MATLAB reference engine can independently evolve all series-group
electrical and thermal states in `parameter_distributed` mode. The generated
Simulink/ESP32 path deliberately retains one representative state and uses a
repeatable output-expansion proxy for supported output-spread modes. Code
generation is blocked for `parameter_distributed`; an artifact cannot imply
independent embedded group physics that it does not implement. A fully
distributed generated plant is a separate future performance/code-size
decision.

Thermal results are comparative screening only. Pack geometry, inter-group
conduction, busbar heat, enclosure convection, and fan airflow are not
calibrated.

See `HIL_REFACTOR_WORK_PLAN.md` for completion status and
`HIL_REFACTOR_VALIDATION_REPORT.md` for executed evidence and tool-dependent
checks. `ATOMIC_CAN_IMAGE_PROTOCOL.md` defines coherent publication, and
`HARDWARE_QUALIFICATION_PLAN.md` is the remaining bench gate.
