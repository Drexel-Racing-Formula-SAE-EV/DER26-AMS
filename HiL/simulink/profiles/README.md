# Current and power profiles

Profiles carry source, sample time, repeat policy, and explicit scaling mode.
Supported sources are synthetic HPPC, embedded C arrays, CSV, MAT, constant
current, and constant power.

The checked-in embedded profiles are exposed through the `us06_25c`,
`udds_25c`, and `la92_25c` simulation configurations. Charge, regen,
endurance, or user traces use the same CSV/MAT contract rather than a
cell-specific loader.

Scaling modes are:

- `no_scaling`;
- `vehicle_current_replay`;
- `constant_c_rate`;
- `constant_cell_current`;
- `constant_pack_power`.

There is no silent capacity scaling. Modes requiring a reference pack reject
missing reference metadata.

Before a fixed-current Simulink/generated-C run, `hil.resolve_profile_inputs`
freezes current bias and iterative constant-power conversion into an explicit
pack-current trace. This keeps reference, Simulink, and C parity inputs
identical.
