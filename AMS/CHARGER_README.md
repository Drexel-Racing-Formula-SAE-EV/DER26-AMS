# Charger-only v0.5.57 configuration reference

Profile 4 (default) is CAN diagnostic/inhibited. Profile 6 retains the previous
charge-review logic and source-level charge permission; it does NOT autonomously
start charging at boot. The physical Cell 1 of SMB1 remains unobserved, with no
direct per-cell overvoltage protection. The inferred-C1 check is not equivalent
to a measured-cell OV trip.

Existing charger CAN settings: 250 kbit/s, command target 312.0 V and 10.0 A.
BMS inhibit at boot, unchanged runtime fault/interlock conditions, and
balancing inhibited in profile 6. Initial SoC=27% for profile 6 only.

Host tests demonstrate code paths, not electrical validation or a working
charge session. Review `docs/CHARGE_ONLY_CONSOLIDATION_2026-11-08.md`.
