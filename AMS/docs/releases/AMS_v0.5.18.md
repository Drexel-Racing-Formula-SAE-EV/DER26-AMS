# AMS v0.5.18 Fuse Direction and Reset-State Hardening

Date: 2026-09-03
Source revision: `DER26-AMS-v0.5.18-20260903`

Review of the EAC14-80 observer against the accumulator, APM, and HV-box
schematics confirmed that the main 80 A fuse and pack-current shunt are in the
common bidirectional traction path. The previous observer accumulated current
magnitude in both directions but only published and applied a discharge cap.
It also initialized an unknown MCU-reset state through a low-current soak that
could erase retained thermal utilization.

Changes:

- added positive-magnitude fuse charge-current caps for every SoP horizon;
- applied those caps to the negative-signed SoP charge/regen current and power,
  with `AMS_SOP_BIND_FUSE_THERMAL` and the existing fuse-derated reason flags;
- production startup and MCU reset now seed the configured maximum thermal
  state, latch exhaustion, and remain fail-closed until the state decays through
  the existing 0.50 release threshold;
- conservative initialization is authoritative only when the independent
  compile-time model-validation gates are enabled; the current bench build
  remains shadow-only;
- `AMS_FUSE_REASON_INITIAL_STATE_UNKNOWN` remains asserted while the
  conservative reset seed is active, even though the bounded state can safely
  provide authority;
- the cold-soak characterization path no longer resets thermal utilization or
  the exhaustion latch when its timer completes;
- documented the 4.0 state clamp as overload-memory headroom;
- extended the independent long-double oracle and CSV replay to validate both
  discharge and charge caps;
- extended the MiL fuse-consistency metric, combined SoP oracle export, and
  summary schema v8 to retain charge-cap evidence;
- changed fuse-replay defaults so conservative startup and unknown reset match
  production behavior, while known-cold and arbitrary seeded modes remain
  explicit characterization policies.

Safety disposition:

- `AMS_FUSE_MODEL_VALIDATED` and
  `AMS_FUSE_LOW_CURRENT_EXTRAPOLATION_VALIDATED` remain disabled by default;
- no fuse authority is enabled by this release;
- future vehicle authority still requires physical fuse/holder/busbar testing;
- a validated persistent state with CRC and elapsed-off-time handling may later
  improve reset availability, but the present fallback is conservative without
  relying on nonvolatile state.

Validation:

- SoP/SoH production core and directed fuse/strategy tests: PASS;
- 20,000 drive + 20,000 charge metamorphic states: PASS;
- independent long-double fuse oracle, including 50,000 randomized updates:
  PASS;
- strict invalid/reset/restore fuse replays: PASS.
