# DER26 AMS CAN power-interface update

**Date:** 2026-07-22  
**Release candidate:** v0.3.3 CAN Power Interface  
**Scope:** CAN publication, portable ECU consumer, dashboard decoding, tests, and contract documentation. No MPC controller, model, solver, cost function, or MPC scope was added.

## Purpose

This change freezes the CAN-side safety boundary before defining the ECU MPC. It removes the ambiguous flat interface that placed raw horizon arrays beside immediate authority, resolves the three-versus-four horizon mismatch, exposes the AMS-owned resource state without creating a competing ECU authority model, and preserves binding causes by direction and wire horizon.

## Wire contract after this update

The required fail-zero authority bundle remains backward compatible and unchanged:

| CAN ID | Role | Required for scalar authority |
|---:|---|---|
| `0x684` | Active discharge current/power authority | Yes |
| `0x685` | Active charge/regen current/power authority | Yes |
| `0x686` | SoH status | Yes |
| `0x687` | Three-horizon constant-current feasibility envelope | Yes |
| `0x689` | AMS-owned resource and mission status | No, advisory |
| `0x68A` | Per-horizon/per-direction binding and segment metadata | No, advisory |

All six frames use power protocol version 2, the same cycle counter, ID-bound CRC-8/SAE-J1850, standard 11-bit CAN, and DLC 8. The new `0x68A` frame is an optional backward-compatible extension; loss or corruption of `0x689` or `0x68A` does not invalidate the four-frame scalar authority bundle.

## Horizon count resolved

The AMS still solves four internal horizons: `0.1 s`, `1 s`, `10 s`, and `30 s`. The wire envelope carries exactly three: `0.1 s`, `10 s`, and `30 s`. The internal 1-second value is not reconstructed, interpolated, or represented by a phantom fourth slot. When mission strategy selects 1-second capability, that selection is reflected through the active high-resolution DCL/CCL scalar path rather than a fourth envelope element.

The encoder/decoder and ECU API now use an explicit three-entry wire-horizon enum. Charge values in the wire-facing envelope type are non-negative magnitudes.

## Safe ECU consumer API

The portable ECU consumer now separates data by safety role:

1. `der26_power_consumer_get_immediate_authority()` returns only the scalar discharge and shared charge/regen current/power authorities. This is the only API intended for the final inverter-transmit clamp. Outputs are zeroed on failure or staleness.
2. `der26_power_consumer_get_feasibility_envelope()` returns the explicitly named three-horizon **constant-current feasibility** vectors. The type warns that these are not a pointwise schedule. Optional binding arrays are usable only when synchronized `0x68A` metadata is valid.
3. `der26_power_consumer_get_resource_state()` returns AMS-owned fuse utilization, thermal energy to target, minimum core temperature, mission state, and readiness/bootstrap fields from `0x689`. It succeeds only when the advisory frame is fresh, has the active bundle counter, and arrived within the same-cycle 50 ms synchronization window.
4. `der26_power_consumer_get_soh()` returns the accepted SoH status separately.

The protocol does not claim fields it does not transmit. There is no fabricated polarization state, remaining fuse I-squared-t value, or separate charger-versus-regen authorization bit. The current `0x685` authority is intentionally exposed as shared `charge_regen`.

## New `0x68A` binding metadata

The optional frame carries six binding nibbles and six limiting-segment nibbles in this order:

```text
D 0.1 s, D 10 s, D 30 s, C 0.1 s, C 10 s, C 30 s
```

Bindings are validated against the 0-14 `ams_sop_binding_t` range. Segment values are 0-4 or `0xF` for none/unknown. A stale AMS snapshot encodes invalid-input bindings and unknown segments. The active scalar binding remains in `0x684`/`0x685`; `0x68A` is planning/diagnostic metadata only.

## Advisory containment and synchronization

Core-frame transport errors still revoke authority and require two new coherent bundles. Advisory errors are isolated:

- A bad `0x689` invalidates only the cached resource state.
- A bad `0x68A` invalidates only cached binding metadata.
- Neither advisory error revokes valid scalar DCL/CCL.
- Counter equality alone is not accepted because the four-bit counter wraps. The consumer also requires the advisory arrival time to be within 50 ms after the accepted core bundle.
- Unknown unrelated CAN IDs do not disturb the authority state machine.

## AMS sender changes

- The immutable CAN snapshot now preserves full four-horizon binding and limiting-segment arrays internally.
- `0x684`/`0x685` continue to publish the active scalar binding and segment.
- `0x687` remains the three-wire-horizon feasibility envelope.
- `0x689` remains advisory.
- `0x68A` is transmitted after `0x689` with the same cycle counter. A failure to send either advisory frame does not mark the four-frame authority transmission as failed.

## Dashboard changes

The passive ESP32 dashboard/logger now recognizes all six power frames while retaining only `0x684`-`0x687` as required for power freshness. JSON output includes:

- `binding_metadata_fresh`
- `discharge_binding_by_horizon[3]`
- `charge_binding_by_horizon[3]`
- `discharge_segment_by_horizon[3]`
- `charge_segment_by_horizon[3]`

## Validation performed

The following completed successfully after the changes:

```text
make power-consumer
make power-core
make dashboard-decoder
make test
make unit
make analyze
make whole-source-analyze
make target-project-gate
```

Additional checks:

- Portable production consumer compiled cleanly under `-Wconversion`, `-Wsign-conversion`, `-Wshadow`, `-Wdouble-promotion`, `-Wcast-align`, `-Wcast-qual`, `-Wundef`, `-Wformat=2`, and `-Wfloat-equal`.
- Consumer tests cover all 24 required-frame permutations, duplicate and skew rejection, wrong-DLC fail-closed behavior, two-good-bundle qualification, exact three-horizon layout, synchronized advisory resources, per-horizon metadata, bad-advisory containment, advisory counter mismatch, same-counter-but-outside-skew rejection, unrelated-ID containment, newer zero-authority override, core corruption/requalification, independent direction inhibition, staleness, and counter wrap.
- The portable consumer test also passed a combined AddressSanitizer/UndefinedBehaviorSanitizer build.
- Dashboard render tests validate the new JSON arrays and freshness field.
- Host CAN transmission tests confirm independent failure of `0x689` and `0x68A` does not fail the required authority bundle.

## Deliberately not included

This update does not define how an MPC uses short horizons, does not choose an MPC state vector, and does not implement a local ECU fuse or thermal authority model. Those decisions come after this CAN boundary is reviewed and frozen.

## Environment limitations

A monolithic `make ci` run was also attempted. It completed the unit and SoP/SoH production-core stages, then the execution environment timed out while the unchanged 40,000-state SoP metamorphic campaign was running. The CAN-relevant targets and static-analysis gates listed above were therefore run individually to completion. The container does not contain `arm-none-eabi-gcc`, so a new STM32 ELF/map was not produced in this pass; target project metadata was checked by `make target-project-gate` and whole-source bench/vehicle profile compilation was checked with the available host GCC analysis path.
