# DER26 AMS v0.3.3 — CAN Power Interface

**Date:** 2026-07-22

## Added

- Optional `0x68A` per-horizon/per-direction SoP binding and limiting-segment metadata.
- Strongly typed portable ECU getters for immediate authority, constant-current feasibility, AMS-owned resources, and SoH.
- Same-cycle counter plus timing synchronization for advisory resource metadata.
- Dashboard decode and JSON output for binding metadata.

## Corrected

- Removed the phantom fourth/1-second slot from the wire envelope API. The wire explicitly contains `0.1/10/30 s`; all four horizons remain internal to the AMS.
- Raw horizon arrays are no longer adjacent to the final-clamp scalar in one flat ECU output type.
- Bad advisory frames invalidate only their own cached metadata and cannot leave an older same-counter value visible.

## Compatibility

- Required fail-zero authority frames `0x684`-`0x687` are unchanged.
- Power protocol remains version 2.
- `0x689` and `0x68A` are independently advisory and cannot authorize or preserve torque by themselves.

## Scope

No MPC implementation or MPC-scope decision is included in this release.
