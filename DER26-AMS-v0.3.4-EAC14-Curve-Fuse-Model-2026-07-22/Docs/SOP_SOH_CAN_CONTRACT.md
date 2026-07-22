# DER26 SoP/SoH CAN contract, power protocol version 2

**Revision:** v0.3.3 CAN consumer/interface hardening (2026-07-22)

## Safety role

These frames carry a dynamic physical envelope from the AMS to the ECU. They do
not command torque. The ECU remains responsible for converting torque/speed to
a conservative DC current and DC power request and limiting both against DCL or
CCL.

CRC and counters provide transmission-integrity and freshness evidence; they
are not cryptographic authentication. The vehicle CAN threat model must be
handled separately if hostile-node resistance is required.

## Transport requirements

- Classic CAN, 11-bit standard data frames, DLC 8.
- Nominal publication rate: 10 Hz as one required four-frame authority bundle
  plus two independently advisory frames: strategy/resource status (`0x689`)
  and per-horizon binding metadata (`0x68A`).
- All six published frames in a cycle carry the same 4-bit rolling counter.
  Only `0x684`-`0x687` are required for scalar authority.
- Counter advances modulo 16 for the next bundle.
- Receiver maximum bundle skew: 50 ms.
- Receiver maximum completed-bundle age: 250 ms.
- Receiver must reject extended, remote, wrong-DLC, bad-version, bad-CRC,
  duplicate, incomplete, or discontinuous bundles.
- Two consecutive complete valid bundles are required after startup or any
  transport fault before limits may become nonzero.

## Common byte 0 and CRC

```text
byte 0 bits 7:4 = power protocol version (2)
byte 0 bits 3:0 = rolling bundle counter
byte 7          = CRC-8/SAE-J1850
```

CRC parameters are polynomial 0x1D, initial value 0xFF, xor-out 0xFF, no
reflection. The CRC input is the standard CAN ID high byte, CAN ID low byte,
then payload bytes 0 through 6. Including the ID prevents a correct payload from
being accepted under another power-frame ID.

## 0x684 active discharge-current limit (DCL)

| Byte | Field | Encoding |
|---:|---|---|
| 0 | Version/counter | Common format |
| 1 | Flags | See flag table |
| 2-3 | Active DCL | Big-endian unsigned, 0.1 A/bit; 1 s capability in Qualify, 30 s-derived cap in Endurance/Limp |
| 4-5 | Active discharge power | Big-endian unsigned, 10 W/bit |
| 6 high nibble | Binding | `ams_sop_binding_t` value |
| 6 low nibble | Limiting segment | 0-4, 15 = none/unknown |
| 7 | CRC | Common format |

## 0x685 charge/regen-current limit (CCL)

The layout is identical to 0x684. Current and power are unsigned magnitudes;
the charge direction is implied by the CAN ID. The target C solver internally
uses negative current for charge/regen.

| Byte | Field | Encoding |
|---:|---|---|
| 2-3 | 1 s CCL magnitude | Big-endian unsigned, 0.1 A/bit |
| 4-5 | 1 s charge power magnitude | Big-endian unsigned, 10 W/bit |

## Limit flags for 0x684 and 0x685

| Bit | Name | Meaning |
|---:|---|---|
| 0 | VALID | Solver result and measurement snapshot are valid/fresh |
| 1 | AUTHORITY_VALID | Complete topology and policy allow publication |
| 2 | CAPACITY_PRIOR | Conservative capacity prior is in use |
| 3 | RESISTANCE_PRIOR | Conservative resistance prior is in use |
| 4 | AMBIENT_PROXY | Hottest surface is used as ambient proxy |
| 5 | SLEWED | Recovery/increase is rate-limited |
| 6 | DIRECTION_INHIBIT | This direction is not authorized |
| 7 | FALLBACK | Invalid/stale solver fallback is active |

Nonzero current is allowed only when VALID and AUTHORITY_VALID are both set and
DIRECTION_INHIBIT and FALLBACK are both clear. Prior, proxy, or slew flags are
diagnostic and do not invalidate an otherwise conservative result.

## 0x686 State of Health

| Byte | Field | Encoding |
|---:|---|---|
| 0 | Version/counter | Common format |
| 1 | Capacity SoH mean | 1 percent/bit |
| 2 | Capacity SoH lower bound | 1 percent/bit |
| 3 | Resistance growth upper bound | 1 percent/bit; 100 = new-reference R0 |
| 4 | Combined conservative SoH | 1 percent/bit |
| 5 bit 7 | Capacity valid | 1 only after required observations |
| 5 bits 6:0 | Capacity confidence | 0-100 percent |
| 6 bit 7 | Resistance valid | 1 only when all five segments qualify |
| 6 bits 6:0 | Resistance confidence | 0-100 percent |
| 7 | CRC | Common format |

SoH is diagnostic and an input to the AMS's conservative predictor. The ECU
must not increase a DCL/CCL because a decoded SoH value appears favorable.

## 0x687 active horizon envelope

The active scalar selected by the AMS mission policy is carried at higher
resolution in `0x684`/`0x685`. This frame carries exactly three
strategy-limited constant-current feasibility horizons at 1 A/bit: 0.1, 10,
and 30 seconds. The internal 1 second result is not a fourth wire-array slot
and is not independently transmitted. When the mission strategy selects the
1 second capability, that selection is reflected in the higher-resolution
active scalar frames. All four physical horizons are still solved internally.
Endurance and Limp may cap transmitted short horizons to a longer-horizon
value.

These values are **constant-current feasibility boundaries from the current
battery state**, not a pointwise current schedule. A downstream controller may
not repeatedly request the 0.1 second value while resetting its dwell timer.

| Byte | Field | Encoding |
|---:|---|---|
| 0 | Version/counter | Common format |
| 1 | 0.1 s discharge | 1 A/bit |
| 2 | 10 s discharge | 1 A/bit |
| 3 | 30 s discharge | 1 A/bit |
| 4 | 0.1 s charge magnitude | 1 A/bit |
| 5 | 10 s charge magnitude | 1 A/bit |
| 6 | 30 s charge magnitude | 1 A/bit |
| 7 | CRC | Common format |

## Binding values

| Value | Meaning |
|---:|---|
| 0 | None/current ceiling reached without a physical predictor binding |
| 1 | Cell undervoltage |
| 2 | Cell overvoltage |
| 3 | Low SOC |
| 4 | High SOC |
| 5 | Core temperature |
| 6 | Surface temperature |
| 7 | Low charge temperature |
| 8 | System current path |
| 9 | Direction inhibited |
| 10 | Model domain |
| 11 | Invalid input |
| 12 | Nested horizon envelope |
| 13 | Main-fuse thermal budget |
| 14 | Mission-profile derating |

## 0x688 ECU-to-AMS mission request

This is a soft derating request, not torque authority and not a battery-safety
override. It uses mission protocol version 1 and the same ID-bound CRC method.

| Byte | Field | Encoding |
|---:|---|---|
| 0 | Version/counter | Version 1 high nibble; modulo-16 low nibble |
| 1 | Requested profile | 0 Endurance, 1 Qualify, 2 Limp Home |
| 2 bit 0 | Stationary confirmed | Required only to enter Qualify |
| 2 bits 7:1 | Reserved | Must be zero |
| 3-6 | Reserved | Must be zero |
| 7 | CRC | CRC-8/SAE-J1850 bound to ID `0x688` |

The AMS requires two consecutive, counter-continuous requests for the same
profile. Maximum age is 250 ms. CRC, version, semantic, counter, or freshness
failure selects Endurance immediately. Limp Home also latches automatically
when the weakest segment's 3-sigma SoC lower bound reaches 30%. CRC/counter
protection is not cryptographic authentication.

## 0x689 advisory strategy status

This frame carries power protocol version 2 and the same counter as the power
publication cycle, but it is not a fifth required member of the fail-zero ECU
bundle.

| Byte | Field | Encoding |
|---:|---|---|
| 0 | Version/counter | Common power format |
| 1 bits 1:0 | Active profile | 0 Endurance, 1 Qualify, 2 Limp Home |
| 1 bits 3:2 | Recommended horizon index | 0/1/2/3 = 0.1/1/10/30 s |
| 1 bit 4 | Thermal ready | All modeled cell nodes at or above 25 degC |
| 1 bit 5 | Fuse authority valid | Characterized model and initialized state |
| 1 bit 6 | Limp latched | Automatic or retained limp state |
| 1 bit 7 | Mission fallback | Stale/invalid request or stale power snapshot |
| 2 | Fuse utilization | 1 percent/bit; may exceed 100 until saturation at 255 |
| 3 | Minimum core temperature | unsigned value minus 40 degC |
| 4-5 | Thermal energy to 25 degC | big-endian, 0.1 Wh/bit |
| 6 | R0 bootstrap progress | 0-100 percent |
| 7 | CRC | Common format bound to ID `0x689` |

## 0x68A advisory per-horizon binding metadata

This optional frame preserves the binding cause and limiting segment for each
of the three wire horizons and each direction. Its loss never invalidates the
scalar DCL/CCL authority bundle. A consumer may use it only when its counter
matches the active four-frame bundle and it is fresh.

The six nibbles are ordered as:

```text
DCL 0.1 s, DCL 10 s, DCL 30 s, CCL 0.1 s, CCL 10 s, CCL 30 s
```

| Byte | Field | Encoding |
|---:|---|---|
| 0 | Version/counter | Common power format |
| 1 high/low | DCL 0.1 s / DCL 10 s binding | 4-bit `ams_sop_binding_t` values |
| 2 high/low | DCL 30 s / CCL 0.1 s binding | 4-bit values |
| 3 high/low | CCL 10 s / CCL 30 s binding | 4-bit values |
| 4 high/low | DCL 0.1 s / DCL 10 s segment | 0-4, 15 none/unknown |
| 5 high/low | DCL 30 s / CCL 0.1 s segment | 0-4, 15 none/unknown |
| 6 high/low | CCL 10 s / CCL 30 s segment | 0-4, 15 none/unknown |
| 7 | CRC | Common format bound to ID `0x68A` |

The active scalar binding and segment remain in `0x684`/`0x685`. `0x68A` is
planning/diagnostic metadata only.

## Sender behavior

The estimator publishes an immutable snapshot after each new measurement epoch.
The CAN task rechecks both measurement and solve timestamps immediately before
encoding. An invalid or older-than-250-ms snapshot is encoded with zero limits,
VALID/AUTHORITY_VALID clear, and FALLBACK set. Each required four-frame call
uses one counter value. The separately advisory `0x689` and `0x68A` frames use
the same cycle counter, but loss of either does not invalidate the primary
bundle.

If one CAN transmission fails, receiver coherence rules prevent the partial
bundle from becoming active. The sender must also retain its existing CAN error
and bus-off supervision; CRC/counter handling does not replace bus recovery.

## ECU acceptance state machine

The portable reference is in `Tools/sop_reference/ecu_power_consumer.c`.
Required behavior is:

1. Reject non-standard-data/DLC-8 frames before decoding.
2. Validate protocol version and ID-bound CRC.
3. Stage one counter value only; accept the four IDs in any order, once each.
4. Reject partial replacement, duplicates, or bundle skew greater than 50 ms.
5. On completion, require exact modulo-16 progression from the previous
   completed bundle.
6. Require two consecutive good completed bundles after boot or any error.
7. Zero-initialize output on every read and return no authority after 250 ms.
8. Gate discharge and charge directions independently from their flags.
9. Reject semantically impossible but CRC-correct data: DCL above 120 A, CCL
   above 15 A, discharge power above 40 kW, charge power above 5 kW, unknown
   binding/segment values, contradictory validity/fallback flags, nonzero data
   while inhibited, invalid SoH ranges/confidence, a non-nested envelope, or an
   envelope inconsistent with the active high-resolution scalar selected by
   mission strategy.

Unknown unrelated CAN IDs, `0x688`, and optional `0x689`/`0x68A` frames must
not disturb the hard-bundle state machine. Malformed frames using one of the
four required power IDs invalidate authority. A malformed advisory frame
invalidates only that advisory channel so an older same-counter value cannot
remain visible after retransmission or modulo-16 wrap; scalar authority stays
valid.

## ECU typed-consumer boundary

The portable consumer in `Tools/sop_reference/ecu_power_consumer.*` deliberately
separates three interfaces:

1. `der26_power_consumer_get_immediate_authority()` returns only the scalar
   discharge and charge/regen current and power limits used by the final torque
   clamp. This is the safe default API.
2. `der26_power_consumer_get_feasibility_envelope()` returns exactly three
   explicitly named constant-current feasibility horizons. The optional
   per-horizon binding arrays are valid only when a synchronized `0x68A` was
   accepted.
3. `der26_power_consumer_get_resource_state()` returns AMS-owned fuse
   utilization and thermal-energy state only when `0x689` is fresh, its
   counter matches the active authority bundle, and it arrived within the
   same-cycle 50 ms synchronization window. The ECU must not run a competing
   authoritative fuse observer.

Power protocol v2 does not publish polarization state, remaining fuse I²t, or
a separate charger-versus-regen authorization bit. Consumers must not fabricate
those fields.

## Torque-limit application

The ECU must enforce both current and power. For positive motoring torque, form
a conservative DC-side request from motor speed, requested torque, inverter
efficiency lower bound, auxiliaries, and DC-bus voltage. Clamp such that:

```text
I_dc_request <= DCL
P_dc_request <= discharge_power_limit
```

For negative torque, use the analogous charge/regen quantities and CCL. At low
DC-bus voltage or uncertain inverter efficiency, the more conservative bound
wins. Do not map amperes directly to pedal percentage or motor torque with a
fixed constant. Revalidate the limit in the final torque-transmit path so a
newer zero limit cannot be bypassed by an older queued command.

On stale/invalid power data, positive torque authority must become zero if the
vehicle policy declares SoP mandatory. The independent inverter-enable and
shutdown outputs must still drop through their existing fail-safe paths; CAN is
not the only torque-removal mechanism.

## Conformance tests

- Target encoder/decoder and bit-flip tests:
  `AMS/host_tests/sop/sop_test.c`
- Passive dashboard decoder:
  `Tools/esp32_ams_dashboard_logger/tests/decoder_smoke_test.c`
- ECU atomic-bundle consumer:
  `Tools/sop_reference/tests/ecu_power_consumer_test.c`

Any incompatible byte-layout, scaling, flag, age, or recovery change requires
a protocol-version increment and simultaneous AMS, ECU, dashboard, test, and
documentation updates. A new optional ID may remain on protocol v2 only when
existing required-frame semantics and fail-zero behavior remain backward
compatible, as with `0x68A`.
