# AMS Passive Logger CAN Contract v1

## Classification

The original `0x690-0x6B0` stream is passive, read-only telemetry. DER26-CAN-V4 extends the live passive range through `0x6C0`; the original v1 payloads below remain unchanged. Deep tuning payloads `0x6B5..0x6C0` are specified in `AMS_TUNING_CAN_SD_CONTRACT.md`. It exists for the ECU
SD recorder, dashboard/logger nodes, bench analysis, and post-run diagnosis.
It does not control BMS_OK, balancing, charger output, AIRs, watchdog feeding,
or ECU torque authority.

All frames use standard 11-bit identifiers, DLC 8, and big-endian multibyte
fields unless a raw-byte field is explicitly named. The payload definitions in
this file are the original v1 layouts; the live DER26-CAN-V4 logger protocol
version is `3` and the detail snapshot version remains `2`.

## Scheduling and snapshot behavior

The CAN task runs on the 10 Hz fast period. Compact ECU and power-authority
frames are sent first. Detail telemetry is divided into `NSMBS` phases:

- Phase zero carries summary frames and the current phase's detailed data.
- Later phases carry the corresponding SMB cell, temperature, masks, and
  diagnostics.
- `0x6AD` identifies snapshot version, logger sequence, phase, phase count, and
  the source measurement sequence.
- With the temporary single-SMB build, phase count is one and all detail is sent
  each 100 ms cycle.
- With the five-SMB vehicle topology, one segment's detail is sent per cycle and
  a full detailed sweep completes every five cycles.

If the compact/power bundle cannot be transmitted, slower detail traffic is
suppressed for that cycle. CAN error, bus-off, recovery, compact failure, detail
failure, and deadline counters remain available in CLI/CAN diagnostics.

## Summary identifiers

| ID | Name | Main contents |
|---:|---|---|
| `0x690` | Heartbeat | protocol, sequence, AMS state, BMS/AIR/IMD/fault/inhibit flags, validity and current-fault flags, uptime |
| `0x691` | Fault reasons | voltage, temperature, current pending/latched reasons and current mode |
| `0x692` | Pack electrical | pack voltage 0.1 V, current 0.1 A, min/max cell mV |
| `0x693` | Temperature/fan | max/min/average 0.1 C, max fan command, thermal flags |
| `0x694` | Voltage health | min/max locations, usable/updated/stale/PEC-failed cell counts |
| `0x695` | Temperature health | min/max locations, usable/updated/stale/invalid sensor counts |
| `0x696` | Charger | target/read voltage/current and charger/charge-stop flags |
| `0x697` | Current detail | current, selected range, reason, fault reasons, pending time |
| `0x698` | ADBMS6830 link | HAL/xfer/op status, errors, PEC and command-counter masks |
| `0x699` | ADBMS6830 counters | error/counter counts, PEC pass mask, last command |
| `0x69A` | ADBMS2950 link | HAL/xfer/op status, errors, PEC mask, enabled flag, IC count |
| `0x69B` | Task health | heartbeat stale/seen/safety masks and logger heartbeat count |
| `0x69C` | CAN diagnostics | HAL CAN error code, bus-off/error/recovery counts and state |
| `0x69D` | Safety diagnostics | reset flags, panic reason/count, output-block count and inhibit/fault flags |
| `0x69E` | Watchdog diagnostics | runtime/start/feed state, block reason/counts, feed age |
| `0x69F` | ADBMS diagnostics | scan/status/config/open-wire counts and degraded/HIL/SMB state |

## Detail identifiers

| ID | Name | Main contents |
|---:|---|---|
| `0x6A0` | Cell detail | segment/start-cell followed by three cell voltages in mV |
| `0x6A1` | Temperature detail | segment/start-sensor followed by three temperatures in 0.1 C |
| `0x6A2` | Voltage masks | updated/stale/open/short or other per-segment voltage masks |
| `0x6A3` | Temperature masks A | first set of per-segment temperature fault/valid masks |
| `0x6A4` | Temperature masks B | second set of per-segment temperature fault/valid masks |
| `0x6A5` | Voltage PEC | per-segment PEC and communication status |
| `0x6A6` | Current ADC | high/low ADC counts, selected range/reason, freshness/valid/calibration flags |
| `0x6A7` | Charger detail | read current, disable-reason mask and TX/RX counters |
| `0x6A8` | Temperature diagnostics | thermistor/fan diagnostic summary |
| `0x6A9` | Temperature diagnostics A | phased detailed temperature diagnostic masks |
| `0x6AA` | Temperature diagnostics B | phased detailed temperature diagnostic masks |
| `0x6AB` | Voltage diagnostics | phased jump/stuck/redundancy diagnostics |
| `0x6AC` | RTOS diagnostics | free/min heap, stack-warning mask, minimum high-water and RTOS fault flags |
| `0x6AD` | Snapshot metadata | version, sequence, phase, phase count, source measurement sequence |

## ADBMS2950/APM identifiers

### `0x6AE` — APM sample

| Bytes | Field | Encoding |
|---:|---|---|
| 0-1 | Current channel 1 | signed 0.01 A; `INT16_MIN` when invalid |
| 2-3 | Current channel 2 | signed 0.01 A; `INT16_MIN` when invalid |
| 4-5 | VBAT channel 1 | unsigned 0.1 V; `UINT16_MAX` when invalid |
| 6-7 | VBAT channel 2 | unsigned 0.1 V; `UINT16_MAX` when invalid |

### `0x6AF` — APM health

| Byte | Field | Meaning |
|---:|---|---|
| 0 | Flags | initialized, SID valid, config valid, sample valid, current valid, pack-voltage valid, REFUP, snapshot active |
| 1 | Last stage | raw ADBMS2950 driver stage |
| 2 | Last reason | raw ADBMS2950 failure reason |
| 3 | Device ID | expected ADBMS2950B device identifier |
| 4-5 | I1 conversion count | unsigned big-endian |
| 6-7 | Sample age | milliseconds, saturated at 65535 |

### `0x6B0` — APM raw

| Bytes | Field | Meaning |
|---:|---|---|
| 0-3 | I1 raw | signed raw current code, big-endian representation |
| 4-5 | VB1 raw | signed raw voltage code, big-endian |
| 6 | I1 conversion phase | raw phase/counter state |
| 7 | Calibration profile | DER 100 micro-ohm or EVAL 50 micro-ohm profile enum |

The standalone EVAL image can publish valid APM current while SMB voltage and
temperature authority remain unavailable. That is intentional for isolated
ADBMS2950 bench work and must not be interpreted as complete accumulator safety
readiness.

## Receiver rules

- Treat the live passive range `0x690-0x6C0` as observability only.
- Use `0x6AD` to associate phased detail with a source measurement sequence.
- Preserve invalid sentinels rather than converting them to zero.
- Detect missing phases and sequence stagnation in logger tooling, not in torque
  authority.
- Continue using compact `0x680-0x683` and dynamic power frames for ECU safety
  and torque decisions.


## DER26-CAN-V4 passive extensions

The V4 scheduler adds `0x6B1..0x6BE` without changing the authority boundary:

- `0x6B1`: optional legacy compatibility relocation; disabled in the vehicle build.
- `0x6B2`: estimator diagnostic detail.
- `0x6B3`: TX scheduler/recovery diagnostics.
- `0x6B4`: estimator voltage-product comparison. Byte 0 is the active estimator index; byte 1 carries RAW/AVG8/IIR validity plus the compile-selected estimator source; bytes 2-3 are signed AVG8-minus-RAW mV; bytes 4-5 are signed IIR-minus-RAW mV; bytes 6-7 are the low 16 bits of the measurement sequence.
- `0x6B5..0x6C0`: disposable test-day tuning records documented separately.

`0x6B4` is observational only. RAW C voltage remains the battery-safety voltage source regardless of the estimator-source experiment.
