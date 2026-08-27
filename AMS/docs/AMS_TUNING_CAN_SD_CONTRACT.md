# DER26 passive estimator/SoP/fuse tuning contract

This extension is read-only test telemetry. It is never an input to BMS_OK,
AIR, balancing, charger, DCL/CCL, torque, or shutdown decisions. IDs
`0x6B5..0x6C0` are always placed in the disposable CAN `DETAIL` class after
critical and protected AMS traffic.

The stream is present only in a 500-kbit/s image. A 250-kbit/s image rejects an
attempt to enable it at compile time. At runtime it latches off for the rest of
the boot after a protected deadline/latency violation, protected HAL load
failure, CAN-task deadline miss, bus-off/recovery, or scheduler/build integrity
failure. Losing tuning traffic is not an AMS fault and cannot restore authority.

All multibyte values are big-endian. Segment and cell indices are zero-based.
The ECU raw `CAN###.BIN` receive timestamp remains the transport timestamp.

| ID | Rate | Payload |
|---|---:|---|
| `0x6B5` | 10 Hz/segment | segment, snapshot seq8, SoC 0.01%, Vp1 mV, Vp2 mV |
| `0x6B6` | 10 Hz/segment | segment, snapshot seq8, measured segment mV, predicted segment mV, innovation mV |
| `0x6B7` | 2 Hz/segment/page | covariance, adaptive measurement R, reject/clamp/fault counters |
| `0x6B8` | 2 Hz/segment/page | R0/growth/confidence plus reference R0, variance and reject flags |
| `0x6B9` | 2 Hz/segment/page | RAW/AVG8/IIR voltage, electrical/thermal inputs, epoch keys/ages, health counters |
| `0x6BA` | 5 Hz/horizon | raw-model, strategy-limited, final discharge current (0.1 A) |
| `0x6BB` | 5 Hz/horizon | raw-model, strategy-limited, final charge current (0.1 A, signed) |
| `0x6BC` | 5 Hz/horizon/page | binding segment/cell and voltage/power extrema |
| `0x6BD` | 5 Hz/page | fuse utilization, temperature, derating, current/melt-time context |
| `0x6BE` | 5 Hz/horizon | fuse cap, hardware cap, low16 SoP reason flags |
| `0x6BF` | 10 Hz/page | full estimator step, source tick, measurement sequence, power validity/authority |
| `0x6C0` | 5 Hz/horizon | charge power and full 32-bit SoP reason flags |

`0x6B9` byte 1 uses bits 7:6 as page and bits 5:0 as snapshot sequence:

- page 0: RAW, AVG8 and IIR segment voltage in mV (`0xFFFF` invalid);
- page 1: pack current 0.1 A, EKF surface temperature 0.1 C, core temperature 0.1 C;
- page 2: low16 measurement sequence, low16 current-window sequence,
  measurement/current ages in 10 ms units;
- page 3: fresh thermistor count, model-domain flags, low16 estimator step,
  saturated SoH accept/reject counts.

`0x6BD` page 2 carries the full 16-bit fuse reason vector plus typical and usable melt time.

The estimator task is the sole snapshot writer. It builds into one of two
static buffers; the CAN task pins and copies the published buffer. If the
inactive buffer is busy, the tuning publication is dropped. There is no heap,
wait, FatFs call, or CAN ISR formatting on this path.

The ECU decoder preserves every accepted frame in `CAN###.BIN` and produces:

- `CAN###_ams_ekf.csv`
- `CAN###_ams_ekf_cov.csv`
- `CAN###_ams_soh.csv`
- `CAN###_ams_sop.csv`
- `CAN###_ams_fuse.csv`

These supplement the existing cell, temperature, snapshot, and APM tables.
Decoder changes can therefore be replayed against the immutable raw log.

The conservative 500-kbit/s planning model is 19.36% AMS-only, 38.26% with
required ECU/CM200 traffic, and 42.04% with optional CM200 logging assumptions.
Those values are planning gates, not a substitute for measured utilization,
protected response time, target linker-map RAM, or task stack high-water tests.
