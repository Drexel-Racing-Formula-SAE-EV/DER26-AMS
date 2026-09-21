# DER26 passive estimator/SoP/fuse tuning contract

This extension is read-only test telemetry. It is never an input to BMS_OK,
AIR, balancing, charger, DCL/CCL, torque, or shutdown decisions. IDs
`0x6B5..0x6C0` are always placed in the disposable CAN `DETAIL` class after
critical and protected AMS traffic.

The stream is present in the nominal 1 Mbit/s image and remains available in a
500-kbit/s fallback image. A 250-kbit/s image rejects an attempt to enable it
at compile time. At runtime it latches off for the rest of
the boot after a protected deadline/latency violation, protected HAL load
failure, CAN-task deadline miss, bus-off/recovery, or scheduler/build integrity
failure. Losing tuning traffic is not an AMS fault and cannot restore authority.

All multibyte values are big-endian. Segment and cell indices are zero-based.
The ECU raw `CAN###.BIN` receive timestamp remains the transport timestamp.

| ID | Rate | Payload |
|---|---:|---|
| `0x6B5` | 10 Hz/segment | segment, snapshot seq8, SoC 0.01%, Vp1 mV, Vp2 mV |
| `0x6B6` | 10 Hz/segment | segment, snapshot seq8, measured segment mV, predicted segment mV, innovation mV |
| `0x6B7` | 2 Hz/segment/page | full 3x3 covariance, adaptive measurement R, exact prior innovation sigma, reject/clamp/fault counters |
| `0x6B8` | 2 Hz/segment/page | R0/growth/confidence plus reference R0, variance and reject flags |
| `0x6B9` | 2 Hz/segment/page | RAW/AVG8/IIR voltage, electrical/thermal inputs, epoch keys/ages, health counters |
| `0x6BA` | 5 Hz/horizon | raw-model, strategy-limited, final discharge current (0.1 A) |
| `0x6BB` | 5 Hz/horizon | raw-model, strategy-limited, final charge current (0.1 A, signed) |
| `0x6BC` | 5 Hz/horizon/page | binding segment/cell and voltage/power extrema |
| `0x6BD` | 5 Hz/page | fuse utilization, temperature, derating, current/melt-time context |
| `0x6BE` | 5 Hz/horizon | fuse cap, hardware cap, low16 SoP reason flags |
| `0x6BF` | 10 Hz global + 1 Hz/segment acquisition | full estimator step, source tick, measurement sequence, power validity/authority, acquisition state/reason/candidate |
| `0x6C0` | 5 Hz/horizon | charge power and full 32-bit SoP reason flags |


`0x6B7` byte 1 uses bits 7:6 as page and bits 5:0 as snapshot sequence:

- page 0: `P_soc`, `P_vp1`, `P_vp2`, each unsigned and scaled by `1e9`;
- page 1: scalar `P_r0` scaled by `1e12`, adaptive segment-voltage `R` scaled
  by `1e9`, and low16 `dt_clamp_count`;
- page 2: low16 innovation-reject count, low16 EKF fault flags, and the exact
  prior innovation sigma `sqrt(S)` in segment mV;
- page 3: signed `P_soc,vp1`, `P_soc,vp2`, and `P_vp1,vp2`, each scaled by
  `1e7`. `INT16_MIN` is the invalid sentinel for these signed cross terms.

The page-3 cross terms complete the symmetric production covariance required for
offline three-state NEES. The page-2 innovation sigma is generated directly from
the prior scalar innovation variance used by the production gate, so offline NIS
does not reconstruct hidden prior covariance.

`0x6B9` byte 1 uses bits 7:6 as page and bits 5:0 as snapshot sequence:

- page 0: RAW, AVG8 and IIR segment voltage in mV (`0xFFFF` invalid);
- page 1: pack current 0.1 A, EKF surface temperature 0.1 C, core temperature 0.1 C;
- page 2: low16 measurement sequence, low16 current-window sequence,
  measurement/current ages in 10 ms units;
- page 3: fresh thermistor count, model-domain flags, low16 covariance-repair
  count, saturated SoH accept/reject counts. The full estimator step already exists
  in `0x6BF` global page 0.


`0x6BF` keeps its existing 10 Hz global pages (`payload[0] == 0` and `1`).
At 1 Hz it additionally emits one acquisition page per segment with
`payload[0] = 0x80 | segment`:

- byte 1: snapshot sequence;
- byte 2: acquisition state;
- byte 3: acquisition reason;
- byte 4: fit-window sample count;
- byte 5: saturated acquisition reject count;
- bytes 6-7: candidate SoC in 0.01% units (`0xFFFF` when unavailable).

This is observational only. A missing acquisition page cannot create or restore
SoP/BMS authority.

`0x6BD` page 2 carries the full 16-bit fuse reason vector plus typical and usable melt time.

The estimator task is the sole snapshot writer. It builds into one of two
static buffers; the CAN task pins and copies the published buffer. If the
inactive buffer is busy, the tuning publication is dropped. There is no heap,
wait, FatFs call, or CAN ISR formatting on this path.

The ECU raw logger is expected to preserve every accepted frame in `CAN###.BIN`.
The ECU decoder source is maintained outside this AMS repository, so protocol-v4
decoding of the new cross-covariance/acquisition pages must be updated and verified in
that ECU source line before claiming the derived CSV schema is current. Existing
expected derived products include `CAN###_ams_ekf.csv`,
`CAN###_ams_ekf_cov.csv`, `CAN###_ams_soh.csv`, `CAN###_ams_sop.csv`, and
`CAN###_ams_fuse.csv`. Because the raw log is immutable, decoder updates can be
replayed later without losing the on-vehicle frames.

The conservative 1-Mbit/s planning model is 9.88% AMS-only, 19.33% with
required ECU/CM200 traffic, and 21.22% with optional CM200 logging assumptions.
Those values are planning gates, not a substitute for measured utilization,
protected response time, target linker-map RAM, or task stack high-water tests.
