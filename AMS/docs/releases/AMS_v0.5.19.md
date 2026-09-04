# AMS v0.5.19 Five-SMB Passive Ring Observer

Source revision: `DER26-AMS-v0.5.19-20260903`

This release makes the normal five-SMB `BENCH_VALIDATION` image useful on an
unloaded LV ring that does not include the DHAB current sensor or a validated
thermistor-mux bus.

## Changes

- The five segment EKFs may acquire advisory OCV-based SoC with an explicit
  open-ring zero-current assumption when no valid DHAB current is available.
- If no usable thermistor sample exists, the advisory EKFs use a fixed 25 C
  surface temperature.
- The fallback is compile-time restricted to the five-SMB
  `BENCH_VALIDATION` profile.
- Measurement current and temperature validity are not synthesized. Capacity
  SoH, resistance SoH, SoP, BMS_OK and balancing therefore remain unavailable
  or inhibited.
- The CLI banner identifies passive-ring mode and `estimator` prints all five
  segment SoC/acquisition states.
- The legacy CLI version value is aligned with the source revision.

## Bench-use limits

Use this image only with the cell stack electrically unloaded: no charger, no
load and no path capable of pack current. Keep the cells near 25 C. The fixed
temperature value is a bench assumption, not a measured temperature.

The Rev5 SMB BOM still contains 100-ohm GPIO4/GPIO5 pull-ups. Automatic
thermistor scanning remains disabled until that network is corrected and
validated.

## Expected observation

After five complete SMB voltage images and roughly 20 seconds of uninterrupted
rest, `estimator` should show five instances with `valid:1`, `acq:2` and
`reason:10`. The reported SoC is advisory OCV-based SoC. `power` should remain
non-authoritative, and SoH validity should remain zero.

