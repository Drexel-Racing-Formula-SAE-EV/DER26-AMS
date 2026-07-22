# AMS software hardening release notes — 2026-07-17

## Scope

This revision implements software-only closures from the 2026-07-17
whole-system review. It deliberately does not encode guessed polarity,
calibration, timing, or electrical behavior for hardware that has not been
measured.

Baseline archive SHA-256:

`3d17abcf9c608e18ab7fb001286cf25976b42ea2f260a2010645c06e153cdffd`

## Implemented closures

- `STATE_START` can no longer authorize BMS_OK. Because the current hardware
  owns precharge, START is treated as software initialization; a fully healthy
  input set advances to the discharge/drive policy, then waits for a fresh
  current-policy publication before a permit can be evaluated.
- Normal ADBMS acquisition is 10 Hz; the intentionally inhibited hardware
  bring-up profile remains 1 Hz. Status, configuration, and open-wire periods
  are specified in milliseconds and converted to scan counts from the selected
  profile rate. Target timing/jitter evidence is still required.
- A complete ADBMS6830 S-cell open-wire diagnostic now runs even and odd
  phases, waits with bounded timing, reads the five populated register groups,
  validates PEC and command counters, maps failures to 15 physical cell
  channels per SMB, suppresses affected-channel freshness, and latches an
  ADBMS diagnostic fault until reset on incomplete or detected-open results.
- Current ADC acquisition uses the longest STM32F767 sample time to accommodate
  the schematic divider/filter impedance. This is a conservative software
  correction, not a substitute for oscilloscope settling and calibration.
- The fan task is included in the safety heartbeat policy.
- IMD period/high-time capture is published through an odd/even sequence
  snapshot; the task rejects in-progress, changed, or stale capture data.
- The bxCAN hardware filter accepts only the charger extended data-frame ID in
  normal builds. Explicit HIL builds add only the five approved standard IDs.
  Filter-configuration failure is checked, and the ISR/task semantic allowlist
  remains active.
- The advisory estimator rejects invalid current and will not coulomb-count it;
  voltage and temperature quality are required before measurement correction.
- All nine application tasks use static TCB/stack storage. The ADBMS mutex and
  FreeRTOS idle task, timer task, and timer queue are statically backed.
- The retained fault ring has schema version 2, persistent boot and event
  sequences, CRC-32, commit-last publication, metadata reconstruction,
  rollover handling, and injected torn/corrupt-record tests. Old unversioned
  history is cleared on first boot because its integrity cannot be established.
- Target-specific ownership and barrier assumptions are documented in
  `Core/docs/CONCURRENCY_OWNERSHIP.md`.

## Deliberately not claimed or enabled

- Current-sensor polarity, gain, offset over temperature, fast-trip latency,
  and protection thresholds are not physically validated.
- IMD decoding remains disabled by default pending the exact installed manual,
  pull network/polarity confirmation, timer-clock measurement, and injected
  hardware faults.
- AIR auxiliary/precharge/bus-voltage monitoring remains disabled because the
  current board lacks the required protected physical-state inputs.
- `fuse_fault` still has no hardware producer; fuse monitoring is not claimed.
- The Q2/BMS safety-output circuit has not been electrically cleared.
- Thermistor curve/tolerance/mux settling and balancing thermal limits remain
  physical validation items.
- Full-ring bidirectional isoSPI failover is not implemented; String A remains
  the five-SMB path and String B the one-APM path.
- ADBMS2950 measurements remain advisory until shunt/divider polarity,
  calibration, and final-ring operation are physically proven.
- Hardware watchdog support remains disabled by default until target LSI,
  reset-output behavior, and repeated-fault policy are validated.
- There is no firmware-owned contactor/precharge controller or released ECU
  operating-mode request protocol. The current hardware precharge circuit
  remains authoritative.

## Required target evidence before release

1. Cross-compile and link the exact tree with the ARM toolchain and warnings as
   errors; retain ELF, map, flash/RAM, symbols, and stack budgets.
2. Run current-limited LV tests with BMS_OK and balancing physically inhibited.
3. Verify SPI mode/rate/CS, five-SMB timing, APM path, and each injected
   open-wire location using a logic analyzer and safe simulator.
4. Measure ADC settling and current polarity/gain/offset, thermistor behavior,
   IMD PWM/status electrical mapping, and watchdog timeout/reset behavior.
5. Resolve and test the Q2 output stage before reconnecting the shutdown loop.
6. Re-run the full target/HIL fault matrix before any HV or vehicle operation.

Passing host tests establishes software behavior under the test model only. It
does not clear the firmware or hardware for energized HV operation.
