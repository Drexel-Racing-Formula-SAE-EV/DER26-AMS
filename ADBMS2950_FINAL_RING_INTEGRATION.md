# ADBMS2950 Final-Ring Integration

Authored by Mahad Faisal, 2026.

## Scope

This revision integrates the APM ADBMS2950 into the final AMS firmware. It does
not include a separate evaluation-board branch or evaluation-board pinout.

Final isoSPI order:

```text
AMS String A -> 5 x ADBMS6830 SMB -> 1 x ADBMS2950 APM -> AMS String B
```

String A is the production five-SMB view. String B is the one-device APM view.
The two drivers share the same SPI6 transaction lock and never publish a mixed
SMB/APM response buffer.

## Initialization Ownership

1. The ADBMS6830 driver selects String A, performs the one global chain reset,
   configures the five SMBs, and verifies both configuration groups.
2. Only after the SMB subset is ready, the ADBMS2950 driver selects String B.
3. The APM driver does not issue a second reset.
4. RDSID must identify the expected ADBMS2950 derivative before any APM-specific
   write is sent.
5. CFGA and CFGB are written and read back byte-for-byte.
6. GPO1/GPO2, which control the APM high-voltage dividers, default to push-pull
   low through `AMS_APM_ENABLE_HV_DIVIDERS=0`.
7. A failed configuration write or readback leaves initialization failed and
   triggers one bounded best-effort divider-off write. Cleanup success cannot
   hide the original failure.

## Periodic Sampling

The existing ADBMS6830 ADCV command is compatible with the ADBMS2950 ADI1
conversion command, so the String-A conversion starts both device types. The
firmware then:

1. waits the complete conversion interval;
2. reads the five SMBs through the individual cell-register group commands on
   String A;
3. reads APM RDSTAT and RDIVB1 as a one-device response on String B;
4. accepts the APM sample only when transport, data PEC, calibration state,
   reset/clear sentinels, and the RDSTAT/RDIVB1 command-counter pair are valid;
5. preserves the last good numerical value but clears validity on any failure.

The read-all shortcut is not used for the mixed chain.

## Safety Boundary

The APM is enabled and observable, but remains `ADVISORY_NON_GATING`:

- APM current and voltage cannot assert or deassert BMS_OK.
- An APM initialization or sample failure does not erase a verified SMB state.
- The existing DHAB current sensor remains the authoritative current input.
- APM pack voltage is marked invalid while the high-voltage dividers are off.
- No contactor, charger, balance, or shutdown decision uses APM data.

Promotion to a safety input requires final-board validation of shunt value and
polarity, divider population and ratio, current/voltage accuracy, timing,
plausibility thresholds, fault persistence, and failure recovery.

## CLI

The final firmware provides:

```text
apm status
apm health
apm sid
apm config
apm sample
apm scope [1-100]
apm clear
apm enable
apm disable
bringup apm2950
```

`apm sid`, `apm config`, `apm sample`, and `apm scope` are refused while the
periodic ADBMS scan owns the bus. `apm scope` repeats a read-only RDSID identity
transaction and bounds the repeat count from 1 through 100.

## Host Verification

Run from `AMS/host_tests`:

```bash
make apm-sil
```

This gate runs the actual ADBMS2950 driver unit/injection tests plus system SIL
tests for final-ring ownership, advisory failure behavior, and CLI commands.
The complete `make ci`, sanitizer, static-analysis, feature-profile, and stress
matrix should also pass before target testing.

Validated on the delivered source tree:

- 38 actual-driver/unit tests passed;
- 88 comprehensive system SIL/injection tests passed;
- the dedicated `apm-sil` gate passed;
- production feature and BMS_OK ownership gates passed;
- bring-up, CAN-fed ADBMS HIL, IMD-enabled and IWDG-enabled profiles passed;
- AddressSanitizer plus UndefinedBehaviorSanitizer passed;
- standalone UndefinedBehaviorSanitizer passed;
- GCC static analysis passed;
- 50,000 deterministic fuzz cycles and 12,000 concurrent stress cycles passed;
- strict non-host GCC syntax checking passed for every modified production
  translation unit.

Covered APM faults include:

- invalid topology and capacity;
- wrong product identity;
- transport failure;
- data PEC failure;
- bounded write-PEC generation with no payload overrun or mutation;
- configuration write/readback failure;
- fail-low divider cleanup;
- uncalibrated current ADC;
- incoherent command counters;
- reset/clear sentinel results;
- stale-value preservation;
- scan/CLI collision prevention;
- APM failure remaining non-gating.

## Hardware Validation Still Required

An ARM cross-compiler was not available, so no STM32 ELF was linked. Host tests
do not prove SPI mode or timing on the STM32 target, transformer and
isoSPI polarity, physical ring order, APM shunt polarity, divider wiring,
measurement accuracy, target stack/timing, or behavior under real faults.

Start with BMS_OK physically inhibited, balancing inhibited, divider controls
off, no HV/contactors/charger, a current-limited LV supply, and read-only SID,
configuration, status, and sample commands. Do not treat this package as
authorization for HV operation.

The implementation was checked against the supplied reference material and
public device documentation. No third-party reference source or confidential
document is redistributed in this package.
