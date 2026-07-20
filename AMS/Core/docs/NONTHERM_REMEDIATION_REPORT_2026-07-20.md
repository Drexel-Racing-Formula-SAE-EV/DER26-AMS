# DER26 AMS non-thermistor remediation report

Date: 2026-07-20
Baseline: `a46811a`
Source review: `DER26_AMS_Pre_SoP_SoH_Firmware_Deep_Review_2026-07-20.md`

## Scope

This revision addresses software-resolvable findings from the pre-SoP/SoH
review. Thermistor-model review and changes were explicitly excluded. No
physical validation claim is made for the DHAB polarity, IMD, AIR feedback,
fuse sensing, balancing current, fan airflow, ADBMS timing, watchdog reset, CAN
bus utilization or STM32 target execution time.

## Finding disposition

| Review item | Disposition |
|---|---|
| P0-1 ADBMS deadline | Reworked into a phased schedule with execution/deadline diagnostics and measured verified balance on/off intervals; target timing remains required. |
| P0-2 balance on-time collapse | Unconditional recovery was removed. Recovery is conditional on prior balancing, minimum on-time is enforced, and the actual verified apply-to-clear and clear-to-voltage intervals are recorded and tested; bleed-current/thermal proof remains hardware work. |
| P0-3 vehicle profile | Added explicit bench, HIL and locked vehicle profiles plus a versioned build manifest. Vehicle compilation separately requires current-polarity and calibration-procedure evidence, along with the other target gates, and forbids bench/HIL mutation. |
| P0-4 current sign/regen | Not inferable in software. The vehicle profile remains compile-blocked until signed physical current and CM200 policy are validated. |
| P1-1 coherent measurements | Added immutable timestamped measurement snapshots with reader-pinned static double buffering, gap-preserving sequence numbers, drop-on-contention diagnostics and estimator stale-publication rejection. The estimator copy is static task-owned storage rather than a large automatic object on its 4 KiB stack; compile-time RAM ceilings guard future growth. |
| P1-2 estimator time base | Estimator now consumes each voltage epoch once and derives `dt` from consecutive voltage-completion timestamps. |
| P1-3 current interval data | Added timestamped current windows with latest/average/RMS/min/max, signed and absolute charge integrals, invalid-gap accounting and lifetime totals. |
| P1-4 calibration integrity | Added fixed-width CRC/versioned calibration records with ID/time/temperature/reference rails/uncertainty and live-zero proof on restore. Restore reconstructs the live zero using the record's reference rails and requires agreement with the stored offsets inside bounded uncertainty/resolution limits, rejecting a valid-CRC record from the wrong board. Any service calibration mutation invalidates the active current epoch and scalar readiness until a fresh owner-task sample. Calibration provenance is carried in the immutable current epoch, and a missing or changed record rejects resistance-SoH confidence. Estimator confidence additionally requires an explicit physical calibration-procedure gate. No NVM adapter exists yet, so persistence is not claimed. |
| P1-5 temperature timing/model | Excluded by user instruction in this remediation. |
| P1-6 R0 observability | Added explicit observation gates, reject reasons/counters, reference R0, growth ratio, variance/confidence and advisory validity. Advisory-valid and last-observable flags now clear when the accepted observation becomes stale; historical convergence is retained. No persistent SoH history is claimed. |
| P1-7 weakest segment | Pack and five-segment estimator topology are now explicit build choices. Pack remains the default; segment topology requires target WCET and local-input validation before authority. |
| P1-8 model domain | Low/high LUT clamp and core-temperature clamp are exported as explicit domain flags. |
| P1-9 R0 units | Summary now distinguishes representative-cell R0 from effective pack R0 and tests the series/parallel conversion. |
| P1-10 CAN burst | Slow traffic is phased. Critical charger traffic is first; compact failure suppresses detail; traffic-class and deadline counters are exposed. Whole-vehicle utilization remains a release calculation. |
| P1-11 CAN coherence | Measurement payloads use an immutable age-checked snapshot, never a live-state fallback, and compact non-measurement fields are frozen once per bundle. The present wire contract still lacks a sequence/timestamp in every frame and is not an authoritative SoP contract. |
| P2-1 watchdog | Vehicle profile requires IWDG and physical reset validation evidence; host gates remain advisory. |
| P2-2 fan proof | No tach/airflow hardware exists; command is not treated as proof of cooling. |
| P2-3 target build hygiene | Corrected stale DER25 CubeIDE paths/identity, enabled target `-Wall -Wextra`, and added an XML/project metadata CI gate. ARM compile/link/map/stack/WCET evidence is still required. |
| P2-4 incomplete inputs | Fuse producer, physical AIR feedback, validated IMD, APM authority and automatic dual-direction isoSPI failover remain explicitly absent/advisory and fail closed where safety-relevant. |

## Safety boundary

The estimator and resistance-SoH outputs remain advisory. They do not own
`BMS_OK`, AIRs, shutdown, charging, balancing or torque limits. Dynamic SoP has
not been implemented in this revision. A failed or stale estimator selects no
new authority; existing independent threshold and shutdown paths remain in
control.

## Remaining release evidence

1. ARM Debug and Release compile/link with exact compiler version and flags.
2. Archive ELF, map, section sizes and stack high-water measurements.
3. Measure ADBMS phase timing, balance recovery/on-time and task WCET/jitter.
4. Validate DHAB channel mapping, polarity, reference rails, offsets and range
   handover with known signed current.
5. Implement and power-loss-test a redundant/wear-managed calibration storage
   adapter before claiming persistence.
6. Validate IWDG reset behavior, IMD input, AIR feedback, fuse producer and CAN
   stale-data behavior on target hardware.
7. Recalculate whole-vehicle CAN utilization and define the future versioned
   SoP frame contract.

## Verification performed

The following passed against the final pre-commit diff with GCC 13.3.0:

- Strict C11 comprehensive build with `-Wall -Wextra -Werror` and 104
  host injection/SIL cases.
- Aggregate `make ci`: 45 isolated AMS unit tests, the comprehensive suite,
  ADBMS2950/final-ring SIL, bench/HIL/locked-vehicle profile gates,
  pack/five-segment estimator topology gates, production feature gates,
  static-allocation and state-ownership gates, CubeIDE project metadata gate,
  and GCC static analyzer.
- AddressSanitizer plus UndefinedBehaviorSanitizer for unit and comprehensive
  paths.
- Standalone UndefinedBehaviorSanitizer for unit and comprehensive paths.
- 50,000-cycle deterministic fuzz plus 12,000-cycle concurrent stress.
- Separate hardware-bring-up, CAN-fed ADBMS HIL, IMD-enabled,
  AIR-feedback-stub, and IWDG-enabled builds/tests.
- Clean `git diff --check` and a changed-file audit confirming that no
  thermistor production source or generated model data was modified by this
  remediation. The inherited thermistor tests still ran as part of aggregate
  CI but are not claimed as work in this scope.

This environment did not provide `arm-none-eabi-gcc`, Clang or cppcheck. Host
success does not substitute for an ARM target compile/link, final ELF/map,
target stack/WCET evidence or physical fault injection.
