/*
 * ams_build_profile.h
 * Author: Mahad Faisal (2026)
 *
 * One explicit build intent owns the safety-relevant feature defaults.  The
 * default image is deliberately a physically inhibited bench image.  A
 * vehicle image is impossible to compile until each target-validation gate is
 * acknowledged by the release build.
 */

#ifndef INC_AMS_BUILD_PROFILE_H_
#define INC_AMS_BUILD_PROFILE_H_

#include "ams_version.h"

#define AMS_PROFILE_BENCH   1
#define AMS_PROFILE_HIL     2
#define AMS_PROFILE_VEHICLE 3
#define AMS_PROFILE_TESTDAY 4
#define AMS_PROFILE_BENCH_VALIDATION 5

/* TESTDAY is an observation/bring-up image. Generic service CLI must never
 * turn it into an authority image at runtime. A future evidenced release gets
 * a separately named source profile/configuration instead of overriding these. */
#define AMS_TESTDAY_BMS_RELEASE_ALLOWED      0
#define AMS_TESTDAY_BALANCE_RELEASE_ALLOWED  0
#define AMS_BENCH_VALIDATION_BMS_RELEASE_ALLOWED      0
#define AMS_BENCH_VALIDATION_BALANCE_RELEASE_ALLOWED  0

/* Optional isolated one-SMB hardware-validation variant.  The normal
 * BENCH_VALIDATION image remains the five-SMB String-A topology.  Defining
 * this to 1 selects one logical SMB on String B while retaining the same
 * compile-time BMS_OK/balancing inhibits. */
#ifndef AMS_BENCH_VALIDATION_SINGLE_SMB
#define AMS_BENCH_VALIDATION_SINGLE_SMB 0
#endif

/* Final authority gates used at the hardware-write/application boundary.
 * These are intentionally compile-time expressions, not mutable app flags.
 * TESTDAY and BENCH_VALIDATION are observation/bring-up images and cannot
 * acquire BMS_OK or balancing authority through the service CLI. */
#define AMS_PROFILE_BMS_RUNTIME_AUTHORITY_ALLOWED \
    (((AMS_BUILD_PROFILE != AMS_PROFILE_TESTDAY) || AMS_TESTDAY_BMS_RELEASE_ALLOWED) && \
     ((AMS_BUILD_PROFILE != AMS_PROFILE_BENCH_VALIDATION) || AMS_BENCH_VALIDATION_BMS_RELEASE_ALLOWED))
#define AMS_PROFILE_BALANCE_RUNTIME_AUTHORITY_ALLOWED \
    (((AMS_BUILD_PROFILE != AMS_PROFILE_TESTDAY) || AMS_TESTDAY_BALANCE_RELEASE_ALLOWED) && \
     ((AMS_BUILD_PROFILE != AMS_PROFILE_BENCH_VALIDATION) || AMS_BENCH_VALIDATION_BALANCE_RELEASE_ALLOWED))
#define AMS_BUILD_CONFIG_FINGERPRINT 0xA5051405u

#ifndef AMS_BUILD_PROFILE
#define AMS_BUILD_PROFILE AMS_PROFILE_BENCH
#endif

/* Passive five-SMB ring observer used for LV bench characterization when the
 * accumulator DHAB and the thermistor mux bus are not connected.  This only
 * permits the segment EKFs to acquire an OCV-based advisory SoC using an
 * explicit zero-current/open-ring assumption and a 25 C temperature fallback.
 * It does not change measurement-valid bits, SoH/SoP authority, BMS_OK, or
 * balancing.  Disable it for any bench setup that can carry pack current. */
#ifndef AMS_ENABLE_BENCH_PASSIVE_RING_ESTIMATOR
#if (AMS_BUILD_PROFILE == AMS_PROFILE_BENCH_VALIDATION) && \
    !AMS_BENCH_VALIDATION_SINGLE_SMB
#define AMS_ENABLE_BENCH_PASSIVE_RING_ESTIMATOR 1
#else
#define AMS_ENABLE_BENCH_PASSIVE_RING_ESTIMATOR 0
#endif
#endif

#define AMS_BENCH_PASSIVE_RING_TEMP_C 25.0f
#define AMS_BENCH_PASSIVE_RING_CURRENT_UNCERTAINTY_A 0.25f

#define AMS_ESTIMATOR_TOPOLOGY_PACK     1
#define AMS_ESTIMATOR_TOPOLOGY_SEGMENTS 2

/* Cell-voltage measurement authority. The normal mode requires the independent
 * C-ADC/S-ADC comparison. C_ONLY_MVP is an explicit, compile-time degraded
 * mode for controlled bring-up while the known SMB S-input routing defect is
 * being repaired. It is never selected automatically after an S-channel fault. */
#define AMS_VOLTAGE_MODE_REDUNDANT_CS 0
#define AMS_VOLTAGE_MODE_C_ONLY_MVP   1

#ifndef AMS_VOLTAGE_MODE
#if (AMS_BUILD_PROFILE == AMS_PROFILE_TESTDAY) || \
    (AMS_BUILD_PROFILE == AMS_PROFILE_BENCH_VALIDATION)
#define AMS_VOLTAGE_MODE AMS_VOLTAGE_MODE_C_ONLY_MVP
#else
#define AMS_VOLTAGE_MODE AMS_VOLTAGE_MODE_REDUNDANT_CS
#endif
#endif

#ifndef AMS_C_ONLY_MVP_RELEASE_REVIEWED
#define AMS_C_ONLY_MVP_RELEASE_REVIEWED 0
#endif

/* The ADBMS6830 discharge timeout is intentionally disabled in every current
 * profile.  Passive balancing uses the explicit DCC/PWM policy and does not
 * require DCTO.  A future design that intentionally uses the device timer must
 * opt in at build time and route writes through the dedicated timer reason;
 * ordinary init, balance, recovery and configuration-stress paths remain
 * forbidden from arming it. */
#ifndef AMS_ENABLE_ADBMS_DISCHARGE_TIMER
#define AMS_ENABLE_ADBMS_DISCHARGE_TIMER 0
#endif

/* ADBMS6830 transport/diagnostic hardening.  The awake-session optimization
 * is transport-only: raw cell authority and the existing three-scan persistent
 * read-fault qualification remain unchanged.  The session guard is deliberately
 * below the 4.3 ms minimum isoSPI idle timeout and is target-calibrated using
 * the timing metrics added with this release. */
#ifndef AMS_ENABLE_ADBMS_AWAKE_SESSION
#define AMS_ENABLE_ADBMS_AWAKE_SESSION 1
#endif
#ifndef AMS_ADBMS_SESSION_GUARD_US
#define AMS_ADBMS_SESSION_GUARD_US 3000u
#endif
#ifndef AMS_ADBMS_COOPERATIVE_WAIT_MIN_US
#define AMS_ADBMS_COOPERATIVE_WAIT_MIN_US 1000u
#endif

/* SPI6 is 108 MHz on the reviewed clock tree.  Keep /256 as the conservative
 * default until /128 or /64 is proven on the complete AMS->6822->6830 chain.
 * The code and CI support all three so the hardware A/B is a build-only change. */
#ifndef AMS_ADBMS_SPI_PRESCALER_DIV
#define AMS_ADBMS_SPI_PRESCALER_DIV 256
#endif

/* Enable the silicon measurement/diagnostic products without allowing them to
 * replace raw C-voltage safety authority.  FC=3 is the datasheet 21 Hz IIR. */
#ifndef AMS_ADBMS_IIR_FC
#define AMS_ADBMS_IIR_FC 3u
#endif
#ifndef AMS_ENABLE_ADBMS_AVG8_VOLTAGE
#define AMS_ENABLE_ADBMS_AVG8_VOLTAGE 1
#endif
#ifndef AMS_ENABLE_ADBMS_FILTERED_VOLTAGE
#define AMS_ENABLE_ADBMS_FILTERED_VOLTAGE 1
#endif
#ifndef AMS_ENABLE_ADBMS_STARTUP_POST
#define AMS_ENABLE_ADBMS_STARTUP_POST 1
#endif
#ifndef AMS_ENABLE_ADBMS_AUX2_REDUNDANCY
#if (AMS_BUILD_PROFILE == AMS_PROFILE_TESTDAY) || \
    (AMS_BUILD_PROFILE == AMS_PROFILE_BENCH_VALIDATION)
#define AMS_ENABLE_ADBMS_AUX2_REDUNDANCY 0
#else
#define AMS_ENABLE_ADBMS_AUX2_REDUNDANCY 1
#endif
#endif
#ifndef AMS_ENABLE_ADBMS_THERM_OPEN_WIRE_DIAG
#if (AMS_BUILD_PROFILE == AMS_PROFILE_TESTDAY) || \
    (AMS_BUILD_PROFILE == AMS_PROFILE_BENCH_VALIDATION)
#define AMS_ENABLE_ADBMS_THERM_OPEN_WIRE_DIAG 0
#else
#define AMS_ENABLE_ADBMS_THERM_OPEN_WIRE_DIAG 1
#endif
#endif
#ifndef AMS_ADBMS_AUX2_COMPARE_THRESHOLD_MV
#define AMS_ADBMS_AUX2_COMPARE_THRESHOLD_MV 20u
#endif
#ifndef AMS_ADBMS_THERM_OPEN_WIRE_PERIOD_MS
/* One mux position every 2 s gives all 24 positions diagnostic stimulus in
 * about 48 s without turning the intrusive open-wire check into scan-rate
 * traffic.  This remains observational until the real mux/NTC network is
 * characterized. */
#define AMS_ADBMS_THERM_OPEN_WIRE_PERIOD_MS 2000u
#endif
/* Initial observational thresholds for the muxed NTC open-wire stimulus.
 * They are deliberately not BMS_OK gates until hardware characterization
 * freezes the normal/open/series-resistance distributions. */
#ifndef AMS_ADBMS_THERM_OW_MIN_DELTA_MV
#define AMS_ADBMS_THERM_OW_MIN_DELTA_MV 10u
#endif
#ifndef AMS_ADBMS_THERM_OW_RECOVERY_TOL_MV
#define AMS_ADBMS_THERM_OW_RECOVERY_TOL_MV 20u
#endif
#ifndef AMS_ADBMS_AUX2_POSITIONS_PER_SECOND
#define AMS_ADBMS_AUX2_POSITIONS_PER_SECOND 4u
#endif

#define AMS_ESTIMATOR_VOLTAGE_SOURCE_RAW   0
#define AMS_ESTIMATOR_VOLTAGE_SOURCE_AVG8  1
#define AMS_ESTIMATOR_VOLTAGE_SOURCE_IIR   2
#ifndef AMS_ESTIMATOR_VOLTAGE_SOURCE
/* Research-selectable estimator input. Safety, OV/UV, freshness and SoP
 * electrical limits continue to use the raw C-voltage image. */
#define AMS_ESTIMATOR_VOLTAGE_SOURCE AMS_ESTIMATOR_VOLTAGE_SOURCE_RAW
#endif

/* The software implementation for ADI's continuous-C / periodic-S diagnostic
 * sequence is present, but current SMB hardware has the known S2N-S15N routing
 * defect.  It is impossible to enable this path in a release until the ECO is
 * explicitly acknowledged. */
#ifndef AMS_S_PATH_ECO_VALIDATED
#define AMS_S_PATH_ECO_VALIDATED 0
#endif
#ifndef AMS_ENABLE_PERIODIC_S_DIAGNOSTIC
#define AMS_ENABLE_PERIODIC_S_DIAGNOSTIC 0
#endif
#ifndef AMS_ADBMS_S_DIAGNOSTIC_PERIOD_MS
#define AMS_ADBMS_S_DIAGNOSTIC_PERIOD_MS 100u
#endif

#ifndef AMS_VEHICLE_PROFILE_REVIEWED
#define AMS_VEHICLE_PROFILE_REVIEWED 0
#endif
#ifndef AMS_IMD_TARGET_VALIDATED
#define AMS_IMD_TARGET_VALIDATED 0
#endif
#ifndef AMS_IWDG_TARGET_VALIDATED
#define AMS_IWDG_TARGET_VALIDATED 0
#endif
#ifndef AMS_CURRENT_POLARITY_VALIDATED
#define AMS_CURRENT_POLARITY_VALIDATED 0
#endif
#ifndef AMS_CURRENT_CALIBRATION_VALIDATED
#define AMS_CURRENT_CALIBRATION_VALIDATED 0
#endif
#ifndef AMS_BALANCE_TARGET_VALIDATED
#define AMS_BALANCE_TARGET_VALIDATED 0
#endif
#ifndef AMS_CAN_VEHICLE_CONTRACT_VALIDATED
#define AMS_CAN_VEHICLE_CONTRACT_VALIDATED 0
#endif
#ifndef AMS_SOP_MODEL_VALIDATED
#define AMS_SOP_MODEL_VALIDATED 0
#endif
#ifndef AMS_SOP_CALIBRATION_VALIDATED
#define AMS_SOP_CALIBRATION_VALIDATED 0
#endif
#ifndef AMS_SOP_CAN_CONTRACT_VALIDATED
#define AMS_SOP_CAN_CONTRACT_VALIDATED 0
#endif
#ifndef AMS_REGEN_TARGET_VALIDATED
#define AMS_REGEN_TARGET_VALIDATED 0
#endif
#ifndef AMS_MISSION_CAN_CONTRACT_VALIDATED
#define AMS_MISSION_CAN_CONTRACT_VALIDATED 0
#endif
#ifndef AMS_FUSE_MODEL_VALIDATED
#define AMS_FUSE_MODEL_VALIDATED 0
#endif
/* The installed EAC14-80 model currently relies on an explicit low-current
 * continuation below the lowest digitized manufacturer curve point (~154 A).
 * The normal vehicle DCL is below that point, so a vehicle release must
 * separately acknowledge measured/manufacturer evidence for that 80-154 A
 * region rather than silently treating "model validated" as sufficient. */
#ifndef AMS_FUSE_LOW_CURRENT_EXTRAPOLATION_VALIDATED
#define AMS_FUSE_LOW_CURRENT_EXTRAPOLATION_VALIDATED 0
#endif

/* The open-wire implementation can use the C path while the known S-input
 * routing defect is under repair. Automatic C-path testing is release-gated
 * because it temporarily repurposes the authoritative C conversion registers;
 * the accumulator wrapper must restore and revalidate a normal C image before
 * returning. */
#ifndef AMS_C_OPEN_WIRE_TARGET_VALIDATED
#define AMS_C_OPEN_WIRE_TARGET_VALIDATED 0
#endif
/* Current SMB Rev5 boards have unvalidated 100-ohm GPIO4/GPIO5 pull-ups.
 * Firmware timing changes cannot make that electrical load acceptable. */
#ifndef AMS_TEMP_PULLUPS_TARGET_VALIDATED
#define AMS_TEMP_PULLUPS_TARGET_VALIDATED 0
#endif
/* Temperature thresholds are safety-relevant policy, independent of whether
 * the mux/pull-up hardware works.  The current constants remain deliberately
 * release-gated until they are checked against the installed cells, cooling
 * system and applicable competition requirements. */
#ifndef AMS_TEMP_THRESHOLDS_VALIDATED
#define AMS_TEMP_THRESHOLDS_VALIDATED 0
#endif

/* These observers are diagnostic only until their hardware inputs and
 * calibration are separately validated. */
#ifndef AMS_PARALLEL_CONNECTION_OBSERVER_VALIDATED
#define AMS_PARALLEL_CONNECTION_OBSERVER_VALIDATED 0
#endif
#ifndef AMS_MAIN_FUSE_PLAUSIBILITY_VALIDATED
#define AMS_MAIN_FUSE_PLAUSIBILITY_VALIDATED 0
#endif
#ifndef AMS_MAIN_FUSE_LOAD_VOLTAGE_TARGET_VALIDATED
#define AMS_MAIN_FUSE_LOAD_VOLTAGE_TARGET_VALIDATED 0
#endif
/* Defined here as well as in app.h because standalone driver/profile test
 * translation units include this file without including the application
 * aggregate header. */
#ifndef AMS_ENABLE_AIR_AUX_FEEDBACK
#define AMS_ENABLE_AIR_AUX_FEEDBACK 0
#endif

#if AMS_BUILD_PROFILE == AMS_PROFILE_BENCH

#define AMS_BUILD_PROFILE_NAME "bench"
#ifndef AMS_HW_BRINGUP
#define AMS_HW_BRINGUP 1
#endif
#ifndef AMS_HIL_REPLACE_ADBMS
#define AMS_HIL_REPLACE_ADBMS 0
#endif
#ifndef AMS_ENABLE_HIL_CAN
#define AMS_ENABLE_HIL_CAN 0
#endif
#ifndef AMS_ENABLE_MISSION_CAN
#define AMS_ENABLE_MISSION_CAN 0
#endif
#ifndef AMS_ENABLE_SERVICE_CLI
#define AMS_ENABLE_SERVICE_CLI 1
#endif
#ifndef AMS_ENABLE_IMD
#define AMS_ENABLE_IMD 0
#endif
#ifndef AMS_ENABLE_IWDG
#define AMS_ENABLE_IWDG 0
#endif
#ifndef AMS_ENABLE_AUTO_TEMP_MUX_SCAN
#define AMS_ENABLE_AUTO_TEMP_MUX_SCAN 0
#endif
#ifndef AMS_ENABLE_AUTO_C_OPEN_WIRE
#define AMS_ENABLE_AUTO_C_OPEN_WIRE 0
#endif
#ifndef AMS_ENABLE_ADBMS_FAULT_INJECTION
#define AMS_ENABLE_ADBMS_FAULT_INJECTION 1
#endif
#define AMS_PROFILE_BMS_OUTPUT_INHIBIT_DEFAULT 1
#define AMS_PROFILE_BALANCE_INHIBIT_DEFAULT 1
#define AMS_SOP_AUTHORITY_REQUIRED 0

#elif AMS_BUILD_PROFILE == AMS_PROFILE_BENCH_VALIDATION

/* Hardware-validation bench image. This deliberately runs the production-rate
 * physical measurement/CAN paths needed to validate the assembled system, but
 * BMS_OK and balancing are compile-time locked off. IMD is enabled for LV
 * bench verification; intrusive automatic open-wire/thermistor diagnostics and
 * fault injection remain disabled. */
#define AMS_BUILD_PROFILE_NAME "bench_validation"
#ifndef AMS_HW_BRINGUP
#define AMS_HW_BRINGUP 0
#endif
#ifndef AMS_HIL_REPLACE_ADBMS
#define AMS_HIL_REPLACE_ADBMS 0
#endif
#ifndef AMS_ENABLE_HIL_CAN
#define AMS_ENABLE_HIL_CAN 0
#endif
#ifndef AMS_ENABLE_MISSION_CAN
#define AMS_ENABLE_MISSION_CAN 0
#endif
#ifndef AMS_ENABLE_SERVICE_CLI
#define AMS_ENABLE_SERVICE_CLI 1
#endif
#ifndef AMS_ENABLE_IMD
#define AMS_ENABLE_IMD 1
#endif
#ifndef AMS_ENABLE_IWDG
#define AMS_ENABLE_IWDG 0
#endif
#ifndef AMS_ENABLE_AUTO_TEMP_MUX_SCAN
#define AMS_ENABLE_AUTO_TEMP_MUX_SCAN 0
#endif
#ifndef AMS_ENABLE_AUTO_C_OPEN_WIRE
#define AMS_ENABLE_AUTO_C_OPEN_WIRE 0
#endif
#ifndef AMS_ENABLE_ADBMS_FAULT_INJECTION
#define AMS_ENABLE_ADBMS_FAULT_INJECTION 0
#endif
#define AMS_PROFILE_BMS_OUTPUT_INHIBIT_DEFAULT 1
#define AMS_PROFILE_BALANCE_INHIBIT_DEFAULT 1
#define AMS_SOP_AUTHORITY_REQUIRED 0

#elif AMS_BUILD_PROFILE == AMS_PROFILE_TESTDAY

/* Five-segment acquisition/tuning image for controlled track/bench testing.
 * All real measurement paths execute at their intended rate, but physical
 * BMS permission and balancing remain source-owned inhibited. */
#define AMS_BUILD_PROFILE_NAME "testday"
#ifndef AMS_HW_BRINGUP
#define AMS_HW_BRINGUP 0
#endif
#ifndef AMS_HIL_REPLACE_ADBMS
#define AMS_HIL_REPLACE_ADBMS 0
#endif
#ifndef AMS_ENABLE_HIL_CAN
#define AMS_ENABLE_HIL_CAN 0
#endif
#ifndef AMS_ENABLE_MISSION_CAN
#define AMS_ENABLE_MISSION_CAN 0
#endif
#ifndef AMS_ENABLE_SERVICE_CLI
#define AMS_ENABLE_SERVICE_CLI 1
#endif
#ifndef AMS_ENABLE_IMD
#define AMS_ENABLE_IMD 1
#endif
#ifndef AMS_ENABLE_IWDG
#define AMS_ENABLE_IWDG 0
#endif
#ifndef AMS_ENABLE_AUTO_TEMP_MUX_SCAN
#define AMS_ENABLE_AUTO_TEMP_MUX_SCAN 1
#endif
#ifndef AMS_ENABLE_AUTO_C_OPEN_WIRE
#define AMS_ENABLE_AUTO_C_OPEN_WIRE 0
#endif
#ifndef AMS_ENABLE_ADBMS_FAULT_INJECTION
#define AMS_ENABLE_ADBMS_FAULT_INJECTION 0
#endif
#define AMS_PROFILE_BMS_OUTPUT_INHIBIT_DEFAULT 1
#define AMS_PROFILE_BALANCE_INHIBIT_DEFAULT 1
#define AMS_SOP_AUTHORITY_REQUIRED 0

#elif AMS_BUILD_PROFILE == AMS_PROFILE_HIL

#define AMS_BUILD_PROFILE_NAME "hil"
#ifndef AMS_HW_BRINGUP
#define AMS_HW_BRINGUP 0
#endif
#ifndef AMS_HIL_REPLACE_ADBMS
#define AMS_HIL_REPLACE_ADBMS 1
#endif
#ifndef AMS_ENABLE_HIL_CAN
#define AMS_ENABLE_HIL_CAN 1
#endif
#ifndef AMS_ENABLE_MISSION_CAN
#define AMS_ENABLE_MISSION_CAN 0
#endif
#ifndef AMS_ENABLE_SERVICE_CLI
#define AMS_ENABLE_SERVICE_CLI 1
#endif
#ifndef AMS_ENABLE_IMD
#define AMS_ENABLE_IMD 0
#endif
#ifndef AMS_ENABLE_IWDG
#define AMS_ENABLE_IWDG 0
#endif
#ifndef AMS_ENABLE_AUTO_TEMP_MUX_SCAN
#define AMS_ENABLE_AUTO_TEMP_MUX_SCAN 0
#endif
#ifndef AMS_ENABLE_AUTO_C_OPEN_WIRE
#define AMS_ENABLE_AUTO_C_OPEN_WIRE 0
#endif
#ifndef AMS_ENABLE_ADBMS_FAULT_INJECTION
#define AMS_ENABLE_ADBMS_FAULT_INJECTION 0
#endif
#define AMS_PROFILE_BMS_OUTPUT_INHIBIT_DEFAULT 1
#define AMS_PROFILE_BALANCE_INHIBIT_DEFAULT 1
#define AMS_SOP_AUTHORITY_REQUIRED 0

#elif AMS_BUILD_PROFILE == AMS_PROFILE_VEHICLE

#define AMS_BUILD_PROFILE_NAME "vehicle"
#ifndef AMS_HW_BRINGUP
#define AMS_HW_BRINGUP 0
#endif
#ifndef AMS_HIL_REPLACE_ADBMS
#define AMS_HIL_REPLACE_ADBMS 0
#endif
#ifndef AMS_ENABLE_HIL_CAN
#define AMS_ENABLE_HIL_CAN 0
#endif
#ifndef AMS_ENABLE_MISSION_CAN
#define AMS_ENABLE_MISSION_CAN 1
#endif
#ifndef AMS_ENABLE_SERVICE_CLI
#define AMS_ENABLE_SERVICE_CLI 0
#endif
#ifndef AMS_ENABLE_IMD
#define AMS_ENABLE_IMD 1
#endif
#ifndef AMS_ENABLE_IWDG
#define AMS_ENABLE_IWDG 1
#endif
#ifndef AMS_ENABLE_AUTO_TEMP_MUX_SCAN
#define AMS_ENABLE_AUTO_TEMP_MUX_SCAN 1
#endif
#ifndef AMS_ENABLE_AUTO_C_OPEN_WIRE
#define AMS_ENABLE_AUTO_C_OPEN_WIRE 1
#endif
#ifndef AMS_ENABLE_ADBMS_FAULT_INJECTION
#define AMS_ENABLE_ADBMS_FAULT_INJECTION 0
#endif
#define AMS_PROFILE_BMS_OUTPUT_INHIBIT_DEFAULT 0
#define AMS_PROFILE_BALANCE_INHIBIT_DEFAULT 0
#define AMS_SOP_AUTHORITY_REQUIRED 1

#if !AMS_VEHICLE_PROFILE_REVIEWED
#error "Vehicle profile requires AMS_VEHICLE_PROFILE_REVIEWED=1"
#endif
#if !AMS_IMD_TARGET_VALIDATED
#error "Vehicle profile requires physically validated IMD input"
#endif
#if !AMS_IWDG_TARGET_VALIDATED
#error "Vehicle profile requires target watchdog reset validation"
#endif
#if !AMS_CURRENT_POLARITY_VALIDATED
#error "Vehicle profile requires signed DHAB discharge/charge validation"
#endif
#if !AMS_CURRENT_CALIBRATION_VALIDATED
#error "Vehicle profile requires a validated DHAB calibration and uncertainty procedure"
#endif
#if !AMS_BALANCE_TARGET_VALIDATED
#error "Vehicle profile requires measured balance current/on-time/thermal validation"
#endif
#if !AMS_CAN_VEHICLE_CONTRACT_VALIDATED
#error "Vehicle profile requires reviewed whole-vehicle CAN timing and stale-data policy"
#endif
#if !AMS_SOP_MODEL_VALIDATED
#error "Vehicle profile requires target-validated SoP electrothermal model and WCET evidence"
#endif
#if !AMS_SOP_CALIBRATION_VALIDATED
#error "Vehicle profile requires reviewed SoP uncertainty and hardware-limit calibration"
#endif
#if !AMS_SOP_CAN_CONTRACT_VALIDATED
#error "Vehicle profile requires ECU-tested SoP/SoH CAN freshness and fail-zero contract"
#endif
#if !AMS_MISSION_CAN_CONTRACT_VALIDATED
#error "Vehicle profile requires validated ECU-to-AMS mission request CAN contract"
#endif
#if !AMS_FUSE_MODEL_VALIDATED
#error "Vehicle profile requires installed EAC14-80 thermal-model calibration"
#endif
#if !AMS_FUSE_LOW_CURRENT_EXTRAPOLATION_VALIDATED
#error "Vehicle profile requires evidence for EAC14-80 low-current curve extrapolation"
#endif
#if AMS_ENABLE_AUTO_C_OPEN_WIRE && !AMS_C_OPEN_WIRE_TARGET_VALIDATED
#error "Vehicle automatic C-path open-wire requires target validation and restore-scan evidence"
#endif
#if AMS_ENABLE_AUTO_TEMP_MUX_SCAN && !AMS_TEMP_PULLUPS_TARGET_VALIDATED
#error "Vehicle automatic temperature scan requires corrected and validated SMB GPIO pull-ups"
#endif
#if !AMS_TEMP_THRESHOLDS_VALIDATED
#error "Vehicle profile requires reviewed accumulator temperature thresholds"
#endif
#ifndef AMS_BUILD_GIT_COMMIT
#error "Vehicle profile requires an explicit immutable source commit in the build manifest"
#endif
#ifndef AMS_CURRENT_CALIBRATION_REVISION
#error "Vehicle profile requires an explicit current-calibration revision"
#endif
#ifndef AMS_CAN_CONTRACT_REVISION
#error "Vehicle profile requires an explicit CAN contract revision"
#endif
#ifndef AMS_THRESHOLD_REVISION
#error "Vehicle profile requires an explicit threshold revision"
#endif
#ifndef AMS_ESTIMATOR_MODEL_REVISION
#error "Vehicle profile requires an explicit estimator-model revision"
#endif
#ifndef AMS_SOP_MODEL_REVISION
#error "Vehicle profile requires an explicit SoP model revision"
#endif
#ifndef AMS_SOH_MODEL_REVISION
#error "Vehicle profile requires an explicit SoH model revision"
#endif
#if AMS_HW_BRINGUP || AMS_HIL_REPLACE_ADBMS || AMS_ENABLE_HIL_CAN || \
    AMS_ENABLE_SERVICE_CLI
#error "Vehicle profile forbids bring-up, HIL injection and service mutation"
#endif
#if !AMS_ENABLE_IMD || !AMS_ENABLE_IWDG || !AMS_ENABLE_MISSION_CAN
#error "Vehicle profile requires IMD, IWDG and supervised mission CAN"
#endif

#else
#error "AMS_BUILD_PROFILE must be BENCH, HIL, TESTDAY, BENCH_VALIDATION or VEHICLE"
#endif

/* DER26-CAN-V4 publication policy. Safety/status/power authority remains at
 * 10 Hz. The complete passive cell/temp/detail snapshot is generated at 2 Hz
 * and serialized asynchronously by the bxCAN TX-complete service. Measurement,
 * protection and estimator execution rates are unchanged. */
#ifndef AMS_CAN_DETAIL_FULL_PERIOD_MS
#define AMS_CAN_DETAIL_FULL_PERIOD_MS 500u
#endif
#ifndef AMS_CAN_PROTECTED_PERIOD_MS
#define AMS_CAN_PROTECTED_PERIOD_MS 100u
#endif
#ifndef AMS_ENABLE_LEGACY_CAN_TELEMETRY
#define AMS_ENABLE_LEGACY_CAN_TELEMETRY 0
#endif

#if AMS_CAN_DETAIL_FULL_PERIOD_MS < 100u
#error "AMS CAN detail period must be >=100 ms"
#endif
#if AMS_CAN_PROTECTED_PERIOD_MS != 100u
#error "DER26-CAN-V4 protected publication contract is 100 ms / 10 Hz"
#endif
#if (AMS_ENABLE_LEGACY_CAN_TELEMETRY != 0) && (AMS_ENABLE_LEGACY_CAN_TELEMETRY != 1)
#error "AMS_ENABLE_LEGACY_CAN_TELEMETRY must be 0 or 1"
#endif
#if (AMS_BUILD_PROFILE == AMS_PROFILE_VEHICLE) && AMS_ENABLE_LEGACY_CAN_TELEMETRY
#error "DER26-CAN-V4 vehicle build forbids legacy bulk telemetry compatibility frames"
#endif
/* Compatibility alias for older host tests/tools. It is no longer used by the
 * production TX scheduler and must not be used to pace physical transmission. */
#ifndef AMS_CAN_DETAIL_DIVIDER
#define AMS_CAN_DETAIL_DIVIDER 5u
#endif


#if (AMS_ENABLE_ADBMS_AWAKE_SESSION != 0) && (AMS_ENABLE_ADBMS_AWAKE_SESSION != 1)
#error "AMS_ENABLE_ADBMS_AWAKE_SESSION must be 0 or 1"
#endif
#if (AMS_ADBMS_SESSION_GUARD_US >= 4300u)
#error "ADBMS session guard must remain below the 4.3 ms minimum isoSPI idle timeout"
#endif
#if (AMS_ADBMS_SPI_PRESCALER_DIV != 256) && \
    (AMS_ADBMS_SPI_PRESCALER_DIV != 128) && \
    (AMS_ADBMS_SPI_PRESCALER_DIV != 64)
#error "ADBMS SPI prescaler experiment supports only /256, /128, or /64"
#endif
#if (AMS_ADBMS_IIR_FC > 7u)
#error "ADBMS6830 IIR FC field is 3 bits"
#endif
#if (AMS_ADBMS_AUX2_POSITIONS_PER_SECOND == 0u) || \
    (AMS_ADBMS_AUX2_POSITIONS_PER_SECOND > 1000u)
#error "AUX2 diagnostic cadence must be 1..1000 positions/second"
#endif
#if (AMS_ESTIMATOR_VOLTAGE_SOURCE != AMS_ESTIMATOR_VOLTAGE_SOURCE_RAW) && \
    (AMS_ESTIMATOR_VOLTAGE_SOURCE != AMS_ESTIMATOR_VOLTAGE_SOURCE_AVG8) && \
    (AMS_ESTIMATOR_VOLTAGE_SOURCE != AMS_ESTIMATOR_VOLTAGE_SOURCE_IIR)
#error "AMS_ESTIMATOR_VOLTAGE_SOURCE must be RAW, AVG8, or IIR"
#endif
#if AMS_ENABLE_PERIODIC_S_DIAGNOSTIC && !AMS_S_PATH_ECO_VALIDATED
#error "Periodic S diagnostic requires AMS_S_PATH_ECO_VALIDATED=1"
#endif
#if (AMS_ENABLE_ADBMS_AVG8_VOLTAGE != 0) && (AMS_ENABLE_ADBMS_AVG8_VOLTAGE != 1)
#error "AMS_ENABLE_ADBMS_AVG8_VOLTAGE must be 0 or 1"
#endif
#if (AMS_ENABLE_ADBMS_FILTERED_VOLTAGE != 0) && (AMS_ENABLE_ADBMS_FILTERED_VOLTAGE != 1)
#error "AMS_ENABLE_ADBMS_FILTERED_VOLTAGE must be 0 or 1"
#endif
#if (AMS_ENABLE_ADBMS_AUX2_REDUNDANCY != 0) && (AMS_ENABLE_ADBMS_AUX2_REDUNDANCY != 1)
#error "AMS_ENABLE_ADBMS_AUX2_REDUNDANCY must be 0 or 1"
#endif

#if (AMS_ENABLE_AUTO_TEMP_MUX_SCAN != 0) && (AMS_ENABLE_AUTO_TEMP_MUX_SCAN != 1)
#error "AMS_ENABLE_AUTO_TEMP_MUX_SCAN must be 0 or 1"
#endif

#if (AMS_ENABLE_BENCH_PASSIVE_RING_ESTIMATOR != 0) && \
    (AMS_ENABLE_BENCH_PASSIVE_RING_ESTIMATOR != 1)
#error "AMS_ENABLE_BENCH_PASSIVE_RING_ESTIMATOR must be 0 or 1"
#endif

#if AMS_ENABLE_BENCH_PASSIVE_RING_ESTIMATOR && \
    ((AMS_BUILD_PROFILE != AMS_PROFILE_BENCH_VALIDATION) || \
     AMS_BENCH_VALIDATION_SINGLE_SMB)
#error "Passive ring estimator is restricted to the five-SMB BENCH_VALIDATION profile"
#endif

#if (AMS_ENABLE_ADBMS_FAULT_INJECTION != 0) && \
    (AMS_ENABLE_ADBMS_FAULT_INJECTION != 1)
#error "AMS_ENABLE_ADBMS_FAULT_INJECTION must be 0 or 1"
#endif

#if (AMS_ENABLE_ADBMS_DISCHARGE_TIMER != 0) && \
    (AMS_ENABLE_ADBMS_DISCHARGE_TIMER != 1)
#error "AMS_ENABLE_ADBMS_DISCHARGE_TIMER must be 0 or 1"
#endif

#if (AMS_ENABLE_AUTO_C_OPEN_WIRE != 0) && (AMS_ENABLE_AUTO_C_OPEN_WIRE != 1)
#error "AMS_ENABLE_AUTO_C_OPEN_WIRE must be 0 or 1"
#endif

#if (AMS_TEMP_PULLUPS_TARGET_VALIDATED != 0) && \
    (AMS_TEMP_PULLUPS_TARGET_VALIDATED != 1)
#error "AMS_TEMP_PULLUPS_TARGET_VALIDATED must be 0 or 1"
#endif

#if (AMS_TEMP_THRESHOLDS_VALIDATED != 0) && \
    (AMS_TEMP_THRESHOLDS_VALIDATED != 1)
#error "AMS_TEMP_THRESHOLDS_VALIDATED must be 0 or 1"
#endif

#if (AMS_C_OPEN_WIRE_TARGET_VALIDATED != 0) && \
    (AMS_C_OPEN_WIRE_TARGET_VALIDATED != 1)
#error "AMS_C_OPEN_WIRE_TARGET_VALIDATED must be 0 or 1"
#endif

#if (AMS_PARALLEL_CONNECTION_OBSERVER_VALIDATED != 0) && \
    (AMS_PARALLEL_CONNECTION_OBSERVER_VALIDATED != 1)
#error "AMS_PARALLEL_CONNECTION_OBSERVER_VALIDATED must be 0 or 1"
#endif

#if (AMS_MAIN_FUSE_PLAUSIBILITY_VALIDATED != 0) && \
    (AMS_MAIN_FUSE_PLAUSIBILITY_VALIDATED != 1)
#error "AMS_MAIN_FUSE_PLAUSIBILITY_VALIDATED must be 0 or 1"
#endif

#if (AMS_MAIN_FUSE_LOAD_VOLTAGE_TARGET_VALIDATED != 0) && \
    (AMS_MAIN_FUSE_LOAD_VOLTAGE_TARGET_VALIDATED != 1)
#error "AMS_MAIN_FUSE_LOAD_VOLTAGE_TARGET_VALIDATED must be 0 or 1"
#endif

#if AMS_PARALLEL_CONNECTION_OBSERVER_VALIDATED && \
    (!AMS_CURRENT_POLARITY_VALIDATED || !AMS_CURRENT_CALIBRATION_VALIDATED)
#error "Parallel-connection observer validation requires validated current polarity and calibration"
#endif

#if AMS_MAIN_FUSE_PLAUSIBILITY_VALIDATED && \
    (!AMS_ENABLE_AIR_AUX_FEEDBACK || !AMS_CURRENT_CALIBRATION_VALIDATED || \
     !AMS_MAIN_FUSE_LOAD_VOLTAGE_TARGET_VALIDATED)
#error "Main-fuse plausibility validation requires authoritative AIR feedback, calibrated current and independent load-side voltage"
#endif

#if AMS_ENABLE_ADBMS_FAULT_INJECTION && !AMS_ENABLE_SERVICE_CLI
#error "ADBMS fault injection requires the service CLI"
#endif

#if (AMS_VOLTAGE_MODE != AMS_VOLTAGE_MODE_REDUNDANT_CS) && \
    (AMS_VOLTAGE_MODE != AMS_VOLTAGE_MODE_C_ONLY_MVP)
#error "AMS_VOLTAGE_MODE must be REDUNDANT_CS or C_ONLY_MVP"
#endif

#if (AMS_BUILD_PROFILE == AMS_PROFILE_VEHICLE) && \
    (AMS_VOLTAGE_MODE == AMS_VOLTAGE_MODE_C_ONLY_MVP) && \
    !AMS_C_ONLY_MVP_RELEASE_REVIEWED
#error "Vehicle C-only voltage mode requires AMS_C_ONLY_MVP_RELEASE_REVIEWED=1"
#endif

#ifndef AMS_ESTIMATOR_DEFAULT_TOPOLOGY
#if (AMS_BUILD_PROFILE == AMS_PROFILE_VEHICLE) || \
    (AMS_BUILD_PROFILE == AMS_PROFILE_TESTDAY) || \
    ((AMS_BUILD_PROFILE == AMS_PROFILE_BENCH_VALIDATION) && !AMS_BENCH_VALIDATION_SINGLE_SMB)
#define AMS_ESTIMATOR_DEFAULT_TOPOLOGY AMS_ESTIMATOR_TOPOLOGY_SEGMENTS
#else
#define AMS_ESTIMATOR_DEFAULT_TOPOLOGY AMS_ESTIMATOR_TOPOLOGY_PACK
#endif
#endif

#if (AMS_ESTIMATOR_DEFAULT_TOPOLOGY != AMS_ESTIMATOR_TOPOLOGY_PACK) && \
    (AMS_ESTIMATOR_DEFAULT_TOPOLOGY != AMS_ESTIMATOR_TOPOLOGY_SEGMENTS)
#error "AMS_ESTIMATOR_DEFAULT_TOPOLOGY must be PACK or SEGMENTS"
#endif

/* The current HIL measurement frame carries one pack voltage and one pack
 * temperature.  It cannot truthfully drive five local segment estimators. */
#if (AMS_BUILD_PROFILE == AMS_PROFILE_HIL) && \
    (AMS_ESTIMATOR_DEFAULT_TOPOLOGY != AMS_ESTIMATOR_TOPOLOGY_PACK)
#error "Current HIL CAN contract supports pack estimator topology only"
#endif

#if (AMS_BUILD_PROFILE == AMS_PROFILE_HIL) && \
    (!AMS_ENABLE_HIL_CAN || !AMS_HIL_REPLACE_ADBMS)
#error "HIL profile requires CAN injection and ADBMS replacement"
#endif

#ifndef AMS_BUILD_GIT_COMMIT
#define AMS_BUILD_GIT_COMMIT AMS_SOURCE_REVISION
#endif
#ifndef AMS_CURRENT_CALIBRATION_REVISION
#define AMS_CURRENT_CALIBRATION_REVISION "DHAB-unvalidated-v0"
#endif
#ifndef AMS_CAN_CONTRACT_REVISION
#define AMS_CAN_CONTRACT_REVISION "DER26-CAN-V4-1M-TXSCHED1-LOGGER3-POWER2-CURRENT2"
#endif
#ifndef AMS_THRESHOLD_REVISION
#define AMS_THRESHOLD_REVISION "DER26-SOP-system-v2"
#endif
#ifndef AMS_ESTIMATOR_MODEL_REVISION
#define AMS_ESTIMATOR_MODEL_REVISION "P42A-HPPC-v1"
#endif
#ifndef AMS_SOP_MODEL_REVISION
#define AMS_SOP_MODEL_REVISION "P42A-robust-FHPE-v3-strategy"
#endif
#ifndef AMS_SOH_MODEL_REVISION
#define AMS_SOH_MODEL_REVISION "rest-anchor-capacity-R0-v3"
#endif

#endif /* INC_AMS_BUILD_PROFILE_H_ */
