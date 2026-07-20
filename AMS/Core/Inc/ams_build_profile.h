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

#define AMS_PROFILE_BENCH   1
#define AMS_PROFILE_HIL     2
#define AMS_PROFILE_VEHICLE 3

#define AMS_ESTIMATOR_TOPOLOGY_PACK     1
#define AMS_ESTIMATOR_TOPOLOGY_SEGMENTS 2

#ifndef AMS_BUILD_PROFILE
#define AMS_BUILD_PROFILE AMS_PROFILE_BENCH
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
#ifndef AMS_ENABLE_SERVICE_CLI
#define AMS_ENABLE_SERVICE_CLI 1
#endif
#ifndef AMS_ENABLE_IMD
#define AMS_ENABLE_IMD 0
#endif
#ifndef AMS_ENABLE_IWDG
#define AMS_ENABLE_IWDG 0
#endif
#define AMS_PROFILE_BMS_OUTPUT_INHIBIT_DEFAULT 1
#define AMS_PROFILE_BALANCE_INHIBIT_DEFAULT 1

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
#ifndef AMS_ENABLE_SERVICE_CLI
#define AMS_ENABLE_SERVICE_CLI 1
#endif
#ifndef AMS_ENABLE_IMD
#define AMS_ENABLE_IMD 0
#endif
#ifndef AMS_ENABLE_IWDG
#define AMS_ENABLE_IWDG 0
#endif
#define AMS_PROFILE_BMS_OUTPUT_INHIBIT_DEFAULT 1
#define AMS_PROFILE_BALANCE_INHIBIT_DEFAULT 1

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
#ifndef AMS_ENABLE_SERVICE_CLI
#define AMS_ENABLE_SERVICE_CLI 0
#endif
#ifndef AMS_ENABLE_IMD
#define AMS_ENABLE_IMD 1
#endif
#ifndef AMS_ENABLE_IWDG
#define AMS_ENABLE_IWDG 1
#endif
#define AMS_PROFILE_BMS_OUTPUT_INHIBIT_DEFAULT 0
#define AMS_PROFILE_BALANCE_INHIBIT_DEFAULT 0

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
#if AMS_HW_BRINGUP || AMS_HIL_REPLACE_ADBMS || AMS_ENABLE_HIL_CAN || \
    AMS_ENABLE_SERVICE_CLI
#error "Vehicle profile forbids bring-up, HIL injection and service mutation"
#endif
#if !AMS_ENABLE_IMD || !AMS_ENABLE_IWDG
#error "Vehicle profile requires IMD and IWDG"
#endif

#else
#error "AMS_BUILD_PROFILE must be AMS_PROFILE_BENCH, AMS_PROFILE_HIL or AMS_PROFILE_VEHICLE"
#endif

#ifndef AMS_ESTIMATOR_DEFAULT_TOPOLOGY
#define AMS_ESTIMATOR_DEFAULT_TOPOLOGY AMS_ESTIMATOR_TOPOLOGY_PACK
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
#define AMS_BUILD_GIT_COMMIT "unknown"
#endif
#ifndef AMS_CURRENT_CALIBRATION_REVISION
#define AMS_CURRENT_CALIBRATION_REVISION "DHAB-unvalidated-v0"
#endif
#ifndef AMS_CAN_CONTRACT_REVISION
#define AMS_CAN_CONTRACT_REVISION "ECU1-LOGGER1-PHASED1"
#endif
#ifndef AMS_THRESHOLD_REVISION
#define AMS_THRESHOLD_REVISION "DER26-prevehicle-v1"
#endif
#ifndef AMS_ESTIMATOR_MODEL_REVISION
#define AMS_ESTIMATOR_MODEL_REVISION "P42A-HPPC-v1"
#endif

#endif /* INC_AMS_BUILD_PROFILE_H_ */
