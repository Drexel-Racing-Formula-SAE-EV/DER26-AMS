/*
 * ams_build_profile.h
 *
 * Central build-profile selection for the DER26 AMS firmware.
 *
 * This source package is intentionally branch-ready for low-energy bring-up
 * of one Analog Devices EVAL-ADBMS6830BMSW board through the AMS ADBMS6822
 * isoSPI master.  The eval profile defaults ON in this package.  Production
 * and regression builds explicitly compile with
 * -DAMS_EVAL_ADBMS6830_BMSW=0 and retain the original five-SMB topology.
 */

#ifndef INC_AMS_BUILD_PROFILE_H_
#define INC_AMS_BUILD_PROFILE_H_

#ifndef AMS_EVAL_ADBMS6830_BMSW
#define AMS_EVAL_ADBMS6830_BMSW 1
#endif

#if AMS_EVAL_ADBMS6830_BMSW

/* The eval-board branch is a controlled bench build. */
#ifndef AMS_HW_BRINGUP
#define AMS_HW_BRINGUP 1
#endif

#ifndef AMS_HW_BRINGUP_BMS_OK_RELEASED_DEFAULT
#define AMS_HW_BRINGUP_BMS_OK_RELEASED_DEFAULT 0
#endif

#ifndef AMS_HW_BRINGUP_BALANCE_INHIBIT_DEFAULT
#define AMS_HW_BRINGUP_BALANCE_INHIBIT_DEFAULT 1
#endif

#ifndef AMS_ENABLE_SERVICE_CLI
#define AMS_ENABLE_SERVICE_CLI 1
#endif

/* One EVAL-ADBMS6830BMSW has one ADBMS6830B and sixteen cell channels. */
#define AMS_ADBMS_MONITOR_COUNT              1
#define AMS_ADBMS_CELL_COUNT                16

/* Monitor-only bench policy.  SRST, wake, ADC-conversion and read commands
 * remain available; configuration writes and all actuating diagnostics do
 * not. */
#define AMS_ADBMS_FULL_CONFIG_ON_INIT        0
#define AMS_ADBMS_TEMP_MUX_ENABLED           0
#define AMS_ADBMS_BALANCE_WRITES_ENABLED     0
#define AMS_ADBMS_OPEN_WIRE_ENABLED          0
#define AMS_ADBMS_RESTRICTED_BENCH_COMMANDS  1
#define AMS_EVAL_FAN_OUTPUTS_ENABLED         0
#define AMS_EVAL_ESTIMATOR_ENABLED           0

/* Reset-default REFON=0 adds up to 4.4 ms of reference start-up before the
 * 8 ms redundant cell conversion.  Use a conservative fixed wait because
 * continuous ADC polling never reports a terminal "done" state. */
#define AMS_ADBMS_CELL_CONVERSION_WAIT_US     15000u

#define AMS_BUILD_PROFILE_NAME "eval-6830bmsw"

#else

#define AMS_ADBMS_MONITOR_COUNT              5
#define AMS_ADBMS_CELL_COUNT                15
#define AMS_ADBMS_FULL_CONFIG_ON_INIT        1
#define AMS_ADBMS_TEMP_MUX_ENABLED           1
#define AMS_ADBMS_BALANCE_WRITES_ENABLED     1
#define AMS_ADBMS_OPEN_WIRE_ENABLED          1
#define AMS_ADBMS_RESTRICTED_BENCH_COMMANDS  0
#define AMS_EVAL_FAN_OUTPUTS_ENABLED         1
#define AMS_EVAL_ESTIMATOR_ENABLED           1

/* Full configuration keeps the reference powered; retain the complete
 * 8 ms C-ADC/S-ADC redundant-conversion interval. */
#define AMS_ADBMS_CELL_CONVERSION_WAIT_US     8000u

#define AMS_BUILD_PROFILE_NAME "der26-five-smb"

#endif

/* Refuse unsafe or internally contradictory eval builds instead of silently
 * accepting command-line overrides. */
#if AMS_EVAL_ADBMS6830_BMSW && !AMS_HW_BRINGUP
#error "EVAL-ADBMS6830BMSW profile requires AMS_HW_BRINGUP=1"
#endif

#if AMS_EVAL_ADBMS6830_BMSW && AMS_HW_BRINGUP_BMS_OK_RELEASED_DEFAULT
#error "EVAL-ADBMS6830BMSW profile permanently requires BMS_OK inhibited"
#endif

#if AMS_EVAL_ADBMS6830_BMSW && !AMS_HW_BRINGUP_BALANCE_INHIBIT_DEFAULT
#error "EVAL-ADBMS6830BMSW profile permanently requires balancing inhibited"
#endif

#endif /* INC_AMS_BUILD_PROFILE_H_ */
