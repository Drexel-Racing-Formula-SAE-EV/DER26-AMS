/* Vehicle-bus source variant. TESTDAY is inhibited; VEHICLE requires evidence. */
#ifndef AMS_VEHICLE_PROJECT_H
#define AMS_VEHICLE_PROJECT_H
#define AMS_CHARGER_ONLY_BUILD 0
#define AMS_VEHICLE_ONLY_BUILD 1
#ifndef AMS_ENABLE_APM_2950
#define AMS_ENABLE_APM_2950 0
#endif
#if AMS_ENABLE_APM_2950
#error "Five-SMB vehicle variant excludes ADBMS2950/APM"
#endif
#ifndef AMS_BUILD_PROFILE
#define AMS_BUILD_PROFILE 4
#endif
#if AMS_BUILD_PROFILE != 4 && AMS_BUILD_PROFILE != 3
#error "Vehicle project supports TESTDAY communications or gated VEHICLE release"
#endif
#ifndef DER26_CAN_BITRATE_KBPS
#define DER26_CAN_BITRATE_KBPS 1000u
#endif
#if DER26_CAN_BITRATE_KBPS != 1000u
#error "Vehicle project requires 1 Mbit/s"
#endif
#define AMS_CHARGE_REVIEW_C1_UNOBSERVED_CHARGE_ALLOWED 0
#define AMS_C1_INFERRED_STOP_V 4.60f
#define AMS_C1_INFERRED_MIN_V 2.00f
#ifndef AMS_ENABLE_MISSION_CAN
#define AMS_ENABLE_MISSION_CAN 1
#endif
#if !AMS_ENABLE_MISSION_CAN
#error "Vehicle project requires ECU mission CAN"
#endif
/* Dedicated safety/status bundles enabled; optional tuning traffic disabled. */
#ifndef AMS_ENABLE_TUNING_CAN
#define AMS_ENABLE_TUNING_CAN 0
#endif
#ifndef AMS_ENABLE_FAN_TASK
#define AMS_ENABLE_FAN_TASK 0
#endif
#if AMS_ENABLE_FAN_TASK
#error "Installed vehicle variant has no fan task"
#endif
#if AMS_BUILD_PROFILE == 3
#define AMS_SOURCE_REVISION "DER26-AMS-v0.5.57-VEHICLE-1M-TIMING7-PHASED-GATED"
#else
#define AMS_SOURCE_REVISION "DER26-AMS-v0.5.57-VEHICLE-COMMS-1M-TIMING7-PHASED-INHIBITED"
#endif
#endif
