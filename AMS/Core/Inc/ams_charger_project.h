/* Diagnostic 250-kbit/s project: 75 physical series, 74 observed. Both
 * configurations remain BMS-inhibited until complete protection is proven. */
#ifndef AMS_CHARGER_PROJECT_H
#define AMS_CHARGER_PROJECT_H

#define AMS_CHARGER_ONLY_BUILD 1
#define AMS_VEHICLE_ONLY_BUILD 0
#ifndef AMS_BUILD_PROFILE
#define AMS_BUILD_PROFILE 4
#endif
#if (AMS_BUILD_PROFILE != 4) && (AMS_BUILD_PROFILE != 6)
#error "Charger project requires TESTDAY CAN-test or CHARGE_REVIEW profile"
#endif
#ifndef DER26_CAN_BITRATE_KBPS
#define DER26_CAN_BITRATE_KBPS 250u
#endif
#if DER26_CAN_BITRATE_KBPS != 250u
#error "Charger project requires 250 kbit/s; use the separate vehicle project for 1 Mbit/s"
#endif
#ifndef AMS_ENABLE_TUNING_CAN
#define AMS_ENABLE_TUNING_CAN 0
#endif
#if AMS_ENABLE_TUNING_CAN
#error "Charger project does not permit tuning CAN"
#endif
#ifndef AMS_ENABLE_MISSION_CAN
#define AMS_ENABLE_MISSION_CAN 0
#endif
#if AMS_ENABLE_MISSION_CAN
#error "Standalone charger project does not accept ECU mission commands"
#endif
/* Profile 6 charging permission with SMB1/C1 UNMEASURED.  Operator-ordered
 * override of the previous compile-time lock.  Nothing in this firmware can
 * observe or stop a C1 overvoltage directly; the only compensating control is
 * the coarse inferred-C1 tripwire in canbus_task.c.  Set to 0 to restore the
 * original no-charge diagnostic mode. */
#ifndef AMS_CHARGE_REVIEW_C1_UNOBSERVED_CHARGE_ALLOWED
#define AMS_CHARGE_REVIEW_C1_UNOBSERVED_CHARGE_ALLOWED 1
#endif
/* Inferred C1 = charger-reported output voltage - sum of 74 measured cells.
 * Charger voltage sense is coarse, so this only catches gross C1 excursions.
 * Charging stops when inferred C1 exceeds this value, or is implausibly low. */
#ifndef AMS_C1_INFERRED_STOP_V
#define AMS_C1_INFERRED_STOP_V 4.60f
#endif
#ifndef AMS_C1_INFERRED_MIN_V
#define AMS_C1_INFERRED_MIN_V 2.00f
#endif
#if AMS_BUILD_PROFILE == 6
#if AMS_CHARGE_REVIEW_C1_UNOBSERVED_CHARGE_ALLOWED
#define AMS_SOURCE_REVISION "DER26-AMS-v0.5.57-75S-74OBS-C1-UNPROTECTED-CHARGE-250-20261108"
#else
#define AMS_SOURCE_REVISION "DER26-AMS-v0.5.57-75S-74OBS-DIAG-250-20261108"
#endif
#else
#define AMS_SOURCE_REVISION "DER26-AMS-v0.5.57-CHARGER-CAN-TEST-250-20261108"
#endif
#endif
