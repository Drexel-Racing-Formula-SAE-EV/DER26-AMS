/*
 * Compile-only checks for the three explicit AMS build intents.
 *
 * The vehicle invocation is intentionally compiled twice by the Makefile:
 * once with every release-evidence acknowledgement, which must succeed, and
 * once without those acknowledgements, which must fail in
 * ams_build_profile.h before any application source is accepted.
 */

#include "ams_build_profile.h"
#include "ext_drivers/accumulator.h"

#ifndef AMS_EXPECTED_PROFILE
#error "AMS_EXPECTED_PROFILE must be supplied by the profile-gates target"
#endif

_Static_assert(AMS_BUILD_PROFILE == AMS_EXPECTED_PROFILE,
               "compiled profile does not match requested profile");
_Static_assert((AMS_ESTIMATOR_DEFAULT_TOPOLOGY == AMS_ESTIMATOR_TOPOLOGY_PACK) ||
               (AMS_ESTIMATOR_DEFAULT_TOPOLOGY == AMS_ESTIMATOR_TOPOLOGY_SEGMENTS),
               "estimator topology must be explicit and supported");
#if (AMS_EXPECTED_PROFILE == AMS_PROFILE_TESTDAY) || \
    (AMS_EXPECTED_PROFILE == AMS_PROFILE_BENCH_VALIDATION)
_Static_assert(AMS_VOLTAGE_MODE == AMS_VOLTAGE_MODE_C_ONLY_MVP,
               "observation profiles must explicitly expose the degraded C-only path");
#else
_Static_assert(AMS_VOLTAGE_MODE == AMS_VOLTAGE_MODE_REDUNDANT_CS,
               "standard profiles must default to redundant C/S authority");
#endif

#if AMS_EXPECTED_PROFILE == AMS_PROFILE_BENCH
_Static_assert(AMS_HW_BRINGUP == 1, "bench profile must use bring-up policy");
_Static_assert(AMS_ENABLE_SERVICE_CLI == 1, "bench profile requires service CLI");
_Static_assert(AMS_ENABLE_HIL_CAN == 0, "bench profile must not inject HIL CAN");
_Static_assert(AMS_ENABLE_IMD == 0, "bench profile does not claim IMD validation");
_Static_assert(AMS_ENABLE_IWDG == 0, "bench profile does not claim IWDG validation");
_Static_assert(AMS_PROFILE_BMS_OUTPUT_INHIBIT_DEFAULT == 1,
               "bench profile must inhibit physical BMS output");
_Static_assert(AMS_PROFILE_BALANCE_INHIBIT_DEFAULT == 1,
               "bench profile must inhibit balancing");
#elif AMS_EXPECTED_PROFILE == AMS_PROFILE_HIL
_Static_assert(AMS_HW_BRINGUP == 0, "HIL is a distinct build intent");
_Static_assert(AMS_ENABLE_SERVICE_CLI == 1, "HIL profile requires service CLI");
_Static_assert(AMS_ENABLE_HIL_CAN == 1, "HIL profile requires CAN injection");
_Static_assert(AMS_HIL_REPLACE_ADBMS == 1, "HIL profile requires ADBMS replacement");
_Static_assert(AMS_PROFILE_BMS_OUTPUT_INHIBIT_DEFAULT == 1,
               "HIL profile must inhibit physical BMS output");
_Static_assert(AMS_PROFILE_BALANCE_INHIBIT_DEFAULT == 1,
               "HIL profile must inhibit balancing");
#elif AMS_EXPECTED_PROFILE == AMS_PROFILE_BENCH_VALIDATION
_Static_assert(AMS_HW_BRINGUP == 0, "bench validation must use 10 Hz acquisition");
_Static_assert(AMS_ENABLE_SERVICE_CLI == 1, "bench validation requires service CLI");
_Static_assert(AMS_ENABLE_HIL_CAN == 0, "bench validation must use physical measurements");
_Static_assert(AMS_HIL_REPLACE_ADBMS == 0, "bench validation requires physical ADBMS");
_Static_assert(AMS_ENABLE_IMD == 1, "bench validation enables the real IMD path");
_Static_assert(AMS_ENABLE_IWDG == 0, "unvalidated watchdog stays disabled");
_Static_assert(AMS_ENABLE_AUTO_TEMP_MUX_SCAN == 0,
               "automatic temperature scan stays off for staged bench validation");
_Static_assert(AMS_ENABLE_AUTO_C_OPEN_WIRE == 0,
               "intrusive automatic C open-wire stays disabled");
_Static_assert(AMS_ENABLE_ADBMS_AUX2_REDUNDANCY == 0,
               "unvalidated AUX2 redundancy stays disabled");
_Static_assert(AMS_ENABLE_ADBMS_THERM_OPEN_WIRE_DIAG == 0,
               "unvalidated automatic thermistor open-wire stays disabled");
_Static_assert(AMS_ENABLE_ADBMS_FAULT_INJECTION == 0,
               "fault injection stays disabled on hardware-validation bench");
_Static_assert(AMS_PROFILE_BMS_OUTPUT_INHIBIT_DEFAULT == 1,
               "bench validation must inhibit physical BMS output");
_Static_assert(AMS_PROFILE_BALANCE_INHIBIT_DEFAULT == 1,
               "bench validation must inhibit balancing");
_Static_assert(AMS_BENCH_VALIDATION_BMS_RELEASE_ALLOWED == 0,
               "bench validation BMS release must remain source-locked");
_Static_assert(AMS_BENCH_VALIDATION_BALANCE_RELEASE_ALLOWED == 0,
               "bench validation balancing release must remain source-locked");
_Static_assert(AMS_PROFILE_BMS_RUNTIME_AUTHORITY_ALLOWED == 0,
               "bench validation final BMS writer must remain compile-locked");
_Static_assert(AMS_PROFILE_BALANCE_RUNTIME_AUTHORITY_ALLOWED == 0,
               "bench validation final balancing path must remain compile-locked");
_Static_assert(NSMBS == 5, "bench validation uses five-SMB production topology");
_Static_assert(ACCUMULATOR_PHYSICAL_CHAIN_COUNT == 5,
               "bench validation excludes the APM final-ring device");
_Static_assert(AMS_ENABLE_APM_2950 == 0,
               "bench validation baseline keeps APM advisory path off");
_Static_assert(ACCUMULATOR_SMB_STRING == STRING_A,
               "bench validation five-SMB chain must use production String A");
_Static_assert(AMS_ESTIMATOR_DEFAULT_TOPOLOGY == AMS_ESTIMATOR_TOPOLOGY_SEGMENTS,
               "bench validation requires five segment DADEKFs for CAN/logging validation");
#elif AMS_EXPECTED_PROFILE == AMS_PROFILE_TESTDAY
_Static_assert(AMS_HW_BRINGUP == 0, "test day must use 10 Hz production acquisition");
_Static_assert(AMS_ENABLE_SERVICE_CLI == 1, "test day requires read/service CLI");
_Static_assert(AMS_ENABLE_HIL_CAN == 0, "test day must use physical measurements");
_Static_assert(AMS_HIL_REPLACE_ADBMS == 0, "test day requires physical ADBMS");
_Static_assert(AMS_ENABLE_IMD == 1, "test day exercises the real IMD path");
_Static_assert(AMS_ENABLE_IWDG == 0, "unvalidated watchdog stays disabled");
_Static_assert(AMS_ENABLE_AUTO_TEMP_MUX_SCAN == 1,
               "test day exercises all primary thermistors");
_Static_assert(AMS_ENABLE_AUTO_C_OPEN_WIRE == 0,
               "intrusive automatic C open-wire stays disabled");
_Static_assert(AMS_ENABLE_ADBMS_AUX2_REDUNDANCY == 0,
               "unvalidated AUX2 redundancy stays disabled");
_Static_assert(AMS_ENABLE_ADBMS_THERM_OPEN_WIRE_DIAG == 0,
               "unvalidated automatic thermistor open-wire stays disabled");
_Static_assert(AMS_PROFILE_BMS_OUTPUT_INHIBIT_DEFAULT == 1,
               "test day must inhibit physical BMS output");
_Static_assert(AMS_PROFILE_BALANCE_INHIBIT_DEFAULT == 1,
               "test day must inhibit balancing");
_Static_assert(AMS_TESTDAY_BMS_RELEASE_ALLOWED == 0,
               "test day BMS release must remain source-locked");
_Static_assert(AMS_TESTDAY_BALANCE_RELEASE_ALLOWED == 0,
               "test day balancing release must remain source-locked");
_Static_assert(AMS_PROFILE_BMS_RUNTIME_AUTHORITY_ALLOWED == 0,
               "test day final BMS hardware writer must remain compile-locked");
_Static_assert(AMS_PROFILE_BALANCE_RUNTIME_AUTHORITY_ALLOWED == 0,
               "test day final balancing application must remain compile-locked");
_Static_assert(NSMBS == 5, "test day requires five SMBs");
_Static_assert(ACCUMULATOR_PHYSICAL_CHAIN_COUNT == 5,
               "test day baseline excludes the APM final-ring device");
_Static_assert(AMS_ENABLE_APM_2950 == 0,
               "test day baseline keeps APM advisory path off");
_Static_assert(ACCUMULATOR_SMB_STRING == STRING_A,
               "five-SMB test-day chain must use production String A");
_Static_assert(AMS_ESTIMATOR_DEFAULT_TOPOLOGY == AMS_ESTIMATOR_TOPOLOGY_SEGMENTS,
               "test day requires five segment DADEKFs");
#elif AMS_EXPECTED_PROFILE == AMS_PROFILE_VEHICLE
_Static_assert(AMS_HW_BRINGUP == 0, "vehicle profile forbids bring-up behavior");
_Static_assert(AMS_ENABLE_SERVICE_CLI == 0, "vehicle profile forbids service mutation");
_Static_assert(AMS_ENABLE_HIL_CAN == 0, "vehicle profile forbids HIL CAN");
_Static_assert(AMS_HIL_REPLACE_ADBMS == 0, "vehicle profile requires physical ADBMS");
_Static_assert(AMS_ENABLE_IMD == 1, "vehicle profile requires validated IMD");
_Static_assert(AMS_ENABLE_IWDG == 1, "vehicle profile requires validated IWDG");
_Static_assert(AMS_PROFILE_BMS_OUTPUT_INHIBIT_DEFAULT == 0,
               "validated vehicle profile may release through runtime gates");
_Static_assert(AMS_PROFILE_BALANCE_INHIBIT_DEFAULT == 0,
               "validated vehicle profile may balance through runtime gates");
_Static_assert(AMS_PROFILE_BMS_RUNTIME_AUTHORITY_ALLOWED == 1,
               "vehicle BMS hardware writer must remain available behind runtime gates");
_Static_assert(AMS_PROFILE_BALANCE_RUNTIME_AUTHORITY_ALLOWED == 1,
               "vehicle balancing must remain available behind runtime gates");
_Static_assert(NSMBS == 5, "vehicle profile requires five SMBs");
_Static_assert(ACCUMULATOR_PHYSICAL_CHAIN_COUNT == 6,
               "vehicle profile requires five physical chain devices");
#else
#error "unsupported expected profile"
#endif

int main(void)
{
    return 0;
}
