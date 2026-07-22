/*
 * Compile-only checks for the three explicit AMS build intents.
 *
 * The vehicle invocation is intentionally compiled twice by the Makefile:
 * once with every release-evidence acknowledgement, which must succeed, and
 * once without those acknowledgements, which must fail in
 * ams_build_profile.h before any application source is accepted.
 */

#include "ams_build_profile.h"

#ifndef AMS_EXPECTED_PROFILE
#error "AMS_EXPECTED_PROFILE must be supplied by the profile-gates target"
#endif

_Static_assert(AMS_BUILD_PROFILE == AMS_EXPECTED_PROFILE,
               "compiled profile does not match requested profile");
_Static_assert((AMS_ESTIMATOR_DEFAULT_TOPOLOGY == AMS_ESTIMATOR_TOPOLOGY_PACK) ||
               (AMS_ESTIMATOR_DEFAULT_TOPOLOGY == AMS_ESTIMATOR_TOPOLOGY_SEGMENTS),
               "estimator topology must be explicit and supported");

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
#else
#error "unsupported expected profile"
#endif

int main(void)
{
    return 0;
}
