/* Compile/link/run gate for the estimator topology selected by the build. */

#include "estimator/ams_soc_ekf.h"
#include "ams_cell_observation.h"

#include <stdint.h>

#ifndef AMS_EXPECTED_ESTIMATOR_INSTANCES
#error "AMS_EXPECTED_ESTIMATOR_INSTANCES must be supplied"
#endif

int main(void)
{
    ams_estimator_t estimator;
    ams_estimator_init_default(&estimator);

    if((estimator.enabled == 0u) ||
       (estimator.instance_count != AMS_EXPECTED_ESTIMATOR_INSTANCES))
    {
        return 1;
    }

    /* Segment EKFs and coulomb counting must use the same startup prior. */
    if (estimator.cc_soc != AMS_EKF_DEFAULT_SOC_INIT)
    {
        return 4;
    }
    for (uint8_t i = 0u; i < estimator.instance_count; ++i)
    {
        if ((estimator.inst[i].soc != AMS_EKF_DEFAULT_SOC_INIT) ||
            (estimator.inst[i].cfg.soc_init != AMS_EKF_DEFAULT_SOC_INIT))
        {
            return 5;
        }
    }
/* Both vehicle build configurations must use the same 27% startup prior. */
    if (AMS_EKF_DEFAULT_SOC_INIT != 0.27f)
    {
        return 6;
    }

#if AMS_ESTIMATOR_DEFAULT_TOPOLOGY == AMS_ESTIMATOR_TOPOLOGY_PACK
    if((estimator.instance_count != 1u) ||
       (estimator.inst[0].cfg.first_series_group != 0u) ||
       (estimator.inst[0].cfg.series_group_count != AMS_EKF_PACK_SERIES_GROUPS))
    {
        return 2;
    }
#elif AMS_ESTIMATOR_DEFAULT_TOPOLOGY == AMS_ESTIMATOR_TOPOLOGY_SEGMENTS
    for(uint8_t segment = 0u; segment < 5u; segment++)
    {
        if((estimator.inst[segment].cfg.first_series_group !=
            (uint16_t)(segment * 15u + 15u - ams_observed_segment_cell_count(segment))) ||
           (estimator.inst[segment].cfg.series_group_count != ams_observed_segment_cell_count(segment)))
        {
            return 3;
        }
    }
#endif

    return 0;
}
