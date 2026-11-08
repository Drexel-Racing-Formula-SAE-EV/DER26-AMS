#ifndef AMS_CELL_OBSERVATION_H
#define AMS_CELL_OBSERVATION_H
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "ams_build_profile.h"
/* Five physical 15s segments, including SMB1/C1. The C1 measurement is
 * unavailable: masking the channel is diagnostic only, never charge authority. */
#define AMS_PHYSICAL_SERIES_CELLS 75u
#define AMS_MONITORED_SERIES_CELLS 74u

#ifndef AMS_TESTDAY_C1_EXCLUDED_IC_MASK
#if (AMS_BUILD_PROFILE == AMS_PROFILE_TESTDAY) || \
    (AMS_BUILD_PROFILE == AMS_PROFILE_CHARGE_REVIEW)
#define AMS_TESTDAY_C1_EXCLUDED_IC_MASK 0x0001u
#else
#define AMS_TESTDAY_C1_EXCLUDED_IC_MASK 0u
#endif
#endif
#if AMS_TESTDAY_C1_EXCLUDED_IC_MASK && \
    (AMS_BUILD_PROFILE != AMS_PROFILE_TESTDAY) && \
    (AMS_BUILD_PROFILE != AMS_PROFILE_CHARGE_REVIEW)
#error "C1 exclusion is confined to TESTDAY and CHARGE_REVIEW"
#endif
#if (AMS_BUILD_PROFILE == AMS_PROFILE_CHARGE_REVIEW) && (AMS_TESTDAY_C1_EXCLUDED_IC_MASK != 0x0001u)
#error "75s diagnostics requires SMB1/C1 to remain unavailable"
#endif
#if (AMS_TESTDAY_C1_EXCLUDED_IC_MASK & ~0x001Fu)
#error "Missing-cell SMB mask must identify the five physical SMBs"
#endif
/* True => charging must stop. Inferred C1 = charger output voltage - sum of
 * the 74 measured cells. Fails closed on stale/missing/non-finite inputs. */
static inline bool ams_c1_inferred_stop(bool charger_fresh, float charger_v, float observed_sum_v)
{
    float c1 = charger_v - observed_sum_v;
    return !charger_fresh || !isfinite(c1) || (observed_sum_v <= 0.0f) ||
           (charger_v <= 0.0f) || (c1 > AMS_C1_INFERRED_STOP_V) ||
           (c1 < AMS_C1_INFERRED_MIN_V);
}
static inline uint16_t ams_observed_cell_mask(uint8_t seg)
{
    if(seg >= 5u) return 0u;
    return (uint16_t)(0x7FFFu & ~(((AMS_TESTDAY_C1_EXCLUDED_IC_MASK >> seg) & 1u) ? 1u : 0u));
}
static inline uint8_t ams_observed_segment_cell_count(uint8_t seg)
{
    return (uint8_t)((ams_observed_cell_mask(seg) == 0x7FFEu) ? 14u : 15u);
}
#endif
