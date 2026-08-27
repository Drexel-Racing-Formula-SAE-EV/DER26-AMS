#include "ext_drivers/parallel_connection_observer.h"

#include <stddef.h>
#include <string.h>

#include "ams_build_profile.h"

#ifndef AMS_PARALLEL_CONNECTION_OBSERVER_VALIDATED
#define AMS_PARALLEL_CONNECTION_OBSERVER_VALIDATED 0
#endif

static float observer_absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

static bool observer_topology(const accumulator_t *acc,
                              uint8_t *segment_count,
                              uint8_t *cell_count,
                              uint16_t *required_mask)
{
    uint8_t segments;
    uint8_t cells;

    if((acc == NULL) || (segment_count == NULL) || (cell_count == NULL) ||
       (required_mask == NULL))
    {
        return false;
    }

    if((acc->smb.ics == NULL) ||
       (acc->smb.num_ics <= 0) ||
       (acc->smb.num_ics > NSMBS) ||
       (acc->smb.num_ics > (int)ADBMS6830_MAX_TRACKED_ICS) ||
       (acc->smb.ics_capacity < (uint8_t)acc->smb.num_ics))
    {
        return false;
    }

    segments = (uint8_t)acc->smb.num_ics;
    cells = acc->smb.monitored_cell_count;
    if((cells == 0u) || (cells > NCELLS))
    {
        return false;
    }

    *segment_count = segments;
    *cell_count = cells;
    *required_mask = (cells >= 16u) ? UINT16_MAX :
                     (uint16_t)((1UL << cells) - 1UL);
    return true;
}

static bool observer_voltage_image_fresh(const accumulator_t *acc,
                                         uint32_t now_ms,
                                         uint8_t segment_count,
                                         uint8_t cell_count,
                                         uint16_t required_mask)
{
    if((acc == NULL) || !acc->voltage_full_usable ||
       (segment_count == 0u) || (segment_count > NSMBS) ||
       (cell_count == 0u) || (cell_count > NCELLS))
    {
        return false;
    }

    for(uint8_t seg = 0u; seg < segment_count; seg++)
    {
        if((acc->usable_voltage_mask[seg] & required_mask) != required_mask)
        {
            return false;
        }
        for(uint8_t cell = 0u; cell < cell_count; cell++)
        {
            if((now_ms - acc->cell_voltage_last_update_ms[seg][cell]) >
               AMS_PARALLEL_OBSERVER_MAX_SAMPLE_AGE_MS)
            {
                return false;
            }
        }
    }
    return true;
}

static float observer_median(float *values, uint8_t count)
{
    if((values == NULL) || (count == 0u))
    {
        return 0.0f;
    }

    for(uint8_t i = 1u; i < count; i++)
    {
        float key = values[i];
        uint8_t j = i;
        while((j > 0u) && (values[j - 1u] > key))
        {
            values[j] = values[j - 1u];
            j--;
        }
        values[j] = key;
    }

    if((count & 1u) != 0u)
    {
        return values[count / 2u];
    }
    return 0.5f * (values[(count / 2u) - 1u] + values[count / 2u]);
}

static void observer_capture(ams_parallel_connection_observer_t *observer,
                             const accumulator_t *acc,
                             float current_a,
                             uint32_t now_ms,
                             uint8_t segment_count,
                             uint8_t cell_count,
                             uint16_t required_mask)
{
    observer->last_tick = now_ms;
    observer->last_current_a = current_a;
    memset(observer->last_valid_mask, 0, sizeof(observer->last_valid_mask));
    memset(observer->last_cell_mv, 0, sizeof(observer->last_cell_mv));
    for(uint8_t seg = 0u; seg < segment_count; seg++)
    {
        observer->last_valid_mask[seg] =
            (uint16_t)(acc->usable_voltage_mask[seg] & required_mask);
        for(uint8_t cell = 0u; cell < cell_count; cell++)
        {
            observer->last_cell_mv[seg][cell] = acc->cell_voltage_mv[seg][cell];
        }
    }
    observer->initialized = true;
}

static void observer_clear_current_result(
    ams_parallel_connection_observer_t *observer)
{
    observer->advisory_valid = false;
    observer->suspect = false;
    memset(observer->candidate_mask, 0, sizeof(observer->candidate_mask));
    memset(observer->suspect_mask, 0, sizeof(observer->suspect_mask));
}

static void observer_clear_unused_topology(
    ams_parallel_connection_observer_t *observer,
    uint8_t segment_count,
    uint8_t cell_count)
{
    for(uint8_t seg = 0u; seg < segment_count; seg++)
    {
        for(uint8_t cell = cell_count; cell < NCELLS; cell++)
        {
            observer->evidence_count[seg][cell] = 0u;
            observer->last_group_resistance_mohm[seg][cell] = 0.0f;
            observer->last_cell_mv[seg][cell] = 0u;
        }
    }

    for(uint8_t seg = segment_count; seg < NSMBS; seg++)
    {
        observer->candidate_mask[seg] = 0u;
        observer->suspect_mask[seg] = 0u;
        observer->last_valid_mask[seg] = 0u;
        observer->last_segment_median_mohm[seg] = 0.0f;
        memset(observer->evidence_count[seg],
               0,
               sizeof(observer->evidence_count[seg]));
        memset(observer->last_group_resistance_mohm[seg],
               0,
               sizeof(observer->last_group_resistance_mohm[seg]));
        memset(observer->last_cell_mv[seg],
               0,
               sizeof(observer->last_cell_mv[seg]));
    }
}

void ams_parallel_connection_observer_init(
    ams_parallel_connection_observer_t *observer)
{
    if(observer == NULL)
    {
        return;
    }

    *observer = (ams_parallel_connection_observer_t){0};
    observer->target_validated =
        (AMS_PARALLEL_CONNECTION_OBSERVER_VALIDATED != 0) &&
        (AMS_CURRENT_POLARITY_VALIDATED != 0) &&
        (AMS_CURRENT_CALIBRATION_VALIDATED != 0);
    observer->reason = AMS_PARALLEL_OBSERVER_WAITING;
}

void ams_parallel_connection_observer_step(
    ams_parallel_connection_observer_t *observer,
    const accumulator_t *acc,
    float current_a,
    bool current_valid,
    bool balancing_active,
    uint32_t now_ms)
{
    float delta_i;
    uint32_t age_ms;
    uint8_t segment_count = 0u;
    uint8_t cell_count = 0u;
    uint16_t required_mask = 0u;

    if((observer == NULL) || (acc == NULL))
    {
        return;
    }

    observer->input_valid = current_valid &&
                            observer_topology(acc,
                                              &segment_count,
                                              &cell_count,
                                              &required_mask) &&
                            observer_voltage_image_fresh(acc,
                                                         now_ms,
                                                         segment_count,
                                                         cell_count,
                                                         required_mask);

    if(!observer->input_valid)
    {
        observer_clear_current_result(observer);
        observer->initialized = false;
        observer->reason = AMS_PARALLEL_OBSERVER_INPUT_INVALID;
        return;
    }

    observer_clear_unused_topology(observer, segment_count, cell_count);

    if(balancing_active)
    {
        observer_clear_current_result(observer);
        observer->initialized = false;
        observer->reason = AMS_PARALLEL_OBSERVER_BALANCING_ACTIVE;
        return;
    }

    if(!observer->initialized)
    {
        observer->reason = AMS_PARALLEL_OBSERVER_WAITING;
        observer_capture(observer,
                         acc,
                         current_a,
                         now_ms,
                         segment_count,
                         cell_count,
                         required_mask);
        return;
    }

    age_ms = now_ms - observer->last_tick;
    delta_i = observer_absf(current_a - observer->last_current_a);
    if((age_ms == 0u) || (age_ms > AMS_PARALLEL_OBSERVER_MAX_SAMPLE_AGE_MS) ||
       (delta_i < AMS_PARALLEL_OBSERVER_MIN_CURRENT_STEP_A) ||
       (delta_i > AMS_PARALLEL_OBSERVER_MAX_CURRENT_STEP_A))
    {
        observer->reason = AMS_PARALLEL_OBSERVER_NO_CURRENT_STEP;
        observer_capture(observer,
                         acc,
                         current_a,
                         now_ms,
                         segment_count,
                         cell_count,
                         required_mask);
        return;
    }

    observer->advisory_valid = true;
    if(observer->accepted_step_count != UINT32_MAX)
    {
        observer->accepted_step_count++;
    }
    observer->suspect = false;
    memset(observer->candidate_mask, 0, sizeof(observer->candidate_mask));
    memset(observer->suspect_mask, 0, sizeof(observer->suspect_mask));

    for(uint8_t seg = 0u; seg < segment_count; seg++)
    {
        float resistance[NCELLS];
        uint8_t resistance_cell[NCELLS];
        uint8_t valid_count = 0u;
        uint16_t common_mask = (uint16_t)(acc->usable_voltage_mask[seg] &
                                          observer->last_valid_mask[seg] &
                                          required_mask);
        uint16_t candidate_mask = 0u;
        uint16_t suspect_mask = 0u;

        observer->last_segment_median_mohm[seg] = 0.0f;
        for(uint8_t cell = 0u; cell < cell_count; cell++)
        {
            uint16_t bit = (uint16_t)(1u << cell);
            if((common_mask & bit) == 0u)
            {
                observer->last_group_resistance_mohm[seg][cell] = 0.0f;
                continue;
            }

            float delta_v_mv = observer_absf(
                (float)acc->cell_voltage_mv[seg][cell] -
                (float)observer->last_cell_mv[seg][cell]);
            float r_mohm = delta_v_mv / delta_i;
            observer->last_group_resistance_mohm[seg][cell] = r_mohm;
            resistance[valid_count] = r_mohm;
            resistance_cell[valid_count] = cell;
            valid_count++;
        }

        if(valid_count >= 3u)
        {
            float median_work[NCELLS];
            memcpy(median_work, resistance, (size_t)valid_count * sizeof(float));
            float median = observer_median(median_work, valid_count);
            float relative_limit =
                median * ((float)AMS_PARALLEL_OBSERVER_RELATIVE_NUM /
                          (float)AMS_PARALLEL_OBSERVER_RELATIVE_DEN);
            float limit = relative_limit;
            if(limit < (median + AMS_PARALLEL_OBSERVER_MIN_EXCESS_MOHM))
            {
                limit = median + AMS_PARALLEL_OBSERVER_MIN_EXCESS_MOHM;
            }
            observer->last_segment_median_mohm[seg] = median;

            for(uint8_t i = 0u; i < valid_count; i++)
            {
                uint8_t cell = resistance_cell[i];
                uint16_t bit = (uint16_t)(1u << cell);
                bool candidate = resistance[i] > limit;

                if(candidate)
                {
                    candidate_mask |= bit;
                    if(observer->evidence_count[seg][cell] < UINT8_MAX)
                    {
                        observer->evidence_count[seg][cell]++;
                    }
                }
                else if(observer->evidence_count[seg][cell] > 0u)
                {
                    observer->evidence_count[seg][cell]--;
                }

                if(observer->evidence_count[seg][cell] >=
                   AMS_PARALLEL_OBSERVER_CONFIRM_EVENTS)
                {
                    suspect_mask |= bit;
                }
            }
        }

        observer->candidate_mask[seg] = candidate_mask;
        observer->suspect_mask[seg] = suspect_mask;
        observer->suspect = observer->suspect || (suspect_mask != 0u);
    }

    observer->reason = observer->suspect ?
                       AMS_PARALLEL_OBSERVER_SUSPECT :
                       AMS_PARALLEL_OBSERVER_STEP_ACCEPTED;
    observer_capture(observer,
                     acc,
                     current_a,
                     now_ms,
                     segment_count,
                     cell_count,
                     required_mask);
}

const char *ams_parallel_observer_reason_str(
    ams_parallel_observer_reason_t reason)
{
    switch(reason)
    {
    case AMS_PARALLEL_OBSERVER_WAITING: return "waiting";
    case AMS_PARALLEL_OBSERVER_INPUT_INVALID: return "input_invalid";
    case AMS_PARALLEL_OBSERVER_BALANCING_ACTIVE: return "balancing_active";
    case AMS_PARALLEL_OBSERVER_NO_CURRENT_STEP: return "no_current_step";
    case AMS_PARALLEL_OBSERVER_STEP_ACCEPTED: return "step_accepted";
    case AMS_PARALLEL_OBSERVER_SUSPECT: return "possible_parallel_connection_degradation";
    default: return "unknown";
    }
}
