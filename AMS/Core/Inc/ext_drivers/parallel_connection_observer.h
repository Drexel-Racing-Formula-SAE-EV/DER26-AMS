/*
 * parallel_connection_observer.h
 *
 * Advisory detector for a degraded parallel-cell connection (for example one
 * opened fusible wire bond) using repeated short current-step observations.
 * The ADBMS6830 measures the complete parallel group, so this observer cannot
 * identify an individual bond and must never be presented as deterministic
 * wire-bond protection.
 */
#ifndef INC_EXT_DRIVERS_PARALLEL_CONNECTION_OBSERVER_H_
#define INC_EXT_DRIVERS_PARALLEL_CONNECTION_OBSERVER_H_

#include <stdbool.h>
#include <stdint.h>

#include "ext_drivers/accumulator.h"

#define AMS_PARALLEL_OBSERVER_CONFIRM_EVENTS       5u
#define AMS_PARALLEL_OBSERVER_MIN_CURRENT_STEP_A   10.0f
#define AMS_PARALLEL_OBSERVER_MAX_CURRENT_STEP_A   400.0f
#define AMS_PARALLEL_OBSERVER_MAX_SAMPLE_AGE_MS    500u
#define AMS_PARALLEL_OBSERVER_MIN_EXCESS_MOHM      0.50f
#define AMS_PARALLEL_OBSERVER_RELATIVE_NUM         3u
#define AMS_PARALLEL_OBSERVER_RELATIVE_DEN         2u

typedef enum
{
    AMS_PARALLEL_OBSERVER_WAITING = 0,
    AMS_PARALLEL_OBSERVER_INPUT_INVALID,
    AMS_PARALLEL_OBSERVER_BALANCING_ACTIVE,
    AMS_PARALLEL_OBSERVER_NO_CURRENT_STEP,
    AMS_PARALLEL_OBSERVER_STEP_ACCEPTED,
    AMS_PARALLEL_OBSERVER_SUSPECT
} ams_parallel_observer_reason_t;

typedef struct
{
    bool initialized;
    bool input_valid;
    bool advisory_valid;
    bool target_validated;
    bool suspect;
    ams_parallel_observer_reason_t reason;

    uint32_t last_tick;
    float last_current_a;
    uint16_t last_cell_mv[NSMBS][NCELLS];
    uint16_t last_valid_mask[NSMBS];

    uint32_t accepted_step_count;
    uint16_t candidate_mask[NSMBS];
    uint16_t suspect_mask[NSMBS];
    uint8_t evidence_count[NSMBS][NCELLS];
    float last_group_resistance_mohm[NSMBS][NCELLS];
    float last_segment_median_mohm[NSMBS];
} ams_parallel_connection_observer_t;

void ams_parallel_connection_observer_init(
    ams_parallel_connection_observer_t *observer);

void ams_parallel_connection_observer_step(
    ams_parallel_connection_observer_t *observer,
    const accumulator_t *acc,
    float current_a,
    bool current_valid,
    bool balancing_active,
    uint32_t now_ms);

const char *ams_parallel_observer_reason_str(
    ams_parallel_observer_reason_t reason);

#endif /* INC_EXT_DRIVERS_PARALLEL_CONNECTION_OBSERVER_H_ */
