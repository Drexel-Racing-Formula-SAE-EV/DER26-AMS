/*
 * ams_measurement.h
 * Author: Mahad Faisal (2026)
 *
 * Timestamped, immutable measurement publication for estimator/SoH/SoP and
 * telemetry consumers. Safety fault ownership remains in the existing tasks;
 * this layer prevents lower-priority consumers from traversing live driver
 * storage while an acquisition task is updating it.
 */

#ifndef INC_MEASUREMENT_AMS_MEASUREMENT_H_
#define INC_MEASUREMENT_AMS_MEASUREMENT_H_

#include <stdbool.h>
#include <stdint.h>

#include "ext_drivers/accumulator.h"

#ifdef __cplusplus
extern "C" {
#endif

#define AMS_CURRENT_WINDOW_MAX_SAMPLE_AGE_MS 100u
#define AMS_CURRENT_WINDOW_MAX_INTEGRATION_GAP_MS 100u

#define AMS_MEAS_VALID_VOLTAGE       (1u << 0u)
#define AMS_MEAS_VALID_TEMPERATURE   (1u << 1u)
#define AMS_MEAS_VALID_CURRENT       (1u << 2u)
#define AMS_MEAS_BALANCE_RECOVERED   (1u << 3u)
#define AMS_MEAS_BALANCE_WAS_ACTIVE  (1u << 4u)
#define AMS_MEAS_CURRENT_TIMING_VALID (1u << 5u)
#define AMS_CURRENT_ACQUISITION_MAX_MS 10u

typedef struct
{
    uint32_t sequence;
    uint32_t start_tick;
    uint32_t end_tick;
    uint32_t sample_count;
    uint32_t invalid_sample_count;
    uint32_t latest_sample_tick;
    float latest_A;
    float filtered_A;
    float average_A;
    float rms_A;
    float min_A;
    float max_A;
    double charge_As;
    double absolute_charge_As;
    double total_charge_As;
    double total_absolute_charge_As;
    uint32_t total_invalid_sample_count;
    uint32_t calibration_id;
    bool calibration_record_confident;
    bool valid;
} ams_current_window_t;

typedef struct
{
    ams_current_window_t active;
    uint32_t next_sequence;
    uint32_t last_sample_tick;
    uint32_t integration_tick;
    float last_current_A;
    float last_filtered_A;
    double total_charge_As;
    double total_absolute_charge_As;
    uint32_t total_invalid_sample_count;
    double active_current_squared_A2s;
    uint32_t last_calibration_id;
    bool initialized;
    bool last_sample_valid;
    bool last_calibration_record_confident;
    bool active_calibration_provenance_initialized;
} ams_current_window_accumulator_t;

typedef struct
{
    uint32_t sequence;
    uint32_t acquisition_start_tick;
    uint32_t voltage_complete_tick;
    uint32_t publication_tick;

    uint16_t cell_mv[NSMBS][NCELLS];
    uint32_t cell_age_ms[NSMBS][NCELLS];
    uint16_t cell_usable_mask[NSMBS];
    uint16_t voltage_updated_mask[NSMBS];
    uint16_t voltage_stale_mask[NSMBS];
    uint16_t voltage_pec_fail_mask[NSMBS];
    uint16_t voltage_jump_mask[NSMBS];
    uint16_t voltage_stuck_mask[NSMBS];

    int16_t temp_deci_c[NSMBS][NTEMPS];
    uint32_t temp_age_ms[NSMBS][NTEMPS];
    uint32_t temp_usable_mask[NSMBS];
    uint32_t temp_updated_mask[NSMBS];
    uint32_t temp_stale_mask[NSMBS];
    uint32_t temp_invalid_mask[NSMBS];
    uint32_t temp_open_mask[NSMBS];
    uint32_t temp_short_mask[NSMBS];
    uint32_t temp_jump_mask[NSMBS];
    uint32_t temp_rate_rise_mask[NSMBS];

    ams_current_window_t current;
    uint16_t balancing_mask[NSMBS];
    uint32_t balance_off_ms;
    uint32_t validity_flags;
} ams_measurement_snapshot_t;

typedef struct
{
    ams_measurement_snapshot_t buffer[2];
    uint16_t reader_count[2];
    uint32_t publication_drop_count;
    uint32_t next_sequence;
    uint32_t write_sequence;
    uint8_t published_index;
    uint8_t write_index;
    bool published;
    bool write_in_progress;
} ams_measurement_store_t;

void ams_current_window_init(ams_current_window_accumulator_t *acc,
                             uint32_t now);
void ams_current_window_update(ams_current_window_accumulator_t *acc,
                               uint32_t now,
                               float current_A,
                               float filtered_A,
                               bool valid,
                               bool calibration_record_confident,
                               uint32_t calibration_id);
bool ams_current_window_rotate(ams_current_window_accumulator_t *acc,
                               uint32_t boundary_tick,
                               ams_current_window_t *completed);

void ams_measurement_store_init(ams_measurement_store_t *store);
ams_measurement_snapshot_t *ams_measurement_store_begin_write(
    ams_measurement_store_t *store);
bool ams_measurement_store_abort_write(ams_measurement_store_t *store,
                                       ams_measurement_snapshot_t *snapshot);
void ams_measurement_snapshot_prepare(ams_measurement_snapshot_t *snapshot,
                                      const accumulator_t *acc,
                                      const ams_current_window_t *current,
                                      uint32_t acquisition_start_tick,
                                      uint32_t voltage_complete_tick,
                                      uint32_t publication_tick,
                                      const uint16_t balancing_mask[NSMBS],
                                      uint32_t balance_off_ms,
                                      uint32_t validity_flags);
uint32_t ams_measurement_store_publish(ams_measurement_store_t *store,
                                       ams_measurement_snapshot_t *snapshot);
bool ams_measurement_store_copy_latest(ams_measurement_store_t *store,
                                       ams_measurement_snapshot_t *snapshot);

#ifdef __cplusplus
}
#endif

#endif /* INC_MEASUREMENT_AMS_MEASUREMENT_H_ */
