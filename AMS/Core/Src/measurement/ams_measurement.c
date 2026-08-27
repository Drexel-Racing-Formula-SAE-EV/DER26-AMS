/*
 * ams_measurement.c
 * Author: Mahad Faisal (2026)
 */

#include "measurement/ams_measurement.h"

#include "FreeRTOS.h"
#include "task.h"

#include <float.h>
#include <math.h>
#include <string.h>

/* Keep the reader-pinned publication design explicit in the target memory
 * budget.  These are compile-time ceilings, not measured target stack/RAM
 * evidence; the final ELF/map and target high-water marks remain release
 * gates. */
_Static_assert(sizeof(ams_measurement_snapshot_t) <= 2048u,
               "measurement snapshot exceeded reviewed RAM ceiling");
_Static_assert(sizeof(ams_measurement_store_t) <= 4096u,
               "measurement store exceeded reviewed RAM ceiling");

static uint32_t saturating_increment_u32(uint32_t value)
{
    return (value == UINT32_MAX) ? UINT32_MAX : (value + 1u);
}

static uint32_t sequence_increment_u32(uint32_t value)
{
    value++;
    return (value == 0u) ? 1u : value;
}

static void current_record_invalid(ams_current_window_accumulator_t *acc)
{
    if(acc == NULL)
    {
        return;
    }

    acc->active.invalid_sample_count =
        saturating_increment_u32(acc->active.invalid_sample_count);
    acc->total_invalid_sample_count =
        saturating_increment_u32(acc->total_invalid_sample_count);
}

static bool current_value_valid(float current_A)
{
    return isfinite(current_A) && (fabsf(current_A) <= 1500.0f);
}

static void current_merge_calibration_provenance(
    ams_current_window_accumulator_t *acc,
    bool calibration_record_confident,
    uint32_t calibration_id)
{
    if(acc == NULL)
    {
        return;
    }

    calibration_record_confident = calibration_record_confident &&
                                    (calibration_id != 0u);
    if(!calibration_record_confident)
    {
        calibration_id = 0u;
    }

    if(!acc->active_calibration_provenance_initialized)
    {
        acc->active.calibration_record_confident =
            calibration_record_confident;
        acc->active.calibration_id = calibration_id;
        acc->active_calibration_provenance_initialized = true;
        return;
    }

    if(!acc->active.calibration_record_confident ||
       !calibration_record_confident ||
       (acc->active.calibration_id != calibration_id))
    {
        /* A voltage epoch that spans uncalibrated current or two calibration
         * records remains valid for safety/current integration, but cannot
         * be used as a resistance-SoH observation. */
        acc->active.calibration_record_confident = false;
        acc->active.calibration_id = 0u;
    }
}

static void current_integrate(ams_current_window_accumulator_t *acc,
                              float from_A,
                              float to_A,
                              uint32_t dt_ms)
{
    if((acc == NULL) || (dt_ms == 0u))
    {
        return;
    }

    double dt_s = (double)dt_ms / 1000.0;
    double from = (double)from_A;
    double to = (double)to_A;
    double delta_charge = 0.5 * (from + to) * dt_s;
    double delta_abs_charge = 0.5 * (fabs(from) + fabs(to)) * dt_s;
    double delta_squared = 0.5 * ((from * from) + (to * to)) * dt_s;

    acc->active.charge_As += delta_charge;
    acc->active.absolute_charge_As += delta_abs_charge;
    acc->active_current_squared_A2s += delta_squared;
    acc->total_charge_As += delta_charge;
    acc->total_absolute_charge_As += delta_abs_charge;
}

void ams_current_window_init(ams_current_window_accumulator_t *acc,
                             uint32_t now)
{
    if(acc == NULL)
    {
        return;
    }

    memset(acc, 0, sizeof(*acc));
    acc->active.start_tick = now;
    acc->active.end_tick = now;
    acc->initialized = true;
}

void ams_current_window_update(ams_current_window_accumulator_t *acc,
                               uint32_t now,
                               float current_A,
                               float filtered_A,
                               bool valid,
                               bool calibration_record_confident,
                               uint32_t calibration_id)
{
    if(acc == NULL)
    {
        return;
    }
    if(!acc->initialized)
    {
        ams_current_window_init(acc, now);
    }

    valid = valid && current_value_valid(current_A);
    if(!valid)
    {
        current_record_invalid(acc);
        acc->active.end_tick = now;
        acc->last_sample_valid = false;
        return;
    }

    if(!isfinite(filtered_A))
    {
        filtered_A = current_A;
    }

    current_merge_calibration_provenance(acc,
                                         calibration_record_confident,
                                         calibration_id);

    if(acc->last_sample_valid)
    {
        uint32_t dt_ms = (uint32_t)(now - acc->integration_tick);
        if(dt_ms <= AMS_CURRENT_WINDOW_MAX_INTEGRATION_GAP_MS)
        {
            current_integrate(acc, acc->last_current_A, current_A, dt_ms);
        }
        else
        {
            current_record_invalid(acc);
        }
    }
    else if(acc->active.sample_count == 0u)
    {
        /* Cover the interval from the voltage boundary (or initialization)
         * to the first sample with the first observed value.  This bounded
         * zero-order hold avoids silently losing charge at every window
         * boundary while still rejecting a late first sample. */
        uint32_t initial_gap_ms = (uint32_t)(now - acc->active.start_tick);
        if(initial_gap_ms <= AMS_CURRENT_WINDOW_MAX_INTEGRATION_GAP_MS)
        {
            current_integrate(acc, current_A, current_A, initial_gap_ms);
        }
        else
        {
            current_record_invalid(acc);
        }
    }

    if(acc->active.sample_count == 0u)
    {
        acc->active.min_A = current_A;
        acc->active.max_A = current_A;
    }
    else
    {
        if(current_A < acc->active.min_A)
        {
            acc->active.min_A = current_A;
        }
        if(current_A > acc->active.max_A)
        {
            acc->active.max_A = current_A;
        }
    }

    acc->active.sample_count = saturating_increment_u32(acc->active.sample_count);
    acc->active.end_tick = now;
    acc->active.latest_sample_tick = now;
    acc->active.latest_A = current_A;
    acc->active.filtered_A = filtered_A;
    acc->last_sample_tick = now;
    acc->integration_tick = now;
    acc->last_current_A = current_A;
    acc->last_filtered_A = filtered_A;
    acc->last_calibration_record_confident =
        calibration_record_confident && (calibration_id != 0u);
    acc->last_calibration_id = acc->last_calibration_record_confident ?
                               calibration_id : 0u;
    acc->last_sample_valid = true;
}

void ams_current_window_set_sensor_metadata(
    ams_current_window_accumulator_t *acc,
    uint16_t uncertainty_mA,
    uint8_t selected_range)
{
    if(acc == NULL)
    {
        return;
    }
    if(!acc->initialized)
    {
        return;
    }

    if(uncertainty_mA > acc->active.uncertainty_mA)
    {
        acc->active.uncertainty_mA = uncertainty_mA;
    }

    if(acc->active.selected_range == 0u)
    {
        acc->active.selected_range = selected_range;
    }
    else if(acc->active.selected_range != selected_range)
    {
        acc->active.selected_range = 0u;
    }
}

bool ams_current_window_rotate(ams_current_window_accumulator_t *acc,
                               uint32_t boundary_tick,
                               ams_current_window_t *completed)
{
    if((acc == NULL) || (completed == NULL))
    {
        return false;
    }
    if(!acc->initialized)
    {
        ams_current_window_init(acc, boundary_tick);
    }

    uint32_t latest_age_ms = UINT32_MAX;
    if(acc->last_sample_valid)
    {
        latest_age_ms = (uint32_t)(boundary_tick - acc->last_sample_tick);
        uint32_t pending_ms = (uint32_t)(boundary_tick - acc->integration_tick);
        if((latest_age_ms <= AMS_CURRENT_WINDOW_MAX_SAMPLE_AGE_MS) &&
           (pending_ms <= AMS_CURRENT_WINDOW_MAX_INTEGRATION_GAP_MS))
        {
            current_integrate(acc,
                              acc->last_current_A,
                              acc->last_current_A,
                              pending_ms);
            acc->integration_tick = boundary_tick;
        }
        else
        {
            current_record_invalid(acc);
        }
    }

    acc->active.end_tick = boundary_tick;
    acc->active.sequence = sequence_increment_u32(acc->next_sequence);
    acc->next_sequence = acc->active.sequence;
    acc->active.total_charge_As = acc->total_charge_As;
    acc->active.total_absolute_charge_As = acc->total_absolute_charge_As;
    acc->active.total_invalid_sample_count = acc->total_invalid_sample_count;

    uint32_t duration_ms = (uint32_t)(boundary_tick - acc->active.start_tick);
    if(duration_ms > 0u)
    {
        double duration_s = (double)duration_ms / 1000.0;
        acc->active.average_A = (float)(acc->active.charge_As / duration_s);
        double mean_square = acc->active_current_squared_A2s / duration_s;
        acc->active.rms_A = (float)sqrt((mean_square > 0.0) ? mean_square : 0.0);
    }
    else if(acc->active.sample_count > 0u)
    {
        acc->active.average_A = acc->active.latest_A;
        acc->active.rms_A = fabsf(acc->active.latest_A);
    }

    acc->active.valid = (acc->active.sample_count > 0u) &&
                        (acc->active.invalid_sample_count == 0u) &&
                        acc->last_sample_valid &&
                        (latest_age_ms <= AMS_CURRENT_WINDOW_MAX_SAMPLE_AGE_MS);
    *completed = acc->active;

    memset(&acc->active, 0, sizeof(acc->active));
    acc->active.start_tick = boundary_tick;
    acc->active.end_tick = boundary_tick;
    acc->active_calibration_provenance_initialized = false;
    if(acc->last_sample_valid)
    {
        acc->active.latest_sample_tick = acc->last_sample_tick;
        acc->active.latest_A = acc->last_current_A;
        acc->active.filtered_A = acc->last_filtered_A;
        acc->active.min_A = acc->last_current_A;
        acc->active.max_A = acc->last_current_A;
        current_merge_calibration_provenance(
            acc,
            acc->last_calibration_record_confident,
            acc->last_calibration_id);
    }
    acc->active_current_squared_A2s = 0.0;

    return completed->valid;
}

void ams_measurement_store_init(ams_measurement_store_t *store)
{
    if(store != NULL)
    {
        memset(store, 0, sizeof(*store));
    }
}

ams_measurement_snapshot_t *ams_measurement_store_begin_write(
    ams_measurement_store_t *store)
{
    if(store == NULL)
    {
        return NULL;
    }

    ams_measurement_snapshot_t *snapshot = NULL;

    taskENTER_CRITICAL();
    uint32_t sequence = store->next_sequence + 1u;
    if(sequence == 0u)
    {
        sequence = 1u;
    }
    store->next_sequence = sequence;

    uint8_t index = store->published ?
                    (uint8_t)(store->published_index ^ 1u) : 0u;
    if(!store->write_in_progress && (store->reader_count[index] == 0u))
    {
        store->write_index = index;
        store->write_sequence = sequence;
        store->write_in_progress = true;
        snapshot = &store->buffer[index];
    }
    else
    {
        store->publication_drop_count =
            saturating_increment_u32(store->publication_drop_count);
    }
    taskEXIT_CRITICAL();

    return snapshot;
}

void ams_measurement_snapshot_prepare(ams_measurement_snapshot_t *snapshot,
                                      const accumulator_t *acc,
                                      const ams_current_window_t *current,
                                      uint32_t acquisition_start_tick,
                                      uint32_t voltage_complete_tick,
                                      uint32_t publication_tick,
                                      const uint16_t balancing_mask[NSMBS],
                                      uint32_t balance_off_ms,
                                      uint32_t validity_flags)
{
    if(snapshot == NULL)
    {
        return;
    }

    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->acquisition_start_tick = acquisition_start_tick;
    snapshot->voltage_complete_tick = voltage_complete_tick;
    snapshot->publication_tick = publication_tick;
    snapshot->balance_off_ms = balance_off_ms;
    snapshot->validity_flags = validity_flags;

    if(current != NULL)
    {
        snapshot->current = *current;
    }
    if(balancing_mask != NULL)
    {
        memcpy(snapshot->balancing_mask,
               balancing_mask,
               sizeof(snapshot->balancing_mask));
    }
    if(acc == NULL)
    {
        return;
    }

    uint8_t ic_count = accumulator_configured_smb_count(acc);
    for(uint8_t seg = 0u; seg < NSMBS; seg++)
    {
        if(seg >= ic_count)
        {
            continue;
        }
        snapshot->cell_usable_mask[seg] = acc->usable_voltage_mask[seg];
        snapshot->cell_avg8_usable_mask[seg] = acc->avg8_usable_voltage_mask[seg];
        snapshot->cell_iir_usable_mask[seg] = acc->iir_usable_voltage_mask[seg];
        snapshot->temp_usable_mask[seg] = acc->usable_temp_mask[seg];
        for(uint8_t cell = 0u; cell < NCELLS; cell++)
        {
            snapshot->cell_mv[seg][cell] = acc->cell_voltage_mv[seg][cell];
            snapshot->cell_avg8_mv[seg][cell] = acc->cell_voltage_avg8_mv[seg][cell];
            snapshot->cell_iir_mv[seg][cell] = acc->cell_voltage_iir_mv[seg][cell];
            snapshot->cell_age_ms[seg][cell] =
                (uint32_t)(voltage_complete_tick -
                           acc->cell_voltage_last_update_ms[seg][cell]);
        }
        for(uint8_t sensor = 0u; sensor < NTEMPS; sensor++)
        {
            snapshot->temp_deci_c[seg][sensor] = acc->temp_deci_c[seg][sensor];
            snapshot->temp_age_ms[seg][sensor] =
                (uint32_t)(publication_tick -
                           acc->temp_last_update_ms[seg][sensor]);
        }
    }
}

uint32_t ams_measurement_store_publish(ams_measurement_store_t *store,
                                       ams_measurement_snapshot_t *snapshot)
{
    if((store == NULL) || (snapshot == NULL))
    {
        return 0u;
    }

    taskENTER_CRITICAL();
    uint8_t index = store->write_index;
    if(!store->write_in_progress ||
       (index > 1u) ||
       (snapshot != &store->buffer[index]) ||
       (store->write_sequence == 0u) ||
       (store->reader_count[index] != 0u))
    {
        taskEXIT_CRITICAL();
        return 0u;
    }

    uint32_t sequence = store->write_sequence;
    snapshot->sequence = sequence;
    store->published_index = index;
    store->published = true;
    store->write_in_progress = false;
    store->write_sequence = 0u;
    taskEXIT_CRITICAL();
    return sequence;
}

bool ams_measurement_store_copy_latest(ams_measurement_store_t *store,
                                       ams_measurement_snapshot_t *snapshot)
{
    if((store == NULL) || (snapshot == NULL))
    {
        return false;
    }

    bool available = false;
    uint8_t index = 0u;

    taskENTER_CRITICAL();
    if(store->published)
    {
        index = store->published_index;
        if(store->reader_count[index] != UINT16_MAX)
        {
            store->reader_count[index]++;
            available = true;
        }
    }
    taskEXIT_CRITICAL();

    if(!available)
    {
        return false;
    }

    *snapshot = store->buffer[index];

    taskENTER_CRITICAL();
    if(store->reader_count[index] > 0u)
    {
        store->reader_count[index]--;
    }
    taskEXIT_CRITICAL();
    return available;
}
