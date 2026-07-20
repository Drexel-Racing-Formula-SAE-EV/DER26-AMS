/*
 * estimator_task.c
 * Author: Mahad Faisal (2026)
 *
 * Runs the advisory P42A DAEKF estimator at 10 Hz. The estimator is deliberately
 * non-authoritative: it does not control BMS_OK, AIRs, charging, shutdown, or
 * balancing. It only publishes state into app.estimator for telemetry/debug.
 */

#include "tasks/estimator_task.h"

#include "estimator/ams_soc_ekf.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

void estimator_task_fn(void *argument);

#define ESTIMATOR_STACK_WORDS       AMS_STACK_ESTIMATOR_WORDS
#define ESTIMATOR_HIL_TIMEOUT_TICKS 500U
#define ESTIMATOR_HW_TIMEOUT_TICKS  500U
#define ESTIMATOR_HW_MIN_TEMP_C     (-40.0f)
#define ESTIMATOR_HW_MAX_TEMP_C     120.0f
#define ESTIMATOR_EPOCH_MIN_DT_MS    1U
#define ESTIMATOR_EPOCH_MAX_DT_MS    1000U

static StaticTask_t estimator_task_tcb;
static StackType_t estimator_task_stack[ESTIMATOR_STACK_WORDS];
static TaskHandle_t estimator_task_handle = NULL;

static bool collect_group_voltage(const ams_measurement_snapshot_t *measurement,
                                  const ams_ekf_config_t *cfg,
                                  float *v_meas_V,
                                  uint16_t *valid_count)
{
    if ((measurement == NULL) || (cfg == NULL) ||
        (v_meas_V == NULL) || (valid_count == NULL))
    {
        return false;
    }

    float sum_v = 0.0f;
    uint16_t count = 0U;
    for (uint16_t n = 0U; n < cfg->series_group_count; n++)
    {
        uint16_t group = (uint16_t)(cfg->first_series_group + n);
        uint8_t seg = (uint8_t)(group / NCELLS);
        uint8_t cell = (uint8_t)(group % NCELLS);

        if ((seg >= NSMBS) || (cell >= NCELLS) ||
            ((measurement->cell_usable_mask[seg] & (uint16_t)(1u << cell)) == 0u))
        {
            continue;
        }

        float v = (float)measurement->cell_mv[seg][cell] / 1000.0f;
        if(isfinite(v) && (v >= 0.5f) && (v <= 5.0f))
        {
            sum_v += v;
            count++;
        }
    }

    *v_meas_V = sum_v;
    *valid_count = count;
    return (count == cfg->series_group_count);
}

static bool collect_group_temp(const ams_measurement_snapshot_t *measurement,
                               const ams_ekf_config_t *cfg,
                               float *temp_c)
{
    if ((measurement == NULL) || (cfg == NULL) || (temp_c == NULL))
    {
        return false;
    }

    float sum = 0.0f;
    uint16_t count = 0U;
    uint8_t first_seg = (uint8_t)(cfg->first_series_group / NCELLS);
    uint8_t last_seg = (uint8_t)((cfg->first_series_group + cfg->series_group_count - 1U) / NCELLS);

    for (uint8_t seg = first_seg; (seg <= last_seg) && (seg < NSMBS); seg++)
    {
        for (uint8_t sensor = 0U; sensor < NTEMPS; sensor++)
        {
            if((measurement->temp_usable_mask[seg] & (1UL << sensor)) == 0u)
            {
                continue;
            }
            float t = (float)measurement->temp_deci_c[seg][sensor] / 10.0f;
            if(isfinite(t) &&
               (t >= ESTIMATOR_HW_MIN_TEMP_C) &&
               (t <= ESTIMATOR_HW_MAX_TEMP_C))
            {
                sum += t;
                count++;
            }
        }
    }

    if (count > 0U)
    {
        *temp_c = sum / (float)count;
        return true;
    }

    *temp_c = 25.0f;
    return false;
}

static void estimator_saturating_add(uint32_t *value, uint32_t increment)
{
    if(value == NULL)
    {
        return;
    }
    if((UINT32_MAX - *value) < increment)
    {
        *value = UINT32_MAX;
    }
    else
    {
        *value += increment;
    }
}

static uint32_t estimator_sequence_distance(uint32_t newer, uint32_t older)
{
    if(newer > older)
    {
        return newer - older;
    }
    if(newer < older)
    {
        /* Measurement sequence zero is reserved, so the wrap is
         * UINT32_MAX -> 1 rather than UINT32_MAX -> 0. */
        return (UINT32_MAX - older) + newer;
    }
    return 0u;
}

static void estimator_mark_instances_fault(ams_estimator_t *est,
                                           uint32_t fault)
{
    if(est == NULL)
    {
        return;
    }

    for(uint8_t i = 0u;
        (i < est->instance_count) && (i < AMS_EKF_MAX_INSTANCES);
        i++)
    {
        if(est->inst[i].cfg.enabled != 0u)
        {
            est->inst[i].valid = 0u;
            est->inst[i].fault_flags = fault;
        }
    }
}

static bool estimator_dt_from_ticks(uint32_t newer_tick,
                                    uint32_t older_tick,
                                    float *dt_s)
{
    if(dt_s == NULL)
    {
        return false;
    }

    uint32_t dt_ms = (uint32_t)(newer_tick - older_tick);
    if((dt_ms < ESTIMATOR_EPOCH_MIN_DT_MS) ||
       (dt_ms > ESTIMATOR_EPOCH_MAX_DT_MS))
    {
        return false;
    }

    *dt_s = (float)dt_ms / 1000.0f;
    return true;
}

static bool estimator_step_with_soh(ams_estimator_t *est,
                                    uint8_t instance_index,
                                    float i_pack_A,
                                    float v_meas_V,
                                    float t_surf_C,
                                    float dt_s,
                                    uint32_t measurement_sequence,
                                    uint32_t measurement_tick,
                                    bool epoch_coherent,
                                    bool balance_recovered,
                                    bool current_calibration_confident)
{
    if((est == NULL) || (instance_index >= est->instance_count) ||
       (instance_index >= AMS_EKF_MAX_INSTANCES))
    {
        return false;
    }

    ams_ekf_instance_t *inst = &est->inst[instance_index];
    ams_resistance_soh_t *soh = &est->resistance_soh[instance_index];
    uint32_t reject_flags = ams_resistance_soh_gate(
        inst,
        i_pack_A,
        epoch_coherent,
        balance_recovered,
        current_calibration_confident);
    ams_ekf_r0_update_result_t r0_result =
        AMS_EKF_R0_UPDATE_NOT_REQUESTED;
    bool step_ok = ams_ekf_step_gated(inst,
                                      i_pack_A,
                                      v_meas_V,
                                      t_surf_C,
                                      dt_s,
                                      reject_flags == AMS_SOH_REJECT_NONE,
                                      &r0_result);
    ams_resistance_soh_record(soh,
                              inst,
                              measurement_sequence,
                              measurement_tick,
                              current_calibration_confident,
                              reject_flags,
                              r0_result,
                              step_ok);
    return step_ok;
}

static void estimator_record_unusable_soh_epoch(
    ams_estimator_t *est,
    uint8_t instance_index,
    float i_pack_A,
    uint32_t measurement_sequence,
    uint32_t measurement_tick,
    bool balance_recovered,
    bool current_calibration_confident)
{
    if((est == NULL) || (instance_index >= est->instance_count) ||
       (instance_index >= AMS_EKF_MAX_INSTANCES))
    {
        return;
    }

    ams_ekf_instance_t *inst = &est->inst[instance_index];
    uint32_t reject_flags = ams_resistance_soh_gate(
        inst,
        i_pack_A,
        false,
        balance_recovered,
        current_calibration_confident);
    ams_resistance_soh_record(&est->resistance_soh[instance_index],
                              inst,
                              measurement_sequence,
                              measurement_tick,
                              current_calibration_confident,
                              reject_flags,
                              AMS_EKF_R0_UPDATE_NOT_REQUESTED,
                              false);
}

#if AMS_ENABLE_HIL_CAN
static bool hil_meas_fresh(const ams_hil_meas_t *measurement, uint32_t now)
{
    if(measurement == NULL)
    {
        return false;
    }

    return ((measurement->fresh != 0U) &&
            ((now - measurement->last_rx_tick) <= ESTIMATOR_HIL_TIMEOUT_TICKS) &&
            isfinite(measurement->v_pack_V) &&
            isfinite(measurement->i_pack_A) &&
            isfinite(measurement->t_surf_C));
}
#endif

TaskHandle_t estimator_task_start(app_data_t *data)
{
    if (data == NULL)
    {
        return NULL;
    }

    if (estimator_task_handle == NULL)
    {
        estimator_task_handle = xTaskCreateStatic(estimator_task_fn,
                                                   "estimator task",
                                                   ESTIMATOR_STACK_WORDS,
                                                   (void *)data,
                                                   EST_PRIO,
                                                   estimator_task_stack,
                                                   &estimator_task_tcb);
    }

    return estimator_task_handle;
}

bool estimator_task_update(app_data_t *data, uint32_t now, float cc_dt_s)
{
#if !AMS_ENABLE_HIL_CAN
    (void)cc_dt_s;
#endif
    if ((data == NULL) || (data->estimator.enabled == 0U) ||
        (data->estimator.instance_count == 0U))
    {
        return false;
    }

#if AMS_ENABLE_HIL_CAN
    ams_hil_meas_t hil_measurement;
    taskENTER_CRITICAL();
    hil_measurement = data->hil.meas;
    taskEXIT_CRITICAL();
    bool use_hil = hil_meas_fresh(&hil_measurement, now);
#else
    const bool use_hil = false;
#endif

    if(use_hil)
    {
#if AMS_ENABLE_HIL_CAN
        bool new_hil_epoch = (data->estimator.hil_counter_seen == 0u) ||
                             (hil_measurement.counter !=
                              data->estimator.last_hil_counter) ||
                             (hil_measurement.last_rx_tick !=
                              data->estimator.last_hil_tick);
        if(!new_hil_epoch)
        {
            estimator_saturating_add(
                &data->estimator.repeated_measurement_count, 1u);
            return !data->estimator_fault;
        }

        float dt_s = cc_dt_s;
        bool dt_valid = true;
        if(data->estimator.hil_counter_seen != 0u)
        {
            dt_valid = estimator_dt_from_ticks(hil_measurement.last_rx_tick,
                                               data->estimator.last_hil_tick,
                                               &dt_s);
        }
        else
        {
            if(!isfinite(dt_s) || (dt_s < 0.001f) || (dt_s > 1.0f))
            {
                dt_s = AMS_EKF_DEFAULT_DT_S;
            }
        }

        data->estimator.hil_counter_seen = 1u;
        data->estimator.last_hil_counter = hil_measurement.counter;
        data->estimator.last_hil_tick = hil_measurement.last_rx_tick;

        if(!dt_valid)
        {
            estimator_saturating_add(
                &data->estimator.epoch_timing_fault_count, 1u);
            estimator_mark_instances_fault(&data->estimator,
                                           AMS_EKF_FAULT_EPOCH_TIMING);
            for(uint8_t i = 0u;
                (i < data->estimator.instance_count) &&
                (i < AMS_EKF_MAX_INSTANCES);
                i++)
            {
                if(data->estimator.inst[i].cfg.enabled != 0u)
                {
                    estimator_record_unusable_soh_epoch(
                        &data->estimator,
                        i,
                        hil_measurement.i_pack_A,
                        (uint32_t)hil_measurement.counter,
                        hil_measurement.last_rx_tick,
                        true,
                        true);
                }
            }
        }
        else
        {
            (void)ams_estimator_cc_step(&data->estimator,
                                        hil_measurement.i_pack_A,
                                        dt_s);
            for(uint8_t i = 0u;
                (i < data->estimator.instance_count) &&
                (i < AMS_EKF_MAX_INSTANCES);
                i++)
            {
                ams_ekf_instance_t *inst = &data->estimator.inst[i];
                if(inst->cfg.enabled == 0u)
                {
                    continue;
                }

                if((inst->cfg.first_series_group == 0u) &&
                   (inst->cfg.series_group_count == AMS_EKF_PACK_SERIES_GROUPS))
                {
                    (void)estimator_step_with_soh(
                        &data->estimator,
                        i,
                        hil_measurement.i_pack_A,
                        hil_measurement.v_pack_V,
                        hil_measurement.t_surf_C,
                        dt_s,
                        (uint32_t)hil_measurement.counter,
                        hil_measurement.last_rx_tick,
                        true,
                        true,
                        true);
                    inst->last_measurement_sequence =
                        (uint32_t)hil_measurement.counter;
                    inst->last_voltage_tick = hil_measurement.last_rx_tick;
                }
                else
                {
                    inst->valid = 0u;
                    inst->fault_flags = AMS_EKF_FAULT_STALE_INPUT;
                    estimator_record_unusable_soh_epoch(
                        &data->estimator,
                        i,
                        hil_measurement.i_pack_A,
                        (uint32_t)hil_measurement.counter,
                        hil_measurement.last_rx_tick,
                        true,
                        true);
                }
            }
        }

        ams_estimator_refresh_summary(&data->estimator,
                                      AMS_ESTIMATOR_INPUT_HIL_CAN,
                                      now);
        data->estimator_fault =
            (data->estimator.fault_flags != AMS_EKF_FAULT_NONE);
        return !data->estimator_fault;
#endif
    }

    /* The estimator task is the sole caller in production. Keep the large
     * immutable epoch copy in static task-owned RAM rather than consuming a
     * substantial fraction of the 4 KiB estimator stack before entering the
     * EKF call chain. */
    static ams_measurement_snapshot_t measurement;
    if(!ams_measurement_store_copy_latest(&data->measurement_store,
                                          &measurement))
    {
        estimator_mark_instances_fault(&data->estimator,
                                       AMS_EKF_FAULT_STALE_INPUT);
        ams_estimator_refresh_summary(&data->estimator,
                                      AMS_ESTIMATOR_INPUT_HARDWARE,
                                      now);
        data->estimator_fault = true;
        return false;
    }

    /* A stopped ADBMS publisher must not leave the last advisory estimate
     * healthy forever merely because the static snapshot remains readable. */
    if((uint32_t)(now - measurement.publication_tick) >
       ESTIMATOR_HW_TIMEOUT_TICKS)
    {
        estimator_mark_instances_fault(&data->estimator,
                                       AMS_EKF_FAULT_STALE_INPUT);
        ams_estimator_refresh_summary(&data->estimator,
                                      AMS_ESTIMATOR_INPUT_HARDWARE,
                                      now);
        data->estimator_fault = true;
        return false;
    }

    if(measurement.sequence ==
       data->estimator.last_consumed_measurement_sequence)
    {
        estimator_saturating_add(&data->estimator.repeated_measurement_count,
                                 1u);
        return !data->estimator_fault;
    }

    uint32_t sequence_distance = 1u;
    if(data->estimator.last_consumed_measurement_sequence != 0u)
    {
        sequence_distance = estimator_sequence_distance(
            measurement.sequence,
            data->estimator.last_consumed_measurement_sequence);
        if(sequence_distance > 1u)
        {
            estimator_saturating_add(&data->estimator.missed_measurement_count,
                                     sequence_distance - 1u);
        }
    }

    float dt_s = AMS_EKF_DEFAULT_DT_S;
    bool dt_valid = true;
    if(data->estimator.last_consumed_measurement_sequence != 0u)
    {
        dt_valid = estimator_dt_from_ticks(measurement.voltage_complete_tick,
                                           data->estimator.last_voltage_tick,
                                           &dt_s);
    }

    bool current_contiguous = measurement.current.valid;
    double charge_delta_As = measurement.current.charge_As;
    if(data->estimator.current_total_initialized != 0u)
    {
        charge_delta_As = measurement.current.total_charge_As -
                          data->estimator.last_current_total_charge_As;
        current_contiguous = current_contiguous &&
            (measurement.current.total_invalid_sample_count ==
             data->estimator.last_current_total_invalid_sample_count);
    }

    data->estimator.last_current_total_charge_As =
        measurement.current.total_charge_As;
    data->estimator.last_current_total_invalid_sample_count =
        measurement.current.total_invalid_sample_count;
    data->estimator.current_total_initialized = 1u;

    bool charge_valid = current_contiguous && isfinite(charge_delta_As);
    if(charge_valid)
    {
        charge_valid = ams_estimator_cc_apply_charge(&data->estimator,
                                                     charge_delta_As);
    }

    float i_pack_A = measurement.current.average_A;
    if((data->estimator.last_consumed_measurement_sequence != 0u) &&
       dt_valid && isfinite(charge_delta_As))
    {
        i_pack_A = (float)(charge_delta_As / (double)dt_s);
    }

    bool hardware_inputs_ready =
        ((measurement.validity_flags & AMS_MEAS_VALID_VOLTAGE) != 0u) &&
        ((measurement.validity_flags & AMS_MEAS_VALID_TEMPERATURE) != 0u) &&
        ((measurement.validity_flags & AMS_MEAS_VALID_CURRENT) != 0u) &&
        ((measurement.validity_flags & AMS_MEAS_BALANCE_RECOVERED) != 0u) &&
        dt_valid && charge_valid && isfinite(i_pack_A);
    const bool balance_recovered =
        ((measurement.validity_flags & AMS_MEAS_BALANCE_RECOVERED) != 0u);
    const bool current_calibration_confident =
        (AMS_CURRENT_POLARITY_VALIDATED != 0) &&
        (AMS_CURRENT_CALIBRATION_VALIDATED != 0) &&
        ((measurement.validity_flags & AMS_MEAS_CURRENT_TIMING_VALID) != 0u) &&
        measurement.current.calibration_record_confident &&
        (measurement.current.calibration_id != 0u);

    if(!dt_valid)
    {
        estimator_saturating_add(&data->estimator.epoch_timing_fault_count, 1u);
    }

    for(uint8_t i = 0u;
        (i < data->estimator.instance_count) && (i < AMS_EKF_MAX_INSTANCES);
        i++)
    {
        ams_ekf_instance_t *inst = &data->estimator.inst[i];
        if(inst->cfg.enabled == 0u)
        {
            continue;
        }

        float v_meas_V = 0.0f;
        float t_surf_C = 25.0f;
        uint16_t valid_voltage_count = 0u;
        bool voltage_ok = collect_group_voltage(&measurement,
                                                &inst->cfg,
                                                &v_meas_V,
                                                &valid_voltage_count);
        bool temp_ok = collect_group_temp(&measurement,
                                          &inst->cfg,
                                          &t_surf_C);

        if(hardware_inputs_ready && voltage_ok && temp_ok)
        {
            (void)estimator_step_with_soh(
                &data->estimator,
                i,
                i_pack_A,
                v_meas_V,
                t_surf_C,
                dt_s,
                measurement.sequence,
                measurement.voltage_complete_tick,
                true,
                balance_recovered,
                current_calibration_confident);
        }
        else
        {
            inst->valid = 0u;
            inst->fault_flags = dt_valid ? AMS_EKF_FAULT_BAD_INPUT :
                                           AMS_EKF_FAULT_EPOCH_TIMING;
            estimator_record_unusable_soh_epoch(
                &data->estimator,
                i,
                i_pack_A,
                measurement.sequence,
                measurement.voltage_complete_tick,
                balance_recovered,
                current_calibration_confident);
        }
        inst->last_measurement_sequence = measurement.sequence;
        inst->last_voltage_tick = measurement.voltage_complete_tick;
    }

    data->estimator.last_consumed_measurement_sequence = measurement.sequence;
    data->estimator.last_voltage_tick = measurement.voltage_complete_tick;
    ams_estimator_refresh_summary(&data->estimator,
                                  AMS_ESTIMATOR_INPUT_HARDWARE,
                                  now);
    data->estimator_fault =
        (data->estimator.fault_flags != AMS_EKF_FAULT_NONE);
    return !data->estimator_fault;
}

void estimator_task_fn(void *argument)
{
    app_data_t *data = (app_data_t *)argument;
    if (data == NULL)
    {
        vTaskDelete(NULL);
        return;
    }

    ams_estimator_init_default(&data->estimator);

    uint32_t last_entry = osKernelGetTickCount();
    uint32_t entry;
    for (;;)
    {
        entry = osKernelGetTickCount();
        float cc_dt_s = (float)(entry - last_entry) / 1000.0f;
        last_entry = entry;

        (void)estimator_task_update(data, entry, cc_dt_s);
        osDelayUntil(entry + (1000U / ESTIMATOR_FREQ));
    }
}
