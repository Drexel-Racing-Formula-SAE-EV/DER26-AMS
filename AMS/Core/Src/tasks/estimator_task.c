/*
 * estimator_task.c
 * Author: Mahad Faisal (2026)
 *
 * Runs the P42A DADEKF estimator and the SoH/finite-horizon SoP pipeline at
 * 10 Hz. The estimator never directly drives BMS_OK, AIRs, charging, shutdown,
 * or balancing. In a vehicle build its heartbeat and fail-zero power contract
 * are nevertheless required before an external ECU may request torque.
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

static bool collect_group_voltage_source(
    const ams_measurement_snapshot_t *measurement,
    const ams_ekf_config_t *cfg,
    uint8_t source,
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

        if ((seg >= NSMBS) || (cell >= NCELLS))
        {
            continue;
        }

        const uint16_t bit = (uint16_t)(1u << cell);
        uint16_t source_mv = 0u;
        bool source_valid = false;
        if(source == AMS_ESTIMATOR_VOLTAGE_SOURCE_AVG8)
        {
            source_valid = ((measurement->cell_avg8_usable_mask[seg] & bit) != 0u);
            source_mv = measurement->cell_avg8_mv[seg][cell];
        }
        else if(source == AMS_ESTIMATOR_VOLTAGE_SOURCE_IIR)
        {
            source_valid = ((measurement->cell_iir_usable_mask[seg] & bit) != 0u);
            source_mv = measurement->cell_iir_mv[seg][cell];
        }
        else
        {
            source_valid = ((measurement->cell_usable_mask[seg] & bit) != 0u);
            source_mv = measurement->cell_mv[seg][cell];
        }

        if(!source_valid)
        {
            continue;
        }

        float v = (float)source_mv / 1000.0f;
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

static void estimator_capture_voltage_products(
    ams_estimator_t *est,
    const ams_measurement_snapshot_t *measurement,
    uint8_t instance_index)
{
    if((est == NULL) || (measurement == NULL) ||
       (instance_index >= est->instance_count) ||
       (instance_index >= AMS_EKF_MAX_INSTANCES))
    {
        return;
    }

    const ams_ekf_config_t *cfg = &est->inst[instance_index].cfg;
    const uint16_t bit = (uint16_t)(1u << instance_index);
    float voltage = 0.0f;
    uint16_t count = 0u;

    est->voltage_raw_valid_mask &= (uint16_t)~bit;
    est->voltage_avg8_valid_mask &= (uint16_t)~bit;
    est->voltage_iir_valid_mask &= (uint16_t)~bit;

    if(collect_group_voltage_source(measurement, cfg,
                                    AMS_ESTIMATOR_VOLTAGE_SOURCE_RAW,
                                    &voltage, &count))
    {
        est->voltage_raw_valid_mask |= bit;
    }
    est->voltage_raw_V[instance_index] = voltage;
    est->voltage_raw_valid_count[instance_index] = count;

    voltage = 0.0f;
    count = 0u;
    if(collect_group_voltage_source(measurement, cfg,
                                    AMS_ESTIMATOR_VOLTAGE_SOURCE_AVG8,
                                    &voltage, &count))
    {
        est->voltage_avg8_valid_mask |= bit;
    }
    est->voltage_avg8_V[instance_index] = voltage;
    est->voltage_avg8_valid_count[instance_index] = count;

    voltage = 0.0f;
    count = 0u;
    if(collect_group_voltage_source(measurement, cfg,
                                    AMS_ESTIMATOR_VOLTAGE_SOURCE_IIR,
                                    &voltage, &count))
    {
        est->voltage_iir_valid_mask |= bit;
    }
    est->voltage_iir_V[instance_index] = voltage;
    est->voltage_iir_valid_count[instance_index] = count;
}

static bool estimator_selected_voltage(
    const ams_estimator_t *est,
    uint8_t instance_index,
    float *voltage_V,
    uint16_t *valid_count)
{
    if((est == NULL) || (voltage_V == NULL) || (valid_count == NULL) ||
       (instance_index >= est->instance_count) ||
       (instance_index >= AMS_EKF_MAX_INSTANCES))
    {
        return false;
    }

    const uint16_t bit = (uint16_t)(1u << instance_index);
#if AMS_ESTIMATOR_VOLTAGE_SOURCE == AMS_ESTIMATOR_VOLTAGE_SOURCE_AVG8
    *voltage_V = est->voltage_avg8_V[instance_index];
    *valid_count = est->voltage_avg8_valid_count[instance_index];
    return ((est->voltage_avg8_valid_mask & bit) != 0u);
#elif AMS_ESTIMATOR_VOLTAGE_SOURCE == AMS_ESTIMATOR_VOLTAGE_SOURCE_IIR
    *voltage_V = est->voltage_iir_V[instance_index];
    *valid_count = est->voltage_iir_valid_count[instance_index];
    return ((est->voltage_iir_valid_mask & bit) != 0u);
#else
    *voltage_V = est->voltage_raw_V[instance_index];
    *valid_count = est->voltage_raw_valid_count[instance_index];
    return ((est->voltage_raw_valid_mask & bit) != 0u);
#endif
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

static void estimator_publish_power_snapshot(app_data_t *data)
{
    if(data == NULL)
    {
        return;
    }

    const ams_power_can_snapshot_t snapshot =
        data->power_state.can_snapshot;
    taskENTER_CRITICAL();
    data->power_can_snapshot = snapshot;
    data->power_limit_fault =
        (snapshot.valid == 0u) || (snapshot.authority_valid == 0u);
    taskEXIT_CRITICAL();
}

static uint16_t estimator_count_bits32(uint32_t value)
{
    uint16_t count = 0u;
    while(value != 0u)
    {
        count = (uint16_t)(count + (uint16_t)(value & 1u));
        value >>= 1u;
    }
    return count;
}

/* The estimator task is the sole writer. The CAN task pins one published
 * buffer while copying, so construction is lock-free and publication needs
 * only a short pointer/index swap. A busy inactive buffer drops diagnostics. */
static void estimator_publish_tuning_snapshot(
    app_data_t *data,
    const ams_measurement_snapshot_t *measurement,
    uint32_t now)
{
#if AMS_ENABLE_TUNING_CAN
    if((data == NULL) || (measurement == NULL))
    {
        return;
    }

    ams_tuning_store_t *store = &data->tuning_store;
    uint8_t write_index;
    taskENTER_CRITICAL();
    write_index = store->published ?
        (uint8_t)(store->published_index ^ 1u) : 0u;
    if(store->reader_count[write_index] != 0u)
    {
        if(store->publication_drop_count != UINT32_MAX)
        {
            store->publication_drop_count++;
        }
        taskEXIT_CRITICAL();
        return;
    }
    taskEXIT_CRITICAL();

    ams_tuning_snapshot_t *out = &store->buffer[write_index];
    memset(out, 0, sizeof(*out));
    out->snapshot_sequence = store->next_sequence;
    out->measurement_sequence = measurement->sequence;
    out->estimator_step = data->estimator.step_count;
    out->source_tick_ms = now;
    out->instance_count = data->estimator.instance_count;
    if(out->instance_count > AMS_EKF_MAX_INSTANCES)
    {
        out->instance_count = AMS_EKF_MAX_INSTANCES;
    }

    for(uint8_t i = 0u; i < out->instance_count; i++)
    {
        const ams_ekf_instance_t *ekf = &data->estimator.inst[i];
        const ams_resistance_soh_t *soh =
            &data->estimator.resistance_soh[i];
        ams_tuning_segment_t *seg = &out->segment[i];
        seg->soc = ekf->soc;
        seg->vp1_v = ekf->vp1_V;
        seg->vp2_v = ekf->vp2_V;
        seg->r0_ohm = ekf->r0_ohm;
        seg->t_core_c = ekf->t_core_C;
        seg->p_soc = ekf->p_soc;
        seg->p_vp1 = ekf->p_vp1;
        seg->p_vp2 = ekf->p_vp2;
        seg->p_r0 = ekf->p_r0;
        seg->r_meas_v2 = ekf->r_meas_V2;
        seg->v_pred_v = ekf->v_pred_V;
        seg->innovation_v = ekf->innovation_V;
        seg->measured_v = ekf->last_v_meas_V;
        seg->current_a = ekf->last_i_pack_A;
        seg->surface_temp_c = ekf->last_t_surf_C;
        seg->step_count = ekf->step_count;
        seg->innovation_reject_count = ekf->innovation_reject_count;
        seg->dt_clamp_count = ekf->dt_clamp_count;
        seg->fault_flags = ekf->fault_flags;
        seg->measurement_sequence = ekf->last_measurement_sequence;
        seg->current_sequence = measurement->current.sequence;
        seg->measurement_age_ms = now - measurement->publication_tick;
        seg->current_age_ms = now - measurement->current.latest_sample_tick;
        seg->model_domain_flags = ekf->model_domain_flags;
        seg->valid = ekf->valid;

        const uint16_t bit = (uint16_t)(1u << i);
        if((data->estimator.voltage_raw_valid_mask & bit) != 0u)
        {
            seg->voltage_valid_flags |= 0x01u;
            seg->voltage_raw_v = data->estimator.voltage_raw_V[i];
        }
        if((data->estimator.voltage_avg8_valid_mask & bit) != 0u)
        {
            seg->voltage_valid_flags |= 0x02u;
            seg->voltage_avg8_v = data->estimator.voltage_avg8_V[i];
        }
        if((data->estimator.voltage_iir_valid_mask & bit) != 0u)
        {
            seg->voltage_valid_flags |= 0x04u;
            seg->voltage_iir_v = data->estimator.voltage_iir_V[i];
        }
        if(i < NSMBS)
        {
            seg->fresh_temp_count =
                estimator_count_bits32(measurement->temp_usable_mask[i]);
        }

        seg->reference_r0_ohm = soh->reference_cell_r0_ohm;
        seg->resistance_growth_ratio = soh->resistance_growth_ratio;
        seg->r0_variance_ohm2 = soh->r0_variance_ohm2;
        seg->soh_reject_flags = soh->last_reject_flags;
        seg->soh_confidence_pct = soh->observation_confidence_pct;
        seg->soh_status_flags = soh->status_flags;
        seg->soh_accepted_count = (soh->accepted_count > 255u) ?
                                  255u : (uint8_t)soh->accepted_count;
        seg->soh_rejected_count = (soh->rejected_count > 255u) ?
                                  255u : (uint8_t)soh->rejected_count;
    }

    const ams_power_state_t *power = &data->power_state;
    out->reason_flags = power->published_result.reason_flags;
    out->power_valid = power->published_result.valid;
    out->power_authority_valid = power->published_result.authority_valid;
    for(uint8_t h = 0u; h < AMS_SOP_HORIZONS; h++)
    {
        ams_tuning_sop_horizon_t *dst = &out->horizon[h];
        dst->raw_model_discharge_a =
            power->raw_result.model_discharge_current_a[h];
        dst->raw_model_charge_a = power->raw_result.model_charge_current_a[h];
        dst->strategy_discharge_a =
            power->strategy_limited_result.discharge_current_a[h];
        dst->strategy_charge_a =
            power->strategy_limited_result.charge_current_a[h];
        dst->final_discharge_a = power->published_result.discharge_current_a[h];
        dst->final_charge_a = power->published_result.charge_current_a[h];
        dst->discharge_power_w = power->published_result.discharge_power_w[h];
        dst->charge_power_w = power->published_result.charge_power_w[h];
        dst->discharge_min_cell_v =
            power->published_result.discharge_extrema[h].minimum_cell_voltage_v;
        dst->charge_max_cell_v =
            power->published_result.charge_extrema[h].maximum_cell_voltage_v;
        dst->discharge_binding = power->published_result.discharge_binding[h];
        dst->charge_binding = power->published_result.charge_binding[h];
        dst->discharge_segment =
            power->published_result.discharge_limiting_segment[h];
        dst->discharge_cell = power->published_result.discharge_limiting_cell[h];
        dst->charge_segment = power->published_result.charge_limiting_segment[h];
        dst->charge_cell = power->published_result.charge_limiting_cell[h];
        out->fuse_cap_a[h] = power->fuse_result.discharge_current_cap_a[h];
        out->hardware_discharge_cap_a[h] =
            power->sop_config.discharge_current_max_a[h];
    }
    out->fuse_utilization = power->fuse_result.utilization;
    out->fuse_temperature_c = power->fuse_result.estimated_fuse_temperature_c;
    out->fuse_derating = power->fuse_result.temperature_derating;
    out->fuse_effective_current_a = power->fuse_result.effective_current_a;
    out->fuse_equivalent_current_a = power->fuse_result.equivalent_25c_current_a;
    out->fuse_typical_melt_time_s = power->fuse_result.typical_melt_time_s;
    out->fuse_usable_melt_time_s = power->fuse_result.usable_melt_time_s;
    out->fuse_reason_flags = power->fuse_result.reason_flags;
    out->fuse_valid = power->fuse_result.valid;
    out->fuse_authority_valid = power->fuse_result.authority_valid;
    out->fuse_budget_exhausted = power->fuse_result.budget_exhausted;

    taskENTER_CRITICAL();
    store->published_index = write_index;
    store->published = true;
    store->next_sequence++;
    taskEXIT_CRITICAL();
#else
    (void)data;
    (void)measurement;
    (void)now;
#endif
}

static void estimator_invalidate_power(app_data_t *data,
                                       uint32_t now,
                                       uint32_t reason_flags)
{
    if(data == NULL)
    {
        return;
    }
    ams_power_state_invalidate(&data->power_state, now, reason_flags);
    estimator_publish_power_snapshot(data);
}

static bool estimator_update_power(app_data_t *data,
                                   const ams_measurement_snapshot_t *measurement,
                                   uint32_t now,
                                   float elapsed_s)
{
    if((data == NULL) || (measurement == NULL))
    {
        return false;
    }

    if(!ams_sop_config_valid(&data->power_state.sop_config) ||
       !ams_soh_config_valid(&data->power_state.soh_config))
    {
        ams_power_state_init(&data->power_state);
    }

    ams_power_policy_t policy;
    memset(&policy, 0, sizeof(policy));
    taskENTER_CRITICAL();
    const state_t state = data->state;
    const bool bms_permitted = data->bms_state &&
                               !data->bms_output_inhibit &&
                               !data->hard_fault;
    const bool charger_healthy = !data->charger_fault;
#if AMS_ENABLE_MISSION_CAN
    const ams_mission_request_state_t mission_request =
        data->mission_request;
#endif
    taskEXIT_CRITICAL();

    if(state == STATE_DISCARGE)
    {
        policy.operating_mode = AMS_SOP_MODE_DRIVE;
        policy.discharge_authorized = bms_permitted ? 1u : 0u;
        policy.regen_authorized =
            (bms_permitted && (AMS_REGEN_TARGET_VALIDATED != 0)) ? 1u : 0u;
    }
    else if(state == STATE_CHARGE)
    {
        policy.operating_mode = AMS_SOP_MODE_CHARGE;
        policy.charger_authorized =
            (bms_permitted && charger_healthy) ? 1u : 0u;
    }
    else
    {
        policy.operating_mode = AMS_SOP_MODE_IDLE;
    }
    policy.current_calibrated =
        (AMS_CURRENT_CALIBRATION_VALIDATED != 0) ? 1u : 0u;
    policy.current_polarity_validated =
        (AMS_CURRENT_POLARITY_VALIDATED != 0) ? 1u : 0u;
    policy.fuse_model_validated =
        (AMS_FUSE_MODEL_VALIDATED != 0) ? 1u : 0u;
#if AMS_ENABLE_MISSION_CAN
    policy.requested_mission = mission_request.requested_profile;
    policy.mission_request_valid =
        ams_mission_request_fresh(&mission_request, now) ? 1u : 0u;
    policy.stationary_confirmed = mission_request.stationary_confirmed;
#else
    policy.requested_mission = AMS_MISSION_ENDURANCE;
    policy.mission_request_valid = 0u;
    policy.stationary_confirmed = 0u;
#endif

    const bool valid = ams_power_state_update(&data->power_state,
                                               measurement,
                                               &data->estimator,
                                               &policy,
                                               now,
                                               elapsed_s);
    estimator_publish_power_snapshot(data);
    return valid;
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
        data->estimator.voltage_compare_sequence = 0u;
        data->estimator.voltage_raw_valid_mask = 0u;
        data->estimator.voltage_avg8_valid_mask = 0u;
        data->estimator.voltage_iir_valid_mask = 0u;
        bool new_hil_epoch = (data->estimator.hil_counter_seen == 0u) ||
                             (hil_measurement.counter !=
                              data->estimator.last_hil_counter) ||
                             (hil_measurement.last_rx_tick !=
                              data->estimator.last_hil_tick);
        if(!new_hil_epoch)
        {
            estimator_saturating_add(
                &data->estimator.repeated_measurement_count, 1u);
            if((uint32_t)(now - data->power_can_snapshot.measurement_timestamp_ms) >
               (uint32_t)data->power_state.sop_config.max_measurement_age_ms)
            {
                estimator_invalidate_power(data, now,
                    AMS_SOP_REASON_MEASUREMENT_STALE |
                    AMS_SOP_REASON_INCOMPLETE_TOPOLOGY);
            }
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
        /* The current HIL contract has one pack estimator frame. Keep the
         * authority output fail-zero until segment-state HIL frames exist. */
        estimator_invalidate_power(data, now,
            AMS_SOP_REASON_INCOMPLETE_TOPOLOGY);
        return !data->estimator_fault;
#endif
    }

    /* The estimator task is the sole caller in production. Keep the large
     * immutable epoch copy in static task-owned RAM rather than consuming a
     * substantial fraction of the 6 KiB estimator stack before entering the
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
        estimator_invalidate_power(data, now,
                                   AMS_SOP_REASON_MEASUREMENT_INVALID);
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
        estimator_invalidate_power(data, now,
                                   AMS_SOP_REASON_MEASUREMENT_STALE);
        return false;
    }

    if(measurement.sequence ==
       data->estimator.last_consumed_measurement_sequence)
    {
        estimator_saturating_add(&data->estimator.repeated_measurement_count,
                                 1u);
        if((uint32_t)(now - measurement.publication_tick) >
           (uint32_t)data->power_state.sop_config.max_measurement_age_ms)
        {
            estimator_invalidate_power(data, now,
                                       AMS_SOP_REASON_MEASUREMENT_STALE);
        }
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
        measurement.current.calibration_record_confident &&
        (measurement.current.calibration_id != 0u);

    if(!dt_valid)
    {
        estimator_saturating_add(&data->estimator.epoch_timing_fault_count, 1u);
    }

    data->estimator.voltage_compare_sequence = measurement.sequence;
    data->estimator.voltage_raw_valid_mask = 0u;
    data->estimator.voltage_avg8_valid_mask = 0u;
    data->estimator.voltage_iir_valid_mask = 0u;

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
        estimator_capture_voltage_products(&data->estimator, &measurement, i);
        bool voltage_ok = estimator_selected_voltage(&data->estimator, i,
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
    (void)estimator_update_power(data, &measurement, now, dt_s);
    estimator_publish_tuning_snapshot(data, &measurement, now);
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
    ams_power_state_init(&data->power_state);
    estimator_publish_power_snapshot(data);

    uint32_t last_entry = osKernelGetTickCount();
    uint32_t entry;
    for (;;)
    {
        entry = osKernelGetTickCount();
        float cc_dt_s = (float)(entry - last_entry) / 1000.0f;
        last_entry = entry;

        (void)estimator_task_update(data, entry, cc_dt_s);
        ams_heartbeat_kick(data, AMS_HEARTBEAT_ESTIMATOR,
                           osKernelGetTickCount());
        osDelayUntil(entry + (1000U / ESTIMATOR_FREQ));
    }
}
