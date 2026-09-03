#include "sop/ams_power_state.h"

#include <math.h>
#include <string.h>

static uint32_t saturating_increment(uint32_t value)
{
    return (value == UINT32_MAX) ? UINT32_MAX : value + 1u;
}

static uint32_t next_generation(uint32_t generation)
{
    generation++;
    return (generation == 0u) ? 1u : generation;
}

static float clampf_local(float value, float lower, float upper)
{
    if(value < lower)
    {
        return lower;
    }
    if(value > upper)
    {
        return upper;
    }
    return value;
}

static bool estimator_has_segment_topology(const ams_estimator_t *estimator)
{
    if((estimator == NULL) ||
       (estimator->instance_count != AMS_SOP_SEGMENTS))
    {
        return false;
    }

    for(uint8_t segment = 0u; segment < AMS_SOP_SEGMENTS; segment++)
    {
        const ams_ekf_instance_t *instance = &estimator->inst[segment];
        if((instance->cfg.enabled == 0u) ||
           (instance->cfg.first_series_group !=
            (uint16_t)(segment * AMS_SOP_CELLS_PER_SEGMENT)) ||
           (instance->cfg.series_group_count !=
            AMS_SOP_CELLS_PER_SEGMENT))
        {
            return false;
        }
    }
    return true;
}

static bool snapshot_all_cells_and_temps(const ams_measurement_snapshot_t *m)
{
    if(m == NULL)
    {
        return false;
    }
    for(uint8_t segment = 0u; segment < AMS_SOP_SEGMENTS; segment++)
    {
        if((m->cell_usable_mask[segment] != AMS_SOP_FULL_CELL_MASK) ||
           ((m->temp_usable_mask[segment] & 0x00FFFFFFu) != 0x00FFFFFFu))
        {
            return false;
        }
    }
    return true;
}

static void snapshot_statistics(const ams_measurement_snapshot_t *measurement,
                                float *minimum_cell_v,
                                float *maximum_cell_v,
                                float *average_temp_c,
                                float segment_max_temp_c[AMS_SOP_SEGMENTS])
{
    float minimum_v = INFINITY;
    float maximum_v = -INFINITY;
    float temperature_sum = 0.0f;
    uint32_t temperature_count = 0u;

    for(uint8_t segment = 0u; segment < AMS_SOP_SEGMENTS; segment++)
    {
        segment_max_temp_c[segment] = -INFINITY;
        for(uint8_t cell = 0u; cell < AMS_SOP_CELLS_PER_SEGMENT; cell++)
        {
            const float voltage_v =
                (float)measurement->cell_mv[segment][cell] / 1000.0f;
            if(voltage_v < minimum_v)
            {
                minimum_v = voltage_v;
            }
            if(voltage_v > maximum_v)
            {
                maximum_v = voltage_v;
            }
        }
        for(uint8_t sensor = 0u; sensor < NTEMPS; sensor++)
        {
            const float temperature_c =
                (float)measurement->temp_deci_c[segment][sensor] / 10.0f;
            temperature_sum += temperature_c;
            temperature_count++;
            if(temperature_c > segment_max_temp_c[segment])
            {
                segment_max_temp_c[segment] = temperature_c;
            }
        }
    }

    *minimum_cell_v = minimum_v;
    *maximum_cell_v = maximum_v;
    *average_temp_c = (temperature_count > 0u) ?
        temperature_sum / (float)temperature_count : NAN;
}

static void build_soh_input(const ams_measurement_snapshot_t *measurement,
                            const ams_estimator_t *estimator,
                            const ams_power_policy_t *policy,
                            uint32_t now_ms,
                            float elapsed_s,
                            float min_cell_v,
                            float max_cell_v,
                            float average_temp_c,
                            ams_soh_input_t *input)
{
    memset(input, 0, sizeof(*input));
    input->measurement_sequence = measurement->sequence;
    input->measurement_timestamp_ms = measurement->publication_tick;
    input->now_ms = now_ms;
    input->elapsed_s = elapsed_s;
    input->pack_current_a = measurement->current.average_A;
    input->pack_current_uncertainty_a =
        (float)measurement->current.uncertainty_mA / 1000.0f;
    input->total_charge_as = measurement->current.total_charge_As;
    input->average_cell_temp_c = average_temp_c;
    input->cell_voltage_spread_v = max_cell_v - min_cell_v;
    input->measurement_valid =
        ((measurement->validity_flags &
          (AMS_MEAS_VALID_VOLTAGE | AMS_MEAS_VALID_TEMPERATURE |
           AMS_MEAS_VALID_CURRENT)) ==
         (AMS_MEAS_VALID_VOLTAGE | AMS_MEAS_VALID_TEMPERATURE |
          AMS_MEAS_VALID_CURRENT)) ? 1u : 0u;
    input->estimator_valid = (estimator->fault_flags == 0u) ? 1u : 0u;
    input->current_calibrated = policy->current_calibrated &&
        measurement->current.calibration_record_confident &&
        (measurement->current.calibration_id != 0u) &&
        (measurement->current.uncertainty_mA != 0u);
    input->current_polarity_validated = policy->current_polarity_validated;
    input->balance_recovered =
        ((measurement->validity_flags & AMS_MEAS_BALANCE_RECOVERED) != 0u) ?
        1u : 0u;

    float soc_sum = 0.0f;
    for(uint8_t segment = 0u; segment < AMS_SOH_SEGMENTS; segment++)
    {
        const ams_ekf_instance_t *instance = &estimator->inst[segment];
        const ams_resistance_soh_t *resistance =
            &estimator->resistance_soh[segment];
        soc_sum += instance->soc;
        const float soc_sigma = (isfinite(instance->p_soc) &&
                                 (instance->p_soc >= 0.0f)) ?
            sqrtf(instance->p_soc) : INFINITY;
        if(soc_sigma > input->maximum_soc_sigma)
        {
            input->maximum_soc_sigma = soc_sigma;
        }
        const float series_count =
            (float)instance->cfg.series_group_count;
        const float innovation_per_cell =
            (series_count > 0.0f) ?
                fabsf(instance->innovation_V) / series_count : INFINITY;
        if(innovation_per_cell >
           input->maximum_abs_innovation_v_per_cell)
        {
            input->maximum_abs_innovation_v_per_cell = innovation_per_cell;
        }
        const float polarization_v = fabsf(instance->vp1_V) +
                                     fabsf(instance->vp2_V);
        if(polarization_v > input->maximum_abs_polarization_v)
        {
            input->maximum_abs_polarization_v = polarization_v;
        }
        input->segment_resistance_growth_ratio[segment] =
            resistance->resistance_growth_ratio;
        input->segment_resistance_confidence_pct[segment] =
            resistance->observation_confidence_pct;
        input->segment_resistance_valid[segment] =
            (((resistance->status_flags & AMS_SOH_STATUS_ADVISORY_VALID) != 0u) &&
             ((resistance->status_flags & AMS_SOH_STATUS_LAST_OBSERVABLE) != 0u)) ?
            1u : 0u;
        if(instance->valid == 0u)
        {
            input->estimator_valid = 0u;
        }
    }
    input->pack_soc = soc_sum / (float)AMS_SOH_SEGMENTS;
}

static float segment_resistance_upper(const ams_power_state_t *state,
                                      uint8_t segment,
                                      uint8_t *valid)
{
    const uint8_t bit = (uint8_t)(1u << segment);
    const float retained_upper =
        state->soh.segment_resistance_growth_upper[segment];
    if((state->soh.segment_resistance_valid_mask & bit) == 0u ||
       !isfinite(retained_upper) || (retained_upper < 1.0f) ||
       (retained_upper > 3.0f))
    {
        *valid = 0u;
        return state->sop_config.default_resistance_soh_upper;
    }

    *valid = 1u;
    return retained_upper;
}

static void build_sop_input(const ams_power_state_t *state,
                            const ams_measurement_snapshot_t *measurement,
                            const ams_estimator_t *estimator,
                            const ams_power_policy_t *policy,
                            const float segment_max_temp_c[AMS_SOP_SEGMENTS],
                            uint32_t now_ms,
                            ams_sop_input_t *input)
{
    memset(input, 0, sizeof(*input));
    input->measurement_sequence = measurement->sequence;
    input->measurement_timestamp_ms = measurement->publication_tick;
    input->now_ms = now_ms;
    input->pack_current_a = measurement->current.average_A;
    input->pack_current_uncertainty_a =
        (float)measurement->current.uncertainty_mA / 1000.0f;
    input->operating_mode = policy->operating_mode;
    input->measurement_valid =
        snapshot_all_cells_and_temps(measurement) &&
        ((measurement->validity_flags &
          (AMS_MEAS_VALID_VOLTAGE | AMS_MEAS_VALID_TEMPERATURE |
           AMS_MEAS_VALID_CURRENT)) ==
         (AMS_MEAS_VALID_VOLTAGE | AMS_MEAS_VALID_TEMPERATURE |
          AMS_MEAS_VALID_CURRENT));
    input->estimator_segment_topology =
        estimator_has_segment_topology(estimator) ? 1u : 0u;
    input->estimator_valid =
        (input->estimator_segment_topology != 0u) &&
        (estimator->fault_flags == 0u);
    input->estimator_acquired = input->estimator_segment_topology;
    input->current_calibrated = policy->current_calibrated &&
        measurement->current.calibration_record_confident &&
        (measurement->current.calibration_id != 0u) &&
        (measurement->current.uncertainty_mA != 0u);
    input->current_polarity_validated = policy->current_polarity_validated;
    input->balance_recovered =
        ((measurement->validity_flags & AMS_MEAS_BALANCE_RECOVERED) != 0u) ?
        1u : 0u;
    input->discharge_authorized = policy->discharge_authorized;
    input->charger_authorized = policy->charger_authorized;
    input->regen_authorized = policy->regen_authorized;

    /* This hardware revision has no independent ambient sensor.  Using the
     * hottest measured surface as ambient starts with zero cooling benefit and
     * is deliberately conservative; the reason flag remains visible. */
    input->ambient_temp_c = segment_max_temp_c[0];
    for(uint8_t segment = 1u; segment < AMS_SOP_SEGMENTS; segment++)
    {
        if(segment_max_temp_c[segment] > input->ambient_temp_c)
        {
            input->ambient_temp_c = segment_max_temp_c[segment];
        }
    }
    input->ambient_measured = 0u;

    for(uint8_t segment = 0u; segment < AMS_SOP_SEGMENTS; segment++)
    {
        const ams_ekf_instance_t *instance = &estimator->inst[segment];
        ams_sop_segment_input_t *out = &input->segment[segment];
        out->soc = instance->soc;
        out->vp1_v = instance->vp1_V;
        out->vp2_v = instance->vp2_V;
        out->r0_ohm = instance->r0_ohm;
        out->core_temp_c = instance->t_core_C;
        out->surface_max_temp_c = segment_max_temp_c[segment];
        out->p_soc = instance->p_soc;
        out->p_vp1 = instance->p_vp1;
        out->p_vp2 = instance->p_vp2;
        out->p_r0 = instance->p_r0;
        out->innovation_v = instance->innovation_V;
        out->capacity_soh_lower = state->soh.result.capacity_soh_lower;
        out->capacity_soh_valid = state->soh.result.capacity_valid;
        out->resistance_soh_upper = segment_resistance_upper(
            state, segment,
            &out->resistance_soh_valid);
        out->cell_usable_mask = measurement->cell_usable_mask[segment];
        out->estimator_valid = instance->valid;
        out->model_domain_flags = instance->model_domain_flags;
        out->max_cell_age_ms = 0u;
        for(uint8_t cell = 0u; cell < AMS_SOP_CELLS_PER_SEGMENT; cell++)
        {
            out->cell_voltage_v[cell] =
                (float)measurement->cell_mv[segment][cell] / 1000.0f;
            if(measurement->cell_age_ms[segment][cell] > out->max_cell_age_ms)
            {
                out->max_cell_age_ms =
                    measurement->cell_age_ms[segment][cell];
            }
        }
        if(instance->valid == 0u)
        {
            input->estimator_valid = 0u;
        }
        if(instance->acquisition.state != AMS_EKF_ACQ_COMPLETE)
        {
            input->estimator_acquired = 0u;
        }
    }
}

static void refresh_can_snapshot(ams_power_state_t *state)
{
    ams_power_can_snapshot_t *can = &state->can_snapshot;
    const ams_sop_result_t *result = &state->published_result;
    const uint32_t generation = next_generation(can->generation);
    memset(can, 0, sizeof(*can));
    can->generation = generation;
    can->measurement_sequence = result->measurement_sequence;
    can->measurement_timestamp_ms = result->measurement_timestamp_ms;
    can->solve_timestamp_ms = result->solve_timestamp_ms;
    can->reason_flags = result->reason_flags;
    memcpy(can->discharge_current_a, result->discharge_current_a,
           sizeof(can->discharge_current_a));
    memcpy(can->charge_current_a, result->charge_current_a,
           sizeof(can->charge_current_a));
    can->discharge_power_w_1s = result->discharge_power_w[1];
    can->charge_power_w_1s = result->charge_power_w[1];
    can->capacity_soh = state->soh.result.capacity_soh;
    can->capacity_soh_lower = state->soh.result.capacity_soh_lower;
    can->resistance_growth_upper =
        state->soh.result.resistance_growth_upper;
    for(uint8_t horizon = 0u; horizon < AMS_SOP_HORIZONS; horizon++)
    {
        can->discharge_binding[horizon] =
            (uint8_t)result->discharge_binding[horizon];
        can->charge_binding[horizon] =
            (uint8_t)result->charge_binding[horizon];
        can->discharge_limiting_segment[horizon] =
            result->discharge_limiting_segment[horizon];
        can->charge_limiting_segment[horizon] =
            result->charge_limiting_segment[horizon];
    }
    can->capacity_confidence_pct =
        state->soh.result.capacity_confidence_pct;
    can->resistance_confidence_pct =
        state->soh.result.resistance_confidence_pct;
    can->capacity_valid = state->soh.result.capacity_valid;
    can->resistance_valid = state->soh.result.resistance_valid;
    can->fuse_utilization = state->strategy_result.fuse_utilization;
    can->minimum_core_temp_c = state->strategy_result.minimum_core_temp_c;
    can->thermal_energy_to_target_wh =
        state->strategy_result.thermal_energy_to_target_wh;
    can->strategy_reason_flags = state->strategy_result.reason_flags;
    can->mission_profile = (uint8_t)state->strategy_result.active_profile;
    can->mission_horizon_index =
        state->strategy_result.recommended_horizon_index;
    can->thermal_ready = state->strategy_result.thermal_ready;
    can->r0_bootstrap_progress_pct =
        state->strategy_result.resistance_bootstrap_progress_pct;
    can->fuse_authority_valid =
        state->strategy_result.fuse_authority_valid;
    can->limp_latched = state->strategy_result.limp_latched;
    can->valid = result->valid;
    can->authority_valid = result->authority_valid;
}

void ams_power_state_init(ams_power_state_t *state)
{
    if(state == NULL)
    {
        return;
    }
    memset(state, 0, sizeof(*state));
    ams_sop_default_config(&state->sop_config);
    ams_soh_default_config(&state->soh_config);
    ams_fuse_observer_default_config(&state->fuse_config);
    ams_power_strategy_default_config(&state->strategy_config);
    ams_soh_init(&state->soh, &state->soh_config);
    ams_fuse_observer_init(&state->fuse);
    ams_power_strategy_init(&state->strategy);
    ams_power_state_invalidate(state, 0u,
                               AMS_SOP_REASON_MEASUREMENT_INVALID);
}

static float minimum_segment_soc_lower(const ams_sop_input_t *input,
                                       const ams_sop_config_t *cfg)
{
    float minimum = 1.0f;
    for(uint8_t segment = 0u; segment < AMS_SOP_SEGMENTS; segment++)
    {
        const float sigma = sqrtf(fmaxf(0.0f,
            input->segment[segment].p_soc));
        const float lower = clampf_local(
            input->segment[segment].soc - cfg->sigma_multiplier * sigma,
            0.0f, 1.0f);
        if(lower < minimum)
        {
            minimum = lower;
        }
    }
    return minimum;
}

static float maximum_segment_soc_upper(const ams_sop_input_t *input,
                                       const ams_sop_config_t *cfg)
{
    float maximum = 0.0f;
    for(uint8_t segment = 0u; segment < AMS_SOP_SEGMENTS; segment++)
    {
        const float sigma = sqrtf(fmaxf(0.0f,
            input->segment[segment].p_soc));
        const float upper = clampf_local(
            input->segment[segment].soc + cfg->sigma_multiplier * sigma,
            0.0f, 1.0f);
        if(upper > maximum)
        {
            maximum = upper;
        }
    }
    return maximum;
}

static bool result_has_binding(const ams_sop_result_t *result,
                               ams_sop_binding_t binding,
                               bool discharge)
{
    if(result == NULL)
    {
        return false;
    }
    for(uint8_t h = 0u; h < AMS_SOP_HORIZONS; h++)
    {
        const ams_sop_binding_t actual = discharge ?
            result->discharge_binding[h] : result->charge_binding[h];
        if(actual == binding)
        {
            return true;
        }
    }
    return false;
}

static void update_soc_recovery(ams_power_state_t *state,
                                const ams_sop_input_t *input,
                                const ams_measurement_snapshot_t *measurement,
                                const ams_sop_result_t *previous,
                                ams_sop_recovery_context_t *context)
{
    memset(context, 0, sizeof(*context));
    const float minimum_soc = minimum_segment_soc_lower(input,
                                                        &state->sop_config);
    const float maximum_soc = maximum_segment_soc_upper(input,
                                                        &state->sop_config);
    const double total_charge_as = measurement->current.total_charge_As;

    if((state->discharge_soc_hold_active == 0u) &&
       result_has_binding(previous, AMS_SOP_BIND_SOC_LOW, true))
    {
        state->discharge_soc_hold_active = 1u;
        state->discharge_soc_hold_reference = minimum_soc;
        state->discharge_soc_hold_charge_as = total_charge_as;
    }
    if((state->charge_soc_hold_active == 0u) &&
       result_has_binding(previous, AMS_SOP_BIND_SOC_HIGH, false))
    {
        state->charge_soc_hold_active = 1u;
        state->charge_soc_hold_reference = maximum_soc;
        state->charge_soc_hold_charge_as = total_charge_as;
    }

    if((state->discharge_soc_hold_active != 0u) &&
       (minimum_soc >= state->discharge_soc_hold_reference +
                       state->sop_config.soc_recovery_delta) &&
       (total_charge_as <= state->discharge_soc_hold_charge_as -
                           state->sop_config.soc_recovery_charge_as))
    {
        context->discharge_soc_recovered = 1u;
        state->discharge_soc_hold_active = 0u;
    }
    if((state->charge_soc_hold_active != 0u) &&
       (maximum_soc <= state->charge_soc_hold_reference -
                       state->sop_config.soc_recovery_delta) &&
       (total_charge_as >= state->charge_soc_hold_charge_as +
                           state->sop_config.soc_recovery_charge_as))
    {
        context->charge_soc_recovered = 1u;
        state->charge_soc_hold_active = 0u;
    }

    context->fuse_state_valid = state->fuse_result.authority_valid;
    context->fuse_utilization = state->fuse_result.utilization;
}

static void build_strategy_input(const ams_power_state_t *state,
                                 const ams_sop_input_t *sop_input,
                                 const ams_power_policy_t *policy,
                                 ams_power_strategy_input_t *strategy_input)
{
    memset(strategy_input, 0, sizeof(*strategy_input));
    strategy_input->requested_profile = policy->requested_mission;
    strategy_input->request_valid = policy->mission_request_valid;
    strategy_input->stationary_confirmed = policy->stationary_confirmed;
    strategy_input->minimum_segment_soc_lower =
        minimum_segment_soc_lower(sop_input, &state->sop_config);
    strategy_input->pack_current_a = sop_input->pack_current_a;
    strategy_input->resistance_confidence_pct =
        state->soh.result.resistance_confidence_pct;
    for(uint8_t segment = 0u; segment < AMS_SOP_SEGMENTS; segment++)
    {
        strategy_input->segment_core_temp_c[segment] =
            sop_input->segment[segment].core_temp_c;
        strategy_input->segment_surface_temp_c[segment] =
            sop_input->segment[segment].surface_max_temp_c;
        strategy_input->segment_r0_ohm[segment] =
            sop_input->segment[segment].r0_ohm;
    }
}

void ams_power_state_invalidate(ams_power_state_t *state,
                                uint32_t now_ms,
                                uint32_t reason_flags)
{
    if(state == NULL)
    {
        return;
    }
    memset(&state->raw_result, 0, sizeof(state->raw_result));
    memset(&state->strategy_limited_result, 0,
           sizeof(state->strategy_limited_result));
    memset(&state->published_result, 0, sizeof(state->published_result));
    state->raw_result.solve_timestamp_ms = now_ms;
    state->raw_result.reason_flags = reason_flags;
    state->raw_result.fallback_active = 1u;
    state->published_result = state->raw_result;
    state->last_update_tick = now_ms;
    state->invalid_count = saturating_increment(state->invalid_count);
    refresh_can_snapshot(state);
}

bool ams_power_state_update(ams_power_state_t *state,
                            const ams_measurement_snapshot_t *measurement,
                            const ams_estimator_t *estimator,
                            const ams_power_policy_t *policy,
                            uint32_t now_ms,
                            float elapsed_s)
{
    if((state == NULL) || (measurement == NULL) || (estimator == NULL) ||
       (policy == NULL) || !isfinite(elapsed_s) || (elapsed_s <= 0.0f))
    {
        if(state != NULL)
        {
            ams_power_state_invalidate(state, now_ms,
                                       AMS_SOP_REASON_NUMERIC);
        }
        return false;
    }

    state->update_count = saturating_increment(state->update_count);
    state->last_update_tick = now_ms;

#if NSMBS < AMS_SOP_SEGMENTS
    /* The SoP/SoH state is explicitly a five-physical-segment model, while
     * BENCH/HIL may intentionally compile a one-SMB measurement store.  Never
     * index a smaller measurement snapshot as five segments merely to discover
     * later that the estimator topology is incomplete.  Fail closed before
     * touching any segment arrays. */
    ams_power_state_invalidate(state, now_ms,
                               AMS_SOP_REASON_INCOMPLETE_TOPOLOGY);
    return false;
#else
    if(!estimator_has_segment_topology(estimator))
    {
        ams_power_state_invalidate(state, now_ms,
                                   AMS_SOP_REASON_INCOMPLETE_TOPOLOGY);
        return false;
    }

    float minimum_cell_v;
    float maximum_cell_v;
    float average_temp_c;
    float segment_max_temp_c[AMS_SOP_SEGMENTS];
    snapshot_statistics(measurement, &minimum_cell_v, &maximum_cell_v,
                        &average_temp_c, segment_max_temp_c);

    ams_soh_input_t soh_input;
    build_soh_input(measurement, estimator, policy, now_ms, elapsed_s,
                    minimum_cell_v, maximum_cell_v, average_temp_c,
                    &soh_input);
    (void)ams_soh_update(&state->soh, &state->soh_config, &soh_input);

    ams_sop_input_t sop_input;
    build_sop_input(state, measurement, estimator, policy,
                    segment_max_temp_c, now_ms, &sop_input);

    ams_sop_result_t previous = state->published_result;
    const ams_sop_status_t status = ams_sop_solve(
        &sop_input, &state->sop_config, &state->raw_result);
    state->last_solver_status = (uint32_t)status;
    if(status == AMS_SOP_OK)
    {
        ams_fuse_observer_input_t fuse_input;
        memset(&fuse_input, 0, sizeof(fuse_input));
        fuse_input.pack_current_a = sop_input.pack_current_a;
        fuse_input.current_uncertainty_a =
            sop_input.pack_current_uncertainty_a;
        fuse_input.temperature_proxy_c = sop_input.ambient_temp_c;
        fuse_input.elapsed_s = elapsed_s;
        fuse_input.measurement_valid = sop_input.measurement_valid;
        fuse_input.current_calibrated = sop_input.current_calibrated;
        fuse_input.current_polarity_validated =
            sop_input.current_polarity_validated;
        fuse_input.temperature_measured_at_fuse = 0u;
        fuse_input.model_validated = policy->fuse_model_validated;
        (void)ams_fuse_observer_update(&state->fuse, &state->fuse_config,
                                      &state->sop_config, &fuse_input,
                                      &state->fuse_result);

        ams_sop_recovery_context_t recovery_context;
        update_soc_recovery(state, &sop_input, measurement, &previous,
                            &recovery_context);
        /* Schedule recovery on the physical envelope first, then apply the
         * subtractive fuse and mission caps as the final publication step.
         * This keeps Endurance tied to the actually published 30 s recovery
         * and prevents a stale/invalid fuse state from creating headroom. */
        ams_sop_apply_recovery(&state->raw_result, &previous,
                               &state->sop_config, &recovery_context,
                               elapsed_s, &state->published_result);

        ams_power_strategy_input_t strategy_input;
        build_strategy_input(state, &sop_input, policy, &strategy_input);
        if(!ams_power_strategy_update(&state->strategy,
                                      &state->strategy_config,
                                      &strategy_input,
                                      &state->fuse_result,
                                      &state->published_result,
                                      &state->strategy_limited_result,
                                      &state->strategy_result))
        {
            ams_power_state_invalidate(state, now_ms,
                                       AMS_SOP_REASON_NUMERIC);
            state->numeric_failure_count =
                saturating_increment(state->numeric_failure_count);
            return false;
        }
        if(policy->mission_request_valid == 0u)
        {
            state->strategy_limited_result.reason_flags |=
                AMS_SOP_REASON_MISSION_FALLBACK;
        }
        state->published_result = state->strategy_limited_result;
        state->valid_count = saturating_increment(state->valid_count);
        state->last_valid_tick = now_ms;
        refresh_can_snapshot(state);
        return true;
    }

    state->published_result = state->raw_result;
    state->invalid_count = saturating_increment(state->invalid_count);
    if(status == AMS_SOP_NUMERIC_FAILURE)
    {
        state->numeric_failure_count =
            saturating_increment(state->numeric_failure_count);
    }
    refresh_can_snapshot(state);
    return false;
#endif
}
