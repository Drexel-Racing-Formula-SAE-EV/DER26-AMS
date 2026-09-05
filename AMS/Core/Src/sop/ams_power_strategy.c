#include "sop/ams_power_strategy.h"

#include "sop/ams_power_can.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

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

static bool valid_profile(uint8_t profile)
{
    return profile <= (uint8_t)AMS_MISSION_LIMP_HOME;
}

void ams_mission_request_init(ams_mission_request_state_t *state)
{
    if(state != NULL)
    {
        memset(state, 0, sizeof(*state));
        state->requested_profile = AMS_MISSION_ENDURANCE;
    }
}

void ams_mission_request_encode(ams_mission_profile_t profile,
                                uint8_t counter,
                                bool stationary_confirmed,
                                uint8_t payload[8])
{
    if(payload == NULL)
    {
        return;
    }
    memset(payload, 0, 8u);
    payload[0] = (uint8_t)((AMS_MISSION_PROTOCOL_VERSION << 4u) |
                           (counter & 0x0Fu));
    payload[1] = valid_profile((uint8_t)profile) ?
        (uint8_t)profile : (uint8_t)AMS_MISSION_ENDURANCE;
    payload[2] = stationary_confirmed ?
        AMS_MISSION_REQUEST_FLAG_STATIONARY : 0u;
    payload[7] = ams_power_can_crc8(AMS_MISSION_CAN_REQUEST_ID, payload);
}

bool ams_mission_request_ingest(ams_mission_request_state_t *state,
                                const uint8_t payload[8],
                                uint32_t now_ms)
{
    if((state == NULL) || (payload == NULL))
    {
        return false;
    }

    const uint8_t version = payload[0] >> 4u;
    const uint8_t counter = payload[0] & 0x0Fu;
    const uint8_t profile = payload[1];
    const bool reserved_clear = (payload[2] &
        (uint8_t)~AMS_MISSION_REQUEST_FLAG_STATIONARY) == 0u &&
        payload[3] == 0u && payload[4] == 0u &&
        payload[5] == 0u && payload[6] == 0u;
    const bool crc_valid = payload[7] ==
        ams_power_can_crc8(AMS_MISSION_CAN_REQUEST_ID, payload);
    const bool sequence_valid = (state->seen == 0u) ||
        (counter == (uint8_t)((state->counter + 1u) & 0x0Fu));

    if((version != AMS_MISSION_PROTOCOL_VERSION) ||
       !valid_profile(profile) || !reserved_clear || !crc_valid ||
       !sequence_valid)
    {
        state->valid = 0u;
        state->good_streak = 0u;
        if(state->rejected_count != UINT32_MAX)
        {
            state->rejected_count++;
        }
        return false;
    }

    if((state->seen != 0u) &&
       (state->requested_profile == (ams_mission_profile_t)profile))
    {
        if(state->good_streak < UINT8_MAX)
        {
            state->good_streak++;
        }
    }
    else
    {
        state->good_streak = 1u;
    }
    state->seen = 1u;
    state->counter = counter;
    state->last_rx_ms = now_ms;
    state->requested_profile = (ams_mission_profile_t)profile;
    state->stationary_confirmed =
        ((payload[2] & AMS_MISSION_REQUEST_FLAG_STATIONARY) != 0u) ? 1u : 0u;
    state->valid = (state->good_streak >= 2u) ? 1u : 0u;
    if(state->accepted_count != UINT32_MAX)
    {
        state->accepted_count++;
    }
    return state->valid != 0u;
}

bool ams_mission_request_fresh(const ams_mission_request_state_t *state,
                               uint32_t now_ms)
{
    return (state != NULL) && (state->valid != 0u) &&
           ((uint32_t)(now_ms - state->last_rx_ms) <=
            AMS_MISSION_REQUEST_MAX_AGE_MS);
}

void ams_power_strategy_default_config(ams_power_strategy_config_t *cfg)
{
    if(cfg == NULL)
    {
        return;
    }
    memset(cfg, 0, sizeof(*cfg));
    cfg->limp_trigger_soc_lower = 0.30f;
    cfg->limp_current_max_a = 35.0f;
    cfg->thermal_ready_target_c = 25.0f;
    cfg->core_capacity_j_per_k_per_cell = 55.0f;
    cfg->surface_capacity_j_per_k_per_cell = 15.0f;
    cfg->cells_per_segment = 90.0f;
    cfg->resistance_confidence_required_pct = 100u;
}

bool ams_power_strategy_config_valid(const ams_power_strategy_config_t *cfg)
{
    return (cfg != NULL) && isfinite(cfg->limp_trigger_soc_lower) &&
           (cfg->limp_trigger_soc_lower > 0.0f) &&
           (cfg->limp_trigger_soc_lower < 1.0f) &&
           isfinite(cfg->limp_current_max_a) &&
           (cfg->limp_current_max_a > 0.0f) &&
           isfinite(cfg->thermal_ready_target_c) &&
           isfinite(cfg->core_capacity_j_per_k_per_cell) &&
           (cfg->core_capacity_j_per_k_per_cell > 0.0f) &&
           isfinite(cfg->surface_capacity_j_per_k_per_cell) &&
           (cfg->surface_capacity_j_per_k_per_cell > 0.0f) &&
           isfinite(cfg->cells_per_segment) &&
           (cfg->cells_per_segment > 0.0f) &&
           (cfg->resistance_confidence_required_pct > 0u) &&
           (cfg->resistance_confidence_required_pct <= 100u);
}

void ams_power_strategy_init(ams_power_strategy_state_t *state)
{
    if(state != NULL)
    {
        memset(state, 0, sizeof(*state));
        state->active_profile = AMS_MISSION_ENDURANCE;
    }
}

static void cap_discharge(ams_sop_result_t *result,
                          float cap_a,
                          ams_sop_binding_t binding)
{
    for(uint8_t h = 0u; h < AMS_SOP_HORIZONS; h++)
    {
        if(result->discharge_current_a[h] > cap_a)
        {
            const float old_a = result->discharge_current_a[h];
            const float ratio = (old_a > 0.0f) ? cap_a / old_a : 0.0f;
            result->discharge_current_a[h] = cap_a;
            result->discharge_power_w[h] *= ratio;
            result->discharge_binding[h] = binding;
            result->discharge_limiting_segment[h] = AMS_SOP_INVALID_INDEX;
            result->discharge_limiting_cell[h] = AMS_SOP_INVALID_INDEX;
        }
    }
}

static bool cap_charge_horizon(ams_sop_result_t *result,
                               uint8_t horizon,
                               float cap_magnitude_a,
                               ams_sop_binding_t binding)
{
    const float old_magnitude_a = fmaxf(
        0.0f, -result->charge_current_a[horizon]);
    if(old_magnitude_a <= cap_magnitude_a)
    {
        return false;
    }

    const float ratio = (old_magnitude_a > 0.0f) ?
        cap_magnitude_a / old_magnitude_a : 0.0f;
    result->charge_current_a[horizon] = -cap_magnitude_a;
    result->charge_power_w[horizon] *= ratio;
    result->charge_binding[horizon] = binding;
    result->charge_limiting_segment[horizon] = AMS_SOP_INVALID_INDEX;
    result->charge_limiting_cell[horizon] = AMS_SOP_INVALID_INDEX;
    return true;
}

static bool strategy_input_valid(const ams_power_strategy_input_t *input)
{
    if((input == NULL) || !valid_profile((uint8_t)input->requested_profile) ||
       !isfinite(input->minimum_segment_soc_lower) ||
       !isfinite(input->pack_current_a))
    {
        return false;
    }
    for(uint8_t segment = 0u; segment < AMS_SOP_SEGMENTS; segment++)
    {
        if(!isfinite(input->segment_core_temp_c[segment]) ||
           !isfinite(input->segment_surface_temp_c[segment]) ||
           !isfinite(input->segment_r0_ohm[segment]) ||
           (input->segment_r0_ohm[segment] <= 0.0f))
        {
            return false;
        }
    }
    return true;
}

bool ams_power_strategy_update(ams_power_strategy_state_t *state,
                               const ams_power_strategy_config_t *cfg,
                               const ams_power_strategy_input_t *input,
                               const ams_fuse_observer_result_t *fuse,
                               const ams_sop_result_t *hard_result,
                               ams_sop_result_t *limited_result,
                               ams_power_strategy_result_t *strategy_result)
{
    if((state == NULL) || !ams_power_strategy_config_valid(cfg) ||
       !strategy_input_valid(input) || (hard_result == NULL) ||
       (limited_result == NULL) || (strategy_result == NULL) ||
       (hard_result->valid == 0u) || (hard_result->authority_valid == 0u))
    {
        if(limited_result != NULL)
        {
            memset(limited_result, 0, sizeof(*limited_result));
            limited_result->fallback_active = 1u;
        }
        if(strategy_result != NULL)
        {
            memset(strategy_result, 0, sizeof(*strategy_result));
            strategy_result->active_profile = AMS_MISSION_ENDURANCE;
            strategy_result->reason_flags = AMS_STRATEGY_REASON_REQUEST_INVALID;
        }
        return false;
    }

    *limited_result = *hard_result;
    memset(strategy_result, 0, sizeof(*strategy_result));
    if(state->update_count != UINT32_MAX)
    {
        state->update_count++;
    }

    if(input->minimum_segment_soc_lower <= cfg->limp_trigger_soc_lower)
    {
        state->limp_latched = 1u;
    }

    if(state->limp_latched != 0u)
    {
        state->active_profile = AMS_MISSION_LIMP_HOME;
        strategy_result->reason_flags |= AMS_STRATEGY_REASON_AUTO_LIMP;
    }
    else if(input->request_valid == 0u)
    {
        state->active_profile = AMS_MISSION_ENDURANCE;
        strategy_result->reason_flags |= AMS_STRATEGY_REASON_REQUEST_STALE;
    }
    else if((input->requested_profile == AMS_MISSION_QUALIFY) &&
            (state->active_profile != AMS_MISSION_QUALIFY) &&
            (input->stationary_confirmed == 0u))
    {
        state->active_profile = AMS_MISSION_ENDURANCE;
        strategy_result->reason_flags |= AMS_STRATEGY_REASON_QUALIFY_BLOCKED;
    }
    else
    {
        state->active_profile = input->requested_profile;
    }

    if((fuse != NULL) && (fuse->valid != 0u))
    {
        strategy_result->fuse_utilization = fuse->utilization;
        strategy_result->fuse_authority_valid = fuse->authority_valid;
        if(fuse->authority_valid != 0u)
        {
            for(uint8_t h = 0u; h < AMS_SOP_HORIZONS; h++)
            {
                if(limited_result->discharge_current_a[h] >
                   fuse->discharge_current_cap_a[h])
                {
                    const float old_a = limited_result->discharge_current_a[h];
                    const float cap_a = fuse->discharge_current_cap_a[h];
                    const float ratio = (old_a > 0.0f) ? cap_a / old_a : 0.0f;
                    limited_result->discharge_current_a[h] = cap_a;
                    limited_result->discharge_power_w[h] *= ratio;
                    limited_result->discharge_binding[h] =
                        AMS_SOP_BIND_FUSE_THERMAL;
                    limited_result->discharge_limiting_segment[h] =
                        AMS_SOP_INVALID_INDEX;
                    limited_result->discharge_limiting_cell[h] =
                        AMS_SOP_INVALID_INDEX;
                    limited_result->reason_flags |=
                        AMS_SOP_REASON_FUSE_DERATED;
                    strategy_result->reason_flags |=
                        AMS_STRATEGY_REASON_FUSE_DERATED;
                }
                if(cap_charge_horizon(
                       limited_result, h, fuse->charge_current_cap_a[h],
                       AMS_SOP_BIND_FUSE_THERMAL))
                {
                    limited_result->reason_flags |=
                        AMS_SOP_REASON_FUSE_DERATED;
                    strategy_result->reason_flags |=
                        AMS_STRATEGY_REASON_FUSE_DERATED;
                }
            }
        }
        else
        {
            limited_result->reason_flags |= AMS_SOP_REASON_FUSE_SHADOW;
            strategy_result->reason_flags |= AMS_STRATEGY_REASON_FUSE_SHADOW;
        }
    }

    float profile_cap_a = limited_result->discharge_current_a[1];
    strategy_result->recommended_horizon_index = 1u;
    if(state->active_profile == AMS_MISSION_ENDURANCE)
    {
        profile_cap_a = limited_result->discharge_current_a[3];
        strategy_result->recommended_horizon_index = 3u;
    }
    else if(state->active_profile == AMS_MISSION_LIMP_HOME)
    {
        profile_cap_a = fminf(limited_result->discharge_current_a[3],
                              cfg->limp_current_max_a);
        strategy_result->recommended_horizon_index = 3u;
    }

    const float before_profile_a = limited_result->discharge_current_a[0];
    cap_discharge(limited_result, profile_cap_a,
                  AMS_SOP_BIND_MISSION_PROFILE);
    if(limited_result->discharge_current_a[0] + 1.0e-3f < before_profile_a)
    {
        limited_result->reason_flags |= AMS_SOP_REASON_MISSION_DERATED;
        strategy_result->reason_flags |=
            AMS_STRATEGY_REASON_MISSION_DERATED;
    }
    if(state->active_profile == AMS_MISSION_LIMP_HOME)
    {
        limited_result->reason_flags |= AMS_SOP_REASON_LIMP_HOME;
    }

    strategy_result->minimum_core_temp_c = INFINITY;
    float energy_j = 0.0f;
    float pack_resistance_ohm = 0.0f;
    for(uint8_t segment = 0u; segment < AMS_SOP_SEGMENTS; segment++)
    {
        const float core_deficit = fmaxf(0.0f,
            cfg->thermal_ready_target_c -
            input->segment_core_temp_c[segment]);
        const float surface_deficit = fmaxf(0.0f,
            cfg->thermal_ready_target_c -
            input->segment_surface_temp_c[segment]);
        energy_j += cfg->cells_per_segment *
            (cfg->core_capacity_j_per_k_per_cell * core_deficit +
             cfg->surface_capacity_j_per_k_per_cell * surface_deficit);
        if(input->segment_core_temp_c[segment] <
           strategy_result->minimum_core_temp_c)
        {
            strategy_result->minimum_core_temp_c =
                input->segment_core_temp_c[segment];
        }
        /* The DADEKF R0 state is per cell; each 15s6p segment contributes
         * 15/6 times that value to pack resistance. */
        pack_resistance_ohm += 2.5f * input->segment_r0_ohm[segment];
    }
    strategy_result->thermal_energy_to_target_wh = energy_j / 3600.0f;
    strategy_result->thermal_ready =
        (energy_j <= 1.0f) ? 1u : 0u;
    if(strategy_result->thermal_ready == 0u)
    {
        strategy_result->reason_flags |= AMS_STRATEGY_REASON_THERMAL_NOT_READY;
    }

    strategy_result->estimated_self_heat_w =
        input->pack_current_a * input->pack_current_a * pack_resistance_ohm;
    strategy_result->estimated_self_heat_time_s =
        (strategy_result->estimated_self_heat_w > 1.0f) ?
        energy_j / strategy_result->estimated_self_heat_w : INFINITY;
    strategy_result->resistance_bootstrap_progress_pct =
        (uint8_t)clampf_local((float)input->resistance_confidence_pct,
                              0.0f, 100.0f);
    if(input->resistance_confidence_pct <
       cfg->resistance_confidence_required_pct)
    {
        strategy_result->reason_flags |= AMS_STRATEGY_REASON_R0_UNQUALIFIED;
    }
    strategy_result->active_profile = state->active_profile;
    strategy_result->recommended_discharge_current_a = profile_cap_a;
    strategy_result->limp_latched = state->limp_latched;
    return true;
}
