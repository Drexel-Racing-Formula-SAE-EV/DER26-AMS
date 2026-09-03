#include "sop/ams_sop.h"

#include "estimator/ams_estimator_lut.h"

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

typedef struct
{
    float soc;
    float vp1_v;
    float vp2_v;
    float core_temp_c;
    float surface_temp_c;
    float soc0;
    float ocv0_v;
    float docv_dsoc_v;
    float docv_dtemp_v_per_c;
    float r0_state_ohm;
    float r0_upper_ohm;
    float r1_ohm;
    float tau1_s;
    float capacity_as;
    float voltage_margin_v;
    float cell_bias_v[AMS_SOP_CELLS_PER_SEGMENT];
} ams_sop_model_segment_t;

typedef struct
{
    uint8_t feasible;
    ams_sop_binding_t binding;
    uint8_t segment;
    uint8_t cell;
    ams_sop_prediction_extrema_t extrema;
    uint32_t steps;
} ams_sop_feasibility_t;

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

static float minf_local(float a, float b)
{
    return (a < b) ? a : b;
}

static float maxf_local(float a, float b)
{
    return (a > b) ? a : b;
}

static bool finite_positive(float value)
{
    return isfinite(value) && (value > 0.0f);
}

static bool finite_nonnegative(float value)
{
    return isfinite(value) && (value >= 0.0f);
}

static uint32_t saturating_add_u32(uint32_t value, uint32_t increment)
{
    return ((UINT32_MAX - value) < increment) ? UINT32_MAX :
                                                   (value + increment);
}

static void result_make_zero(ams_sop_result_t *result)
{
    if(result == NULL)
    {
        return;
    }

    memset(result, 0, sizeof(*result));
    for(uint8_t h = 0u; h < AMS_SOP_HORIZONS; h++)
    {
        result->discharge_binding[h] = AMS_SOP_BIND_INVALID_INPUT;
        result->charge_binding[h] = AMS_SOP_BIND_INVALID_INPUT;
        result->discharge_limiting_segment[h] = AMS_SOP_INVALID_INDEX;
        result->discharge_limiting_cell[h] = AMS_SOP_INVALID_INDEX;
        result->charge_limiting_segment[h] = AMS_SOP_INVALID_INDEX;
        result->charge_limiting_cell[h] = AMS_SOP_INVALID_INDEX;
    }
    result->fallback_active = 1u;
}

void ams_sop_default_config(ams_sop_config_t *cfg)
{
    if(cfg == NULL)
    {
        return;
    }

    memset(cfg, 0, sizeof(*cfg));

    /* P42A absolute range is 2.5..4.2 V.  These operational constraints leave
     * headroom for estimator, measurement, and dynamic-model uncertainty. */
    cfg->cell_uv_operating_v = 2.80f;
    cfg->cell_ov_operating_v = 4.15f;
    cfg->soc_min = 0.05f;
    cfg->soc_max = 0.98f;
    cfg->discharge_core_temp_max_c = 55.0f;
    cfg->discharge_surface_temp_max_c = 55.0f;
    cfg->charge_core_temp_max_c = 42.0f;
    cfg->charge_surface_temp_max_c = 42.0f;
    cfg->charge_temp_min_c = 3.0f;

    cfg->horizons_s[0] = 0.10f;
    cfg->horizons_s[1] = 1.00f;
    cfg->horizons_s[2] = 10.0f;
    cfg->horizons_s[3] = 30.0f;

    /* System path envelope: existing firmware fast trip is 120 A / 100 ms,
     * sustained trip is 85 A / 500 ms, the main fuse is 80 A, and the contactor
     * is rated 100 A.  Long horizons stay at the existing 70 A warning level. */
    cfg->discharge_current_max_a[0] = 118.0f;
    cfg->discharge_current_max_a[1] = 80.0f;
    cfg->discharge_current_max_a[2] = 70.0f;
    cfg->discharge_current_max_a[3] = 70.0f;

    /* The installed charger is nominally 10 A and the charging branches are
     * fused at 15 A.  Regeneration uses the same magnitude envelope only after
     * its independent vehicle commissioning gate is asserted. */
    cfg->charge_current_max_a[0] = 11.5f;
    cfg->charge_current_max_a[1] = 10.0f;
    cfg->charge_current_max_a[2] = 10.0f;
    cfg->charge_current_max_a[3] = 10.0f;

    cfg->cell_capacity_ah = 4.20f;
    cfg->parallel_cells = 6.0f;
    cfg->r2_ohm = 0.004f;
    cfg->c2_f = 12000.0f;

    cfg->core_thermal_capacity_j_per_k = 55.0f;
    cfg->surface_thermal_capacity_j_per_k = 15.0f;
    cfg->core_surface_resistance_k_per_w = 1.5f;
    cfg->surface_ambient_resistance_k_per_w = 8.0f;

    cfg->sigma_multiplier = 3.0f;
    cfg->cell_voltage_measurement_uncertainty_v = 0.005f;
    cfg->model_voltage_margin_v = 0.020f;
    cfg->temperature_measurement_uncertainty_c = 1.5f;
    cfg->model_temperature_margin_c = 1.5f;
    cfg->current_uncertainty_floor_a = 0.50f;
    cfg->max_innovation_per_cell_v = 0.100f;
    cfg->max_measurement_age_ms = 250.0f;
    cfg->default_capacity_soh_lower = 0.80f;
    cfg->default_resistance_soh_upper = 1.25f;

    cfg->fine_step_s = 0.10f;
    cfg->medium_step_s = 0.50f;
    cfg->coarse_step_s = 1.00f;
    cfg->discharge_rise_rate_a_per_s = 40.0f;
    cfg->charge_rise_rate_a_per_s = 5.0f;
    cfg->discharge_voltage_recovery_a_per_s = 800.0f;
    cfg->charge_voltage_recovery_a_per_s = 120.0f;
    cfg->discharge_thermal_recovery_a_per_s = 8.0f;
    cfg->charge_thermal_recovery_a_per_s = 2.0f;
    cfg->discharge_current_path_recovery_a_per_s = 20.0f;
    cfg->charge_current_path_recovery_a_per_s = 4.0f;
    cfg->soc_recovery_rate_a_per_s = 5.0f;
    cfg->soc_recovery_delta = 0.005f;
    /* 0.5% of the nominal 25.2 Ah pack capacity. */
    cfg->soc_recovery_charge_as = 453.6f;
}

bool ams_sop_config_valid(const ams_sop_config_t *cfg)
{
    if(cfg == NULL)
    {
        return false;
    }

    if(!finite_positive(cfg->cell_uv_operating_v) ||
       !finite_positive(cfg->cell_ov_operating_v) ||
       !(cfg->cell_uv_operating_v < cfg->cell_ov_operating_v) ||
       !isfinite(cfg->soc_min) || !isfinite(cfg->soc_max) ||
       (cfg->soc_min < 0.0f) || (cfg->soc_max > 1.0f) ||
       !(cfg->soc_min < cfg->soc_max) ||
       !finite_positive(cfg->discharge_core_temp_max_c) ||
       !finite_positive(cfg->discharge_surface_temp_max_c) ||
       !finite_positive(cfg->charge_core_temp_max_c) ||
       !finite_positive(cfg->charge_surface_temp_max_c) ||
       !isfinite(cfg->charge_temp_min_c) ||
       !(cfg->charge_temp_min_c < cfg->charge_surface_temp_max_c) ||
       !finite_positive(cfg->cell_capacity_ah) ||
       !finite_positive(cfg->parallel_cells) ||
       !finite_positive(cfg->r2_ohm) || !finite_positive(cfg->c2_f) ||
       !finite_positive(cfg->core_thermal_capacity_j_per_k) ||
       !finite_positive(cfg->surface_thermal_capacity_j_per_k) ||
       !finite_positive(cfg->core_surface_resistance_k_per_w) ||
       !finite_positive(cfg->surface_ambient_resistance_k_per_w) ||
       !finite_positive(cfg->sigma_multiplier) ||
       !finite_nonnegative(cfg->cell_voltage_measurement_uncertainty_v) ||
       !finite_nonnegative(cfg->model_voltage_margin_v) ||
       !finite_nonnegative(cfg->temperature_measurement_uncertainty_c) ||
       !finite_nonnegative(cfg->model_temperature_margin_c) ||
       !finite_nonnegative(cfg->current_uncertainty_floor_a) ||
       !finite_positive(cfg->max_innovation_per_cell_v) ||
       !finite_positive(cfg->max_measurement_age_ms) ||
       !isfinite(cfg->default_capacity_soh_lower) ||
       (cfg->default_capacity_soh_lower < 0.50f) ||
       (cfg->default_capacity_soh_lower > 1.05f) ||
       !isfinite(cfg->default_resistance_soh_upper) ||
       (cfg->default_resistance_soh_upper < 1.0f) ||
       (cfg->default_resistance_soh_upper > 3.0f) ||
       !finite_positive(cfg->fine_step_s) ||
       !finite_positive(cfg->medium_step_s) ||
       !finite_positive(cfg->coarse_step_s) ||
       (cfg->fine_step_s > cfg->medium_step_s) ||
       (cfg->medium_step_s > cfg->coarse_step_s) ||
       !finite_positive(cfg->discharge_rise_rate_a_per_s) ||
       !finite_positive(cfg->charge_rise_rate_a_per_s) ||
       !finite_positive(cfg->discharge_voltage_recovery_a_per_s) ||
       !finite_positive(cfg->charge_voltage_recovery_a_per_s) ||
       !finite_positive(cfg->discharge_thermal_recovery_a_per_s) ||
       !finite_positive(cfg->charge_thermal_recovery_a_per_s) ||
       !finite_positive(cfg->discharge_current_path_recovery_a_per_s) ||
       !finite_positive(cfg->charge_current_path_recovery_a_per_s) ||
       !finite_positive(cfg->soc_recovery_rate_a_per_s) ||
       !finite_positive(cfg->soc_recovery_delta) ||
       (cfg->soc_recovery_delta > 0.10f) ||
       !finite_positive(cfg->soc_recovery_charge_as))
    {
        return false;
    }

    float previous_horizon = 0.0f;
    for(uint8_t h = 0u; h < AMS_SOP_HORIZONS; h++)
    {
        if(!finite_positive(cfg->horizons_s[h]) ||
           !(cfg->horizons_s[h] > previous_horizon) ||
           !finite_nonnegative(cfg->discharge_current_max_a[h]) ||
           !finite_nonnegative(cfg->charge_current_max_a[h]))
        {
            return false;
        }
        previous_horizon = cfg->horizons_s[h];
    }

    return true;
}

uint32_t ams_sop_input_reason_flags(const ams_sop_input_t *input,
                                    const ams_sop_config_t *cfg)
{
    if((input == NULL) || !ams_sop_config_valid(cfg))
    {
        return AMS_SOP_REASON_CONFIGURATION;
    }

    uint32_t reasons = AMS_SOP_REASON_NONE;
    if(input->measurement_valid == 0u)
    {
        reasons |= AMS_SOP_REASON_MEASUREMENT_INVALID;
    }
    if(input->estimator_valid == 0u)
    {
        reasons |= AMS_SOP_REASON_ESTIMATOR_INVALID;
    }
    if(input->estimator_acquired == 0u)
    {
        reasons |= AMS_SOP_REASON_ESTIMATOR_UNACQUIRED;
    }
    if(input->current_calibrated == 0u)
    {
        reasons |= AMS_SOP_REASON_CURRENT_UNCALIBRATED;
    }
    if(input->current_polarity_validated == 0u)
    {
        reasons |= AMS_SOP_REASON_CURRENT_POLARITY;
    }
    if(input->estimator_segment_topology == 0u)
    {
        reasons |= AMS_SOP_REASON_INCOMPLETE_TOPOLOGY;
    }
    if(input->balance_recovered == 0u)
    {
        reasons |= AMS_SOP_REASON_BALANCE_RECOVERY;
    }
    if((input->measurement_sequence == 0u) ||
       ((uint32_t)(input->now_ms - input->measurement_timestamp_ms) >
        (uint32_t)cfg->max_measurement_age_ms))
    {
        reasons |= AMS_SOP_REASON_MEASUREMENT_STALE;
    }
    if(!isfinite(input->pack_current_a) ||
       !finite_nonnegative(input->pack_current_uncertainty_a) ||
       !isfinite(input->ambient_temp_c) ||
       (input->ambient_temp_c < -50.0f) ||
       (input->ambient_temp_c > 120.0f) ||
       (input->operating_mode > AMS_SOP_MODE_CHARGE))
    {
        reasons |= AMS_SOP_REASON_NUMERIC;
    }
    if(input->pack_current_uncertainty_a < cfg->current_uncertainty_floor_a)
    {
        reasons |= AMS_SOP_REASON_CURRENT_UNCERTAINTY;
    }
    if(input->ambient_measured == 0u)
    {
        reasons |= AMS_SOP_REASON_AMBIENT_PROXY;
    }

    for(uint8_t segment = 0u; segment < AMS_SOP_SEGMENTS; segment++)
    {
        const ams_sop_segment_input_t *in = &input->segment[segment];
        if((in->estimator_valid == 0u) ||
           (in->cell_usable_mask != AMS_SOP_FULL_CELL_MASK))
        {
            reasons |= AMS_SOP_REASON_ESTIMATOR_INVALID;
        }
        if(in->model_domain_flags != 0u)
        {
            reasons |= AMS_SOP_REASON_MODEL_DOMAIN;
        }
        if(in->max_cell_age_ms > (uint32_t)cfg->max_measurement_age_ms)
        {
            reasons |= AMS_SOP_REASON_MEASUREMENT_STALE;
        }

        if(!isfinite(in->soc) || (in->soc < -0.05f) || (in->soc > 1.05f) ||
           !isfinite(in->vp1_v) || !isfinite(in->vp2_v) ||
           !finite_positive(in->r0_ohm) ||
           !isfinite(in->core_temp_c) || !isfinite(in->surface_max_temp_c) ||
           (in->core_temp_c < -50.0f) || (in->core_temp_c > 120.0f) ||
           (in->surface_max_temp_c < -50.0f) ||
           (in->surface_max_temp_c > 120.0f) ||
           !finite_nonnegative(in->p_soc) ||
           !finite_nonnegative(in->p_vp1) ||
           !finite_nonnegative(in->p_vp2) ||
           !finite_nonnegative(in->p_r0) ||
           !isfinite(in->innovation_v) ||
           (fabsf(in->innovation_v) /
            (float)AMS_SOP_CELLS_PER_SEGMENT >
            cfg->max_innovation_per_cell_v))
        {
            reasons |= AMS_SOP_REASON_NUMERIC;
        }

        if(in->capacity_soh_valid != 0u)
        {
            if(!isfinite(in->capacity_soh_lower) ||
               (in->capacity_soh_lower < 0.50f) ||
               (in->capacity_soh_lower > 1.05f))
            {
                reasons |= AMS_SOP_REASON_NUMERIC;
            }
        }
        else
        {
            reasons |= AMS_SOP_REASON_SOH_CAPACITY_PRIOR;
        }

        if(in->resistance_soh_valid != 0u)
        {
            if(!isfinite(in->resistance_soh_upper) ||
               (in->resistance_soh_upper < 0.75f) ||
               (in->resistance_soh_upper > 3.0f))
            {
                reasons |= AMS_SOP_REASON_NUMERIC;
            }
        }
        else
        {
            reasons |= AMS_SOP_REASON_SOH_RESISTANCE_PRIOR;
        }

        for(uint8_t cell = 0u; cell < AMS_SOP_CELLS_PER_SEGMENT; cell++)
        {
            if(!isfinite(in->cell_voltage_v[cell]) ||
               (in->cell_voltage_v[cell] < 2.0f) ||
               (in->cell_voltage_v[cell] > 4.5f))
            {
                reasons |= AMS_SOP_REASON_NUMERIC;
            }
        }
    }

    return reasons;
}

static uint32_t fatal_input_reasons(void)
{
    return AMS_SOP_REASON_MEASUREMENT_INVALID |
           AMS_SOP_REASON_ESTIMATOR_INVALID |
           AMS_SOP_REASON_ESTIMATOR_UNACQUIRED |
           AMS_SOP_REASON_CURRENT_UNCALIBRATED |
           AMS_SOP_REASON_CURRENT_POLARITY |
           AMS_SOP_REASON_MEASUREMENT_STALE |
           AMS_SOP_REASON_INCOMPLETE_TOPOLOGY |
           AMS_SOP_REASON_MODEL_DOMAIN |
           AMS_SOP_REASON_BALANCE_RECOVERY |
           AMS_SOP_REASON_CONFIGURATION |
           AMS_SOP_REASON_NUMERIC;
}

static bool thermal_step_tustin(float *core_temp_c,
                                float *surface_temp_c,
                                float heat_w,
                                float ambient_temp_c,
                                float dt_s,
                                const ams_sop_config_t *cfg)
{
    if((core_temp_c == NULL) || (surface_temp_c == NULL) || (cfg == NULL) ||
       !isfinite(*core_temp_c) || !isfinite(*surface_temp_c) ||
       !finite_nonnegative(heat_w) || !isfinite(ambient_temp_c) ||
       !finite_positive(dt_s))
    {
        return false;
    }

    const float a = 1.0f / (cfg->core_surface_resistance_k_per_w *
                            cfg->core_thermal_capacity_j_per_k);
    const float b = 1.0f / (cfg->core_surface_resistance_k_per_w *
                            cfg->surface_thermal_capacity_j_per_k);
    const float c = 1.0f / (cfg->surface_ambient_resistance_k_per_w *
                            cfg->surface_thermal_capacity_j_per_k);
    const float half_dt = 0.5f * dt_s;

    /* x[k+1] = inv(I-dt*A/2) * ((I+dt*A/2)x[k] + dt*B*u). */
    const float rhs_core =
        (1.0f - (half_dt * a)) * (*core_temp_c) +
        (half_dt * a) * (*surface_temp_c) +
        dt_s * heat_w / cfg->core_thermal_capacity_j_per_k;
    const float rhs_surface =
        (half_dt * b) * (*core_temp_c) +
        (1.0f - half_dt * (b + c)) * (*surface_temp_c) +
        dt_s * c * ambient_temp_c;

    const float m00 = 1.0f + half_dt * a;
    const float m01 = -half_dt * a;
    const float m10 = -half_dt * b;
    const float m11 = 1.0f + half_dt * (b + c);
    const float determinant = (m00 * m11) - (m01 * m10);
    if(!isfinite(determinant) || (fabsf(determinant) < 1.0e-9f))
    {
        return false;
    }

    const float next_core = ((m11 * rhs_core) - (m01 * rhs_surface)) /
                            determinant;
    const float next_surface = ((-m10 * rhs_core) + (m00 * rhs_surface)) /
                               determinant;
    if(!isfinite(next_core) || !isfinite(next_surface))
    {
        return false;
    }

    *core_temp_c = next_core;
    *surface_temp_c = next_surface;
    return true;
}

static bool initialize_model(const ams_sop_input_t *input,
                             const ams_sop_config_t *cfg,
                             ams_sop_model_segment_t model[AMS_SOP_SEGMENTS])
{
    const float present_cell_current = input->pack_current_a /
                                       cfg->parallel_cells;

    for(uint8_t segment = 0u; segment < AMS_SOP_SEGMENTS; segment++)
    {
        const ams_sop_segment_input_t *in = &input->segment[segment];
        ams_sop_model_segment_t *out = &model[segment];
        memset(out, 0, sizeof(*out));

        const float lut_temp_c = clampf_local(in->core_temp_c, 5.0f, 40.0f);
        const float lut_soc = clampf_local(in->soc, 0.0f, 1.0f);
        const float inv_c1 = ams_p42a_inv_c1(lut_soc, lut_temp_c);
        const float neg_inv_tau1 =
            ams_p42a_neg_inv_tau1(lut_soc, lut_temp_c);
        const float inv_r1 = ams_p42a_inv_r1_from_luts(inv_c1,
                                                       neg_inv_tau1);
        if(!finite_positive(inv_c1) || !isfinite(neg_inv_tau1) ||
           !(neg_inv_tau1 < 0.0f) || !finite_positive(inv_r1))
        {
            return false;
        }

        out->soc = in->soc;
        out->vp1_v = in->vp1_v;
        out->vp2_v = in->vp2_v;
        out->core_temp_c = maxf_local(in->core_temp_c,
                                      in->surface_max_temp_c);
        out->surface_temp_c = in->surface_max_temp_c;
        out->soc0 = in->soc;
        out->r1_ohm = 1.0f / inv_r1;
        out->tau1_s = -1.0f / neg_inv_tau1;

        const float lut_r0 = ams_p42a_r0_ohm(lut_soc, lut_temp_c);
        const float r0_sigma = cfg->sigma_multiplier * sqrtf(in->p_r0);
        const float resistance_upper = (in->resistance_soh_valid != 0u) ?
            in->resistance_soh_upper : cfg->default_resistance_soh_upper;
        out->r0_state_ohm = maxf_local(in->r0_ohm, lut_r0);
        out->r0_upper_ohm = maxf_local(in->r0_ohm + r0_sigma,
                                      lut_r0 * resistance_upper);

        const float capacity_soh = (in->capacity_soh_valid != 0u) ?
            in->capacity_soh_lower : cfg->default_capacity_soh_lower;
        out->capacity_as = cfg->cell_capacity_ah * capacity_soh * 3600.0f;

        out->ocv0_v = ams_p42a_ocv_v(lut_soc, lut_temp_c);
        const float soc_lo = clampf_local(lut_soc - 0.005f, 0.0f, 1.0f);
        const float soc_hi = clampf_local(lut_soc + 0.005f, 0.0f, 1.0f);
        const float soc_span = soc_hi - soc_lo;
        out->docv_dsoc_v = (soc_span > 1.0e-6f) ?
            (ams_p42a_ocv_v(soc_hi, lut_temp_c) -
             ams_p42a_ocv_v(soc_lo, lut_temp_c)) / soc_span : 0.0f;

        const float temp_lo = clampf_local(lut_temp_c - 1.0f, 5.0f, 40.0f);
        const float temp_hi = clampf_local(lut_temp_c + 1.0f, 5.0f, 40.0f);
        const float temp_span = temp_hi - temp_lo;
        out->docv_dtemp_v_per_c = (temp_span > 1.0e-6f) ?
            (ams_p42a_ocv_v(lut_soc, temp_hi) -
             ams_p42a_ocv_v(lut_soc, temp_lo)) / temp_span : 0.0f;

        const float sigma_voltage = cfg->sigma_multiplier *
            (fabsf(out->docv_dsoc_v) * sqrtf(in->p_soc) +
             sqrtf(in->p_vp1) + sqrtf(in->p_vp2) +
             fabsf(present_cell_current) * sqrtf(in->p_r0));
        out->voltage_margin_v =
            cfg->cell_voltage_measurement_uncertainty_v +
            cfg->model_voltage_margin_v + sigma_voltage +
            fabsf(in->innovation_v) /
                (float)AMS_SOP_CELLS_PER_SEGMENT;

        float average_cell_voltage = 0.0f;
        for(uint8_t cell = 0u; cell < AMS_SOP_CELLS_PER_SEGMENT; cell++)
        {
            average_cell_voltage += in->cell_voltage_v[cell];
        }
        average_cell_voltage /= (float)AMS_SOP_CELLS_PER_SEGMENT;

        const float model_voltage_now = out->ocv0_v - in->vp1_v -
            in->vp2_v - (out->r0_state_ohm * present_cell_current);
        const float common_bias = average_cell_voltage - model_voltage_now;
        for(uint8_t cell = 0u; cell < AMS_SOP_CELLS_PER_SEGMENT; cell++)
        {
            out->cell_bias_v[cell] = common_bias +
                (in->cell_voltage_v[cell] - average_cell_voltage);
        }

        if(!finite_positive(out->r0_upper_ohm) ||
           !finite_positive(out->r1_ohm) ||
           !finite_positive(out->tau1_s) ||
           !finite_positive(out->capacity_as) ||
           !finite_nonnegative(out->voltage_margin_v) ||
           !isfinite(out->ocv0_v) || !isfinite(out->docv_dsoc_v) ||
           !isfinite(out->docv_dtemp_v_per_c))
        {
            return false;
        }
    }

    return true;
}

static float prediction_step_size(float elapsed_s,
                                  float horizon_s,
                                  const ams_sop_config_t *cfg)
{
    const float remaining_s = horizon_s - elapsed_s;
    float dt_s;
    float boundary_s;

    if(elapsed_s < 1.0f)
    {
        dt_s = cfg->fine_step_s;
        boundary_s = 1.0f;
    }
    else if(elapsed_s < 10.0f)
    {
        dt_s = cfg->medium_step_s;
        boundary_s = 10.0f;
    }
    else
    {
        dt_s = cfg->coarse_step_s;
        boundary_s = horizon_s;
    }

    dt_s = minf_local(dt_s, remaining_s);
    if(boundary_s > elapsed_s)
    {
        dt_s = minf_local(dt_s, boundary_s - elapsed_s);
    }
    return dt_s;
}

static ams_sop_feasibility_t check_current(
    const ams_sop_input_t *input,
    const ams_sop_config_t *cfg,
    const ams_sop_model_segment_t initialized[AMS_SOP_SEGMENTS],
    float requested_pack_current_a,
    bool discharge,
    float horizon_s)
{
    ams_sop_feasibility_t result;
    ams_sop_model_segment_t state[AMS_SOP_SEGMENTS];
    memset(&result, 0, sizeof(result));
    memcpy(state, initialized, sizeof(state));
    result.feasible = 1u;
    result.binding = AMS_SOP_BIND_NONE;
    result.segment = AMS_SOP_INVALID_INDEX;
    result.cell = AMS_SOP_INVALID_INDEX;
    result.extrema.minimum_cell_voltage_v = FLT_MAX;
    result.extrema.maximum_cell_voltage_v = -FLT_MAX;
    result.extrema.minimum_soc = FLT_MAX;
    result.extrema.maximum_soc = -FLT_MAX;
    result.extrema.maximum_core_temp_c = -FLT_MAX;
    result.extrema.maximum_surface_temp_c = -FLT_MAX;

    const float uncertainty_a = maxf_local(input->pack_current_uncertainty_a,
                                            cfg->current_uncertainty_floor_a);
    const float conservative_pack_current_a = discharge ?
        requested_pack_current_a + uncertainty_a :
        requested_pack_current_a - uncertainty_a;
    const float cell_current_a = conservative_pack_current_a /
                                 cfg->parallel_cells;
    float elapsed_s = 0.0f;

    while(elapsed_s < (horizon_s - 1.0e-6f))
    {
        const float dt_s = prediction_step_size(elapsed_s, horizon_s, cfg);
        if(!finite_positive(dt_s))
        {
            result.feasible = 0u;
            result.binding = AMS_SOP_BIND_MODEL_DOMAIN;
            return result;
        }

        float pack_voltage_v = 0.0f;
        for(uint8_t segment = 0u; segment < AMS_SOP_SEGMENTS; segment++)
        {
            ams_sop_model_segment_t *model = &state[segment];
            const float a1 = expf(-dt_s / model->tau1_s);
            const float a2 = expf(-dt_s / (cfg->r2_ohm * cfg->c2_f));
            if(!isfinite(a1) || !isfinite(a2) ||
               (a1 < 0.0f) || (a1 > 1.0f) ||
               (a2 < 0.0f) || (a2 > 1.0f))
            {
                result.feasible = 0u;
                result.binding = AMS_SOP_BIND_MODEL_DOMAIN;
                result.segment = segment;
                return result;
            }

            model->vp1_v = (a1 * model->vp1_v) +
                (model->r1_ohm * (1.0f - a1) * cell_current_a);
            model->vp2_v = (a2 * model->vp2_v) +
                (cfg->r2_ohm * (1.0f - a2) * cell_current_a);
            model->soc -= cell_current_a * dt_s / model->capacity_as;

            const float heat_w = cell_current_a * cell_current_a *
                (model->r0_upper_ohm + model->r1_ohm + cfg->r2_ohm);
            if(!thermal_step_tustin(&model->core_temp_c,
                                    &model->surface_temp_c,
                                    heat_w,
                                    input->ambient_temp_c,
                                    dt_s,
                                    cfg))
            {
                result.feasible = 0u;
                result.binding = AMS_SOP_BIND_MODEL_DOMAIN;
                result.segment = segment;
                return result;
            }

            const ams_sop_segment_input_t *segment_input =
                &input->segment[segment];
            const float sigma_soc = cfg->sigma_multiplier *
                                    sqrtf(segment_input->p_soc);
            const float conservative_soc = discharge ?
                model->soc - sigma_soc : model->soc + sigma_soc;
            if(discharge && (conservative_soc < cfg->soc_min))
            {
                result.feasible = 0u;
                result.binding = AMS_SOP_BIND_SOC_LOW;
                result.segment = segment;
                return result;
            }
            if(!discharge && (conservative_soc > cfg->soc_max))
            {
                result.feasible = 0u;
                result.binding = AMS_SOP_BIND_SOC_HIGH;
                result.segment = segment;
                return result;
            }

            const float temperature_margin =
                cfg->temperature_measurement_uncertainty_c +
                cfg->model_temperature_margin_c;
            const float core_upper = model->core_temp_c + temperature_margin;
            const float surface_upper = model->surface_temp_c +
                                        temperature_margin;
            if(discharge)
            {
                if(core_upper > cfg->discharge_core_temp_max_c)
                {
                    result.feasible = 0u;
                    result.binding = AMS_SOP_BIND_CORE_TEMP;
                    result.segment = segment;
                    return result;
                }
                if(surface_upper > cfg->discharge_surface_temp_max_c)
                {
                    result.feasible = 0u;
                    result.binding = AMS_SOP_BIND_SURFACE_TEMP;
                    result.segment = segment;
                    return result;
                }
            }
            else
            {
                if((model->surface_temp_c - temperature_margin) <
                   cfg->charge_temp_min_c)
                {
                    result.feasible = 0u;
                    result.binding = AMS_SOP_BIND_CHARGE_TEMP_LOW;
                    result.segment = segment;
                    return result;
                }
                if(core_upper > cfg->charge_core_temp_max_c)
                {
                    result.feasible = 0u;
                    result.binding = AMS_SOP_BIND_CORE_TEMP;
                    result.segment = segment;
                    return result;
                }
                if(surface_upper > cfg->charge_surface_temp_max_c)
                {
                    result.feasible = 0u;
                    result.binding = AMS_SOP_BIND_SURFACE_TEMP;
                    result.segment = segment;
                    return result;
                }
            }

            const float ocv_v = model->ocv0_v +
                model->docv_dsoc_v * (model->soc - model->soc0) +
                model->docv_dtemp_v_per_c *
                    (model->core_temp_c - segment_input->core_temp_c);
            const float base_voltage_v = ocv_v - model->vp1_v -
                model->vp2_v - (model->r0_upper_ohm * cell_current_a);
            for(uint8_t cell = 0u; cell < AMS_SOP_CELLS_PER_SEGMENT; cell++)
            {
                const float cell_voltage_v = base_voltage_v +
                                             model->cell_bias_v[cell];
                if(!isfinite(cell_voltage_v))
                {
                    result.feasible = 0u;
                    result.binding = AMS_SOP_BIND_MODEL_DOMAIN;
                    result.segment = segment;
                    result.cell = cell;
                    return result;
                }

                result.extrema.minimum_cell_voltage_v =
                    minf_local(result.extrema.minimum_cell_voltage_v,
                               cell_voltage_v);
                result.extrema.maximum_cell_voltage_v =
                    maxf_local(result.extrema.maximum_cell_voltage_v,
                               cell_voltage_v);
                pack_voltage_v += cell_voltage_v;

                if(discharge &&
                   ((cell_voltage_v - model->voltage_margin_v) <
                    cfg->cell_uv_operating_v))
                {
                    result.feasible = 0u;
                    result.binding = AMS_SOP_BIND_CELL_UV;
                    result.segment = segment;
                    result.cell = cell;
                    return result;
                }
                if(!discharge &&
                   ((cell_voltage_v + model->voltage_margin_v) >
                    cfg->cell_ov_operating_v))
                {
                    result.feasible = 0u;
                    result.binding = AMS_SOP_BIND_CELL_OV;
                    result.segment = segment;
                    result.cell = cell;
                    return result;
                }
            }

            result.extrema.minimum_soc =
                minf_local(result.extrema.minimum_soc, model->soc);
            result.extrema.maximum_soc =
                maxf_local(result.extrema.maximum_soc, model->soc);
            result.extrema.maximum_core_temp_c =
                maxf_local(result.extrema.maximum_core_temp_c,
                           model->core_temp_c);
            result.extrema.maximum_surface_temp_c =
                maxf_local(result.extrema.maximum_surface_temp_c,
                           model->surface_temp_c);
        }

        result.extrema.pack_voltage_v = pack_voltage_v;
        result.steps = saturating_add_u32(result.steps, 1u);
        elapsed_s += dt_s;
    }

    return result;
}

static float solve_direction(const ams_sop_input_t *input,
                             const ams_sop_config_t *cfg,
                             const ams_sop_model_segment_t model[AMS_SOP_SEGMENTS],
                             float horizon_s,
                             float current_cap_a,
                             bool discharge,
                             ams_sop_feasibility_t *limiting,
                             uint32_t *evaluations,
                             uint32_t *steps)
{
    float lower_a = 0.0f;
    float upper_a = current_cap_a;
    ams_sop_feasibility_t zero = check_current(input, cfg, model,
                                                0.0f,
                                                discharge,
                                                horizon_s);
    *evaluations = saturating_add_u32(*evaluations, 1u);
    *steps = saturating_add_u32(*steps, zero.steps);
    if(zero.feasible == 0u)
    {
        *limiting = zero;
        return 0.0f;
    }

    ams_sop_feasibility_t at_cap = check_current(
        input, cfg, model, discharge ? current_cap_a : -current_cap_a,
        discharge,
        horizon_s);
    *evaluations = saturating_add_u32(*evaluations, 1u);
    *steps = saturating_add_u32(*steps, at_cap.steps);
    if(at_cap.feasible != 0u)
    {
        at_cap.binding = AMS_SOP_BIND_CURRENT_PATH;
        at_cap.segment = AMS_SOP_INVALID_INDEX;
        at_cap.cell = AMS_SOP_INVALID_INDEX;
        *limiting = at_cap;
        return current_cap_a;
    }

    ams_sop_feasibility_t first_infeasible = at_cap;
    for(uint8_t iteration = 0u; iteration < AMS_SOP_BISECTION_ITERS;
        iteration++)
    {
        const float candidate_a = 0.5f * (lower_a + upper_a);
        ams_sop_feasibility_t candidate = check_current(
            input, cfg, model, discharge ? candidate_a : -candidate_a,
            discharge,
            horizon_s);
        *evaluations = saturating_add_u32(*evaluations, 1u);
        *steps = saturating_add_u32(*steps, candidate.steps);
        if(candidate.feasible != 0u)
        {
            lower_a = candidate_a;
        }
        else
        {
            upper_a = candidate_a;
            first_infeasible = candidate;
        }
    }

    *limiting = first_infeasible;
    return lower_a;
}

static void apply_direction_authority(const ams_sop_input_t *input,
                                      ams_sop_result_t *result)
{
    const bool allow_discharge =
        (input->operating_mode == AMS_SOP_MODE_DRIVE) &&
        (input->discharge_authorized != 0u);
    const bool allow_charge =
        ((input->operating_mode == AMS_SOP_MODE_CHARGE) &&
         (input->charger_authorized != 0u)) ||
        ((input->operating_mode == AMS_SOP_MODE_DRIVE) &&
         (input->regen_authorized != 0u));

    if(!allow_discharge)
    {
        result->reason_flags |= AMS_SOP_REASON_DISCHARGE_INHIBITED;
    }
    if(!allow_charge)
    {
        result->reason_flags |=
            (input->operating_mode == AMS_SOP_MODE_CHARGE) ?
                AMS_SOP_REASON_CHARGE_INHIBITED :
                AMS_SOP_REASON_REGEN_INHIBITED;
    }

    for(uint8_t h = 0u; h < AMS_SOP_HORIZONS; h++)
    {
        result->discharge_current_a[h] = allow_discharge ?
            result->model_discharge_current_a[h] : 0.0f;
        result->charge_current_a[h] = allow_charge ?
            result->model_charge_current_a[h] : 0.0f;
        if(!allow_discharge)
        {
            result->discharge_binding[h] = AMS_SOP_BIND_DIRECTION_INHIBIT;
            result->discharge_power_w[h] = 0.0f;
        }
        if(!allow_charge)
        {
            result->charge_binding[h] = AMS_SOP_BIND_DIRECTION_INHIBIT;
            result->charge_power_w[h] = 0.0f;
        }
    }
}

ams_sop_status_t ams_sop_solve(const ams_sop_input_t *input,
                               const ams_sop_config_t *cfg,
                               ams_sop_result_t *result)
{
    if((input == NULL) || (cfg == NULL) || (result == NULL))
    {
        return AMS_SOP_BAD_ARGUMENT;
    }

    result_make_zero(result);
    result->measurement_sequence = input->measurement_sequence;
    result->measurement_timestamp_ms = input->measurement_timestamp_ms;
    result->solve_timestamp_ms = input->now_ms;

    if(!ams_sop_config_valid(cfg))
    {
        result->reason_flags = AMS_SOP_REASON_CONFIGURATION;
        return AMS_SOP_INVALID_CONFIGURATION;
    }

    result->reason_flags = ams_sop_input_reason_flags(input, cfg);
    if((result->reason_flags & fatal_input_reasons()) != 0u)
    {
        return AMS_SOP_INVALID_INPUT;
    }

    ams_sop_model_segment_t model[AMS_SOP_SEGMENTS];
    if(!initialize_model(input, cfg, model))
    {
        result->reason_flags |= AMS_SOP_REASON_NUMERIC;
        return AMS_SOP_NUMERIC_FAILURE;
    }

    result->fallback_active = 0u;
    for(uint8_t h = 0u; h < AMS_SOP_HORIZONS; h++)
    {
        ams_sop_feasibility_t discharge_limit;
        ams_sop_feasibility_t charge_limit;
        const float discharge_a = solve_direction(
            input, cfg, model, cfg->horizons_s[h],
            cfg->discharge_current_max_a[h], true, &discharge_limit,
            &result->feasibility_evaluations, &result->prediction_steps);
        const float charge_magnitude_a = solve_direction(
            input, cfg, model, cfg->horizons_s[h],
            cfg->charge_current_max_a[h], false, &charge_limit,
            &result->feasibility_evaluations, &result->prediction_steps);

        result->model_discharge_current_a[h] = discharge_a;
        result->model_charge_current_a[h] = -charge_magnitude_a;
        result->discharge_binding[h] = discharge_limit.binding;
        result->charge_binding[h] = charge_limit.binding;
        result->discharge_limiting_segment[h] = discharge_limit.segment;
        result->discharge_limiting_cell[h] = discharge_limit.cell;
        result->charge_limiting_segment[h] = charge_limit.segment;
        result->charge_limiting_cell[h] = charge_limit.cell;

        ams_sop_feasibility_t discharge_final = check_current(
            input, cfg, model, discharge_a, true, cfg->horizons_s[h]);
        ams_sop_feasibility_t charge_final = check_current(
            input, cfg, model, -charge_magnitude_a, false,
            cfg->horizons_s[h]);
        result->feasibility_evaluations = saturating_add_u32(
            result->feasibility_evaluations, 2u);
        result->prediction_steps = saturating_add_u32(
            result->prediction_steps,
            saturating_add_u32(discharge_final.steps, charge_final.steps));
        const bool discharge_zero_infeasible =
            (discharge_a <= 1.0e-4f) &&
            (discharge_limit.feasible == 0u);
        const bool charge_zero_infeasible =
            (charge_magnitude_a <= 1.0e-4f) &&
            (charge_limit.feasible == 0u);
        if(((discharge_final.feasible == 0u) &&
            !discharge_zero_infeasible) ||
           ((charge_final.feasible == 0u) && !charge_zero_infeasible))
        {
            result_make_zero(result);
            result->measurement_sequence = input->measurement_sequence;
            result->measurement_timestamp_ms = input->measurement_timestamp_ms;
            result->solve_timestamp_ms = input->now_ms;
            result->reason_flags = AMS_SOP_REASON_NUMERIC;
            return AMS_SOP_NUMERIC_FAILURE;
        }

        if(discharge_zero_infeasible)
        {
            discharge_final = discharge_limit;
        }
        if(charge_zero_infeasible)
        {
            charge_final = charge_limit;
        }

        result->discharge_extrema[h] = discharge_final.extrema;
        result->charge_extrema[h] = charge_final.extrema;
        result->discharge_power_w[h] = maxf_local(
            0.0f, discharge_final.extrema.pack_voltage_v * discharge_a);
        result->charge_power_w[h] = maxf_local(
            0.0f, charge_final.extrema.pack_voltage_v * charge_magnitude_a);

        if((discharge_a <= 1.0e-4f) &&
           (discharge_limit.binding != AMS_SOP_BIND_CURRENT_PATH))
        {
            result->reason_flags |= AMS_SOP_REASON_ZERO_CURRENT_INFEASIBLE;
        }
        if((charge_magnitude_a <= 1.0e-4f) &&
           (charge_limit.binding != AMS_SOP_BIND_CURRENT_PATH))
        {
            result->reason_flags |= AMS_SOP_REASON_ZERO_CURRENT_INFEASIBLE;
        }
    }

    /* A longer pulse must never publish a larger capability than a shorter
     * pulse.  This also contains round-off at grid/horizon boundaries. */
    for(uint8_t h = 1u; h < AMS_SOP_HORIZONS; h++)
    {
        if(result->model_discharge_current_a[h] >
           result->model_discharge_current_a[h - 1u])
        {
            result->model_discharge_current_a[h] =
                result->model_discharge_current_a[h - 1u];
            result->discharge_binding[h] = AMS_SOP_BIND_HORIZON_ENVELOPE;
            const ams_sop_feasibility_t clipped = check_current(
                input, cfg, model, result->model_discharge_current_a[h], true,
                cfg->horizons_s[h]);
            result->feasibility_evaluations = saturating_add_u32(
                result->feasibility_evaluations, 1u);
            result->prediction_steps = saturating_add_u32(
                result->prediction_steps, clipped.steps);
            if(clipped.feasible == 0u)
            {
                result_make_zero(result);
                result->measurement_sequence = input->measurement_sequence;
                result->measurement_timestamp_ms =
                    input->measurement_timestamp_ms;
                result->solve_timestamp_ms = input->now_ms;
                result->reason_flags = AMS_SOP_REASON_NUMERIC;
                return AMS_SOP_NUMERIC_FAILURE;
            }
            result->discharge_extrema[h] = clipped.extrema;
            result->discharge_power_w[h] = maxf_local(
                0.0f, clipped.extrema.pack_voltage_v *
                      result->model_discharge_current_a[h]);
        }
        if(fabsf(result->model_charge_current_a[h]) >
           fabsf(result->model_charge_current_a[h - 1u]))
        {
            result->model_charge_current_a[h] =
                result->model_charge_current_a[h - 1u];
            result->charge_binding[h] = AMS_SOP_BIND_HORIZON_ENVELOPE;
            const ams_sop_feasibility_t clipped = check_current(
                input, cfg, model, result->model_charge_current_a[h], false,
                cfg->horizons_s[h]);
            result->feasibility_evaluations = saturating_add_u32(
                result->feasibility_evaluations, 1u);
            result->prediction_steps = saturating_add_u32(
                result->prediction_steps, clipped.steps);
            if(clipped.feasible == 0u)
            {
                result_make_zero(result);
                result->measurement_sequence = input->measurement_sequence;
                result->measurement_timestamp_ms =
                    input->measurement_timestamp_ms;
                result->solve_timestamp_ms = input->now_ms;
                result->reason_flags = AMS_SOP_REASON_NUMERIC;
                return AMS_SOP_NUMERIC_FAILURE;
            }
            result->charge_extrema[h] = clipped.extrema;
            result->charge_power_w[h] = maxf_local(
                0.0f, clipped.extrema.pack_voltage_v *
                      fabsf(result->model_charge_current_a[h]));
        }
    }

    result->valid = 1u;
    result->authority_valid = 1u;
    apply_direction_authority(input, result);
    return AMS_SOP_OK;
}

ams_sop_status_t ams_sop_evaluate_current(const ams_sop_input_t *input,
                                          const ams_sop_config_t *cfg,
                                          float pack_current_a,
                                          float horizon_s,
                                          ams_sop_evaluation_t *evaluation)
{
    if((input == NULL) || (cfg == NULL) || (evaluation == NULL))
    {
        return AMS_SOP_BAD_ARGUMENT;
    }
    memset(evaluation, 0, sizeof(*evaluation));
    evaluation->limiting_segment = AMS_SOP_INVALID_INDEX;
    evaluation->limiting_cell = AMS_SOP_INVALID_INDEX;

    if(!ams_sop_config_valid(cfg) || !isfinite(pack_current_a) ||
       !finite_positive(horizon_s))
    {
        evaluation->binding = AMS_SOP_BIND_INVALID_INPUT;
        return AMS_SOP_INVALID_CONFIGURATION;
    }
    const uint32_t reasons = ams_sop_input_reason_flags(input, cfg);
    if((reasons & fatal_input_reasons()) != 0u)
    {
        evaluation->binding = AMS_SOP_BIND_INVALID_INPUT;
        return AMS_SOP_INVALID_INPUT;
    }

    ams_sop_model_segment_t model[AMS_SOP_SEGMENTS];
    if(!initialize_model(input, cfg, model))
    {
        evaluation->binding = AMS_SOP_BIND_MODEL_DOMAIN;
        return AMS_SOP_NUMERIC_FAILURE;
    }
    const bool discharge = !signbit(pack_current_a);
    const ams_sop_feasibility_t result = check_current(
        input, cfg, model, pack_current_a, discharge, horizon_s);
    evaluation->extrema = result.extrema;
    evaluation->binding = result.binding;
    evaluation->limiting_segment = result.segment;
    evaluation->limiting_cell = result.cell;
    evaluation->prediction_steps = result.steps;
    evaluation->feasible = result.feasible;
    return AMS_SOP_OK;
}

static float discharge_recovery_rate(const ams_sop_config_t *cfg,
                                     ams_sop_binding_t binding,
                                     const ams_sop_recovery_context_t *context,
                                     uint32_t *reason_flags)
{
    switch(binding)
    {
    case AMS_SOP_BIND_CELL_UV:
        *reason_flags |= AMS_SOP_REASON_RECOVERY_VOLTAGE;
        return cfg->discharge_voltage_recovery_a_per_s;
    case AMS_SOP_BIND_CORE_TEMP:
    case AMS_SOP_BIND_SURFACE_TEMP:
        *reason_flags |= AMS_SOP_REASON_RECOVERY_THERMAL;
        return cfg->discharge_thermal_recovery_a_per_s;
    case AMS_SOP_BIND_SOC_LOW:
        *reason_flags |= AMS_SOP_REASON_RECOVERY_SOC_HOLD;
        return ((context != NULL) &&
                (context->discharge_soc_recovered != 0u)) ?
            cfg->soc_recovery_rate_a_per_s : 0.0f;
    case AMS_SOP_BIND_CURRENT_PATH:
        *reason_flags |= AMS_SOP_REASON_RECOVERY_CURRENT_PATH;
        return cfg->discharge_current_path_recovery_a_per_s;
    case AMS_SOP_BIND_FUSE_THERMAL:
        *reason_flags |= AMS_SOP_REASON_RECOVERY_CURRENT_PATH;
        if((context != NULL) && (context->fuse_state_valid != 0u))
        {
            const float headroom = clampf_local(
                1.0f - context->fuse_utilization, 0.0f, 1.0f);
            return cfg->discharge_current_path_recovery_a_per_s * headroom;
        }
        return 0.0f;
    default:
        return cfg->discharge_rise_rate_a_per_s;
    }
}

static float charge_recovery_rate(const ams_sop_config_t *cfg,
                                  ams_sop_binding_t binding,
                                  const ams_sop_recovery_context_t *context,
                                  uint32_t *reason_flags)
{
    switch(binding)
    {
    case AMS_SOP_BIND_CELL_OV:
        *reason_flags |= AMS_SOP_REASON_RECOVERY_VOLTAGE;
        return cfg->charge_voltage_recovery_a_per_s;
    case AMS_SOP_BIND_CORE_TEMP:
    case AMS_SOP_BIND_SURFACE_TEMP:
    case AMS_SOP_BIND_CHARGE_TEMP_LOW:
        *reason_flags |= AMS_SOP_REASON_RECOVERY_THERMAL;
        return cfg->charge_thermal_recovery_a_per_s;
    case AMS_SOP_BIND_SOC_HIGH:
        *reason_flags |= AMS_SOP_REASON_RECOVERY_SOC_HOLD;
        return ((context != NULL) &&
                (context->charge_soc_recovered != 0u)) ?
            cfg->soc_recovery_rate_a_per_s : 0.0f;
    case AMS_SOP_BIND_CURRENT_PATH:
        *reason_flags |= AMS_SOP_REASON_RECOVERY_CURRENT_PATH;
        return cfg->charge_current_path_recovery_a_per_s;
    case AMS_SOP_BIND_FUSE_THERMAL:
        *reason_flags |= AMS_SOP_REASON_RECOVERY_CURRENT_PATH;
        return ((context != NULL) &&
                (context->fuse_state_valid != 0u)) ?
            cfg->charge_current_path_recovery_a_per_s : 0.0f;
    default:
        return cfg->charge_rise_rate_a_per_s;
    }
}

void ams_sop_apply_recovery(const ams_sop_result_t *raw,
                            const ams_sop_result_t *previous,
                            const ams_sop_config_t *cfg,
                            const ams_sop_recovery_context_t *context,
                            float elapsed_s,
                            ams_sop_result_t *published)
{
    if(published == NULL)
    {
        return;
    }

    if((raw == NULL) || (cfg == NULL) || !ams_sop_config_valid(cfg) ||
       !isfinite(elapsed_s) || (elapsed_s <= 0.0f) ||
       (raw->valid == 0u) || (raw->authority_valid == 0u))
    {
        result_make_zero(published);
        if(raw != NULL)
        {
            published->measurement_sequence = raw->measurement_sequence;
            published->measurement_timestamp_ms =
                raw->measurement_timestamp_ms;
            published->solve_timestamp_ms = raw->solve_timestamp_ms;
            published->reason_flags = raw->reason_flags;
        }
        return;
    }

    *published = *raw;
    const bool previous_valid = (previous != NULL) &&
                                (previous->valid != 0u) &&
                                (previous->authority_valid != 0u);
    for(uint8_t h = 0u; h < AMS_SOP_HORIZONS; h++)
    {
        const float old_discharge_a = previous_valid ?
            maxf_local(0.0f, previous->discharge_current_a[h]) : 0.0f;
        const ams_sop_binding_t previous_discharge_binding = previous_valid ?
            previous->discharge_binding[h] : AMS_SOP_BIND_NONE;
        const float discharge_rate = discharge_recovery_rate(
            cfg, previous_discharge_binding, context,
            &published->reason_flags);
        const float discharge_ceiling_a = old_discharge_a +
            discharge_rate * elapsed_s;
        if(published->discharge_current_a[h] > discharge_ceiling_a)
        {
            const float ratio = (published->discharge_current_a[h] > 0.0f) ?
                discharge_ceiling_a / published->discharge_current_a[h] : 0.0f;
            published->discharge_current_a[h] = discharge_ceiling_a;
            published->discharge_power_w[h] *= ratio;
            published->reason_flags |= AMS_SOP_REASON_LIMIT_SLEWED;
        }

        const float old_charge_magnitude_a = previous_valid ?
            maxf_local(0.0f, -previous->charge_current_a[h]) : 0.0f;
        const ams_sop_binding_t previous_charge_binding = previous_valid ?
            previous->charge_binding[h] : AMS_SOP_BIND_NONE;
        const float charge_rate = charge_recovery_rate(
            cfg, previous_charge_binding, context,
            &published->reason_flags);
        const float charge_ceiling_a = old_charge_magnitude_a +
            charge_rate * elapsed_s;
        const float raw_charge_magnitude_a =
            maxf_local(0.0f, -published->charge_current_a[h]);
        if(raw_charge_magnitude_a > charge_ceiling_a)
        {
            const float ratio = (raw_charge_magnitude_a > 0.0f) ?
                charge_ceiling_a / raw_charge_magnitude_a : 0.0f;
            published->charge_current_a[h] = -charge_ceiling_a;
            published->charge_power_w[h] *= ratio;
            published->reason_flags |= AMS_SOP_REASON_LIMIT_SLEWED;
        }
    }
}

void ams_sop_apply_slew(const ams_sop_result_t *raw,
                        const ams_sop_result_t *previous,
                        const ams_sop_config_t *cfg,
                        float elapsed_s,
                        ams_sop_result_t *published)
{
    ams_sop_apply_recovery(raw, previous, cfg, NULL, elapsed_s, published);
}

const char *ams_sop_binding_name(ams_sop_binding_t binding)
{
    switch(binding)
    {
    case AMS_SOP_BIND_NONE: return "none";
    case AMS_SOP_BIND_CELL_UV: return "cell-uv";
    case AMS_SOP_BIND_CELL_OV: return "cell-ov";
    case AMS_SOP_BIND_SOC_LOW: return "soc-low";
    case AMS_SOP_BIND_SOC_HIGH: return "soc-high";
    case AMS_SOP_BIND_CORE_TEMP: return "core-temp";
    case AMS_SOP_BIND_SURFACE_TEMP: return "surface-temp";
    case AMS_SOP_BIND_CHARGE_TEMP_LOW: return "charge-temp-low";
    case AMS_SOP_BIND_CURRENT_PATH: return "current-path";
    case AMS_SOP_BIND_DIRECTION_INHIBIT: return "direction-inhibit";
    case AMS_SOP_BIND_MODEL_DOMAIN: return "model-domain";
    case AMS_SOP_BIND_INVALID_INPUT: return "invalid-input";
    case AMS_SOP_BIND_HORIZON_ENVELOPE: return "horizon-envelope";
    case AMS_SOP_BIND_FUSE_THERMAL: return "fuse-thermal";
    case AMS_SOP_BIND_MISSION_PROFILE: return "mission-profile";
    default: return "unknown";
    }
}
