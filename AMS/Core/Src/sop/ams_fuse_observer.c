#include "sop/ams_fuse_observer.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/* Rounded centerline readings of the EAC14-80 current-versus-time curve in
 * Eaton ELX1308, January 2026, page 4.  The chart itself spans 0.01 s to
 * 100 s.  The 800 A point is anchored to the tabulated 8020 A^2s typical
 * melting I2t measured at 10 In: 8020 / 800^2 = 0.01253125 s.  The final
 * 850 A point is a rounded read near the lower chart boundary and preserves
 * monotonic interpolation around the tabulated anchor.
 *
 * These are approximate typical centerline readings, not guaranteed minimums.
 * Keep AMS_FUSE_MODEL_VALIDATED disabled until manufacturer and vehicle
 * evidence establishes an authoritative curve and margin. */
typedef struct
{
    float current_a;
    float time_s;
} fuse_curve_point_t;

static const fuse_curve_point_t EAC14_80_TYPICAL_CURVE[] = {
    {154.0f, 100.0f},
    {168.0f, 50.0f},
    {180.9f, 30.0f},
    {192.8f, 20.0f},
    {219.2f, 10.0f},
    {249.2f, 5.0f},
    {274.9f, 3.0f},
    {300.8f, 2.0f},
    {350.9f, 1.0f},
    {411.1f, 0.5f},
    {458.9f, 0.3f},
    {500.2f, 0.2f},
    {576.6f, 0.1f},
    {652.8f, 0.05f},
    {705.8f, 0.03f},
    {752.5f, 0.02f},
    {800.0f, 0.01253125f},
    {850.0f, 0.01078f},
};

#define EAC14_80_CURVE_COUNT \
    ((uint32_t)(sizeof(EAC14_80_TYPICAL_CURVE) / \
                sizeof(EAC14_80_TYPICAL_CURVE[0])))

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

static float interpolate(float x, float x0, float y0, float x1, float y1)
{
    if(x1 <= x0)
    {
        return y0;
    }
    const float fraction = clampf_local((x - x0) / (x1 - x0), 0.0f, 1.0f);
    return y0 + fraction * (y1 - y0);
}

static float log_log_interpolate(float x,
                                 float x0,
                                 float y0,
                                 float x1,
                                 float y1)
{
    if((x <= 0.0f) || (x0 <= 0.0f) || (x1 <= x0) ||
       (y0 <= 0.0f) || (y1 <= 0.0f))
    {
        return y0;
    }
    const float f = clampf_local((logf(x) - logf(x0)) /
                                 (logf(x1) - logf(x0)), 0.0f, 1.0f);
    return expf(logf(y0) + f * (logf(y1) - logf(y0)));
}

void ams_fuse_observer_default_config(ams_fuse_observer_config_t *cfg)
{
    if(cfg == NULL)
    {
        return;
    }

    memset(cfg, 0, sizeof(*cfg));
    cfg->rated_current_a = AMS_FUSE_EAC14_80_RATED_CURRENT_A;
    cfg->curve_time_fraction = 0.25f;
    cfg->cooling_time_constant_s = 300.0f;
    cfg->initialization_soak_s = 300.0f;
    cfg->quiescent_current_a = 5.0f;
    cfg->fuse_temperature_margin_c = 15.0f;
    cfg->minimum_temperature_derating = 0.75f;
    cfg->maximum_state_multiple = 4.0f;
    cfg->low_current_fit_scale_s = 75.6f;
    cfg->low_current_fit_exponent = 3.93f;
    cfg->maximum_curve_time_s = 86400.0f;
    cfg->minimum_curve_time_s = 0.0005f;
}

bool ams_fuse_observer_config_valid(const ams_fuse_observer_config_t *cfg)
{
    return (cfg != NULL) && isfinite(cfg->rated_current_a) &&
           (fabsf(cfg->rated_current_a -
                  AMS_FUSE_EAC14_80_RATED_CURRENT_A) <= 1.0e-3f) &&
           isfinite(cfg->curve_time_fraction) &&
           (cfg->curve_time_fraction > 0.0f) &&
           (cfg->curve_time_fraction <= 1.0f) &&
           isfinite(cfg->cooling_time_constant_s) &&
           (cfg->cooling_time_constant_s > 0.0f) &&
           isfinite(cfg->initialization_soak_s) &&
           (cfg->initialization_soak_s >= 0.0f) &&
           isfinite(cfg->quiescent_current_a) &&
           (cfg->quiescent_current_a >= 0.0f) &&
           isfinite(cfg->fuse_temperature_margin_c) &&
           (cfg->fuse_temperature_margin_c >= 0.0f) &&
           isfinite(cfg->minimum_temperature_derating) &&
           (cfg->minimum_temperature_derating > 0.0f) &&
           (cfg->minimum_temperature_derating <= 1.0f) &&
           isfinite(cfg->maximum_state_multiple) &&
           (cfg->maximum_state_multiple >= 1.0f) &&
           isfinite(cfg->low_current_fit_scale_s) &&
           (cfg->low_current_fit_scale_s > 0.0f) &&
           isfinite(cfg->low_current_fit_exponent) &&
           (cfg->low_current_fit_exponent > 0.0f) &&
           isfinite(cfg->maximum_curve_time_s) &&
           (cfg->maximum_curve_time_s > 0.0f) &&
           isfinite(cfg->minimum_curve_time_s) &&
           (cfg->minimum_curve_time_s > 0.0f) &&
           (cfg->minimum_curve_time_s < cfg->maximum_curve_time_s);
}

void ams_fuse_observer_init(ams_fuse_observer_t *observer)
{
    if(observer != NULL)
    {
        memset(observer, 0, sizeof(*observer));
    }
}

bool ams_fuse_observer_init_conservative(
    ams_fuse_observer_t *observer,
    const ams_fuse_observer_config_t *cfg)
{
    if((observer == NULL) || !ams_fuse_observer_config_valid(cfg))
    {
        return false;
    }

    memset(observer, 0, sizeof(*observer));
    observer->thermal_utilization = cfg->maximum_state_multiple;
    observer->thermal_state_initialized = 1u;
    observer->budget_exhausted = 1u;
    observer->initial_state_conservative = 1u;
    return true;
}

float ams_fuse_temperature_derating(float fuse_temperature_c,
                                     float minimum_derating)
{
    if(!isfinite(fuse_temperature_c) || !isfinite(minimum_derating) ||
       (minimum_derating <= 0.0f) || (minimum_derating > 1.0f))
    {
        return 0.0f;
    }

    /* Rounded readings of the EAC14 page-4 ambient-temperature curve.  Cold
     * uplift is intentionally capped at 1.0; only hot-temperature derating is
     * credited by the safety model. */
    static const float temp_c[] = {
        -40.0f, 0.0f, 25.0f, 40.0f, 60.0f, 80.0f, 100.0f, 125.0f
    };
    static const float factor[] = {
        1.15f, 1.06f, 1.00f, 0.97f, 0.93f, 0.89f, 0.85f, 0.80f
    };
    const uint32_t count = (uint32_t)(sizeof(temp_c) / sizeof(temp_c[0]));

    float derating = factor[count - 1u];
    if(fuse_temperature_c <= temp_c[0])
    {
        derating = factor[0];
    }
    else
    {
        for(uint32_t i = 1u; i < count; ++i)
        {
            if(fuse_temperature_c <= temp_c[i])
            {
                derating = interpolate(fuse_temperature_c,
                                       temp_c[i - 1u], factor[i - 1u],
                                       temp_c[i], factor[i]);
                break;
            }
        }
    }
    return clampf_local(derating, minimum_derating, 1.0f);
}

float ams_fuse_typical_melt_time_s(const ams_fuse_observer_config_t *cfg,
                                   float equivalent_25c_current_a,
                                   uint8_t *extrapolated)
{
    if(extrapolated != NULL)
    {
        *extrapolated = 0u;
    }
    if(!ams_fuse_observer_config_valid(cfg) ||
       !isfinite(equivalent_25c_current_a) ||
       (equivalent_25c_current_a < 0.0f))
    {
        return NAN;
    }
    if(equivalent_25c_current_a <= cfg->rated_current_a)
    {
        return INFINITY;
    }

    const fuse_curve_point_t *first = &EAC14_80_TYPICAL_CURVE[0];
    const fuse_curve_point_t *last =
        &EAC14_80_TYPICAL_CURVE[EAC14_80_CURVE_COUNT - 1u];

    if(equivalent_25c_current_a < first->current_a)
    {
        if(extrapolated != NULL)
        {
            *extrapolated = 1u;
        }
        const float overcurrent = equivalent_25c_current_a /
                                  cfg->rated_current_a - 1.0f;
        if(overcurrent <= 0.0f)
        {
            return INFINITY;
        }
        const float time_s = cfg->low_current_fit_scale_s *
            powf(overcurrent, -cfg->low_current_fit_exponent);
        return fminf(cfg->maximum_curve_time_s,
                     fmaxf(cfg->minimum_curve_time_s, time_s));
    }

    for(uint32_t i = 1u; i < EAC14_80_CURVE_COUNT; ++i)
    {
        if(equivalent_25c_current_a <=
           EAC14_80_TYPICAL_CURVE[i].current_a)
        {
            return log_log_interpolate(
                equivalent_25c_current_a,
                EAC14_80_TYPICAL_CURVE[i - 1u].current_a,
                EAC14_80_TYPICAL_CURVE[i - 1u].time_s,
                EAC14_80_TYPICAL_CURVE[i].current_a,
                EAC14_80_TYPICAL_CURVE[i].time_s);
        }
    }

    if(extrapolated != NULL)
    {
        *extrapolated = 1u;
    }
    const fuse_curve_point_t *previous =
        &EAC14_80_TYPICAL_CURVE[EAC14_80_CURVE_COUNT - 2u];
    const float slope = (logf(last->time_s) - logf(previous->time_s)) /
                        (logf(last->current_a) -
                         logf(previous->current_a));
    const float high_current_time_s = expf(logf(last->time_s) +
        slope * (logf(equivalent_25c_current_a) -
                 logf(last->current_a)));
    return fmaxf(cfg->minimum_curve_time_s,
                 fminf(cfg->maximum_curve_time_s, high_current_time_s));
}

static float curve_source_rate_per_s(const ams_fuse_observer_config_t *cfg,
                                     float equivalent_25c_current_a,
                                     uint8_t *extrapolated,
                                     float *typical_time_s,
                                     float *usable_time_s)
{
    const float typical = ams_fuse_typical_melt_time_s(
        cfg, equivalent_25c_current_a, extrapolated);
    if(typical_time_s != NULL)
    {
        *typical_time_s = typical;
    }
    if(!isfinite(typical))
    {
        if(usable_time_s != NULL)
        {
            *usable_time_s = INFINITY;
        }
        return 0.0f;
    }

    const float usable = clampf_local(
        typical * cfg->curve_time_fraction,
        cfg->minimum_curve_time_s,
        cfg->maximum_curve_time_s);
    if(usable_time_s != NULL)
    {
        *usable_time_s = usable;
    }

    const float ratio = usable / cfg->cooling_time_constant_s;
    const float kernel_s = -cfg->cooling_time_constant_s * expm1f(-ratio);
    if(!isfinite(kernel_s) || (kernel_s <= 0.0f))
    {
        return 1.0f / cfg->minimum_curve_time_s;
    }
    return 1.0f / kernel_s;
}

static float predicted_utilization_production(
    const ams_fuse_observer_t *observer,
    const ams_fuse_observer_config_t *cfg,
    float pack_current_candidate_a,
    float current_uncertainty_a,
    float temperature_derating,
    float horizon_s,
    uint8_t *extrapolated)
{
    const float effective_current_a =
        fmaxf(0.0f, pack_current_candidate_a) + current_uncertainty_a;
    const float equivalent_current_a = effective_current_a /
                                       temperature_derating;
    const float source_rate = curve_source_rate_per_s(
        cfg, equivalent_current_a, extrapolated, NULL, NULL);
    const float decay = expf(-horizon_s / cfg->cooling_time_constant_s);
    return observer->thermal_utilization * decay + source_rate * horizon_s;
}

static float solve_horizon_cap(const ams_fuse_observer_t *observer,
                               const ams_fuse_observer_config_t *cfg,
                               float static_cap_a,
                               float current_uncertainty_a,
                               float temperature_derating,
                               float horizon_s,
                               uint8_t *extrapolated)
{
    if((observer->budget_exhausted != 0u) || (static_cap_a <= 0.0f))
    {
        return 0.0f;
    }

    uint8_t local_extrapolated = 0u;
    const float at_static = predicted_utilization_production(
        observer, cfg, static_cap_a, current_uncertainty_a,
        temperature_derating, horizon_s, &local_extrapolated);
    if(local_extrapolated != 0u && extrapolated != NULL)
    {
        *extrapolated = 1u;
    }
    if(at_static <= 1.0f)
    {
        return static_cap_a;
    }

    float low = 0.0f;
    float high = static_cap_a;
    for(uint8_t iteration = 0u; iteration < 24u; ++iteration)
    {
        const float mid = 0.5f * (low + high);
        local_extrapolated = 0u;
        const float predicted = predicted_utilization_production(
            observer, cfg, mid, current_uncertainty_a,
            temperature_derating, horizon_s, &local_extrapolated);
        if(local_extrapolated != 0u && extrapolated != NULL)
        {
            *extrapolated = 1u;
        }
        if(predicted <= 1.0f)
        {
            low = mid;
        }
        else
        {
            high = mid;
        }
    }
    return low;
}

static void zero_result(ams_fuse_observer_result_t *result)
{
    if(result != NULL)
    {
        memset(result, 0, sizeof(*result));
        result->utilization = 1.0f;
        result->reason_flags = AMS_FUSE_REASON_INPUT_INVALID;
    }
}

bool ams_fuse_observer_update(ams_fuse_observer_t *observer,
                              const ams_fuse_observer_config_t *cfg,
                              const ams_sop_config_t *sop_cfg,
                              const ams_fuse_observer_input_t *input,
                              ams_fuse_observer_result_t *result)
{
    if((observer == NULL) || (result == NULL))
    {
        return false;
    }
    zero_result(result);

    const bool valid = ams_fuse_observer_config_valid(cfg) &&
        ams_sop_config_valid(sop_cfg) && (input != NULL) &&
        isfinite(input->pack_current_a) &&
        isfinite(input->current_uncertainty_a) &&
        (input->current_uncertainty_a >= 0.0f) &&
        isfinite(input->temperature_proxy_c) &&
        isfinite(input->elapsed_s) && (input->elapsed_s > 0.0f) &&
        (input->measurement_valid != 0u) &&
        (input->current_calibrated != 0u) &&
        (input->current_polarity_validated != 0u);
    if(!valid)
    {
        if(observer->invalid_count != UINT32_MAX)
        {
            observer->invalid_count++;
        }
        return false;
    }

    result->reason_flags = AMS_FUSE_REASON_NONE;
    if(observer->update_count != UINT32_MAX)
    {
        observer->update_count++;
    }

    result->effective_current_a = fabsf(input->pack_current_a) +
                                  input->current_uncertainty_a;
    if(observer->thermal_state_initialized == 0u)
    {
        if(result->effective_current_a <= cfg->quiescent_current_a)
        {
            observer->quiescent_time_s += input->elapsed_s;
            if(observer->quiescent_time_s >= cfg->initialization_soak_s)
            {
                observer->thermal_state_initialized = 1u;
                /* The soak establishes authority but does not erase thermal
                 * history.  Any utilization accumulated before/during the
                 * soak must decay through the normal model. */
            }
        }
        else
        {
            observer->quiescent_time_s = 0.0f;
        }
    }

    result->estimated_fuse_temperature_c = input->temperature_proxy_c;
    if(input->temperature_measured_at_fuse == 0u)
    {
        result->estimated_fuse_temperature_c +=
            cfg->fuse_temperature_margin_c;
        result->reason_flags |= AMS_FUSE_REASON_TEMPERATURE_PROXY;
    }
    result->temperature_derating = ams_fuse_temperature_derating(
        result->estimated_fuse_temperature_c,
        cfg->minimum_temperature_derating);
    result->continuous_current_a = cfg->rated_current_a *
                                   result->temperature_derating;
    result->equivalent_25c_current_a = result->effective_current_a /
                                       result->temperature_derating;

    uint8_t current_extrapolated = 0u;
    const float source_rate = curve_source_rate_per_s(
        cfg, result->equivalent_25c_current_a, &current_extrapolated,
        &result->typical_melt_time_s, &result->usable_melt_time_s);
    if(current_extrapolated != 0u)
    {
        result->curve_extrapolated = 1u;
        result->reason_flags |= AMS_FUSE_REASON_CURVE_EXTRAPOLATED;
    }

    const float decay = expf(-input->elapsed_s /
                             cfg->cooling_time_constant_s);
    observer->thermal_utilization =
        observer->thermal_utilization * decay +
        source_rate * input->elapsed_s;
    observer->thermal_utilization = clampf_local(
        observer->thermal_utilization, 0.0f,
        cfg->maximum_state_multiple);

    result->utilization = observer->thermal_utilization;
    result->remaining_utilization = fmaxf(
        0.0f, 1.0f - observer->thermal_utilization);
    if(result->utilization >= 1.0f)
    {
        observer->budget_exhausted = 1u;
    }
    else if(result->utilization <= 0.50f)
    {
        observer->budget_exhausted = 0u;
        observer->initial_state_conservative = 0u;
    }

    for(uint8_t h = 0u; h < AMS_SOP_HORIZONS; ++h)
    {
        uint8_t cap_extrapolated = 0u;
        result->discharge_current_cap_a[h] = solve_horizon_cap(
            observer, cfg, sop_cfg->discharge_current_max_a[h],
            input->current_uncertainty_a, result->temperature_derating,
            sop_cfg->horizons_s[h], &cap_extrapolated);
        if(cap_extrapolated != 0u)
        {
            result->curve_extrapolated = 1u;
            result->reason_flags |= AMS_FUSE_REASON_CURVE_EXTRAPOLATED;
        }
        if(result->discharge_current_cap_a[h] + 1.0e-3f <
           sop_cfg->discharge_current_max_a[h])
        {
            result->reason_flags |= AMS_FUSE_REASON_CURVE_DERATED;
        }

        cap_extrapolated = 0u;
        result->charge_current_cap_a[h] = solve_horizon_cap(
            observer, cfg, sop_cfg->charge_current_max_a[h],
            input->current_uncertainty_a, result->temperature_derating,
            sop_cfg->horizons_s[h], &cap_extrapolated);
        if(cap_extrapolated != 0u)
        {
            result->curve_extrapolated = 1u;
            result->reason_flags |= AMS_FUSE_REASON_CURVE_EXTRAPOLATED;
        }
        if(result->charge_current_cap_a[h] + 1.0e-3f <
           sop_cfg->charge_current_max_a[h])
        {
            result->reason_flags |= AMS_FUSE_REASON_CURVE_DERATED;
        }
    }

    result->valid = 1u;
    result->budget_exhausted = observer->budget_exhausted;
    if(input->model_validated == 0u)
    {
        result->reason_flags |= AMS_FUSE_REASON_MODEL_UNVALIDATED;
    }
    if((observer->thermal_state_initialized == 0u) ||
       (observer->initial_state_conservative != 0u))
    {
        result->reason_flags |= AMS_FUSE_REASON_INITIAL_STATE_UNKNOWN;
    }
    if(observer->budget_exhausted != 0u)
    {
        result->reason_flags |= AMS_FUSE_REASON_BUDGET_EXHAUSTED;
    }
    result->authority_valid = ((input->model_validated != 0u) &&
        (observer->thermal_state_initialized != 0u)) ? 1u : 0u;
    return true;
}
