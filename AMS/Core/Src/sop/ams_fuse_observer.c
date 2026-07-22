#include "sop/ams_fuse_observer.h"

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

static float interpolate(float x, float x0, float y0, float x1, float y1)
{
    if(x1 <= x0)
    {
        return y0;
    }
    const float fraction = clampf_local((x - x0) / (x1 - x0), 0.0f, 1.0f);
    return y0 + fraction * (y1 - y0);
}

void ams_fuse_observer_default_config(ams_fuse_observer_config_t *cfg)
{
    if(cfg == NULL)
    {
        return;
    }

    memset(cfg, 0, sizeof(*cfg));
    cfg->rated_current_a = 80.0f;
    cfg->typical_melting_i2t_a2s = 8020.0f;
    /* A typical melting-I2t value is not a guaranteed minimum.  Only one
     * quarter is made available to the software observer until lot, holder,
     * busbar, ambient, and ageing characterization justifies another value. */
    cfg->usable_i2t_fraction = 0.25f;
    cfg->cooling_time_constant_s = 300.0f;
    cfg->initialization_soak_s = 300.0f;
    cfg->quiescent_current_a = 5.0f;
    cfg->fuse_temperature_margin_c = 15.0f;
    cfg->minimum_temperature_derating = 0.75f;
    cfg->maximum_state_multiple = 4.0f;
}

bool ams_fuse_observer_config_valid(const ams_fuse_observer_config_t *cfg)
{
    return (cfg != NULL) && isfinite(cfg->rated_current_a) &&
           (cfg->rated_current_a > 0.0f) &&
           isfinite(cfg->typical_melting_i2t_a2s) &&
           (cfg->typical_melting_i2t_a2s > 0.0f) &&
           isfinite(cfg->usable_i2t_fraction) &&
           (cfg->usable_i2t_fraction > 0.0f) &&
           (cfg->usable_i2t_fraction <= 1.0f) &&
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
           (cfg->maximum_state_multiple >= 1.0f);
}

void ams_fuse_observer_init(ams_fuse_observer_t *observer)
{
    if(observer != NULL)
    {
        memset(observer, 0, sizeof(*observer));
    }
}

float ams_fuse_temperature_derating(float fuse_temperature_c,
                                     float minimum_derating)
{
    if(!isfinite(fuse_temperature_c) || !isfinite(minimum_derating) ||
       (minimum_derating <= 0.0f) || (minimum_derating > 1.0f))
    {
        return 0.0f;
    }

    /* Conservative piecewise reading of the EAC14 ambient-temperature
     * derating curve.  Installed-holder characterization remains mandatory. */
    float derating;
    if(fuse_temperature_c <= 0.0f)
    {
        derating = 1.03f;
    }
    else if(fuse_temperature_c <= 25.0f)
    {
        derating = interpolate(fuse_temperature_c, 0.0f, 1.03f,
                               25.0f, 1.00f);
    }
    else if(fuse_temperature_c <= 80.0f)
    {
        derating = interpolate(fuse_temperature_c, 25.0f, 1.00f,
                               80.0f, 0.90f);
    }
    else if(fuse_temperature_c <= 125.0f)
    {
        derating = interpolate(fuse_temperature_c, 80.0f, 0.90f,
                               125.0f, 0.80f);
    }
    else
    {
        derating = 0.80f;
    }
    return clampf_local(derating, minimum_derating, 1.0f);
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

    const float current_a = fabsf(input->pack_current_a) +
                            input->current_uncertainty_a;
    if(observer->thermal_state_initialized == 0u)
    {
        if(current_a <= cfg->quiescent_current_a)
        {
            observer->quiescent_time_s += input->elapsed_s;
            if(observer->quiescent_time_s >= cfg->initialization_soak_s)
            {
                observer->thermal_state_initialized = 1u;
                observer->excess_i2t_a2s = 0.0f;
                observer->budget_exhausted = 0u;
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
    result->usable_i2t_a2s = cfg->typical_melting_i2t_a2s *
                             cfg->usable_i2t_fraction;

    const float decay = expf(-input->elapsed_s /
                             cfg->cooling_time_constant_s);
    const float excess_rate_a2 = fmaxf(0.0f,
        current_a * current_a -
        result->continuous_current_a * result->continuous_current_a);
    observer->excess_i2t_a2s = observer->excess_i2t_a2s * decay +
                               excess_rate_a2 * input->elapsed_s;
    observer->excess_i2t_a2s = clampf_local(
        observer->excess_i2t_a2s, 0.0f,
        result->usable_i2t_a2s * cfg->maximum_state_multiple);

    result->utilization = observer->excess_i2t_a2s /
                          result->usable_i2t_a2s;
    result->remaining_i2t_a2s = fmaxf(
        0.0f, result->usable_i2t_a2s - observer->excess_i2t_a2s);
    if(result->utilization >= 1.0f)
    {
        observer->budget_exhausted = 1u;
    }
    else if(result->utilization <= 0.50f)
    {
        observer->budget_exhausted = 0u;
    }

    for(uint8_t h = 0u; h < AMS_SOP_HORIZONS; h++)
    {
        float cap_a = 0.0f;
        if(observer->budget_exhausted == 0u)
        {
            cap_a = sqrtf(result->continuous_current_a *
                          result->continuous_current_a +
                          result->remaining_i2t_a2s /
                          sop_cfg->horizons_s[h]);
        }
        result->discharge_current_cap_a[h] = fminf(
            sop_cfg->discharge_current_max_a[h], cap_a);
        if(result->discharge_current_cap_a[h] + 1.0e-3f <
           sop_cfg->discharge_current_max_a[h])
        {
            result->reason_flags |= AMS_FUSE_REASON_BUDGET_DERATED;
        }
    }

    result->valid = 1u;
    result->budget_exhausted = observer->budget_exhausted;
    if(input->model_validated == 0u)
    {
        result->reason_flags |= AMS_FUSE_REASON_MODEL_UNVALIDATED;
    }
    if(observer->thermal_state_initialized == 0u)
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
