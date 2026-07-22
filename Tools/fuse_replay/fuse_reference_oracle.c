#include "fuse_reference_oracle.h"

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

static long double clamp_ld(long double value,
                            long double lower,
                            long double upper)
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

static long double interp_ld(long double x,
                             long double x0,
                             long double y0,
                             long double x1,
                             long double y1)
{
    if(x1 <= x0)
    {
        return y0;
    }
    const long double f = clamp_ld((x - x0) / (x1 - x0), 0.0L, 1.0L);
    return y0 + f * (y1 - y0);
}

static void result_invalid(fuse_ref_result_t *result)
{
    if(result != NULL)
    {
        memset(result, 0, sizeof(*result));
        result->utilization = 1.0L;
        result->reason_flags = FUSE_REF_REASON_INPUT_INVALID;
    }
}

bool fuse_ref_config_valid(const fuse_ref_config_t *cfg)
{
    if((cfg == NULL) || !isfinite(cfg->rated_current_a) ||
       (cfg->rated_current_a <= 0.0L) ||
       !isfinite(cfg->typical_melting_i2t_a2s) ||
       (cfg->typical_melting_i2t_a2s <= 0.0L) ||
       !isfinite(cfg->usable_i2t_fraction) ||
       (cfg->usable_i2t_fraction <= 0.0L) ||
       (cfg->usable_i2t_fraction > 1.0L) ||
       !isfinite(cfg->cooling_time_constant_s) ||
       (cfg->cooling_time_constant_s <= 0.0L) ||
       !isfinite(cfg->initialization_soak_s) ||
       (cfg->initialization_soak_s < 0.0L) ||
       !isfinite(cfg->quiescent_current_a) ||
       (cfg->quiescent_current_a < 0.0L) ||
       !isfinite(cfg->fuse_temperature_margin_c) ||
       (cfg->fuse_temperature_margin_c < 0.0L) ||
       !isfinite(cfg->minimum_temperature_derating) ||
       (cfg->minimum_temperature_derating <= 0.0L) ||
       (cfg->minimum_temperature_derating > 1.0L) ||
       !isfinite(cfg->maximum_state_multiple) ||
       (cfg->maximum_state_multiple < 1.0L))
    {
        return false;
    }

    for(uint32_t i = 0u; i < FUSE_REF_HORIZON_COUNT; ++i)
    {
        if(!isfinite(cfg->horizons_s[i]) || (cfg->horizons_s[i] <= 0.0L) ||
           !isfinite(cfg->discharge_static_cap_a[i]) ||
           (cfg->discharge_static_cap_a[i] < 0.0L))
        {
            return false;
        }
    }
    return true;
}

void fuse_ref_state_init(fuse_ref_state_t *state)
{
    if(state != NULL)
    {
        memset(state, 0, sizeof(*state));
    }
}

bool fuse_ref_state_seed_utilization(fuse_ref_state_t *state,
                                     const fuse_ref_config_t *cfg,
                                     long double utilization)
{
    if((state == NULL) || !fuse_ref_config_valid(cfg) ||
       !isfinite(utilization) || (utilization < 0.0L) ||
       (utilization > cfg->maximum_state_multiple))
    {
        return false;
    }

    const long double budget = cfg->typical_melting_i2t_a2s *
                               cfg->usable_i2t_fraction;
    state->excess_i2t_a2s = utilization * budget;
    state->quiescent_time_s = cfg->initialization_soak_s;
    state->thermal_state_initialized = 1u;
    state->budget_exhausted = (utilization >= 1.0L) ? 1u : 0u;
    return true;
}

long double fuse_ref_temperature_derating(long double fuse_temperature_c,
                                          long double minimum_derating)
{
    if(!isfinite(fuse_temperature_c) || !isfinite(minimum_derating) ||
       (minimum_derating <= 0.0L) || (minimum_derating > 1.0L))
    {
        return 0.0L;
    }

    long double derating;
    if(fuse_temperature_c <= 0.0L)
    {
        derating = 1.03L;
    }
    else if(fuse_temperature_c <= 25.0L)
    {
        derating = interp_ld(fuse_temperature_c,
                             0.0L, 1.03L, 25.0L, 1.00L);
    }
    else if(fuse_temperature_c <= 80.0L)
    {
        derating = interp_ld(fuse_temperature_c,
                             25.0L, 1.00L, 80.0L, 0.90L);
    }
    else if(fuse_temperature_c <= 125.0L)
    {
        derating = interp_ld(fuse_temperature_c,
                             80.0L, 0.90L, 125.0L, 0.80L);
    }
    else
    {
        derating = 0.80L;
    }
    return clamp_ld(derating, minimum_derating, 1.0L);
}

bool fuse_ref_step_exact_zoh(fuse_ref_state_t *state,
                             const fuse_ref_config_t *cfg,
                             const fuse_ref_input_t *input,
                             fuse_ref_result_t *result)
{
    if((state == NULL) || (result == NULL))
    {
        return false;
    }
    result_invalid(result);

    const bool valid = fuse_ref_config_valid(cfg) && (input != NULL) &&
        isfinite(input->pack_current_a) &&
        isfinite(input->current_uncertainty_a) &&
        (input->current_uncertainty_a >= 0.0L) &&
        isfinite(input->temperature_proxy_c) &&
        isfinite(input->elapsed_s) && (input->elapsed_s > 0.0L) &&
        (input->measurement_valid != 0u) &&
        (input->current_calibrated != 0u) &&
        (input->current_polarity_validated != 0u);
    if(!valid)
    {
        if(state->invalid_count != UINT64_MAX)
        {
            ++state->invalid_count;
        }
        return false;
    }

    if(state->update_count != UINT64_MAX)
    {
        ++state->update_count;
    }
    result->reason_flags = FUSE_REF_REASON_NONE;

    const long double effective_current_a = fabsl(input->pack_current_a) +
                                            input->current_uncertainty_a;
    if(state->thermal_state_initialized == 0u)
    {
        if(effective_current_a <= cfg->quiescent_current_a)
        {
            state->quiescent_time_s += input->elapsed_s;
            if(state->quiescent_time_s >= cfg->initialization_soak_s)
            {
                state->thermal_state_initialized = 1u;
                state->excess_i2t_a2s = 0.0L;
                state->budget_exhausted = 0u;
            }
        }
        else
        {
            state->quiescent_time_s = 0.0L;
        }
    }

    result->estimated_fuse_temperature_c = input->temperature_proxy_c;
    if(input->temperature_measured_at_fuse == 0u)
    {
        result->estimated_fuse_temperature_c +=
            cfg->fuse_temperature_margin_c;
        result->reason_flags |= FUSE_REF_REASON_TEMPERATURE_PROXY;
    }

    result->temperature_derating = fuse_ref_temperature_derating(
        result->estimated_fuse_temperature_c,
        cfg->minimum_temperature_derating);
    result->continuous_current_a = cfg->rated_current_a *
                                   result->temperature_derating;
    result->usable_i2t_a2s = cfg->typical_melting_i2t_a2s *
                             cfg->usable_i2t_fraction;

    const long double ratio = input->elapsed_s /
                              cfg->cooling_time_constant_s;
    const long double decay = expl(-ratio);
    const long double kernel_s = -cfg->cooling_time_constant_s *
                                 expm1l(-ratio);
    const long double excess_rate_a2 = fmaxl(
        0.0L,
        effective_current_a * effective_current_a -
        result->continuous_current_a * result->continuous_current_a);

    state->excess_i2t_a2s = state->excess_i2t_a2s * decay +
                            excess_rate_a2 * kernel_s;
    state->excess_i2t_a2s = clamp_ld(
        state->excess_i2t_a2s,
        0.0L,
        result->usable_i2t_a2s * cfg->maximum_state_multiple);

    result->utilization = state->excess_i2t_a2s /
                          result->usable_i2t_a2s;
    result->remaining_i2t_a2s = fmaxl(
        0.0L,
        result->usable_i2t_a2s - state->excess_i2t_a2s);

    if(result->utilization >= 1.0L)
    {
        state->budget_exhausted = 1u;
    }
    else if(result->utilization <= 0.50L)
    {
        state->budget_exhausted = 0u;
    }

    for(uint32_t h = 0u; h < FUSE_REF_HORIZON_COUNT; ++h)
    {
        long double cap_a = 0.0L;
        if(state->budget_exhausted == 0u)
        {
            cap_a = sqrtl(
                result->continuous_current_a * result->continuous_current_a +
                result->remaining_i2t_a2s / cfg->horizons_s[h]);
        }
        result->discharge_current_cap_a[h] = fminl(
            cfg->discharge_static_cap_a[h], cap_a);
        if(result->discharge_current_cap_a[h] + 1.0e-3L <
           cfg->discharge_static_cap_a[h])
        {
            result->reason_flags |= FUSE_REF_REASON_BUDGET_DERATED;
        }
    }

    result->valid = 1u;
    result->budget_exhausted = state->budget_exhausted;
    if(input->model_validated == 0u)
    {
        result->reason_flags |= FUSE_REF_REASON_MODEL_UNVALIDATED;
    }
    if(state->thermal_state_initialized == 0u)
    {
        result->reason_flags |= FUSE_REF_REASON_INITIAL_STATE_UNKNOWN;
    }
    if(state->budget_exhausted != 0u)
    {
        result->reason_flags |= FUSE_REF_REASON_BUDGET_EXHAUSTED;
    }
    result->authority_valid = ((input->model_validated != 0u) &&
        (state->thermal_state_initialized != 0u)) ? 1u : 0u;
    return true;
}

long double fuse_ref_integrate_trapezoidal(long double initial_i2t_a2s,
                                           long double excess_rate_a2,
                                           long double elapsed_s,
                                           long double cooling_tau_s,
                                           uint32_t subdivisions)
{
    if(!isfinite(initial_i2t_a2s) || (initial_i2t_a2s < 0.0L) ||
       !isfinite(excess_rate_a2) || (excess_rate_a2 < 0.0L) ||
       !isfinite(elapsed_s) || (elapsed_s < 0.0L) ||
       !isfinite(cooling_tau_s) || (cooling_tau_s <= 0.0L) ||
       (subdivisions == 0u))
    {
        return NAN;
    }

    const long double h = elapsed_s / (long double)subdivisions;
    long double x = initial_i2t_a2s;
    for(uint32_t i = 0u; i < subdivisions; ++i)
    {
        const long double slope0 = -x / cooling_tau_s + excess_rate_a2;
        const long double predicted = x + h * slope0;
        const long double slope1 = -predicted / cooling_tau_s +
                                   excess_rate_a2;
        x += 0.5L * h * (slope0 + slope1);
        if(x < 0.0L && x > -64.0L * LDBL_EPSILON)
        {
            x = 0.0L;
        }
    }
    return x;
}
