#include "fuse_reference_oracle.h"

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

/* Independently transcribed rounded EAC14-80 current-versus-time centerline
 * points from Eaton ELX1308 page 4.  The chart spans 0.01 s to 100 s; the
 * 800 A point is independently anchored from 8020 A^2s / 800^2.  Keep this
 * table separate from the production translation unit. */
typedef struct
{
    long double current_a;
    long double time_s;
} ref_curve_point_t;

static const ref_curve_point_t REF_CURVE[] = {
    {154.0L, 100.0L}, {168.0L, 50.0L}, {180.9L, 30.0L},
    {192.8L, 20.0L}, {219.2L, 10.0L}, {249.2L, 5.0L},
    {274.9L, 3.0L}, {300.8L, 2.0L}, {350.9L, 1.0L},
    {411.1L, 0.5L}, {458.9L, 0.3L}, {500.2L, 0.2L},
    {576.6L, 0.1L}, {652.8L, 0.05L}, {705.8L, 0.03L},
    {752.5L, 0.02L}, {800.0L, 0.01253125L}, {850.0L, 0.01078L},
};

#define REF_CURVE_COUNT \
    ((uint32_t)(sizeof(REF_CURVE) / sizeof(REF_CURVE[0])))

static long double clamp_ld(long double value,
                            long double lower,
                            long double upper)
{
    if(value < lower) return lower;
    if(value > upper) return upper;
    return value;
}

static long double interp_ld(long double x,
                             long double x0,
                             long double y0,
                             long double x1,
                             long double y1)
{
    if(x1 <= x0) return y0;
    const long double f = clamp_ld((x - x0) / (x1 - x0), 0.0L, 1.0L);
    return y0 + f * (y1 - y0);
}

static long double log_interp_ld(long double x,
                                 long double x0,
                                 long double y0,
                                 long double x1,
                                 long double y1)
{
    if((x <= 0.0L) || (x0 <= 0.0L) || (x1 <= x0) ||
       (y0 <= 0.0L) || (y1 <= 0.0L))
    {
        return y0;
    }
    const long double f = clamp_ld((logl(x) - logl(x0)) /
                                   (logl(x1) - logl(x0)), 0.0L, 1.0L);
    return expl(logl(y0) + f * (logl(y1) - logl(y0)));
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
       (fabsl(cfg->rated_current_a - 80.0L) > 1.0e-9L) ||
       !isfinite(cfg->curve_time_fraction) ||
       (cfg->curve_time_fraction <= 0.0L) ||
       (cfg->curve_time_fraction > 1.0L) ||
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
       (cfg->maximum_state_multiple < 1.0L) ||
       !isfinite(cfg->low_current_fit_scale_s) ||
       (cfg->low_current_fit_scale_s <= 0.0L) ||
       !isfinite(cfg->low_current_fit_exponent) ||
       (cfg->low_current_fit_exponent <= 0.0L) ||
       !isfinite(cfg->maximum_curve_time_s) ||
       (cfg->maximum_curve_time_s <= 0.0L) ||
       !isfinite(cfg->minimum_curve_time_s) ||
       (cfg->minimum_curve_time_s <= 0.0L) ||
       (cfg->minimum_curve_time_s >= cfg->maximum_curve_time_s))
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
    if(state != NULL) memset(state, 0, sizeof(*state));
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
    state->thermal_utilization = utilization;
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
    static const long double temp_c[] = {
        -40.0L, 0.0L, 25.0L, 40.0L, 60.0L, 80.0L, 100.0L, 125.0L
    };
    static const long double factor[] = {
        1.15L, 1.06L, 1.00L, 0.97L, 0.93L, 0.89L, 0.85L, 0.80L
    };
    const uint32_t count = (uint32_t)(sizeof(temp_c) / sizeof(temp_c[0]));
    long double value = factor[count - 1u];
    if(fuse_temperature_c <= temp_c[0])
    {
        value = factor[0];
    }
    else
    {
        for(uint32_t i = 1u; i < count; ++i)
        {
            if(fuse_temperature_c <= temp_c[i])
            {
                value = interp_ld(fuse_temperature_c,
                                  temp_c[i - 1u], factor[i - 1u],
                                  temp_c[i], factor[i]);
                break;
            }
        }
    }
    return clamp_ld(value, minimum_derating, 1.0L);
}

long double fuse_ref_typical_melt_time_s(const fuse_ref_config_t *cfg,
                                         long double equivalent_25c_current_a,
                                         uint8_t *extrapolated)
{
    if(extrapolated != NULL) *extrapolated = 0u;
    if(!fuse_ref_config_valid(cfg) ||
       !isfinite(equivalent_25c_current_a) ||
       (equivalent_25c_current_a < 0.0L))
    {
        return NAN;
    }
    if(equivalent_25c_current_a <= cfg->rated_current_a)
    {
        return INFINITY;
    }
    if(equivalent_25c_current_a < REF_CURVE[0].current_a)
    {
        if(extrapolated != NULL) *extrapolated = 1u;
        const long double overcurrent =
            equivalent_25c_current_a / cfg->rated_current_a - 1.0L;
        const long double time_s = cfg->low_current_fit_scale_s *
            powl(overcurrent, -cfg->low_current_fit_exponent);
        return fminl(cfg->maximum_curve_time_s,
                     fmaxl(cfg->minimum_curve_time_s, time_s));
    }
    for(uint32_t i = 1u; i < REF_CURVE_COUNT; ++i)
    {
        if(equivalent_25c_current_a <= REF_CURVE[i].current_a)
        {
            return log_interp_ld(equivalent_25c_current_a,
                                 REF_CURVE[i - 1u].current_a,
                                 REF_CURVE[i - 1u].time_s,
                                 REF_CURVE[i].current_a,
                                 REF_CURVE[i].time_s);
        }
    }
    if(extrapolated != NULL) *extrapolated = 1u;
    const ref_curve_point_t *last = &REF_CURVE[REF_CURVE_COUNT - 1u];
    const ref_curve_point_t *prev = &REF_CURVE[REF_CURVE_COUNT - 2u];
    const long double slope = (logl(last->time_s) - logl(prev->time_s)) /
                              (logl(last->current_a) -
                               logl(prev->current_a));
    const long double time_s = expl(logl(last->time_s) + slope *
        (logl(equivalent_25c_current_a) - logl(last->current_a)));
    return fmaxl(cfg->minimum_curve_time_s,
                 fminl(cfg->maximum_curve_time_s, time_s));
}

static long double source_rate(const fuse_ref_config_t *cfg,
                               long double equivalent_current_a,
                               uint8_t *extrapolated,
                               long double *typical_time_s,
                               long double *usable_time_s)
{
    const long double typical = fuse_ref_typical_melt_time_s(
        cfg, equivalent_current_a, extrapolated);
    if(typical_time_s != NULL) *typical_time_s = typical;
    if(!isfinite(typical))
    {
        if(usable_time_s != NULL) *usable_time_s = INFINITY;
        return 0.0L;
    }
    const long double usable = clamp_ld(
        typical * cfg->curve_time_fraction,
        cfg->minimum_curve_time_s, cfg->maximum_curve_time_s);
    if(usable_time_s != NULL) *usable_time_s = usable;
    const long double kernel = -cfg->cooling_time_constant_s *
        expm1l(-usable / cfg->cooling_time_constant_s);
    if(!isfinite(kernel) || (kernel <= 0.0L))
    {
        return 1.0L / cfg->minimum_curve_time_s;
    }
    return 1.0L / kernel;
}

static long double predicted_exact(const fuse_ref_state_t *state,
                                   const fuse_ref_config_t *cfg,
                                   long double pack_candidate_a,
                                   long double uncertainty_a,
                                   long double derating,
                                   long double horizon_s,
                                   uint8_t *extrapolated)
{
    const long double effective = fmaxl(0.0L, pack_candidate_a) +
                                  uncertainty_a;
    const long double equivalent = effective / derating;
    const long double rate = source_rate(cfg, equivalent, extrapolated,
                                         NULL, NULL);
    const long double ratio = horizon_s / cfg->cooling_time_constant_s;
    const long double decay = expl(-ratio);
    const long double kernel = -cfg->cooling_time_constant_s *
                               expm1l(-ratio);
    return state->thermal_utilization * decay + rate * kernel;
}

static long double solve_cap_exact(const fuse_ref_state_t *state,
                                   const fuse_ref_config_t *cfg,
                                   long double static_cap_a,
                                   long double uncertainty_a,
                                   long double derating,
                                   long double horizon_s,
                                   uint8_t *extrapolated)
{
    if((state->budget_exhausted != 0u) || (static_cap_a <= 0.0L))
    {
        return 0.0L;
    }
    uint8_t local = 0u;
    const long double at_static = predicted_exact(
        state, cfg, static_cap_a, uncertainty_a, derating, horizon_s, &local);
    if(local != 0u && extrapolated != NULL) *extrapolated = 1u;
    if(at_static <= 1.0L) return static_cap_a;

    long double low = 0.0L;
    long double high = static_cap_a;
    for(uint32_t i = 0u; i < 64u; ++i)
    {
        const long double mid = 0.5L * (low + high);
        local = 0u;
        const long double value = predicted_exact(
            state, cfg, mid, uncertainty_a, derating, horizon_s, &local);
        if(local != 0u && extrapolated != NULL) *extrapolated = 1u;
        if(value <= 1.0L) low = mid;
        else high = mid;
    }
    return low;
}

bool fuse_ref_step_exact_zoh(fuse_ref_state_t *state,
                             const fuse_ref_config_t *cfg,
                             const fuse_ref_input_t *input,
                             fuse_ref_result_t *result)
{
    if((state == NULL) || (result == NULL)) return false;
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
        if(state->invalid_count != UINT64_MAX) ++state->invalid_count;
        return false;
    }
    if(state->update_count != UINT64_MAX) ++state->update_count;
    result->reason_flags = FUSE_REF_REASON_NONE;

    result->effective_current_a = fabsl(input->pack_current_a) +
                                  input->current_uncertainty_a;
    if(state->thermal_state_initialized == 0u)
    {
        if(result->effective_current_a <= cfg->quiescent_current_a)
        {
            state->quiescent_time_s += input->elapsed_s;
            if(state->quiescent_time_s >= cfg->initialization_soak_s)
            {
                state->thermal_state_initialized = 1u;
                state->thermal_utilization = 0.0L;
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
    result->equivalent_25c_current_a = result->effective_current_a /
                                       result->temperature_derating;

    uint8_t extrapolated = 0u;
    const long double rate = source_rate(
        cfg, result->equivalent_25c_current_a, &extrapolated,
        &result->typical_melt_time_s, &result->usable_melt_time_s);
    if(extrapolated != 0u)
    {
        result->curve_extrapolated = 1u;
        result->reason_flags |= FUSE_REF_REASON_CURVE_EXTRAPOLATED;
    }

    const long double ratio = input->elapsed_s /
                              cfg->cooling_time_constant_s;
    const long double decay = expl(-ratio);
    const long double kernel = -cfg->cooling_time_constant_s *
                               expm1l(-ratio);
    state->thermal_utilization = state->thermal_utilization * decay +
                                 rate * kernel;
    state->thermal_utilization = clamp_ld(
        state->thermal_utilization, 0.0L, cfg->maximum_state_multiple);

    result->utilization = state->thermal_utilization;
    result->remaining_utilization = fmaxl(
        0.0L, 1.0L - state->thermal_utilization);
    if(result->utilization >= 1.0L) state->budget_exhausted = 1u;
    else if(result->utilization <= 0.50L) state->budget_exhausted = 0u;

    for(uint32_t h = 0u; h < FUSE_REF_HORIZON_COUNT; ++h)
    {
        uint8_t cap_extrapolated = 0u;
        result->discharge_current_cap_a[h] = solve_cap_exact(
            state, cfg, cfg->discharge_static_cap_a[h],
            input->current_uncertainty_a, result->temperature_derating,
            cfg->horizons_s[h], &cap_extrapolated);
        if(cap_extrapolated != 0u)
        {
            result->curve_extrapolated = 1u;
            result->reason_flags |= FUSE_REF_REASON_CURVE_EXTRAPOLATED;
        }
        if(result->discharge_current_cap_a[h] + 1.0e-3L <
           cfg->discharge_static_cap_a[h])
        {
            result->reason_flags |= FUSE_REF_REASON_CURVE_DERATED;
        }
    }

    result->valid = 1u;
    result->budget_exhausted = state->budget_exhausted;
    if(input->model_validated == 0u)
        result->reason_flags |= FUSE_REF_REASON_MODEL_UNVALIDATED;
    if(state->thermal_state_initialized == 0u)
        result->reason_flags |= FUSE_REF_REASON_INITIAL_STATE_UNKNOWN;
    if(state->budget_exhausted != 0u)
        result->reason_flags |= FUSE_REF_REASON_BUDGET_EXHAUSTED;
    result->authority_valid = ((input->model_validated != 0u) &&
        (state->thermal_state_initialized != 0u)) ? 1u : 0u;
    return true;
}

long double fuse_ref_integrate_trapezoidal(long double initial_utilization,
                                           long double source_rate_per_s,
                                           long double elapsed_s,
                                           long double cooling_tau_s,
                                           uint32_t subdivisions)
{
    if(!isfinite(initial_utilization) || (initial_utilization < 0.0L) ||
       !isfinite(source_rate_per_s) || (source_rate_per_s < 0.0L) ||
       !isfinite(elapsed_s) || (elapsed_s < 0.0L) ||
       !isfinite(cooling_tau_s) || (cooling_tau_s <= 0.0L) ||
       (subdivisions == 0u))
    {
        return NAN;
    }
    const long double h = elapsed_s / (long double)subdivisions;
    long double x = initial_utilization;
    for(uint32_t i = 0u; i < subdivisions; ++i)
    {
        const long double slope0 = -x / cooling_tau_s + source_rate_per_s;
        const long double predicted = x + h * slope0;
        const long double slope1 = -predicted / cooling_tau_s +
                                   source_rate_per_s;
        x += 0.5L * h * (slope0 + slope1);
        if((x < 0.0L) && (x > -64.0L * LDBL_EPSILON)) x = 0.0L;
    }
    return x;
}
