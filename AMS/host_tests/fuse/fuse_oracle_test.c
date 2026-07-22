#include "fuse_reference_oracle.h"
#include "sop/ams_fuse_observer.h"
#include "sop/ams_sop.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define ASSERT_TRUE(expr) do { \
    if(!(expr)) { \
        fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
        return false; \
    } \
} while(0)

static void map_config(const ams_fuse_observer_config_t *prod,
                       const ams_sop_config_t *sop,
                       fuse_ref_config_t *ref)
{
    memset(ref, 0, sizeof(*ref));
    ref->rated_current_a = prod->rated_current_a;
    ref->typical_melting_i2t_a2s = prod->typical_melting_i2t_a2s;
    ref->usable_i2t_fraction = prod->usable_i2t_fraction;
    ref->cooling_time_constant_s = prod->cooling_time_constant_s;
    ref->initialization_soak_s = prod->initialization_soak_s;
    ref->quiescent_current_a = prod->quiescent_current_a;
    ref->fuse_temperature_margin_c = prod->fuse_temperature_margin_c;
    ref->minimum_temperature_derating = prod->minimum_temperature_derating;
    ref->maximum_state_multiple = prod->maximum_state_multiple;
    for(uint32_t h = 0u; h < FUSE_REF_HORIZON_COUNT; ++h)
    {
        ref->horizons_s[h] = sop->horizons_s[h];
        ref->discharge_static_cap_a[h] = sop->discharge_current_max_a[h];
    }
}

static void map_input(const ams_fuse_observer_input_t *prod,
                      fuse_ref_input_t *ref)
{
    memset(ref, 0, sizeof(*ref));
    ref->pack_current_a = prod->pack_current_a;
    ref->current_uncertainty_a = prod->current_uncertainty_a;
    ref->temperature_proxy_c = prod->temperature_proxy_c;
    ref->elapsed_s = prod->elapsed_s;
    ref->measurement_valid = prod->measurement_valid;
    ref->current_calibrated = prod->current_calibrated;
    ref->current_polarity_validated = prod->current_polarity_validated;
    ref->temperature_measured_at_fuse = prod->temperature_measured_at_fuse;
    ref->model_validated = prod->model_validated;
}

static bool state_close(float production, long double reference)
{
    const long double diff = fabsl((long double)production - reference);
    const long double tolerance = 0.20L + 1.0e-3L * fmaxl(1.0L, fabsl(reference));
    return diff <= tolerance;
}

static bool cap_conservative(float production, long double reference)
{
    return (long double)production <= reference + 0.03L;
}

static bool test_exact_oracle_against_trapezoidal(void)
{
    const long double initial = 123.456L;
    const long double rate = 4567.89L;
    const long double dt = 0.2L;
    const long double tau = 300.0L;
    const long double decay = expl(-dt / tau);
    const long double exact = initial * decay +
        rate * tau * (1.0L - decay);
    const long double trap = fuse_ref_integrate_trapezoidal(
        initial, rate, dt, tau, 10000u);
    ASSERT_TRUE(isfinite(trap));
    ASSERT_TRUE(fabsl(exact - trap) < 1.0e-8L);

    const long double cool_exact = initial * decay;
    const long double cool_trap = fuse_ref_integrate_trapezoidal(
        initial, 0.0L, dt, tau, 10000u);
    ASSERT_TRUE(fabsl(cool_exact - cool_trap) < 1.0e-10L);
    return true;
}

static bool test_reference_temperature_curve(void)
{
    ASSERT_TRUE(fabsl(fuse_ref_temperature_derating(-10.0L, 0.75L) -
                      1.0L) < 1.0e-15L);
    ASSERT_TRUE(fabsl(fuse_ref_temperature_derating(25.0L, 0.75L) -
                      1.0L) < 1.0e-15L);
    ASSERT_TRUE(fabsl(fuse_ref_temperature_derating(80.0L, 0.75L) -
                      0.90L) < 1.0e-15L);
    ASSERT_TRUE(fabsl(fuse_ref_temperature_derating(125.0L, 0.75L) -
                      0.80L) < 1.0e-15L);
    ASSERT_TRUE(fabsl(fuse_ref_temperature_derating(200.0L, 0.85L) -
                      0.85L) < 1.0e-15L);
    return true;
}

static bool test_directed_production_comparison(void)
{
    ams_fuse_observer_config_t pcfg;
    ams_sop_config_t scfg;
    ams_fuse_observer_t pstate;
    ams_fuse_observer_input_t pin;
    ams_fuse_observer_result_t pout;
    fuse_ref_config_t rcfg;
    fuse_ref_state_t rstate;
    fuse_ref_input_t rin;
    fuse_ref_result_t rout;

    ams_fuse_observer_default_config(&pcfg);
    ams_sop_default_config(&scfg);
    map_config(&pcfg, &scfg, &rcfg);
    ASSERT_TRUE(fuse_ref_config_valid(&rcfg));
    ams_fuse_observer_init(&pstate);
    fuse_ref_state_init(&rstate);

    memset(&pin, 0, sizeof(pin));
    pin.current_uncertainty_a = 0.5f;
    pin.temperature_proxy_c = 25.0f;
    pin.elapsed_s = 1.0f;
    pin.measurement_valid = 1u;
    pin.current_calibrated = 1u;
    pin.current_polarity_validated = 1u;
    pin.model_validated = 1u;

    for(uint32_t i = 0u; i < 300u; ++i)
    {
        map_input(&pin, &rin);
        ASSERT_TRUE(ams_fuse_observer_update(&pstate, &pcfg, &scfg,
                                             &pin, &pout));
        ASSERT_TRUE(fuse_ref_step_exact_zoh(&rstate, &rcfg, &rin, &rout));
    }
    ASSERT_TRUE(pout.authority_valid == 1u && rout.authority_valid == 1u);
    ASSERT_TRUE(pstate.thermal_state_initialized == 1u &&
                rstate.thermal_state_initialized == 1u);

    pin.pack_current_a = 100.0f;
    pin.elapsed_s = 0.1f;
    for(uint32_t i = 0u; i < 3u; ++i)
    {
        map_input(&pin, &rin);
        ASSERT_TRUE(ams_fuse_observer_update(&pstate, &pcfg, &scfg,
                                             &pin, &pout));
        ASSERT_TRUE(fuse_ref_step_exact_zoh(&rstate, &rcfg, &rin, &rout));
        ASSERT_TRUE((long double)pstate.excess_i2t_a2s + 1.0e-4L >=
                    rstate.excess_i2t_a2s);
        ASSERT_TRUE(state_close(pstate.excess_i2t_a2s,
                                rstate.excess_i2t_a2s));
        for(uint32_t h = 0u; h < FUSE_REF_HORIZON_COUNT; ++h)
        {
            ASSERT_TRUE(cap_conservative(pout.discharge_current_cap_a[h],
                                         rout.discharge_current_cap_a[h]));
        }
    }

    /* Cooling-only uses the same exponential kernel and should be extremely
     * close even though the two paths use different precision. */
    pin.pack_current_a = 0.0f;
    pin.current_uncertainty_a = 0.0f;
    pin.elapsed_s = 0.1f;
    for(uint32_t i = 0u; i < 100u; ++i)
    {
        map_input(&pin, &rin);
        ASSERT_TRUE(ams_fuse_observer_update(&pstate, &pcfg, &scfg,
                                             &pin, &pout));
        ASSERT_TRUE(fuse_ref_step_exact_zoh(&rstate, &rcfg, &rin, &rout));
        ASSERT_TRUE(state_close(pstate.excess_i2t_a2s,
                                rstate.excess_i2t_a2s));
    }
    return true;
}

static uint32_t rng_state = 0xEAC1480u;
static uint32_t next_u32(void)
{
    uint32_t x = rng_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    rng_state = x;
    return x;
}

static float randf(float lo, float hi)
{
    const float u = (float)(next_u32() >> 8) / (float)(1u << 24);
    return lo + (hi - lo) * u;
}

static bool test_randomized_production_vs_independent_oracle(void)
{
    ams_fuse_observer_config_t pcfg;
    ams_sop_config_t scfg;
    ams_fuse_observer_t pstate;
    ams_fuse_observer_input_t pin;
    ams_fuse_observer_result_t pout;
    fuse_ref_config_t rcfg;
    fuse_ref_state_t rstate;
    fuse_ref_input_t rin;
    fuse_ref_result_t rout;

    ams_fuse_observer_default_config(&pcfg);
    ams_sop_default_config(&scfg);
    map_config(&pcfg, &scfg, &rcfg);
    ams_fuse_observer_init(&pstate);
    fuse_ref_state_init(&rstate);

    /* Start from the same known warm state so the comparison focuses on the
     * integration, derating, cap, and latch calculations. */
    const long double seed_util = 0.35L;
    const long double budget = rcfg.typical_melting_i2t_a2s *
                               rcfg.usable_i2t_fraction;
    pstate.excess_i2t_a2s = (float)(seed_util * budget);
    pstate.quiescent_time_s = pcfg.initialization_soak_s;
    pstate.thermal_state_initialized = 1u;
    ASSERT_TRUE(fuse_ref_state_seed_utilization(&rstate, &rcfg, seed_util));

    memset(&pin, 0, sizeof(pin));
    pin.measurement_valid = 1u;
    pin.current_calibrated = 1u;
    pin.current_polarity_validated = 1u;
    pin.model_validated = 1u;

    uint32_t transition_skew_samples = 0u;
    for(uint32_t i = 0u; i < 50000u; ++i)
    {
        pin.pack_current_a = randf(-35.0f, 125.0f);
        pin.current_uncertainty_a = randf(0.0f, 2.0f);
        pin.temperature_proxy_c = randf(-10.0f, 85.0f);
        pin.elapsed_s = randf(0.01f, 0.20f);
        pin.temperature_measured_at_fuse = (uint8_t)(next_u32() & 1u);
        map_input(&pin, &rin);

        ASSERT_TRUE(ams_fuse_observer_update(&pstate, &pcfg, &scfg,
                                             &pin, &pout));
        ASSERT_TRUE(fuse_ref_step_exact_zoh(&rstate, &rcfg, &rin, &rout));

        /* q*dt is a conservative upper approximation of the exact ZOH
         * injection.  The production observer must therefore never have less
         * thermal state than the exact reference, apart from float noise. */
        ASSERT_TRUE((long double)pstate.excess_i2t_a2s + 0.03L >=
                    rstate.excess_i2t_a2s);
        ASSERT_TRUE(state_close(pstate.excess_i2t_a2s,
                                rstate.excess_i2t_a2s));
        ASSERT_TRUE(pout.authority_valid == rout.authority_valid);

        for(uint32_t h = 0u; h < FUSE_REF_HORIZON_COUNT; ++h)
        {
            ASSERT_TRUE(cap_conservative(pout.discharge_current_cap_a[h],
                                         rout.discharge_current_cap_a[h]));
        }

        /* A conservative production state may latch one sample earlier and
         * clear one sample later.  It must never be less conservative. */
        if(pout.budget_exhausted != rout.budget_exhausted)
        {
            ++transition_skew_samples;
            ASSERT_TRUE((pout.budget_exhausted != 0u) &&
                        (rout.budget_exhausted == 0u));
        }
    }

    ASSERT_TRUE(transition_skew_samples < 100u);
    return true;
}

static bool test_invalid_fail_closed(void)
{
    ams_fuse_observer_config_t pcfg;
    ams_sop_config_t scfg;
    ams_fuse_observer_t pstate;
    ams_fuse_observer_input_t pin;
    ams_fuse_observer_result_t pout;
    fuse_ref_config_t rcfg;
    fuse_ref_state_t rstate;
    fuse_ref_input_t rin;
    fuse_ref_result_t rout;

    ams_fuse_observer_default_config(&pcfg);
    ams_sop_default_config(&scfg);
    map_config(&pcfg, &scfg, &rcfg);
    ams_fuse_observer_init(&pstate);
    fuse_ref_state_init(&rstate);
    memset(&pin, 0, sizeof(pin));
    pin.pack_current_a = NAN;
    pin.elapsed_s = 0.1f;
    pin.measurement_valid = 1u;
    pin.current_calibrated = 1u;
    pin.current_polarity_validated = 1u;
    map_input(&pin, &rin);

    ASSERT_TRUE(!ams_fuse_observer_update(&pstate, &pcfg, &scfg,
                                          &pin, &pout));
    ASSERT_TRUE(!fuse_ref_step_exact_zoh(&rstate, &rcfg, &rin, &rout));
    ASSERT_TRUE(pout.valid == 0u && pout.authority_valid == 0u &&
                pout.utilization == 1.0f);
    ASSERT_TRUE(rout.valid == 0u && rout.authority_valid == 0u &&
                rout.utilization == 1.0L);
    return true;
}

int main(void)
{
    struct test_case
    {
        const char *name;
        bool (*fn)(void);
    } tests[] = {
        {"exact oracle vs trapezoidal integration",
         test_exact_oracle_against_trapezoidal},
        {"reference temperature derating curve",
         test_reference_temperature_curve},
        {"directed production/reference comparison",
         test_directed_production_comparison},
        {"50k randomized production/reference comparison",
         test_randomized_production_vs_independent_oracle},
        {"invalid input fail-closed",
         test_invalid_fail_closed},
    };

    const size_t count = sizeof(tests) / sizeof(tests[0]);
    for(size_t i = 0u; i < count; ++i)
    {
        if(!tests[i].fn())
        {
            fprintf(stderr, "FAILED: %s\n", tests[i].name);
            return EXIT_FAILURE;
        }
        printf("PASS: %s\n", tests[i].name);
    }
    printf("Fuse oracle validation: PASS (%zu tests)\n", count);
    return EXIT_SUCCESS;
}
