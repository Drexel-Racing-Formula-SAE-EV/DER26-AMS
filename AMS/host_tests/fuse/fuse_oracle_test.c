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
    ref->curve_time_fraction = prod->curve_time_fraction;
    ref->cooling_time_constant_s = prod->cooling_time_constant_s;
    ref->initialization_soak_s = prod->initialization_soak_s;
    ref->quiescent_current_a = prod->quiescent_current_a;
    ref->fuse_temperature_margin_c = prod->fuse_temperature_margin_c;
    ref->minimum_temperature_derating = prod->minimum_temperature_derating;
    ref->maximum_state_multiple = prod->maximum_state_multiple;
    ref->low_current_fit_scale_s = prod->low_current_fit_scale_s;
    ref->low_current_fit_exponent = prod->low_current_fit_exponent;
    ref->maximum_curve_time_s = prod->maximum_curve_time_s;
    ref->minimum_curve_time_s = prod->minimum_curve_time_s;
    for(uint32_t h = 0u; h < FUSE_REF_HORIZON_COUNT; ++h)
    {
        ref->horizons_s[h] = sop->horizons_s[h];
        ref->discharge_static_cap_a[h] = sop->discharge_current_max_a[h];
        ref->charge_static_cap_a[h] = sop->charge_current_max_a[h];
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
    const long double tolerance = 2.0e-5L +
        2.0e-3L * fmaxl(1.0e-3L, fabsl(reference));
    return diff <= tolerance;
}

static bool cap_conservative(float production, long double reference)
{
    return (long double)production <= reference + 0.03L;
}

static bool test_exact_oracle_against_trapezoidal(void)
{
    const long double initial = 0.37L;
    const long double rate = 0.021L;
    const long double dt = 0.2L;
    const long double tau = 300.0L;
    const long double decay = expl(-dt / tau);
    const long double exact = initial * decay +
        rate * tau * (1.0L - decay);
    const long double trap = fuse_ref_integrate_trapezoidal(
        initial, rate, dt, tau, 10000u);
    ASSERT_TRUE(isfinite(trap));
    ASSERT_TRUE(fabsl(exact - trap) < 1.0e-10L);

    const long double cool_exact = initial * decay;
    const long double cool_trap = fuse_ref_integrate_trapezoidal(
        initial, 0.0L, dt, tau, 10000u);
    ASSERT_TRUE(fabsl(cool_exact - cool_trap) < 1.0e-12L);
    return true;
}

static bool test_curve_and_temperature_anchors(void)
{
    ams_fuse_observer_config_t pcfg;
    ams_sop_config_t scfg;
    fuse_ref_config_t rcfg;
    ams_fuse_observer_default_config(&pcfg);
    ams_sop_default_config(&scfg);
    map_config(&pcfg, &scfg, &rcfg);

    uint8_t pe = 0u;
    uint8_t re = 0u;
    const float p800 = ams_fuse_typical_melt_time_s(&pcfg, 800.0f, &pe);
    const long double r800 = fuse_ref_typical_melt_time_s(&rcfg, 800.0L, &re);
    ASSERT_TRUE(pe == 0u && re == 0u);
    ASSERT_TRUE(fabsf(p800 - 0.01253125f) < 1.0e-7f);
    ASSERT_TRUE(fabsl(r800 - 0.01253125L) < 1.0e-12L);

    ASSERT_TRUE(fabsf(ams_fuse_typical_melt_time_s(&pcfg, 350.9f, &pe) -
                      1.0f) < 1.0e-5f);
    ASSERT_TRUE(fabsl(fuse_ref_typical_melt_time_s(&rcfg, 154.0L, &re) -
                      100.0L) < 1.0e-9L);
    ASSERT_TRUE(isinf(ams_fuse_typical_melt_time_s(&pcfg, 80.0f, &pe)));
    ASSERT_TRUE(isinf(fuse_ref_typical_melt_time_s(&rcfg, 80.0L, &re)));

    pe = 0u;
    re = 0u;
    const float p100 = ams_fuse_typical_melt_time_s(&pcfg, 100.0f, &pe);
    const long double r100 = fuse_ref_typical_melt_time_s(&rcfg, 100.0L, &re);
    ASSERT_TRUE(pe == 1u && re == 1u);
    ASSERT_TRUE(p100 > 10000.0f && r100 > 10000.0L);
    ASSERT_TRUE(fabsl((long double)p100 - r100) / r100 < 1.0e-4L);

    ASSERT_TRUE(fabsl(fuse_ref_temperature_derating(-10.0L, 0.75L) -
                      1.0L) < 1.0e-15L);
    ASSERT_TRUE(fabsl(fuse_ref_temperature_derating(25.0L, 0.75L) -
                      1.0L) < 1.0e-15L);
    ASSERT_TRUE(fabsl(fuse_ref_temperature_derating(80.0L, 0.75L) -
                      0.89L) < 1.0e-15L);
    ASSERT_TRUE(fabsl(fuse_ref_temperature_derating(125.0L, 0.75L) -
                      0.80L) < 1.0e-15L);
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

    pin.pack_current_a = 180.0f;
    pin.elapsed_s = 0.1f;
    for(uint32_t i = 0u; i < 20u; ++i)
    {
        map_input(&pin, &rin);
        ASSERT_TRUE(ams_fuse_observer_update(&pstate, &pcfg, &scfg,
                                             &pin, &pout));
        ASSERT_TRUE(fuse_ref_step_exact_zoh(&rstate, &rcfg, &rin, &rout));
        ASSERT_TRUE((long double)pstate.thermal_utilization + 1.0e-7L >=
                    rstate.thermal_utilization);
        ASSERT_TRUE(state_close(pstate.thermal_utilization,
                                rstate.thermal_utilization));
        for(uint32_t h = 0u; h < FUSE_REF_HORIZON_COUNT; ++h)
        {
            ASSERT_TRUE(cap_conservative(pout.discharge_current_cap_a[h],
                                         rout.discharge_current_cap_a[h]));
            ASSERT_TRUE(cap_conservative(pout.charge_current_cap_a[h],
                                         rout.charge_current_cap_a[h]));
        }
    }
    ASSERT_TRUE(pout.utilization > 0.0f);

    pin.pack_current_a = 0.0f;
    pin.current_uncertainty_a = 0.0f;
    for(uint32_t i = 0u; i < 100u; ++i)
    {
        map_input(&pin, &rin);
        ASSERT_TRUE(ams_fuse_observer_update(&pstate, &pcfg, &scfg,
                                             &pin, &pout));
        ASSERT_TRUE(fuse_ref_step_exact_zoh(&rstate, &rcfg, &rin, &rout));
        ASSERT_TRUE(state_close(pstate.thermal_utilization,
                                rstate.thermal_utilization));
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

    pstate.thermal_utilization = 0.35f;
    pstate.quiescent_time_s = pcfg.initialization_soak_s;
    pstate.thermal_state_initialized = 1u;
    ASSERT_TRUE(fuse_ref_state_seed_utilization(&rstate, &rcfg, 0.35L));

    memset(&pin, 0, sizeof(pin));
    pin.measurement_valid = 1u;
    pin.current_calibrated = 1u;
    pin.current_polarity_validated = 1u;
    pin.model_validated = 1u;

    uint32_t transition_skew_samples = 0u;
    for(uint32_t i = 0u; i < 50000u; ++i)
    {
        pin.pack_current_a = randf(-80.0f, 220.0f);
        pin.current_uncertainty_a = randf(0.0f, 2.0f);
        pin.temperature_proxy_c = randf(-10.0f, 95.0f);
        pin.elapsed_s = randf(0.01f, 0.20f);
        pin.temperature_measured_at_fuse = (uint8_t)(next_u32() & 1u);
        map_input(&pin, &rin);

        ASSERT_TRUE(ams_fuse_observer_update(&pstate, &pcfg, &scfg,
                                             &pin, &pout));
        ASSERT_TRUE(fuse_ref_step_exact_zoh(&rstate, &rcfg, &rin, &rout));

        ASSERT_TRUE((long double)pstate.thermal_utilization + 2.0e-5L >=
                    rstate.thermal_utilization);
        ASSERT_TRUE(state_close(pstate.thermal_utilization,
                                rstate.thermal_utilization));
        ASSERT_TRUE(pout.authority_valid == rout.authority_valid);
        for(uint32_t h = 0u; h < FUSE_REF_HORIZON_COUNT; ++h)
        {
            ASSERT_TRUE(cap_conservative(pout.discharge_current_cap_a[h],
                                         rout.discharge_current_cap_a[h]));
            ASSERT_TRUE(cap_conservative(pout.charge_current_cap_a[h],
                                         rout.charge_current_cap_a[h]));
        }
        if(pout.budget_exhausted != rout.budget_exhausted)
        {
            ++transition_skew_samples;
            ASSERT_TRUE((pout.budget_exhausted != 0u) &&
                        (rout.budget_exhausted == 0u));
        }
    }
    ASSERT_TRUE(transition_skew_samples < 250u);
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
        {"exact curve-state oracle vs trapezoidal integration",
         test_exact_oracle_against_trapezoidal},
        {"EAC14-80 curve and temperature anchors",
         test_curve_and_temperature_anchors},
        {"directed production/reference comparison",
         test_directed_production_comparison},
        {"50k randomized production/reference comparison",
         test_randomized_production_vs_independent_oracle},
        {"invalid input fail-closed", test_invalid_fail_closed},
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
    printf("Fuse curve-oracle validation: PASS (%zu tests)\n", count);
    return EXIT_SUCCESS;
}
