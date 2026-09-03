#include "estimator/ams_estimator_lut.h"
#include "soh/ams_soh.h"
#include "sop/ams_power_can.h"
#include "sop/ams_fuse_observer.h"
#include "sop/ams_power_strategy.h"
#include "sop/ams_sop.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define TEST_ASSERT(x) do { if(!(x)) { \
    fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #x); \
    return false; } } while(0)

static ams_sop_input_t nominal_sop_input(void)
{
    ams_sop_input_t input;
    memset(&input, 0, sizeof(input));
    input.measurement_sequence = 10u;
    input.measurement_timestamp_ms = 950u;
    input.now_ms = 1000u;
    input.pack_current_a = 0.0f;
    input.pack_current_uncertainty_a = 0.50f;
    input.ambient_temp_c = 27.0f;
    input.operating_mode = AMS_SOP_MODE_DRIVE;
    input.measurement_valid = 1u;
    input.estimator_valid = 1u;
    input.estimator_acquired = 1u;
    input.estimator_segment_topology = 1u;
    input.current_calibrated = 1u;
    input.current_polarity_validated = 1u;
    input.ambient_measured = 1u;
    input.balance_recovered = 1u;
    input.discharge_authorized = 1u;
    input.regen_authorized = 1u;

    for(uint8_t segment = 0u; segment < AMS_SOP_SEGMENTS; segment++)
    {
        ams_sop_segment_input_t *s = &input.segment[segment];
        s->soc = 0.55f;
        s->vp1_v = 0.0f;
        s->vp2_v = 0.0f;
        s->r0_ohm = ams_p42a_r0_ohm(s->soc, 28.0f);
        s->core_temp_c = 28.0f;
        s->surface_max_temp_c = 27.0f;
        s->p_soc = 1.0e-6f;
        s->p_vp1 = 1.0e-7f;
        s->p_vp2 = 1.0e-7f;
        s->p_r0 = 1.0e-9f;
        s->innovation_v = 0.0f;
        s->capacity_soh_lower = 1.0f;
        s->resistance_soh_upper = 1.05f;
        s->cell_usable_mask = AMS_SOP_FULL_CELL_MASK;
        s->estimator_valid = 1u;
        s->capacity_soh_valid = 1u;
        s->resistance_soh_valid = 1u;
        for(uint8_t cell = 0u; cell < AMS_SOP_CELLS_PER_SEGMENT; cell++)
        {
            s->cell_voltage_v[cell] = 3.75f;
        }
    }
    return input;
}

static bool limits_finite_and_nested(const ams_sop_result_t *result,
                                     const ams_sop_config_t *cfg)
{
    for(uint8_t h = 0u; h < AMS_SOP_HORIZONS; h++)
    {
        TEST_ASSERT(isfinite(result->model_discharge_current_a[h]));
        TEST_ASSERT(isfinite(result->model_charge_current_a[h]));
        TEST_ASSERT(isfinite(result->discharge_power_w[h]));
        TEST_ASSERT(isfinite(result->charge_power_w[h]));
        TEST_ASSERT(result->model_discharge_current_a[h] >= 0.0f);
        TEST_ASSERT(result->model_discharge_current_a[h] <=
                    cfg->discharge_current_max_a[h] + 0.01f);
        TEST_ASSERT(result->model_charge_current_a[h] <= 0.0f);
        TEST_ASSERT(-result->model_charge_current_a[h] <=
                    cfg->charge_current_max_a[h] + 0.01f);
        if(h > 0u)
        {
            TEST_ASSERT(result->model_discharge_current_a[h] <=
                        result->model_discharge_current_a[h - 1u] + 0.01f);
            TEST_ASSERT(fabsf(result->model_charge_current_a[h]) <=
                        fabsf(result->model_charge_current_a[h - 1u]) + 0.01f);
        }
    }
    return true;
}

static bool test_nominal_solver(void)
{
    ams_sop_config_t cfg;
    ams_sop_default_config(&cfg);
    TEST_ASSERT(ams_sop_config_valid(&cfg));
    ams_sop_input_t input = nominal_sop_input();
    ams_sop_result_t result;
    TEST_ASSERT(ams_sop_solve(&input, &cfg, &result) == AMS_SOP_OK);
    TEST_ASSERT(result.valid && result.authority_valid &&
                !result.fallback_active);
    TEST_ASSERT(result.feasibility_evaluations >=
                (uint32_t)(AMS_SOP_HORIZONS * 4u));
    TEST_ASSERT(result.prediction_steps > 0u);
    TEST_ASSERT(limits_finite_and_nested(&result, &cfg));
    TEST_ASSERT(result.discharge_current_a[1] > 0.0f);
    TEST_ASSERT(result.charge_current_a[1] < 0.0f);
    return true;
}

static bool test_input_and_config_fail_closed(void)
{
    ams_sop_config_t cfg;
    ams_sop_default_config(&cfg);
    ams_sop_input_t input = nominal_sop_input();
    ams_sop_result_t result;

    input.current_calibrated = 0u;
    TEST_ASSERT(ams_sop_solve(&input, &cfg, &result) ==
                AMS_SOP_INVALID_INPUT);
    TEST_ASSERT(result.fallback_active && !result.valid &&
                result.discharge_current_a[0] == 0.0f);
    TEST_ASSERT((result.reason_flags &
                 AMS_SOP_REASON_CURRENT_UNCALIBRATED) != 0u);

    input = nominal_sop_input();
    input.current_polarity_validated = 0u;
    TEST_ASSERT(ams_sop_solve(&input, &cfg, &result) ==
                AMS_SOP_INVALID_INPUT);
    input = nominal_sop_input();
    input.now_ms = 1300u;
    TEST_ASSERT(ams_sop_solve(&input, &cfg, &result) ==
                AMS_SOP_INVALID_INPUT);
    input = nominal_sop_input();
    input.estimator_segment_topology = 0u;
    TEST_ASSERT(ams_sop_solve(&input, &cfg, &result) ==
                AMS_SOP_INVALID_INPUT);
    input = nominal_sop_input();
    input.estimator_acquired = 0u;
    TEST_ASSERT(ams_sop_solve(&input, &cfg, &result) ==
                AMS_SOP_INVALID_INPUT);
    TEST_ASSERT(result.fallback_active && !result.valid &&
                !result.authority_valid);
    TEST_ASSERT(result.discharge_current_a[0] == 0.0f);
    TEST_ASSERT(result.charge_current_a[0] == 0.0f);
    TEST_ASSERT((result.reason_flags &
                 AMS_SOP_REASON_ESTIMATOR_UNACQUIRED) != 0u);
    input = nominal_sop_input();
    input.balance_recovered = 0u;
    TEST_ASSERT(ams_sop_solve(&input, &cfg, &result) ==
                AMS_SOP_INVALID_INPUT);
    input = nominal_sop_input();
    input.ambient_temp_c = NAN;
    TEST_ASSERT(ams_sop_solve(&input, &cfg, &result) ==
                AMS_SOP_INVALID_INPUT);
    input = nominal_sop_input();
    input.segment[4].cell_voltage_v[14] = NAN;
    TEST_ASSERT(ams_sop_solve(&input, &cfg, &result) ==
                AMS_SOP_INVALID_INPUT);

    ams_sop_default_config(&cfg);
    cfg.horizons_s[2] = NAN;
    TEST_ASSERT(!ams_sop_config_valid(&cfg));
    TEST_ASSERT(ams_sop_solve(&input, &cfg, &result) ==
                AMS_SOP_INVALID_CONFIGURATION);
    ams_sop_default_config(&cfg);
    cfg.discharge_current_max_a[0] = -1.0f;
    TEST_ASSERT(!ams_sop_config_valid(&cfg));
    ams_sop_default_config(&cfg);
    cfg.core_thermal_capacity_j_per_k = 0.0f;
    TEST_ASSERT(!ams_sop_config_valid(&cfg));
    return true;
}

static bool test_direction_authority(void)
{
    ams_sop_config_t cfg;
    ams_sop_default_config(&cfg);
    ams_sop_input_t input = nominal_sop_input();
    ams_sop_result_t result;

    input.regen_authorized = 0u;
    TEST_ASSERT(ams_sop_solve(&input, &cfg, &result) == AMS_SOP_OK);
    TEST_ASSERT(result.model_charge_current_a[1] < 0.0f);
    TEST_ASSERT(result.charge_current_a[1] == 0.0f);
    TEST_ASSERT(result.charge_binding[1] ==
                AMS_SOP_BIND_DIRECTION_INHIBIT);
    TEST_ASSERT((result.reason_flags & AMS_SOP_REASON_REGEN_INHIBITED) != 0u);

    input.operating_mode = AMS_SOP_MODE_IDLE;
    TEST_ASSERT(ams_sop_solve(&input, &cfg, &result) == AMS_SOP_OK);
    TEST_ASSERT(result.discharge_current_a[1] == 0.0f);
    TEST_ASSERT(result.charge_current_a[1] == 0.0f);

    input.operating_mode = AMS_SOP_MODE_CHARGE;
    input.charger_authorized = 1u;
    TEST_ASSERT(ams_sop_solve(&input, &cfg, &result) == AMS_SOP_OK);
    TEST_ASSERT(result.discharge_current_a[1] == 0.0f);
    TEST_ASSERT(result.charge_current_a[1] < 0.0f);
    return true;
}

static bool test_weakest_cell_and_covariance_bind(void)
{
    ams_sop_config_t cfg;
    ams_sop_default_config(&cfg);
    ams_sop_input_t nominal = nominal_sop_input();
    for(uint8_t segment = 0u; segment < AMS_SOP_SEGMENTS; segment++)
    {
        for(uint8_t cell = 0u; cell < AMS_SOP_CELLS_PER_SEGMENT; cell++)
        {
            nominal.segment[segment].cell_voltage_v[cell] = 3.15f;
        }
        nominal.segment[segment].soc = 0.18f;
    }
    ams_sop_result_t baseline;
    TEST_ASSERT(ams_sop_solve(&nominal, &cfg, &baseline) == AMS_SOP_OK);

    ams_sop_input_t weak = nominal;
    weak.segment[4].cell_voltage_v[14] = 2.92f;
    ams_sop_result_t weak_result;
    TEST_ASSERT(ams_sop_solve(&weak, &cfg, &weak_result) == AMS_SOP_OK);
    TEST_ASSERT(weak_result.model_discharge_current_a[1] + 0.05f <
                baseline.model_discharge_current_a[1]);
    TEST_ASSERT(weak_result.discharge_limiting_segment[1] == 4u);
    TEST_ASSERT(weak_result.discharge_limiting_cell[1] == 14u);

    ams_sop_input_t uncertain = nominal;
    for(uint8_t segment = 0u; segment < AMS_SOP_SEGMENTS; segment++)
    {
        uncertain.segment[segment].p_soc = 2.5e-4f;
        uncertain.segment[segment].p_vp1 = 1.0e-4f;
        uncertain.segment[segment].p_vp2 = 1.0e-4f;
        uncertain.segment[segment].p_r0 = 1.0e-6f;
    }
    ams_sop_result_t uncertain_result;
    TEST_ASSERT(ams_sop_solve(&uncertain, &cfg, &uncertain_result) ==
                AMS_SOP_OK);
    TEST_ASSERT(uncertain_result.model_discharge_current_a[1] <
                baseline.model_discharge_current_a[1]);
    return true;
}

static bool test_charge_voltage_and_temperature_bind(void)
{
    ams_sop_config_t cfg;
    ams_sop_default_config(&cfg);
    ams_sop_input_t nominal = nominal_sop_input();
    ams_sop_result_t baseline;
    TEST_ASSERT(ams_sop_solve(&nominal, &cfg, &baseline) == AMS_SOP_OK);

    ams_sop_input_t high = nominal;
    high.segment[3].cell_voltage_v[7] = 4.13f;
    high.segment[3].soc = 0.95f;
    ams_sop_result_t high_result;
    TEST_ASSERT(ams_sop_solve(&high, &cfg, &high_result) == AMS_SOP_OK);
    TEST_ASSERT(fabsf(high_result.model_charge_current_a[1]) <
                fabsf(baseline.model_charge_current_a[1]));

    ams_sop_input_t cold = nominal;
    for(uint8_t segment = 0u; segment < AMS_SOP_SEGMENTS; segment++)
    {
        cold.segment[segment].core_temp_c = 2.0f;
        cold.segment[segment].surface_max_temp_c = 2.0f;
    }
    cold.ambient_temp_c = 2.0f;
    ams_sop_result_t cold_result;
    TEST_ASSERT(ams_sop_solve(&cold, &cfg, &cold_result) == AMS_SOP_OK);
    TEST_ASSERT(cold_result.model_charge_current_a[0] == 0.0f);
    TEST_ASSERT(cold_result.charge_binding[0] ==
                AMS_SOP_BIND_CHARGE_TEMP_LOW);
    return true;
}

static bool test_soh_priors_and_aged_model(void)
{
    ams_sop_config_t cfg;
    ams_sop_default_config(&cfg);
    ams_sop_input_t fresh = nominal_sop_input();
    for(uint8_t segment = 0u; segment < AMS_SOP_SEGMENTS; segment++)
    {
        fresh.segment[segment].soc = 0.16f;
        for(uint8_t cell = 0u; cell < AMS_SOP_CELLS_PER_SEGMENT; cell++)
        {
            fresh.segment[segment].cell_voltage_v[cell] = 3.10f;
        }
    }
    ams_sop_result_t fresh_result;
    TEST_ASSERT(ams_sop_solve(&fresh, &cfg, &fresh_result) == AMS_SOP_OK);

    ams_sop_input_t aged = fresh;
    for(uint8_t segment = 0u; segment < AMS_SOP_SEGMENTS; segment++)
    {
        aged.segment[segment].capacity_soh_lower = 0.60f;
        aged.segment[segment].resistance_soh_upper = 1.80f;
    }
    ams_sop_result_t aged_result;
    TEST_ASSERT(ams_sop_solve(&aged, &cfg, &aged_result) == AMS_SOP_OK);
    TEST_ASSERT(aged_result.model_discharge_current_a[2] <
                fresh_result.model_discharge_current_a[2]);

    ams_sop_input_t prior = fresh;
    for(uint8_t segment = 0u; segment < AMS_SOP_SEGMENTS; segment++)
    {
        prior.segment[segment].capacity_soh_valid = 0u;
        prior.segment[segment].resistance_soh_valid = 0u;
        prior.segment[segment].capacity_soh_lower = NAN;
        prior.segment[segment].resistance_soh_upper = NAN;
    }
    ams_sop_result_t prior_result;
    TEST_ASSERT(ams_sop_solve(&prior, &cfg, &prior_result) == AMS_SOP_OK);
    TEST_ASSERT((prior_result.reason_flags &
                 AMS_SOP_REASON_SOH_CAPACITY_PRIOR) != 0u);
    TEST_ASSERT((prior_result.reason_flags &
                 AMS_SOP_REASON_SOH_RESISTANCE_PRIOR) != 0u);
    return true;
}

static bool test_bisection_against_bruteforce(void)
{
    ams_sop_config_t cfg;
    ams_sop_default_config(&cfg);
    ams_sop_input_t input = nominal_sop_input();
    for(uint8_t segment = 0u; segment < AMS_SOP_SEGMENTS; segment++)
    {
        input.segment[segment].soc = 0.22f;
        for(uint8_t cell = 0u; cell < AMS_SOP_CELLS_PER_SEGMENT; cell++)
        {
            input.segment[segment].cell_voltage_v[cell] = 3.18f;
        }
    }
    ams_sop_result_t result;
    TEST_ASSERT(ams_sop_solve(&input, &cfg, &result) == AMS_SOP_OK);

    for(uint8_t h = 0u; h < AMS_SOP_HORIZONS; h++)
    {
        float brute_a = 0.0f;
        for(float current_a = 0.0f;
            current_a <= cfg.discharge_current_max_a[h]; current_a += 0.25f)
        {
            ams_sop_evaluation_t evaluation;
            TEST_ASSERT(ams_sop_evaluate_current(&input, &cfg, current_a,
                        cfg.horizons_s[h], &evaluation) == AMS_SOP_OK);
            if(evaluation.feasible)
            {
                brute_a = current_a;
            }
            else
            {
                break;
            }
        }
        TEST_ASSERT(result.model_discharge_current_a[h] + 0.01f >= brute_a);
        TEST_ASSERT(result.model_discharge_current_a[h] <= brute_a + 0.26f);
    }
    return true;
}

static bool test_slew_policy(void)
{
    ams_sop_config_t cfg;
    ams_sop_default_config(&cfg);
    ams_sop_input_t input = nominal_sop_input();
    ams_sop_result_t raw;
    ams_sop_result_t previous;
    ams_sop_result_t published;
    memset(&previous, 0, sizeof(previous));
    TEST_ASSERT(ams_sop_solve(&input, &cfg, &raw) == AMS_SOP_OK);
    ams_sop_apply_slew(&raw, &previous, &cfg, 0.1f, &published);
    TEST_ASSERT(published.discharge_current_a[1] <= 4.01f);
    TEST_ASSERT(fabsf(published.charge_current_a[1]) <= 0.51f);
    TEST_ASSERT((published.reason_flags & AMS_SOP_REASON_LIMIT_SLEWED) != 0u);

    previous = published;
    raw.discharge_current_a[1] = 1.0f;
    raw.discharge_power_w[1] = 250.0f;
    ams_sop_apply_slew(&raw, &previous, &cfg, 0.1f, &published);
    TEST_ASSERT(fabsf(published.discharge_current_a[1] - 1.0f) < 0.001f);

    raw.valid = 0u;
    ams_sop_apply_slew(&raw, &previous, &cfg, 0.1f, &published);
    TEST_ASSERT(published.discharge_current_a[1] == 0.0f);
    TEST_ASSERT(published.fallback_active);
    return true;
}

static bool test_cause_scheduled_recovery(void)
{
    ams_sop_config_t cfg;
    ams_sop_default_config(&cfg);
    ams_sop_result_t raw;
    ams_sop_result_t previous;
    ams_sop_result_t published;
    ams_sop_recovery_context_t context;
    memset(&raw, 0, sizeof(raw));
    memset(&previous, 0, sizeof(previous));
    memset(&context, 0, sizeof(context));
    raw.valid = 1u;
    raw.authority_valid = 1u;
    previous.valid = 1u;
    previous.authority_valid = 1u;
    for(uint8_t h = 0u; h < AMS_SOP_HORIZONS; h++)
    {
        raw.discharge_current_a[h] = 80.0f;
        raw.discharge_power_w[h] = 20000.0f;
        raw.charge_current_a[h] = -10.0f;
        raw.charge_power_w[h] = 2500.0f;
        previous.discharge_current_a[h] = 10.0f;
        previous.charge_current_a[h] = -1.0f;
    }

    previous.discharge_binding[1] = AMS_SOP_BIND_CELL_UV;
    ams_sop_apply_recovery(&raw, &previous, &cfg, &context, 0.1f,
                           &published);
    TEST_ASSERT(fabsf(published.discharge_current_a[1] - 80.0f) < 0.01f);
    TEST_ASSERT((published.reason_flags &
                 AMS_SOP_REASON_RECOVERY_VOLTAGE) != 0u);

    previous.discharge_binding[1] = AMS_SOP_BIND_CORE_TEMP;
    ams_sop_apply_recovery(&raw, &previous, &cfg, &context, 0.1f,
                           &published);
    TEST_ASSERT(fabsf(published.discharge_current_a[1] - 10.8f) < 0.01f);
    TEST_ASSERT((published.reason_flags &
                 AMS_SOP_REASON_RECOVERY_THERMAL) != 0u);

    previous.discharge_binding[1] = AMS_SOP_BIND_SOC_LOW;
    ams_sop_apply_recovery(&raw, &previous, &cfg, &context, 0.1f,
                           &published);
    TEST_ASSERT(fabsf(published.discharge_current_a[1] - 10.0f) < 0.01f);
    TEST_ASSERT((published.reason_flags &
                 AMS_SOP_REASON_RECOVERY_SOC_HOLD) != 0u);
    context.discharge_soc_recovered = 1u;
    ams_sop_apply_recovery(&raw, &previous, &cfg, &context, 0.1f,
                           &published);
    TEST_ASSERT(fabsf(published.discharge_current_a[1] - 10.5f) < 0.01f);

    context.discharge_soc_recovered = 0u;
    context.fuse_state_valid = 1u;
    context.fuse_utilization = 0.50f;
    previous.discharge_binding[1] = AMS_SOP_BIND_FUSE_THERMAL;
    ams_sop_apply_recovery(&raw, &previous, &cfg, &context, 0.1f,
                           &published);
    TEST_ASSERT(fabsf(published.discharge_current_a[1] - 11.0f) < 0.01f);
    TEST_ASSERT((published.reason_flags &
                 AMS_SOP_REASON_RECOVERY_CURRENT_PATH) != 0u);

    context.fuse_state_valid = 0u;
    ams_sop_apply_recovery(&raw, &previous, &cfg, &context, 0.1f,
                           &published);
    TEST_ASSERT(fabsf(published.discharge_current_a[1] - 10.0f) < 0.01f);
    return true;
}

static bool test_fuse_observer(void)
{
    ams_fuse_observer_config_t cfg;
    ams_sop_config_t sop_cfg;
    ams_fuse_observer_t observer;
    ams_fuse_observer_input_t input;
    ams_fuse_observer_result_t result;
    ams_fuse_observer_default_config(&cfg);
    ams_sop_default_config(&sop_cfg);
    ams_fuse_observer_init(&observer);
    memset(&input, 0, sizeof(input));
    input.current_uncertainty_a = 0.5f;
    input.temperature_proxy_c = 25.0f;
    input.elapsed_s = 1.0f;
    input.measurement_valid = 1u;
    input.current_calibrated = 1u;
    input.current_polarity_validated = 1u;
    input.model_validated = 1u;

    TEST_ASSERT(ams_fuse_observer_config_valid(&cfg));
    TEST_ASSERT(ams_fuse_temperature_derating(25.0f, 0.75f) == 1.0f);
    for(uint16_t i = 0u; i < 300u; i++)
    {
        TEST_ASSERT(ams_fuse_observer_update(&observer, &cfg, &sop_cfg,
                                             &input, &result));
    }
    TEST_ASSERT(result.valid && result.authority_valid);
    TEST_ASSERT(observer.thermal_state_initialized);
    TEST_ASSERT(result.continuous_current_a < 80.0f);
    for(uint8_t h = 0u; h < AMS_SOP_HORIZONS; h++)
    {
        TEST_ASSERT(result.discharge_current_cap_a[h] <=
                    sop_cfg.discharge_current_max_a[h]);
    }

    /* The curve model no longer treats 118 A as consuming a fixed 8020 A^2s
     * bucket in seconds.  It accumulates slowly according to the time-current
     * curve and remains below exhaustion for this short pulse train. */
    input.pack_current_a = 118.0f;
    input.elapsed_s = 0.1f;
    for(uint16_t i = 0u; i < 100u; i++)
    {
        TEST_ASSERT(ams_fuse_observer_update(&observer, &cfg, &sop_cfg,
                                             &input, &result));
    }
    TEST_ASSERT(result.utilization > 0.0f);
    TEST_ASSERT(!result.budget_exhausted);

    /* A severe 10 In pulse crosses the preliminary curve boundary quickly and
     * exercises the exhaustion latch. */
    input.pack_current_a = 800.0f;
    input.elapsed_s = 0.01f;
    TEST_ASSERT(ams_fuse_observer_update(&observer, &cfg, &sop_cfg,
                                         &input, &result));
    TEST_ASSERT(result.budget_exhausted);
    TEST_ASSERT(result.discharge_current_cap_a[0] == 0.0f);

    input.pack_current_a = 0.0f;
    input.model_validated = 0u;
    TEST_ASSERT(ams_fuse_observer_update(&observer, &cfg, &sop_cfg,
                                         &input, &result));
    TEST_ASSERT(!result.authority_valid);
    TEST_ASSERT((result.reason_flags &
                 AMS_FUSE_REASON_MODEL_UNVALIDATED) != 0u);
    return true;
}

static bool test_mission_strategy_and_request(void)
{
    ams_mission_request_state_t request;
    uint8_t payload[8];
    ams_mission_request_init(&request);
    ams_mission_request_encode(AMS_MISSION_QUALIFY, 1u, true, payload);
    TEST_ASSERT(!ams_mission_request_ingest(&request, payload, 100u));
    ams_mission_request_encode(AMS_MISSION_QUALIFY, 2u, true, payload);
    TEST_ASSERT(ams_mission_request_ingest(&request, payload, 200u));
    TEST_ASSERT(ams_mission_request_fresh(&request, 400u));
    TEST_ASSERT(!ams_mission_request_fresh(&request, 451u));
    payload[7] ^= 0x01u;
    TEST_ASSERT(!ams_mission_request_ingest(&request, payload, 300u));
    TEST_ASSERT(!request.valid);

    ams_sop_config_t sop_cfg;
    ams_sop_default_config(&sop_cfg);
    ams_sop_input_t sop_input = nominal_sop_input();
    ams_sop_result_t hard;
    TEST_ASSERT(ams_sop_solve(&sop_input, &sop_cfg, &hard) == AMS_SOP_OK);

    ams_power_strategy_config_t cfg;
    ams_power_strategy_state_t state;
    ams_power_strategy_input_t input;
    ams_power_strategy_result_t result;
    ams_sop_result_t limited;
    ams_power_strategy_default_config(&cfg);
    ams_power_strategy_init(&state);
    memset(&input, 0, sizeof(input));
    input.requested_profile = AMS_MISSION_ENDURANCE;
    input.request_valid = 1u;
    input.minimum_segment_soc_lower = 0.50f;
    input.resistance_confidence_pct = 50u;
    for(uint8_t segment = 0u; segment < AMS_SOP_SEGMENTS; segment++)
    {
        input.segment_core_temp_c[segment] = 20.0f;
        input.segment_surface_temp_c[segment] = 20.0f;
        input.segment_r0_ohm[segment] = 0.016f;
    }
    TEST_ASSERT(ams_power_strategy_update(&state, &cfg, &input, NULL,
                                          &hard, &limited, &result));
    TEST_ASSERT(result.active_profile == AMS_MISSION_ENDURANCE);
    TEST_ASSERT(result.recommended_horizon_index == 3u);
    TEST_ASSERT(limited.discharge_current_a[0] <=
                hard.discharge_current_a[3] + 0.01f);
    TEST_ASSERT(result.thermal_energy_to_target_wh > 40.0f);
    TEST_ASSERT(!result.thermal_ready);
    TEST_ASSERT(result.resistance_bootstrap_progress_pct == 50u);

    input.requested_profile = AMS_MISSION_QUALIFY;
    input.stationary_confirmed = 0u;
    TEST_ASSERT(ams_power_strategy_update(&state, &cfg, &input, NULL,
                                          &hard, &limited, &result));
    TEST_ASSERT(result.active_profile == AMS_MISSION_ENDURANCE);
    TEST_ASSERT((result.reason_flags &
                 AMS_STRATEGY_REASON_QUALIFY_BLOCKED) != 0u);
    input.stationary_confirmed = 1u;
    TEST_ASSERT(ams_power_strategy_update(&state, &cfg, &input, NULL,
                                          &hard, &limited, &result));
    TEST_ASSERT(result.active_profile == AMS_MISSION_QUALIFY);
    TEST_ASSERT(fabsf(limited.discharge_current_a[1] -
                      hard.discharge_current_a[1]) < 0.01f);

    input.minimum_segment_soc_lower = 0.299f;
    TEST_ASSERT(ams_power_strategy_update(&state, &cfg, &input, NULL,
                                          &hard, &limited, &result));
    TEST_ASSERT(result.active_profile == AMS_MISSION_LIMP_HOME);
    TEST_ASSERT(result.limp_latched);
    TEST_ASSERT(limited.discharge_current_a[0] <= 35.01f);
    input.minimum_segment_soc_lower = 0.80f;
    input.requested_profile = AMS_MISSION_QUALIFY;
    TEST_ASSERT(ams_power_strategy_update(&state, &cfg, &input, NULL,
                                          &hard, &limited, &result));
    TEST_ASSERT(result.active_profile == AMS_MISSION_LIMP_HOME);
    return true;
}

static uint32_t lcg_next(uint32_t *state)
{
    *state = (*state * 1664525u) + 1013904223u;
    return *state;
}

static float lcg_range(uint32_t *state, float lower, float upper)
{
    const float unit = (float)(lcg_next(state) & 0x00FFFFFFu) /
                       16777215.0f;
    return lower + (upper - lower) * unit;
}

static bool test_strategy_fuse_randomized_invariants(void)
{
    uint8_t encoded[8];
    ams_mission_request_encode(AMS_MISSION_QUALIFY, 7u, true, encoded);
    for(uint8_t byte = 0u; byte < 8u; byte++)
    {
        for(uint8_t bit = 0u; bit < 8u; bit++)
        {
            uint8_t corrupt[8];
            memcpy(corrupt, encoded, sizeof(corrupt));
            corrupt[byte] ^= (uint8_t)(1u << bit);
            ams_mission_request_state_t request;
            ams_mission_request_init(&request);
            TEST_ASSERT(!ams_mission_request_ingest(&request, corrupt, 10u));
            TEST_ASSERT(request.rejected_count == 1u);
            TEST_ASSERT(!request.valid);
        }
    }

    ams_fuse_observer_config_t fuse_cfg;
    ams_sop_config_t sop_cfg;
    ams_fuse_observer_t fuse_state;
    ams_fuse_observer_input_t fuse_input;
    ams_fuse_observer_result_t fuse_result;
    ams_fuse_observer_default_config(&fuse_cfg);
    ams_sop_default_config(&sop_cfg);
    ams_fuse_observer_init(&fuse_state);
    fuse_state.thermal_state_initialized = 1u;
    memset(&fuse_input, 0, sizeof(fuse_input));
    fuse_input.measurement_valid = 1u;
    fuse_input.current_calibrated = 1u;
    fuse_input.current_polarity_validated = 1u;
    fuse_input.model_validated = 1u;

    uint32_t seed = 0xEAC1480u;
    for(uint32_t cycle = 0u; cycle < 10000u; cycle++)
    {
        fuse_input.pack_current_a = lcg_range(&seed, -15.0f, 130.0f);
        fuse_input.current_uncertainty_a = lcg_range(&seed, 0.2f, 2.0f);
        fuse_input.temperature_proxy_c = lcg_range(&seed, -20.0f, 90.0f);
        fuse_input.elapsed_s = lcg_range(&seed, 0.01f, 0.20f);
        TEST_ASSERT(ams_fuse_observer_update(&fuse_state, &fuse_cfg,
                                             &sop_cfg, &fuse_input,
                                             &fuse_result));
        TEST_ASSERT(isfinite(fuse_result.utilization));
        TEST_ASSERT(fuse_result.utilization >= 0.0f);
        TEST_ASSERT(isfinite(fuse_result.remaining_utilization));
        for(uint8_t h = 0u; h < AMS_SOP_HORIZONS; h++)
        {
            TEST_ASSERT(isfinite(fuse_result.discharge_current_cap_a[h]));
            TEST_ASSERT(fuse_result.discharge_current_cap_a[h] >= 0.0f);
            TEST_ASSERT(fuse_result.discharge_current_cap_a[h] <=
                        sop_cfg.discharge_current_max_a[h] + 0.001f);
        }
    }

    ams_sop_input_t sop_input = nominal_sop_input();
    ams_sop_result_t hard;
    TEST_ASSERT(ams_sop_solve(&sop_input, &sop_cfg, &hard) == AMS_SOP_OK);
    ams_power_strategy_config_t strategy_cfg;
    ams_power_strategy_default_config(&strategy_cfg);
    for(uint16_t cycle = 0u; cycle < 1000u; cycle++)
    {
        ams_power_strategy_state_t strategy_state;
        ams_power_strategy_input_t strategy_input;
        ams_power_strategy_result_t strategy_result;
        ams_sop_result_t limited;
        ams_power_strategy_init(&strategy_state);
        memset(&strategy_input, 0, sizeof(strategy_input));
        strategy_input.requested_profile = (ams_mission_profile_t)
            (lcg_next(&seed) % 3u);
        strategy_input.request_valid = 1u;
        strategy_input.stationary_confirmed = 1u;
        strategy_input.minimum_segment_soc_lower =
            lcg_range(&seed, 0.20f, 0.90f);
        strategy_input.pack_current_a = lcg_range(&seed, -10.0f, 90.0f);
        strategy_input.resistance_confidence_pct =
            (uint8_t)(lcg_next(&seed) % 101u);
        for(uint8_t segment = 0u; segment < AMS_SOP_SEGMENTS; segment++)
        {
            strategy_input.segment_core_temp_c[segment] =
                lcg_range(&seed, 5.0f, 50.0f);
            strategy_input.segment_surface_temp_c[segment] =
                lcg_range(&seed, 5.0f, 50.0f);
            strategy_input.segment_r0_ohm[segment] =
                lcg_range(&seed, 0.008f, 0.030f);
        }
        TEST_ASSERT(ams_power_strategy_update(
            &strategy_state, &strategy_cfg, &strategy_input, NULL,
            &hard, &limited, &strategy_result));
        TEST_ASSERT(strategy_result.active_profile <=
                    AMS_MISSION_LIMP_HOME);
        for(uint8_t h = 0u; h < AMS_SOP_HORIZONS; h++)
        {
            TEST_ASSERT(isfinite(limited.discharge_current_a[h]));
            TEST_ASSERT(limited.discharge_current_a[h] >= 0.0f);
            TEST_ASSERT(limited.discharge_current_a[h] <=
                        hard.discharge_current_a[h] + 0.001f);
            TEST_ASSERT(limited.model_discharge_current_a[h] ==
                        hard.model_discharge_current_a[h]);
        }
    }
    return true;
}

static bool test_randomized_invariants(void)
{
    ams_sop_config_t cfg;
    ams_sop_default_config(&cfg);
    uint32_t seed = 0x42A75A6u;
    for(uint16_t cycle = 0u; cycle < 300u; cycle++)
    {
        ams_sop_input_t input = nominal_sop_input();
        input.pack_current_a = lcg_range(&seed, -8.0f, 80.0f);
        input.pack_current_uncertainty_a = lcg_range(&seed, 0.50f, 2.0f);
        for(uint8_t segment = 0u; segment < AMS_SOP_SEGMENTS; segment++)
        {
            input.segment[segment].soc = lcg_range(&seed, 0.08f, 0.95f);
            input.segment[segment].core_temp_c = lcg_range(&seed, 8.0f, 39.0f);
            input.segment[segment].surface_max_temp_c =
                input.segment[segment].core_temp_c -
                lcg_range(&seed, 0.0f, 2.0f);
            input.segment[segment].r0_ohm = ams_p42a_r0_ohm(
                input.segment[segment].soc,
                input.segment[segment].core_temp_c);
            const float base_v = lcg_range(&seed, 2.90f, 4.12f);
            for(uint8_t cell = 0u; cell < AMS_SOP_CELLS_PER_SEGMENT; cell++)
            {
                input.segment[segment].cell_voltage_v[cell] = base_v +
                    lcg_range(&seed, -0.010f, 0.010f);
            }
        }
        input.ambient_temp_c = input.segment[0].surface_max_temp_c;
        ams_sop_result_t result;
        TEST_ASSERT(ams_sop_solve(&input, &cfg, &result) == AMS_SOP_OK);
        TEST_ASSERT(result.valid && result.authority_valid);
        TEST_ASSERT(limits_finite_and_nested(&result, &cfg));
    }
    return true;
}

static ams_soh_input_t nominal_soh_input(void)
{
    ams_soh_input_t input;
    memset(&input, 0, sizeof(input));
    input.measurement_sequence = 1u;
    input.measurement_timestamp_ms = 1u;
    input.now_ms = 1u;
    input.elapsed_s = 1.0f;
    input.pack_current_uncertainty_a = 0.05f;
    input.pack_soc = 0.90f;
    input.average_cell_temp_c = 25.0f;
    input.cell_voltage_spread_v = 0.020f;
    input.maximum_soc_sigma = 0.005f;
    input.maximum_abs_innovation_v_per_cell = 0.005f;
    input.maximum_abs_polarization_v = 0.005f;
    input.measurement_valid = 1u;
    input.estimator_valid = 1u;
    input.current_calibrated = 1u;
    input.current_polarity_validated = 1u;
    input.balance_recovered = 1u;
    for(uint8_t segment = 0u; segment < AMS_SOH_SEGMENTS; segment++)
    {
        input.segment_resistance_growth_ratio[segment] = 1.05f;
        input.segment_resistance_confidence_pct[segment] = 80u;
        input.segment_resistance_valid[segment] = 1u;
    }
    return input;
}

static void soh_advance(ams_soh_estimator_t *estimator,
                        const ams_soh_config_t *cfg,
                        ams_soh_input_t *input,
                        uint32_t seconds)
{
    for(uint32_t n = 0u; n < seconds; n++)
    {
        input->now_ms += 1000u;
        input->measurement_timestamp_ms = input->now_ms;
        input->measurement_sequence++;
        assert(ams_soh_update(estimator, cfg, input));
    }
}

static void soh_close_resistance_episode(ams_soh_estimator_t *estimator,
                                         const ams_soh_config_t *cfg,
                                         ams_soh_input_t *input)
{
    uint8_t valid[AMS_SOH_SEGMENTS];
    memcpy(valid, input->segment_resistance_valid, sizeof(valid));
    memset(input->segment_resistance_valid, 0,
           sizeof(input->segment_resistance_valid));
    const uint32_t gap_s =
        (AMS_SOH_RESISTANCE_EPISODE_GAP_MS + 999u) / 1000u;
    soh_advance(estimator, cfg, input, gap_s);
    memcpy(input->segment_resistance_valid, valid, sizeof(valid));
}

static bool test_capacity_soh_observability(void)
{
    ams_soh_config_t cfg;
    ams_soh_default_config(&cfg);
    ams_soh_estimator_t estimator;
    ams_soh_init(&estimator, &cfg);
    ams_soh_input_t input = nominal_soh_input();

    soh_advance(&estimator, &cfg, &input, 60u); /* first rest anchor */
    TEST_ASSERT(estimator.anchor_valid);
    TEST_ASSERT(estimator.result.accepted_capacity_windows == 0u);

    input.pack_current_a = 20.0f;
    input.pack_soc = 0.70f;
    input.total_charge_as = 5.04 * 3600.0;
    input.now_ms += 1000u;
    input.measurement_timestamp_ms = input.now_ms;
    input.measurement_sequence++;
    TEST_ASSERT(ams_soh_update(&estimator, &cfg, &input));
    input.pack_current_a = 0.0f;
    soh_advance(&estimator, &cfg, &input, 60u);
    TEST_ASSERT(estimator.result.accepted_capacity_windows == 1u);
    TEST_ASSERT(fabsf(estimator.result.capacity_ah - 25.2f) < 0.05f);
    TEST_ASSERT(!estimator.result.capacity_valid);

    input.pack_current_a = -20.0f;
    input.pack_soc = 0.90f;
    input.total_charge_as = 0.0;
    input.now_ms += 1000u;
    input.measurement_timestamp_ms = input.now_ms;
    input.measurement_sequence++;
    TEST_ASSERT(ams_soh_update(&estimator, &cfg, &input));
    input.pack_current_a = 0.0f;
    soh_advance(&estimator, &cfg, &input, 60u);
    TEST_ASSERT(estimator.result.accepted_capacity_windows == 2u);
    TEST_ASSERT(estimator.result.capacity_valid);
    TEST_ASSERT(estimator.result.capacity_confidence_pct == 50u);
    TEST_ASSERT(estimator.result.capacity_soh_lower > 0.90f);
    soh_close_resistance_episode(&estimator, &cfg, &input);
    TEST_ASSERT(estimator.result.resistance_valid);
    TEST_ASSERT(estimator.result.resistance_growth_upper > 1.05f);
    return true;
}

static bool test_capacity_soh_rejections_and_persistence(void)
{
    ams_soh_config_t cfg;
    ams_soh_default_config(&cfg);
    ams_soh_estimator_t estimator;
    ams_soh_init(&estimator, &cfg);
    ams_soh_input_t input = nominal_soh_input();
    input.current_calibrated = 0u;
    TEST_ASSERT(!ams_soh_update(&estimator, &cfg, &input));
    TEST_ASSERT(estimator.result.capacity_soh_lower ==
                cfg.prior_capacity_soh_lower);
    TEST_ASSERT((estimator.result.last_reason_flags &
                 AMS_SOH_REASON_CURRENT_CALIBRATION) != 0u);

    input = nominal_soh_input();
    input.maximum_soc_sigma = cfg.maximum_soc_sigma + 0.001f;
    soh_advance(&estimator, &cfg, &input, 60u);
    TEST_ASSERT(!estimator.anchor_valid);
    TEST_ASSERT((estimator.result.last_reason_flags &
                 AMS_SOH_REASON_SOC_UNCERTAINTY) != 0u);

    input = nominal_soh_input();
    input.maximum_abs_polarization_v =
        cfg.maximum_rest_polarization_v + 0.001f;
    soh_advance(&estimator, &cfg, &input, 60u);
    TEST_ASSERT(!estimator.anchor_valid);
    TEST_ASSERT((estimator.result.last_reason_flags &
                 AMS_SOH_REASON_REST_POLARIZATION) != 0u);

    input = nominal_soh_input();
    soh_advance(&estimator, &cfg, &input, 60u);
    input.pack_current_a = 20.0f;
    input.pack_soc = 0.70f;
    /* Wrong sign: positive SOC delta/positive throughput would be rejected. */
    input.total_charge_as = -5.04 * 3600.0;
    input.now_ms += 1000u;
    input.measurement_timestamp_ms = input.now_ms;
    input.measurement_sequence++;
    TEST_ASSERT(ams_soh_update(&estimator, &cfg, &input));
    input.pack_current_a = 0.0f;
    soh_advance(&estimator, &cfg, &input, 60u);
    TEST_ASSERT(estimator.result.rejected_capacity_windows >= 1u);
    TEST_ASSERT((estimator.result.last_reason_flags &
                 AMS_SOH_REASON_DIRECTION) != 0u);

    /* Seed a valid record and prove CRC/schema validation. */
    estimator.result.accepted_capacity_windows = 2u;
    estimator.capacity_mean_ah = 24.0;
    estimator.capacity_m2_ah2 = 0.1;
    estimator.segment_resistance_growth_ratio[0] = 1.10f;
    estimator.segment_resistance_growth_upper[0] = 1.20f;
    estimator.segment_resistance_confidence_pct[0] = 80u;
    estimator.segment_resistance_valid_mask = 0x01u;
    ams_soh_persist_record_t record;
    TEST_ASSERT(ams_soh_export_record(&estimator, 7u, &record));
    ams_soh_estimator_t restored;
    TEST_ASSERT(ams_soh_import_record(&restored, &cfg, &record));
    TEST_ASSERT(restored.result.persistence_valid);
    TEST_ASSERT(fabsf(restored.result.capacity_ah - 24.0f) < 0.01f);
    TEST_ASSERT(!restored.result.capacity_valid);
    TEST_ASSERT(restored.result.capacity_soh_lower ==
                cfg.prior_capacity_soh_lower);
    TEST_ASSERT(!restored.result.resistance_valid);
    TEST_ASSERT(restored.segment_resistance_valid_mask == 0x01u);
    TEST_ASSERT(fabsf(restored.segment_resistance_growth_upper[0] - 1.20f) <
                0.001f);

    estimator.result.capacity_valid = 1u;
    estimator.segment_resistance_valid_mask = AMS_SOH_ALL_SEGMENTS_MASK;
    for(uint8_t segment = 0u; segment < AMS_SOH_SEGMENTS; segment++)
    {
        estimator.segment_resistance_growth_ratio[segment] = 1.10f;
        estimator.segment_resistance_growth_upper[segment] = 1.20f;
        estimator.segment_resistance_confidence_pct[segment] = 80u;
    }
    ams_soh_persist_record_t newer;
    TEST_ASSERT(ams_soh_export_record(&estimator, 8u, &newer));
    TEST_ASSERT(ams_soh_import_record(&restored, &cfg, &newer));
    TEST_ASSERT(restored.result.capacity_valid);
    TEST_ASSERT(restored.result.resistance_valid);
    ams_soh_persist_record_t selected;
    TEST_ASSERT(ams_soh_select_newest_record(&cfg, &record, &newer,
                                             &selected));
    TEST_ASSERT(selected.generation == 8u);

    record.capacity_mean_ah += 1.0;
    /* Invalid persistence must be safe even when the destination has never
     * been initialized.  This catches accidental read/modify/write of the
     * prior reason flags on a rejection path. */
    memset(&restored, 0xA5, sizeof(restored));
    TEST_ASSERT(!ams_soh_import_record(&restored, &cfg, &record));
    TEST_ASSERT(restored.result.last_reason_flags ==
                AMS_SOH_REASON_PERSISTENCE);
    TEST_ASSERT(ams_soh_select_newest_record(&cfg, &record, &newer,
                                             &selected));
    TEST_ASSERT(selected.generation == 8u);
    return true;
}

static bool test_resistance_soh_retention(void)
{
    ams_soh_config_t cfg;
    ams_soh_default_config(&cfg);
    ams_soh_estimator_t estimator;
    ams_soh_init(&estimator, &cfg);
    ams_soh_input_t input = nominal_soh_input();
    for(uint8_t segment = 0u; segment < AMS_SOH_SEGMENTS; segment++)
    {
        input.segment_resistance_growth_ratio[segment] =
            1.45f + 0.02f * (float)segment;
        input.segment_resistance_confidence_pct[segment] = 80u;
        input.segment_resistance_valid[segment] = 1u;
    }
    soh_advance(&estimator, &cfg, &input,
                AMS_SOH_RESISTANCE_EPISODE_MIN_OBSERVATIONS);
    TEST_ASSERT(!estimator.result.resistance_valid);
    soh_close_resistance_episode(&estimator, &cfg, &input);
    TEST_ASSERT(estimator.result.resistance_valid);
    const float retained = estimator.result.resistance_growth_upper;
    TEST_ASSERT(retained > 1.50f);

    for(uint8_t segment = 0u; segment < AMS_SOH_SEGMENTS; segment++)
    {
        input.segment_resistance_growth_ratio[segment] = 1.05f;
        input.segment_resistance_confidence_pct[segment] = 80u;
        input.segment_resistance_valid[segment] = 1u;
    }
    soh_advance(&estimator, &cfg, &input,
                AMS_SOH_RESISTANCE_EPISODE_MIN_OBSERVATIONS);
    soh_close_resistance_episode(&estimator, &cfg, &input);
    TEST_ASSERT(estimator.result.resistance_growth_upper >= retained);

    memset(input.segment_resistance_valid, 0,
           sizeof(input.segment_resistance_valid));
    input.now_ms += 100u;
    input.measurement_timestamp_ms = input.now_ms;
    input.measurement_sequence++;
    TEST_ASSERT(ams_soh_update(&estimator, &cfg, &input));
    TEST_ASSERT(estimator.result.resistance_valid);
    TEST_ASSERT(estimator.result.resistance_growth_upper >= retained);

    /* Resistance-only history must survive a reboot even before a capacity
     * window has become observable. */
    TEST_ASSERT(estimator.result.accepted_capacity_windows == 0u);
    ams_soh_persist_record_t record;
    ams_soh_estimator_t restored;
    TEST_ASSERT(ams_soh_export_record(&estimator, 3u, &record));
    TEST_ASSERT(ams_soh_import_record(&restored, &cfg, &record));
    TEST_ASSERT(restored.result.resistance_valid);
    TEST_ASSERT(!restored.result.capacity_valid);
    TEST_ASSERT(restored.result.capacity_soh_lower ==
                cfg.prior_capacity_soh_lower);
    TEST_ASSERT(restored.result.resistance_growth_upper >= retained);
    return true;
}

static bool test_resistance_soh_transient_spike_rejection(void)
{
    ams_soh_config_t cfg;
    ams_soh_default_config(&cfg);
    ams_soh_estimator_t estimator;
    ams_soh_init(&estimator, &cfg);
    ams_soh_input_t input = nominal_soh_input();

    /* A single qualified-but-bad R0 observation must not become permanent
     * battery ageing. Eight healthy observations plus one 60% spike have a
     * healthy median and should establish a healthy retained value. */
    for(uint8_t n = 0u; n < AMS_SOH_RESISTANCE_EPISODE_MIN_OBSERVATIONS; n++)
    {
        const float ratio = (n == 4u) ? 1.60f : 1.02f;
        for(uint8_t segment = 0u; segment < AMS_SOH_SEGMENTS; segment++)
        {
            input.segment_resistance_growth_ratio[segment] = ratio;
            input.segment_resistance_confidence_pct[segment] = 100u;
            input.segment_resistance_valid[segment] = 1u;
        }
        input.now_ms += 100u;
        input.measurement_timestamp_ms = input.now_ms;
        input.measurement_sequence++;
        TEST_ASSERT(ams_soh_update(&estimator, &cfg, &input));
    }
    TEST_ASSERT(!estimator.result.resistance_valid);
    soh_close_resistance_episode(&estimator, &cfg, &input);
    TEST_ASSERT(estimator.result.resistance_valid);
    TEST_ASSERT(estimator.result.resistance_growth_ratio < 1.05f);
    TEST_ASSERT(estimator.result.resistance_growth_upper < 1.10f);
    const float healthy_retained = estimator.result.resistance_growth_upper;

    /* Sustained ageing still has to move the monotonic retained state. */
    for(uint8_t n = 0u; n < AMS_SOH_RESISTANCE_EPISODE_MIN_OBSERVATIONS; n++)
    {
        for(uint8_t segment = 0u; segment < AMS_SOH_SEGMENTS; segment++)
        {
            input.segment_resistance_growth_ratio[segment] = 1.40f;
            input.segment_resistance_confidence_pct[segment] = 100u;
            input.segment_resistance_valid[segment] = 1u;
        }
        input.now_ms += 100u;
        input.measurement_timestamp_ms = input.now_ms;
        input.measurement_sequence++;
        TEST_ASSERT(ams_soh_update(&estimator, &cfg, &input));
    }
    soh_close_resistance_episode(&estimator, &cfg, &input);
    TEST_ASSERT(estimator.result.resistance_growth_ratio > 1.39f);
    TEST_ASSERT(estimator.result.resistance_growth_upper > healthy_retained);
    return true;
}

static bool test_resistance_soh_correlated_episode_rejection(void)
{
    ams_soh_config_t cfg;
    ams_soh_default_config(&cfg);
    ams_soh_estimator_t estimator;
    ams_soh_init(&estimator, &cfg);
    ams_soh_input_t input = nominal_soh_input();

    /* Reproduce the licensed C5 failure shape: a fresh-R0 episode begins high
     * and decays as the polarization state settles.  The old nine-sample
     * confirmation latched the first 8 s of this correlated burst at ~1.14.
     * Episode-level confirmation must wait for the burst to close and summarize
     * the whole bounded episode instead. */
    const uint8_t observations = 32u;
    for(uint8_t n = 0u; n < observations; n++)
    {
        const float ratio = 1.14f - (0.08f * (float)n /
            (float)(observations - 1u));
        for(uint8_t segment = 0u; segment < AMS_SOH_SEGMENTS; segment++)
        {
            input.segment_resistance_growth_ratio[segment] = ratio;
            input.segment_resistance_confidence_pct[segment] = 100u;
            input.segment_resistance_valid[segment] = 1u;
        }
        input.now_ms += 1000u;
        input.measurement_timestamp_ms = input.now_ms;
        input.measurement_sequence++;
        TEST_ASSERT(ams_soh_update(&estimator, &cfg, &input));
        TEST_ASSERT(!estimator.result.resistance_valid);
    }

    soh_close_resistance_episode(&estimator, &cfg, &input);
    TEST_ASSERT(estimator.result.resistance_valid);
    TEST_ASSERT(estimator.result.resistance_growth_ratio < 1.11f);
    TEST_ASSERT(estimator.result.resistance_growth_ratio > 1.08f);
    return true;
}

static bool test_power_can_contract(void)
{
    ams_power_can_snapshot_t snapshot;
    memset(&snapshot, 0, sizeof(snapshot));
    snapshot.generation = 1u;
    snapshot.measurement_sequence = 12u;
    snapshot.measurement_timestamp_ms = 1000u;
    snapshot.solve_timestamp_ms = 1050u;
    snapshot.valid = 1u;
    snapshot.authority_valid = 1u;
    snapshot.discharge_current_a[0] = 100.0f;
    snapshot.discharge_current_a[1] = 80.0f;
    snapshot.discharge_current_a[2] = 70.0f;
    snapshot.discharge_current_a[3] = 65.0f;
    snapshot.charge_current_a[0] = -11.0f;
    snapshot.charge_current_a[1] = -10.0f;
    snapshot.charge_current_a[2] = -9.0f;
    snapshot.charge_current_a[3] = -8.0f;
    snapshot.discharge_power_w_1s = 22000.0f;
    snapshot.charge_power_w_1s = 3000.0f;
    snapshot.capacity_soh = 0.95f;
    snapshot.capacity_soh_lower = 0.90f;
    snapshot.resistance_growth_upper = 1.20f;
    snapshot.capacity_confidence_pct = 75u;
    snapshot.resistance_confidence_pct = 80u;
    snapshot.capacity_valid = 1u;
    snapshot.resistance_valid = 1u;
    snapshot.discharge_binding[0] = AMS_SOP_BIND_CELL_UV;
    snapshot.discharge_binding[1] = AMS_SOP_BIND_CURRENT_PATH;
    snapshot.discharge_binding[2] = AMS_SOP_BIND_CORE_TEMP;
    snapshot.discharge_binding[3] = AMS_SOP_BIND_MISSION_PROFILE;
    snapshot.charge_binding[0] = AMS_SOP_BIND_CELL_OV;
    snapshot.charge_binding[1] = AMS_SOP_BIND_FUSE_THERMAL;
    snapshot.charge_binding[2] = AMS_SOP_BIND_SURFACE_TEMP;
    snapshot.charge_binding[3] = AMS_SOP_BIND_SOC_HIGH;
    snapshot.discharge_limiting_segment[0] = 0u;
    snapshot.discharge_limiting_segment[1] = AMS_SOP_INVALID_INDEX;
    snapshot.discharge_limiting_segment[2] = 2u;
    snapshot.discharge_limiting_segment[3] = 4u;
    snapshot.charge_limiting_segment[0] = 1u;
    snapshot.charge_limiting_segment[1] = 3u;
    snapshot.charge_limiting_segment[2] = 4u;
    snapshot.charge_limiting_segment[3] = AMS_SOP_INVALID_INDEX;

    uint8_t payload[8];
    ams_power_can_encode_dcl(&snapshot, 5u, 1100u, payload);
    TEST_ASSERT(ams_power_can_frame_valid(AMS_POWER_CAN_DCL_ID, payload));
    ams_power_can_limit_t limit;
    TEST_ASSERT(ams_power_can_decode_limit(AMS_POWER_CAN_DCL_ID,
                                           payload, &limit));
    TEST_ASSERT(limit.counter == 5u);
    TEST_ASSERT((limit.flags & AMS_POWER_CAN_FLAG_VALID) != 0u);
    TEST_ASSERT(fabsf(limit.current_limit_a - 80.0f) < 0.01f);
    TEST_ASSERT(fabsf(limit.power_limit_w - 22000.0f) < 1.0f);
    payload[3] ^= 0x01u;
    TEST_ASSERT(!ams_power_can_frame_valid(AMS_POWER_CAN_DCL_ID, payload));

    ams_power_can_encode_dcl(&snapshot, 6u, 1300u, payload);
    TEST_ASSERT(ams_power_can_decode_limit(AMS_POWER_CAN_DCL_ID,
                                           payload, &limit));
    TEST_ASSERT(limit.current_limit_a == 0.0f);
    TEST_ASSERT((limit.flags & AMS_POWER_CAN_FLAG_VALID) == 0u);
    TEST_ASSERT((limit.flags & AMS_POWER_CAN_FLAG_FALLBACK) != 0u);

    ams_power_can_encode_soh(&snapshot, 7u, 1100u, payload);
    ams_power_can_soh_t soh;
    TEST_ASSERT(ams_power_can_decode_soh(payload, &soh));
    TEST_ASSERT(soh.capacity_valid && soh.resistance_valid);
    TEST_ASSERT(fabsf(soh.capacity_soh - 0.95f) < 0.01f);

    ams_power_can_encode_envelope(&snapshot, 8u, 1100u, payload);
    ams_power_can_envelope_t envelope;
    TEST_ASSERT(ams_power_can_decode_envelope(payload, &envelope));
    TEST_ASSERT(envelope.discharge_constant_current_a
                [AMS_POWER_CAN_HORIZON_0P1_S] == 100.0f);
    TEST_ASSERT(envelope.discharge_constant_current_a
                [AMS_POWER_CAN_HORIZON_30_S] == 65.0f);
    TEST_ASSERT(envelope.charge_constant_current_a
                [AMS_POWER_CAN_HORIZON_30_S] == 8.0f);

    snapshot.mission_profile = AMS_MISSION_LIMP_HOME;
    snapshot.mission_horizon_index = 3u;
    snapshot.fuse_utilization = 0.76f;
    snapshot.minimum_core_temp_c = 22.0f;
    snapshot.thermal_energy_to_target_wh = 26.4f;
    snapshot.thermal_ready = 0u;
    snapshot.fuse_authority_valid = 1u;
    snapshot.limp_latched = 1u;
    snapshot.r0_bootstrap_progress_pct = 65u;
    ams_power_can_encode_strategy(&snapshot, 5u, 1100u, payload);
    ams_power_can_strategy_t strategy;
    TEST_ASSERT(ams_power_can_decode_strategy(payload, &strategy));
    TEST_ASSERT(strategy.counter == 5u);
    TEST_ASSERT(strategy.mission_profile == AMS_MISSION_LIMP_HOME);
    TEST_ASSERT(strategy.mission_horizon_index == 3u);
    TEST_ASSERT(fabsf(strategy.fuse_utilization - 0.76f) < 0.01f);
    TEST_ASSERT(fabsf(strategy.minimum_core_temp_c - 22.0f) < 0.01f);
    TEST_ASSERT(fabsf(strategy.thermal_energy_to_target_wh - 26.4f) < 0.11f);
    TEST_ASSERT(strategy.fuse_authority_valid && strategy.limp_latched);
    TEST_ASSERT(strategy.r0_bootstrap_progress_pct == 65u);

    ams_power_can_encode_bindings(&snapshot, 5u, 1100u, payload);
    ams_power_can_bindings_t bindings;
    TEST_ASSERT(ams_power_can_decode_bindings(payload, &bindings));
    TEST_ASSERT(bindings.counter == 5u);
    TEST_ASSERT(bindings.discharge_binding
                [AMS_POWER_CAN_HORIZON_0P1_S] == AMS_SOP_BIND_CELL_UV);
    TEST_ASSERT(bindings.discharge_binding
                [AMS_POWER_CAN_HORIZON_10_S] == AMS_SOP_BIND_CORE_TEMP);
    TEST_ASSERT(bindings.discharge_binding
                [AMS_POWER_CAN_HORIZON_30_S] ==
                AMS_SOP_BIND_MISSION_PROFILE);
    TEST_ASSERT(bindings.charge_binding
                [AMS_POWER_CAN_HORIZON_0P1_S] == AMS_SOP_BIND_CELL_OV);
    TEST_ASSERT(bindings.charge_binding
                [AMS_POWER_CAN_HORIZON_30_S] == AMS_SOP_BIND_SOC_HIGH);
    TEST_ASSERT(bindings.discharge_limiting_segment
                [AMS_POWER_CAN_HORIZON_30_S] == 4u);
    TEST_ASSERT(bindings.charge_limiting_segment
                [AMS_POWER_CAN_HORIZON_30_S] == 0x0Fu);
    payload[6] = 0x5Eu; /* invalid segment nibble 14 */
    payload[7] = ams_power_can_crc8(AMS_POWER_CAN_BINDINGS_ID, payload);
    TEST_ASSERT(!ams_power_can_decode_bindings(payload, &bindings));
    return true;
}

static bool test_power_state_integration(void)
{
    ams_measurement_snapshot_t measurement;
    ams_estimator_t estimator;
    ams_power_policy_t policy;
    ams_power_state_t state;
    memset(&measurement, 0, sizeof(measurement));
    memset(&estimator, 0, sizeof(estimator));
    memset(&policy, 0, sizeof(policy));

    measurement.sequence = 3u;
    measurement.publication_tick = 950u;
    measurement.voltage_complete_tick = 950u;
    measurement.validity_flags = AMS_MEAS_VALID_VOLTAGE |
        AMS_MEAS_VALID_TEMPERATURE | AMS_MEAS_VALID_CURRENT |
        AMS_MEAS_BALANCE_RECOVERED;
    measurement.current.average_A = 0.0f;
    measurement.current.valid = true;
    measurement.current.calibration_record_confident = true;
    measurement.current.calibration_id = 42u;
    measurement.current.uncertainty_mA = 500u;
    for(uint8_t segment = 0u; segment < AMS_SOP_SEGMENTS; segment++)
    {
        measurement.cell_usable_mask[segment] = AMS_SOP_FULL_CELL_MASK;
        measurement.temp_usable_mask[segment] = 0x00FFFFFFu;
        for(uint8_t cell = 0u; cell < AMS_SOP_CELLS_PER_SEGMENT; cell++)
        {
            measurement.cell_mv[segment][cell] = 3750u;
        }
        for(uint8_t sensor = 0u; sensor < NTEMPS; sensor++)
        {
            measurement.temp_deci_c[segment][sensor] = 270;
        }
    }

    estimator.enabled = 1u;
    estimator.instance_count = AMS_SOP_SEGMENTS;
    for(uint8_t segment = 0u; segment < AMS_SOP_SEGMENTS; segment++)
    {
        ams_ekf_instance_t *instance = &estimator.inst[segment];
        instance->cfg.enabled = 1u;
        instance->cfg.first_series_group =
            (uint16_t)(segment * AMS_SOP_CELLS_PER_SEGMENT);
        instance->cfg.series_group_count = AMS_SOP_CELLS_PER_SEGMENT;
        instance->cfg.parallel_cell_count = 6.0f;
        instance->soc = 0.55f;
        instance->r0_ohm = ams_p42a_r0_ohm(0.55f, 28.0f);
        instance->t_core_C = 28.0f;
        instance->p_soc = 1.0e-6f;
        instance->p_vp1 = 1.0e-7f;
        instance->p_vp2 = 1.0e-7f;
        instance->p_r0 = 1.0e-9f;
        instance->valid = 1u;
        instance->acquisition.state = AMS_EKF_ACQ_COMPLETE;
        instance->acquisition.anchor_count = 1u;
        estimator.resistance_soh[segment].resistance_growth_ratio = 1.05f;
        estimator.resistance_soh[segment].observation_confidence_pct = 80u;
        estimator.resistance_soh[segment].status_flags =
            AMS_SOH_STATUS_ADVISORY_VALID;
    }

    policy.operating_mode = AMS_SOP_MODE_DRIVE;
    policy.discharge_authorized = 1u;
    policy.current_calibrated = 1u;
    policy.current_polarity_validated = 1u;
    ams_power_state_init(&state);
    TEST_ASSERT(ams_power_state_update(&state, &measurement, &estimator,
                                       &policy, 1000u, 0.1f));
    TEST_ASSERT(state.can_snapshot.valid);
    TEST_ASSERT(state.can_snapshot.authority_valid);
    TEST_ASSERT(state.can_snapshot.discharge_current_a[1] > 0.0f);
    TEST_ASSERT(state.can_snapshot.discharge_current_a[1] <= 4.01f);
    TEST_ASSERT((state.can_snapshot.reason_flags &
                 AMS_SOP_REASON_AMBIENT_PROXY) != 0u);
    TEST_ASSERT(state.can_snapshot.mission_profile == AMS_MISSION_ENDURANCE);
    TEST_ASSERT((state.can_snapshot.reason_flags &
                 AMS_SOP_REASON_MISSION_FALLBACK) != 0u);
    for(uint8_t h = 0u; h < AMS_SOP_HORIZONS - 1u; h++)
    {
        TEST_ASSERT(state.can_snapshot.discharge_current_a[h] <=
                    state.can_snapshot.discharge_current_a[3] + 0.001f);
    }

    policy.requested_mission = AMS_MISSION_QUALIFY;
    policy.mission_request_valid = 1u;
    policy.stationary_confirmed = 1u;
    measurement.sequence++;
    measurement.publication_tick = 1050u;
    TEST_ASSERT(ams_power_state_update(&state, &measurement, &estimator,
                                       &policy, 1100u, 0.1f));
    TEST_ASSERT(state.can_snapshot.mission_profile == AMS_MISSION_QUALIFY);
    TEST_ASSERT(state.can_snapshot.mission_horizon_index == 1u);

    for(uint8_t segment = 0u; segment < AMS_SOP_SEGMENTS; segment++)
    {
        estimator.inst[segment].soc = 0.299f;
        estimator.inst[segment].r0_ohm =
            ams_p42a_r0_ohm(0.299f, 28.0f);
    }
    measurement.sequence++;
    measurement.publication_tick = 1150u;
    TEST_ASSERT(ams_power_state_update(&state, &measurement, &estimator,
                                       &policy, 1200u, 0.1f));
    TEST_ASSERT(state.can_snapshot.mission_profile == AMS_MISSION_LIMP_HOME);
    TEST_ASSERT(state.can_snapshot.limp_latched);
    TEST_ASSERT(state.can_snapshot.discharge_current_a[0] <= 35.01f);

    measurement.temp_usable_mask[4] &= ~(1u << 23u);
    measurement.sequence++;
    measurement.publication_tick = 1250u;
    TEST_ASSERT(!ams_power_state_update(&state, &measurement, &estimator,
                                        &policy, 1300u, 0.1f));
    TEST_ASSERT(!state.can_snapshot.valid);
    TEST_ASSERT(state.can_snapshot.discharge_current_a[1] == 0.0f);

    measurement.temp_usable_mask[4] |= (1u << 23u);
    measurement.current.calibration_record_confident = false;
    measurement.sequence++;
    measurement.publication_tick = 1350u;
    ams_power_state_init(&state);
    TEST_ASSERT(!ams_power_state_update(&state, &measurement, &estimator,
                                        &policy, 1400u, 0.1f));
    TEST_ASSERT((state.soh.result.last_reason_flags &
                 AMS_SOH_REASON_CURRENT_CALIBRATION) != 0u);
    TEST_ASSERT(!state.can_snapshot.valid);
    return true;
}

typedef bool (*test_fn_t)(void);
typedef struct { const char *name; test_fn_t fn; } test_case_t;

int main(void)
{
    const test_case_t tests[] = {
        {"nominal predictive solver", test_nominal_solver},
        {"input/config fail-closed matrix", test_input_and_config_fail_closed},
        {"direction authority", test_direction_authority},
        {"weakest-cell/covariance binding", test_weakest_cell_and_covariance_bind},
        {"charge voltage/temperature binding", test_charge_voltage_and_temperature_bind},
        {"SoH priors and aged model", test_soh_priors_and_aged_model},
        {"bisection vs brute force", test_bisection_against_bruteforce},
        {"limit slew policy", test_slew_policy},
        {"cause-scheduled recovery", test_cause_scheduled_recovery},
        {"conservative fuse observer", test_fuse_observer},
        {"mission strategy/request contract", test_mission_strategy_and_request},
        {"strategy/fuse randomized invariants", test_strategy_fuse_randomized_invariants},
        {"randomized invariants", test_randomized_invariants},
        {"capacity SoH observability", test_capacity_soh_observability},
        {"capacity SoH rejection/persistence", test_capacity_soh_rejections_and_persistence},
        {"resistance SoH retention/persistence", test_resistance_soh_retention},
        {"resistance SoH transient-spike rejection", test_resistance_soh_transient_spike_rejection},
        {"resistance SoH correlated-episode rejection", test_resistance_soh_correlated_episode_rejection},
        {"power CAN CRC/freshness contract", test_power_can_contract},
        {"measurement-to-DADEKF-to-power integration", test_power_state_integration},
    };

    for(size_t i = 0u; i < sizeof(tests) / sizeof(tests[0]); i++)
    {
        if(!tests[i].fn())
        {
            return 1;
        }
        printf("PASS %s\n", tests[i].name);
    }
    puts("ALL SOP/SOH PRODUCTION CORE TESTS PASSED");
    return 0;
}
