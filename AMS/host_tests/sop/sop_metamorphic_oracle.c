/*
 * Deterministic black-box metamorphic test for the DER26 SoP solver.
 *
 * This does not reuse the production feasibility internals. It checks
 * directionally necessary properties of the public ams_sop_solve() API:
 * nested horizons, monotonic response to stricter constraints, and fail-zero
 * behavior for invalid inputs. It is structural verification, not physical
 * calibration or an independent electrothermal trajectory oracle.
 */
#include "estimator/ams_estimator_lut.h"
#include "sop/ams_sop.h"

#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef AMS_SOP_METAMORPHIC_STATES
#define AMS_SOP_METAMORPHIC_STATES 20000u
#endif

#define TOLERANCE_A 0.05f
#define MAX_REQUESTED_STATES 1000000u

typedef struct
{
    uint32_t base_drive;
    uint32_t base_charge;
    uint32_t nesting;
    uint32_t lower_min_voltage;
    uint32_t higher_max_voltage;
    uint32_t higher_resistance;
    uint32_t higher_uncertainty;
    uint32_t higher_temperature;
    uint32_t lower_soc;
    uint32_t higher_soc;
    uint32_t tighter_discharge_ceiling;
    uint32_t tighter_charge_ceiling;
    uint32_t cold_charge_block;
    uint32_t invalid_fail_zero;
} violation_counts_t;

typedef struct
{
    const char *tag;
    uint32_t seed;
    bool valid;
} first_failure_t;

static uint32_t rng_state;

static uint32_t xorshift32(void)
{
    uint32_t x = rng_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    rng_state = x;
    return x;
}

static float random_float(float low, float high)
{
    const float unit = (float)(xorshift32() >> 8) / (float)(1u << 24);
    return low + ((high - low) * unit);
}

static void record_violation(uint32_t *count,
                             uint32_t seed,
                             const char *tag,
                             first_failure_t *first)
{
    (*count)++;
    if(!first->valid)
    {
        first->tag = tag;
        first->seed = seed;
        first->valid = true;
    }
}

static uint32_t parse_state_count(int argc, char **argv)
{
    if(argc < 2)
    {
        return AMS_SOP_METAMORPHIC_STATES;
    }

    errno = 0;
    char *end = NULL;
    const unsigned long value = strtoul(argv[1], &end, 10);
    if(errno != 0 || end == argv[1] || *end != '\0' || value == 0ul ||
       value > (unsigned long)MAX_REQUESTED_STATES)
    {
        fprintf(stderr,
                "usage: %s [states-per-direction: 1..%u]\n",
                argv[0],
                MAX_REQUESTED_STATES);
        return 0u;
    }
    return (uint32_t)value;
}

static void build_input(ams_sop_input_t *input,
                        ams_sop_operating_mode_t mode)
{
    memset(input, 0, sizeof(*input));
    input->measurement_sequence = 100u;
    input->measurement_timestamp_ms = 995u;
    input->now_ms = 1000u;
    input->pack_current_a = (mode == AMS_SOP_MODE_CHARGE)
                                ? random_float(-10.0f, 0.0f)
                                : random_float(0.0f, 40.0f);
    input->pack_current_uncertainty_a = random_float(0.2f, 1.5f);
    input->ambient_temp_c = random_float(15.0f, 35.0f);
    input->operating_mode = mode;
    input->measurement_valid = 1u;
    input->estimator_valid = 1u;
    input->estimator_acquired = 1u;
    input->estimator_segment_topology = 1u;
    input->current_calibrated = 1u;
    input->current_polarity_validated = 1u;
    input->ambient_measured = 1u;
    input->balance_recovered = 1u;
    input->discharge_authorized = 1u;
    input->charger_authorized = 1u;
    input->regen_authorized = 1u;

    for(uint8_t segment = 0u; segment < AMS_SOP_SEGMENTS; segment++)
    {
        ams_sop_segment_input_t *state = &input->segment[segment];
        const float soc = random_float(0.30f, 0.80f);
        const float temperature_c = random_float(18.0f, 40.0f);
        const float ocv_v = ams_p42a_ocv_v(soc, temperature_c);

        state->soc = soc;
        state->vp1_v = 0.0f;
        state->vp2_v = 0.0f;
        state->r0_ohm = random_float(0.006f, 0.014f);
        state->core_temp_c = temperature_c;
        state->surface_max_temp_c = temperature_c - random_float(0.0f, 2.0f);
        state->p_soc = 1.0e-4f;
        state->p_vp1 = 1.0e-6f;
        state->p_vp2 = 1.0e-6f;
        state->p_r0 = 1.0e-8f;
        state->innovation_v = 0.0f;
        state->capacity_soh_lower = 0.90f;
        state->resistance_soh_upper = 1.10f;
        state->max_cell_age_ms = 10u;
        state->cell_usable_mask = AMS_SOP_FULL_CELL_MASK;
        state->estimator_valid = 1u;
        state->model_domain_flags = 0u;
        state->capacity_soh_valid = 0u;
        state->resistance_soh_valid = 0u;

        for(uint8_t cell = 0u; cell < AMS_SOP_CELLS_PER_SEGMENT; cell++)
        {
            state->cell_voltage_v[cell] =
                ocv_v + random_float(-0.015f, 0.015f);
        }
    }
}

static bool valid_authoritative_result(ams_sop_status_t status,
                                       const ams_sop_result_t *result)
{
    if(status != AMS_SOP_OK || result == NULL || !result->valid ||
       !result->authority_valid || result->fallback_active)
    {
        return false;
    }

    for(uint8_t horizon = 0u; horizon < AMS_SOP_HORIZONS; horizon++)
    {
        if(!isfinite(result->discharge_current_a[horizon]) ||
           !isfinite(result->charge_current_a[horizon]) ||
           result->discharge_current_a[horizon] < 0.0f ||
           result->charge_current_a[horizon] > 0.0f)
        {
            return false;
        }
    }
    return true;
}

static bool result_is_fail_zero(ams_sop_status_t status,
                                const ams_sop_result_t *result)
{
    if(result == NULL)
    {
        return false;
    }

    if(status == AMS_SOP_OK && result->authority_valid)
    {
        return false;
    }

    for(uint8_t horizon = 0u; horizon < AMS_SOP_HORIZONS; horizon++)
    {
        if(fabsf(result->discharge_current_a[horizon]) > TOLERANCE_A ||
           fabsf(result->charge_current_a[horizon]) > TOLERANCE_A)
        {
            return false;
        }
    }
    return true;
}

static float maximum_discharge(const ams_sop_result_t *result)
{
    float maximum = result->discharge_current_a[0];
    for(uint8_t horizon = 1u; horizon < AMS_SOP_HORIZONS; horizon++)
    {
        if(result->discharge_current_a[horizon] > maximum)
        {
            maximum = result->discharge_current_a[horizon];
        }
    }
    return maximum;
}

static bool discharge_did_not_increase(const ams_sop_result_t *baseline,
                                       const ams_sop_result_t *perturbed)
{
    for(uint8_t horizon = 0u; horizon < AMS_SOP_HORIZONS; horizon++)
    {
        if(perturbed->discharge_current_a[horizon] >
           baseline->discharge_current_a[horizon] + TOLERANCE_A)
        {
            return false;
        }
    }
    return true;
}

static bool charge_magnitude_did_not_increase(const ams_sop_result_t *baseline,
                                               const ams_sop_result_t *perturbed)
{
    for(uint8_t horizon = 0u; horizon < AMS_SOP_HORIZONS; horizon++)
    {
        if(fabsf(perturbed->charge_current_a[horizon]) >
           fabsf(baseline->charge_current_a[horizon]) + TOLERANCE_A)
        {
            return false;
        }
    }
    return true;
}

static bool limits_are_nested(const ams_sop_result_t *result)
{
    for(uint8_t horizon = 1u; horizon < AMS_SOP_HORIZONS; horizon++)
    {
        if(result->discharge_current_a[horizon] >
               result->discharge_current_a[horizon - 1u] + TOLERANCE_A ||
           fabsf(result->charge_current_a[horizon]) >
               fabsf(result->charge_current_a[horizon - 1u]) + TOLERANCE_A)
        {
            return false;
        }
    }
    return true;
}

static void lower_weakest_cell(ams_sop_input_t *input)
{
    float minimum_v = INFINITY;
    uint8_t minimum_segment = 0u;
    uint8_t minimum_cell = 0u;

    for(uint8_t segment = 0u; segment < AMS_SOP_SEGMENTS; segment++)
    {
        for(uint8_t cell = 0u; cell < AMS_SOP_CELLS_PER_SEGMENT; cell++)
        {
            const float voltage_v = input->segment[segment].cell_voltage_v[cell];
            if(voltage_v < minimum_v)
            {
                minimum_v = voltage_v;
                minimum_segment = segment;
                minimum_cell = cell;
            }
        }
    }
    input->segment[minimum_segment].cell_voltage_v[minimum_cell] -= 0.05f;
}

static void raise_strongest_cell(ams_sop_input_t *input)
{
    float maximum_v = -INFINITY;
    uint8_t maximum_segment = 0u;
    uint8_t maximum_cell = 0u;

    for(uint8_t segment = 0u; segment < AMS_SOP_SEGMENTS; segment++)
    {
        for(uint8_t cell = 0u; cell < AMS_SOP_CELLS_PER_SEGMENT; cell++)
        {
            const float voltage_v = input->segment[segment].cell_voltage_v[cell];
            if(voltage_v > maximum_v)
            {
                maximum_v = voltage_v;
                maximum_segment = segment;
                maximum_cell = cell;
            }
        }
    }
    input->segment[maximum_segment].cell_voltage_v[maximum_cell] += 0.05f;
}

static void check_drive_states(uint32_t state_count,
                               const ams_sop_config_t *config,
                               violation_counts_t *violations,
                               first_failure_t *first,
                               uint32_t *solved,
                               uint32_t *nonzero)
{
    for(uint32_t seed = 1u; seed <= state_count; seed++)
    {
        rng_state = (seed * 2654435761u) + 1u;
        ams_sop_input_t baseline_input;
        ams_sop_result_t baseline_result;
        build_input(&baseline_input, AMS_SOP_MODE_DRIVE);
        const ams_sop_status_t baseline_status =
            ams_sop_solve(&baseline_input, config, &baseline_result);

        if(!valid_authoritative_result(baseline_status, &baseline_result))
        {
            record_violation(&violations->base_drive,
                             seed,
                             "valid-drive-base-rejected",
                             first);
            continue;
        }
        (*solved)++;
        if(maximum_discharge(&baseline_result) > 0.1f)
        {
            (*nonzero)++;
        }

        if(!limits_are_nested(&baseline_result))
        {
            record_violation(&violations->nesting,
                             seed,
                             "horizon-nesting",
                             first);
        }

        ams_sop_input_t perturbed_input = baseline_input;
        ams_sop_result_t perturbed_result;
        ams_sop_status_t perturbed_status;

        lower_weakest_cell(&perturbed_input);
        perturbed_status = ams_sop_solve(&perturbed_input,
                                         config,
                                         &perturbed_result);
        if(!valid_authoritative_result(perturbed_status, &perturbed_result) ||
           !discharge_did_not_increase(&baseline_result, &perturbed_result))
        {
            record_violation(&violations->lower_min_voltage,
                             seed,
                             "lower-min-voltage-raised-DCL",
                             first);
        }

        perturbed_input = baseline_input;
        for(uint8_t segment = 0u; segment < AMS_SOP_SEGMENTS; segment++)
        {
            perturbed_input.segment[segment].r0_ohm *= 1.25f;
            perturbed_input.segment[segment].p_r0 *= 4.0f;
        }
        perturbed_status = ams_sop_solve(&perturbed_input,
                                         config,
                                         &perturbed_result);
        if(!valid_authoritative_result(perturbed_status, &perturbed_result) ||
           !discharge_did_not_increase(&baseline_result, &perturbed_result))
        {
            record_violation(&violations->higher_resistance,
                             seed,
                             "higher-resistance-raised-DCL",
                             first);
        }

        perturbed_input = baseline_input;
        perturbed_input.pack_current_uncertainty_a += 5.0f;
        perturbed_status = ams_sop_solve(&perturbed_input,
                                         config,
                                         &perturbed_result);
        if(!valid_authoritative_result(perturbed_status, &perturbed_result) ||
           !discharge_did_not_increase(&baseline_result, &perturbed_result) ||
           !charge_magnitude_did_not_increase(&baseline_result,
                                              &perturbed_result))
        {
            record_violation(&violations->higher_uncertainty,
                             seed,
                             "higher-uncertainty-raised-capability",
                             first);
        }

        perturbed_input = baseline_input;
        for(uint8_t segment = 0u; segment < AMS_SOP_SEGMENTS; segment++)
        {
            perturbed_input.segment[segment].core_temp_c += 8.0f;
            perturbed_input.segment[segment].surface_max_temp_c += 8.0f;
        }
        perturbed_status = ams_sop_solve(&perturbed_input,
                                         config,
                                         &perturbed_result);
        if(!valid_authoritative_result(perturbed_status, &perturbed_result) ||
           !discharge_did_not_increase(&baseline_result, &perturbed_result))
        {
            record_violation(&violations->higher_temperature,
                             seed,
                             "higher-temperature-raised-DCL",
                             first);
        }

        perturbed_input = baseline_input;
        for(uint8_t segment = 0u; segment < AMS_SOP_SEGMENTS; segment++)
        {
            perturbed_input.segment[segment].soc -= 0.05f;
        }
        perturbed_status = ams_sop_solve(&perturbed_input,
                                         config,
                                         &perturbed_result);
        if(!valid_authoritative_result(perturbed_status, &perturbed_result) ||
           !discharge_did_not_increase(&baseline_result, &perturbed_result))
        {
            record_violation(&violations->lower_soc,
                             seed,
                             "lower-soc-raised-DCL",
                             first);
        }

        ams_sop_config_t tighter_config = *config;
        for(uint8_t horizon = 0u; horizon < AMS_SOP_HORIZONS; horizon++)
        {
            tighter_config.discharge_current_max_a[horizon] *= 0.5f;
        }
        perturbed_status = ams_sop_solve(&baseline_input,
                                         &tighter_config,
                                         &perturbed_result);
        if(!valid_authoritative_result(perturbed_status, &perturbed_result) ||
           !discharge_did_not_increase(&baseline_result, &perturbed_result))
        {
            record_violation(&violations->tighter_discharge_ceiling,
                             seed,
                             "tighter-discharge-ceiling-raised-DCL",
                             first);
        }

        perturbed_input = baseline_input;
        perturbed_input.segment[2].cell_voltage_v[7] = NAN;
        perturbed_status = ams_sop_solve(&perturbed_input,
                                         config,
                                         &perturbed_result);
        if(!result_is_fail_zero(perturbed_status, &perturbed_result))
        {
            record_violation(&violations->invalid_fail_zero,
                             seed,
                             "NaN-input-not-fail-zero",
                             first);
        }

        perturbed_input = baseline_input;
        perturbed_input.segment[1].r0_ohm = INFINITY;
        perturbed_status = ams_sop_solve(&perturbed_input,
                                         config,
                                         &perturbed_result);
        if(!result_is_fail_zero(perturbed_status, &perturbed_result))
        {
            record_violation(&violations->invalid_fail_zero,
                             seed,
                             "infinite-input-not-fail-zero",
                             first);
        }

        perturbed_input = baseline_input;
        perturbed_input.now_ms = 1300u;
        perturbed_status = ams_sop_solve(&perturbed_input,
                                         config,
                                         &perturbed_result);
        if(!result_is_fail_zero(perturbed_status, &perturbed_result))
        {
            record_violation(&violations->invalid_fail_zero,
                             seed,
                             "stale-input-not-fail-zero",
                             first);
        }
    }
}

static void check_charge_states(uint32_t state_count,
                                const ams_sop_config_t *config,
                                violation_counts_t *violations,
                                first_failure_t *first,
                                uint32_t *solved,
                                uint32_t *nonzero)
{
    for(uint32_t seed = 1u; seed <= state_count; seed++)
    {
        rng_state = 777u + (seed * 40503u);
        ams_sop_input_t baseline_input;
        ams_sop_result_t baseline_result;
        build_input(&baseline_input, AMS_SOP_MODE_CHARGE);

        for(uint8_t segment = 0u; segment < AMS_SOP_SEGMENTS; segment++)
        {
            ams_sop_segment_input_t *state = &baseline_input.segment[segment];
            state->soc = random_float(0.20f, 0.55f);
            const float ocv_v = ams_p42a_ocv_v(state->soc,
                                               state->core_temp_c);
            for(uint8_t cell = 0u; cell < AMS_SOP_CELLS_PER_SEGMENT; cell++)
            {
                state->cell_voltage_v[cell] =
                    ocv_v + random_float(-0.010f, 0.010f);
            }
        }

        const ams_sop_status_t baseline_status =
            ams_sop_solve(&baseline_input, config, &baseline_result);
        if(!valid_authoritative_result(baseline_status, &baseline_result))
        {
            record_violation(&violations->base_charge,
                             seed,
                             "valid-charge-base-rejected",
                             first);
            continue;
        }
        (*solved)++;
        if(fabsf(baseline_result.charge_current_a[0]) > 0.1f)
        {
            (*nonzero)++;
        }

        if(!limits_are_nested(&baseline_result))
        {
            record_violation(&violations->nesting,
                             seed,
                             "charge-horizon-nesting",
                             first);
        }

        ams_sop_input_t perturbed_input = baseline_input;
        ams_sop_result_t perturbed_result;
        ams_sop_status_t perturbed_status;

        raise_strongest_cell(&perturbed_input);
        perturbed_status = ams_sop_solve(&perturbed_input,
                                         config,
                                         &perturbed_result);
        if(!valid_authoritative_result(perturbed_status, &perturbed_result) ||
           !charge_magnitude_did_not_increase(&baseline_result,
                                              &perturbed_result))
        {
            record_violation(&violations->higher_max_voltage,
                             seed,
                             "higher-max-voltage-raised-CCL",
                             first);
        }

        perturbed_input = baseline_input;
        for(uint8_t segment = 0u; segment < AMS_SOP_SEGMENTS; segment++)
        {
            perturbed_input.segment[segment].r0_ohm *= 1.25f;
            perturbed_input.segment[segment].p_r0 *= 4.0f;
        }
        perturbed_status = ams_sop_solve(&perturbed_input,
                                         config,
                                         &perturbed_result);
        if(!valid_authoritative_result(perturbed_status, &perturbed_result) ||
           !charge_magnitude_did_not_increase(&baseline_result,
                                              &perturbed_result))
        {
            record_violation(&violations->higher_resistance,
                             seed,
                             "higher-resistance-raised-CCL",
                             first);
        }

        perturbed_input = baseline_input;
        for(uint8_t segment = 0u; segment < AMS_SOP_SEGMENTS; segment++)
        {
            perturbed_input.segment[segment].soc += 0.05f;
        }
        perturbed_status = ams_sop_solve(&perturbed_input,
                                         config,
                                         &perturbed_result);
        if(!valid_authoritative_result(perturbed_status, &perturbed_result) ||
           !charge_magnitude_did_not_increase(&baseline_result,
                                              &perturbed_result))
        {
            record_violation(&violations->higher_soc,
                             seed,
                             "higher-soc-raised-CCL",
                             first);
        }

        ams_sop_config_t tighter_config = *config;
        for(uint8_t horizon = 0u; horizon < AMS_SOP_HORIZONS; horizon++)
        {
            tighter_config.charge_current_max_a[horizon] *= 0.5f;
        }
        perturbed_status = ams_sop_solve(&baseline_input,
                                         &tighter_config,
                                         &perturbed_result);
        if(!valid_authoritative_result(perturbed_status, &perturbed_result) ||
           !charge_magnitude_did_not_increase(&baseline_result,
                                              &perturbed_result))
        {
            record_violation(&violations->tighter_charge_ceiling,
                             seed,
                             "tighter-charge-ceiling-raised-CCL",
                             first);
        }

        perturbed_input = baseline_input;
        for(uint8_t segment = 0u; segment < AMS_SOP_SEGMENTS; segment++)
        {
            ams_sop_segment_input_t *state = &perturbed_input.segment[segment];
            state->core_temp_c = 0.5f;
            state->surface_max_temp_c = 0.5f;
            const float ocv_v = ams_p42a_ocv_v(state->soc, 0.5f);
            for(uint8_t cell = 0u; cell < AMS_SOP_CELLS_PER_SEGMENT; cell++)
            {
                state->cell_voltage_v[cell] = ocv_v;
            }
        }
        perturbed_input.ambient_temp_c = 0.5f;
        perturbed_status = ams_sop_solve(&perturbed_input,
                                         config,
                                         &perturbed_result);
        bool charge_blocked = valid_authoritative_result(perturbed_status,
                                                         &perturbed_result);
        for(uint8_t horizon = 0u;
            charge_blocked && horizon < AMS_SOP_HORIZONS;
            horizon++)
        {
            charge_blocked = fabsf(perturbed_result.charge_current_a[horizon]) <=
                             TOLERANCE_A;
        }
        if(!charge_blocked)
        {
            record_violation(&violations->cold_charge_block,
                             seed,
                             "sub-min-temperature-allowed-charge",
                             first);
        }
    }
}

static uint32_t total_violations(const violation_counts_t *v)
{
    return v->base_drive + v->base_charge + v->nesting +
           v->lower_min_voltage + v->higher_max_voltage +
           v->higher_resistance + v->higher_uncertainty +
           v->higher_temperature + v->lower_soc + v->higher_soc +
           v->tighter_discharge_ceiling + v->tighter_charge_ceiling +
           v->cold_charge_block + v->invalid_fail_zero;
}

int main(int argc, char **argv)
{
    const uint32_t state_count = parse_state_count(argc, argv);
    if(state_count == 0u)
    {
        return 2;
    }

    ams_sop_config_t config;
    ams_sop_default_config(&config);
    if(!ams_sop_config_valid(&config))
    {
        fprintf(stderr, "FAIL: default SoP configuration is invalid\n");
        return 2;
    }

    violation_counts_t violations;
    memset(&violations, 0, sizeof(violations));
    first_failure_t first = {0};
    uint32_t drive_solved = 0u;
    uint32_t drive_nonzero = 0u;
    uint32_t charge_solved = 0u;
    uint32_t charge_nonzero = 0u;

    check_drive_states(state_count,
                       &config,
                       &violations,
                       &first,
                       &drive_solved,
                       &drive_nonzero);
    check_charge_states(state_count,
                        &config,
                        &violations,
                        &first,
                        &charge_solved,
                        &charge_nonzero);

    const uint32_t total = total_violations(&violations);
    printf("SoP metamorphic CI: %u drive + %u charge seeded states\n",
           state_count,
           state_count);
    printf("  valid base solves: drive=%u charge=%u; nonzero: DCL=%u CCL=%u\n",
           drive_solved,
           charge_solved,
           drive_nonzero,
           charge_nonzero);
    printf("  violations: baseD=%u baseC=%u nest=%u lowerV=%u higherV=%u "
           "higherR=%u higherUnc=%u higherTemp=%u lowerSoC=%u higherSoC=%u "
           "tightD=%u tightC=%u coldCCL=%u invalid=%u\n",
           violations.base_drive,
           violations.base_charge,
           violations.nesting,
           violations.lower_min_voltage,
           violations.higher_max_voltage,
           violations.higher_resistance,
           violations.higher_uncertainty,
           violations.higher_temperature,
           violations.lower_soc,
           violations.higher_soc,
           violations.tighter_discharge_ceiling,
           violations.tighter_charge_ceiling,
           violations.cold_charge_block,
           violations.invalid_fail_zero);

    if(total == 0u)
    {
        puts("RESULT: PASS (all metamorphic and fail-zero properties held)");
        return 0;
    }

    printf("RESULT: FAIL total=%u first=%s seed=%u\n",
           total,
           first.valid ? first.tag : "unknown",
           first.seed);
    return 1;
}
