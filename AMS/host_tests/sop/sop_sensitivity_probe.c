/*
 * Non-gating SoP characterization utility.
 *
 * This probe is intentionally not a pass/fail test. It shows whether the 30 s
 * discharge result is controlled by the electrothermal model or by a static
 * current-path ceiling at representative operating points.
 */
#include "estimator/ams_estimator_lut.h"
#include "sop/ams_sop.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef struct
{
    const char *name;
    float soc;
    float temperature_c;
} scenario_t;

static void make_input(ams_sop_input_t *input, float soc, float temperature_c)
{
    memset(input, 0, sizeof(*input));
    input->measurement_sequence = 1u;
    input->measurement_timestamp_ms = 995u;
    input->now_ms = 1000u;
    input->pack_current_a = 10.0f;
    input->pack_current_uncertainty_a = 0.5f;
    input->ambient_temp_c = temperature_c;
    input->operating_mode = AMS_SOP_MODE_DRIVE;
    input->measurement_valid = 1u;
    input->estimator_valid = 1u;
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
        state->soc = soc;
        state->core_temp_c = temperature_c;
        state->surface_max_temp_c = temperature_c;
        state->r0_ohm = 0.010f;
        state->p_soc = 1.0e-4f;
        state->p_vp1 = 1.0e-6f;
        state->p_vp2 = 1.0e-6f;
        state->p_r0 = 1.0e-8f;
        state->max_cell_age_ms = 10u;
        state->cell_usable_mask = AMS_SOP_FULL_CELL_MASK;
        state->estimator_valid = 1u;

        const float ocv_v = ams_p42a_ocv_v(soc, temperature_c);
        for(uint8_t cell = 0u; cell < AMS_SOP_CELLS_PER_SEGMENT; cell++)
        {
            state->cell_voltage_v[cell] = ocv_v;
        }
    }
}

static bool solve_30s_dcl(const ams_sop_config_t *config,
                          const ams_sop_input_t *input,
                          float *dcl_a)
{
    ams_sop_result_t result;
    const ams_sop_status_t status = ams_sop_solve(input, config, &result);
    if(status != AMS_SOP_OK || !result.valid || !result.authority_valid)
    {
        return false;
    }
    *dcl_a = result.discharge_current_a[AMS_SOP_HORIZONS - 1u];
    return true;
}

static void print_probe(const char *label,
                        const ams_sop_config_t *config,
                        const ams_sop_input_t *input,
                        float baseline_a)
{
    float value_a = 0.0f;
    if(!solve_30s_dcl(config, input, &value_a))
    {
        printf("  %-31s solve failed\n", label);
        return;
    }
    printf("  %-31s %7.2f A  delta=%+7.2f A\n",
           label,
           value_a,
           value_a - baseline_a);
}

int main(void)
{
    const scenario_t scenarios[] = {
        {"nominal 50% / 30 C", 0.50f, 30.0f},
        {"hot 50% / 48 C", 0.50f, 48.0f},
        {"low SoC 12% / 30 C", 0.12f, 30.0f},
    };

    ams_sop_config_t baseline_config;
    ams_sop_default_config(&baseline_config);

    puts("SoP 30 s DCL sensitivity characterization (non-gating)");
    for(size_t index = 0u;
        index < (sizeof(scenarios) / sizeof(scenarios[0]));
        index++)
    {
        ams_sop_input_t input;
        make_input(&input, scenarios[index].soc, scenarios[index].temperature_c);

        float baseline_a = 0.0f;
        if(!solve_30s_dcl(&baseline_config, &input, &baseline_a))
        {
            printf("\n[%s] baseline solve failed\n", scenarios[index].name);
            continue;
        }
        printf("\n[%s] baseline=%0.2f A\n", scenarios[index].name, baseline_a);

        ams_sop_config_t config = baseline_config;
        config.core_surface_resistance_k_per_w *= 1.20f;
        print_probe("core-surface R +20%", &config, &input, baseline_a);

        config = baseline_config;
        config.surface_ambient_resistance_k_per_w *= 1.20f;
        print_probe("surface-ambient R +20%", &config, &input, baseline_a);

        config = baseline_config;
        config.core_thermal_capacity_j_per_k *= 1.20f;
        print_probe("core thermal capacity +20%", &config, &input, baseline_a);

        config = baseline_config;
        config.sigma_multiplier *= 1.20f;
        print_probe("sigma multiplier +20%", &config, &input, baseline_a);

        config = baseline_config;
        config.discharge_surface_temp_max_c *= 0.90f;
        print_probe("surface Tmax -10%", &config, &input, baseline_a);

        config = baseline_config;
        config.default_resistance_soh_upper *= 1.20f;
        print_probe("R0 prior +20%", &config, &input, baseline_a);
    }
    return 0;
}
