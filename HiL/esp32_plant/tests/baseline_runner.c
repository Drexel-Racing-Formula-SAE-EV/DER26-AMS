#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "drive_profiles.h"
#include "plant_model_adapter.h"

typedef float (*current_function_t)(uint32_t step);

static float current_zero(uint32_t step)
{
    (void)step;
    return 0.0f;
}

static float current_pulse(uint32_t step)
{
    if (step < 50U)
    {
        return 0.0f;
    }
    if (step < 150U)
    {
        return 100.0f;
    }
    return 0.0f;
}

static float current_hppc(uint32_t step)
{
    const uint32_t cycle = step % 600U;
    if (cycle < 100U) return 0.0f;
    if (cycle < 110U) return 100.0f;
    if (cycle < 150U) return 0.0f;
    if (cycle < 160U) return -40.0f;
    if (cycle < 260U) return 60.0f;
    if (cycle < 310U) return 0.0f;
    if (cycle < 360U) return 120.0f;
    if (cycle < 400U) return 0.0f;
    if (cycle < 440U) return -60.0f;
    return 0.0f;
}

static float current_us06(uint32_t step)
{
    return ((float)us06_25_i_10ma[step % US06_25_LEN]) * 0.01f;
}

static float sum_values(const float *values, size_t count)
{
    float sum = 0.0f;
    for (size_t index = 0; index < count; index++)
    {
        sum += values[index];
    }
    return sum;
}

static int run_scenario(const char *name,
                        uint32_t steps,
                        uint32_t decimation,
                        float initial_soc,
                        float initial_temperature_C,
                        float ambient_temperature_C,
                        current_function_t current_function)
{
    plant_output_t output;
    plant_state_t state;
    if (!plant_reset(initial_soc, initial_temperature_C))
    {
        return EXIT_FAILURE;
    }

    for (uint32_t step = 0U; step < steps; step++)
    {
        const float current_A = current_function(step);
        if (!plant_step(current_A, ambient_temperature_C) ||
            !plant_get_outputs(&output) ||
            !plant_get_state(&state))
        {
            return EXIT_FAILURE;
        }

        if ((step % decimation) == 0U || step == (steps - 1U))
        {
            const float group_error =
                sum_values(output.V_group, PLANT_NUM_GROUPS) - output.V_pack;
            const float segment_error =
                sum_values(output.V_segment, PLANT_NUM_SEGMENTS) - output.V_pack;
            printf(
                "%s,%u,%.6f,%.6f,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,"
                "%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n",
                name,
                step,
                (double)step * PLANT_MODEL_SAMPLE_TIME_S,
                (double)current_A,
                (double)output.V_pack,
                (double)output.SoC_true,
                (double)output.T_core,
                (double)output.T_surf,
                (double)output.V_min,
                (double)output.V_max,
                (double)output.T_max,
                (double)output.T_avg,
                (double)state.fast_polarization_V,
                (double)state.slow_polarization_V,
                (double)group_error,
                (double)segment_error,
                (double)ambient_temperature_C);
        }
    }
    return EXIT_SUCCESS;
}

int main(void)
{
    puts(
        "scenario,step,time_s,I_pack_A,V_pack_V,SoC_true,T_core_C,T_surf_C,"
        "V_min_V,V_max_V,T_max_C,T_avg_C,Vp1_V,Vp2_V,"
        "group_sum_error_V,segment_sum_error_V,T_ambient_C");

    if (run_scenario("zero_current", 101U, 10U, 1.0f, 25.0f, 25.0f,
                     current_zero) != EXIT_SUCCESS ||
        run_scenario("pulse_100A", 301U, 10U, 0.80f, 25.0f, 25.0f,
                     current_pulse) != EXIT_SUCCESS ||
        run_scenario("synthetic_hppc", 1201U, 10U, 1.0f, 25.0f, 25.0f,
                     current_hppc) != EXIT_SUCCESS ||
        run_scenario("us06_25C", US06_25_LEN, 50U, 1.0f, 25.0f, 25.0f,
                     current_us06) != EXIT_SUCCESS ||
        run_scenario("hot_init", 101U, 10U, 0.80f, 40.0f, 40.0f,
                     current_zero) != EXIT_SUCCESS ||
        run_scenario("cold_init", 101U, 10U, 0.80f, 5.0f, 5.0f,
                     current_zero) != EXIT_SUCCESS)
    {
        plant_terminate();
        return EXIT_FAILURE;
    }

    plant_terminate();
    return EXIT_SUCCESS;
}
