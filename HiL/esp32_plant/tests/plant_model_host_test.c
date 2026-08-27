#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "plant_model_adapter.h"

#define CHECK(condition)                                                      \
    do                                                                        \
    {                                                                         \
        if (!(condition))                                                     \
        {                                                                     \
            fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
            return EXIT_FAILURE;                                              \
        }                                                                     \
    } while (0)

static float sum_values(const float *values, size_t count)
{
    float sum = 0.0f;
    for (size_t index = 0; index < count; index++)
    {
        sum += values[index];
    }
    return sum;
}

static bool outputs_are_finite(const plant_output_t *output)
{
    if (!isfinite(output->V_pack) || !isfinite(output->T_core) ||
        !isfinite(output->T_surf) || !isfinite(output->SoC_true) ||
        !isfinite(output->V_min) || !isfinite(output->V_max) ||
        !isfinite(output->T_max) || !isfinite(output->T_avg))
    {
        return false;
    }
    for (size_t index = 0; index < PLANT_NUM_GROUPS; index++)
    {
        if (!isfinite(output->V_group[index]) ||
            !isfinite(output->SoC_group[index]))
        {
            return false;
        }
    }
    for (size_t index = 0; index < PLANT_NUM_THERMISTORS; index++)
    {
        if (!isfinite(output->T_sensor[index]))
        {
            return false;
        }
    }
    return true;
}

int main(void)
{
    plant_output_t rest;
    plant_output_t pulse;
    plant_state_t state;

    CHECK(plant_init(NULL));
    plant_terminate();
    CHECK(!plant_reset(-0.01f, 25.0f));
    CHECK(!plant_reset(1.01f, 25.0f));
    CHECK(!plant_reset(0.50f, NAN));
    CHECK(plant_reset(1.0f, 25.0f));
    CHECK(plant_step(0.0f, 25.0f));
    CHECK(plant_get_outputs(&rest));
    CHECK(outputs_are_finite(&rest));
    CHECK(fabsf(sum_values(rest.V_group, PLANT_NUM_GROUPS) - rest.V_pack) <
          2.0e-3f);
    CHECK(fabsf(sum_values(rest.V_segment, PLANT_NUM_SEGMENTS) - rest.V_pack) <
          2.0e-3f);
    CHECK(rest.SoC_true == 1.0f);
    CHECK(rest.V_min <= rest.V_max);
    CHECK(rest.T_core == 25.0f);
    CHECK(rest.T_surf == 25.0f);

    CHECK(plant_reset(0.80f, 25.0f));
    CHECK(plant_step(0.0f, 25.0f));
    CHECK(plant_get_outputs(&rest));
    CHECK(plant_reset(0.80f, 25.0f));
    CHECK(plant_step(100.0f, 25.0f));
    CHECK(plant_get_outputs(&pulse));
    CHECK(pulse.V_pack < rest.V_pack);
    CHECK(plant_step(100.0f, 25.0f));
    CHECK(plant_get_outputs(&pulse));
    CHECK(plant_get_state(&state));
    CHECK(state.soc < 0.80f);
    CHECK(state.fast_polarization_V > 0.0f);
    CHECK(state.slow_polarization_V > 0.0f);

    CHECK(plant_reset(0.50f, 10.0f));
    CHECK(plant_get_state(&state));
    CHECK(fabsf(state.soc - 0.50f) < 1.0e-7f);
    CHECK(fabsf(state.core_temperature_C - 10.0f) < 1.0e-7f);
    CHECK(fabsf(state.surface_temperature_C - 10.0f) < 1.0e-7f);
    CHECK(!plant_step(NAN, 25.0f));
    CHECK(!plant_step(0.0f, NAN));
    CHECK(!plant_get_outputs(NULL));

    plant_terminate();
    puts("PASS plant_model_host_test");
    return EXIT_SUCCESS;
}
