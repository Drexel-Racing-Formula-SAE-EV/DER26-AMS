#include "plant_model_adapter.h"

#include <math.h>
#include <string.h>

#include "plant_model_binding.h"

static plant_generated_model_t s_model;
static plant_generated_dwork_t s_dwork;
static plant_output_t s_last_output;
static bool s_initialized = false;

static bool valid_initial_conditions(float soc, float temperature_C)
{
    return isfinite(soc) && isfinite(temperature_C) &&
           (soc >= 0.0f) && (soc <= 1.0f);
}

static bool output_is_finite(const plant_output_t *output)
{
    if ((output == NULL) ||
        !isfinite(output->V_pack) ||
        !isfinite(output->T_core) ||
        !isfinite(output->T_surf) ||
        !isfinite(output->SoC_true) ||
        !isfinite(output->V_min) ||
        !isfinite(output->V_max) ||
        !isfinite(output->T_max) ||
        !isfinite(output->T_avg))
    {
        return false;
    }
    for (size_t index = 0U; index < PLANT_NUM_GROUPS; index++)
    {
        if (!isfinite(output->V_group[index]) ||
            !isfinite(output->SoC_group[index]))
        {
            return false;
        }
    }
    for (size_t index = 0U; index < PLANT_NUM_SEGMENTS; index++)
    {
        if (!isfinite(output->V_segment[index]))
        {
            return false;
        }
    }
    for (size_t index = 0U; index < PLANT_NUM_THERMISTORS; index++)
    {
        if (!isfinite(output->T_sensor[index]))
        {
            return false;
        }
    }
    return true;
}

bool plant_init(const plant_configuration_t *configuration)
{
    const float initial_soc =
        (configuration != NULL) ? configuration->initial_soc : 1.0f;
    const float initial_temperature_C =
        (configuration != NULL) ? configuration->initial_temperature_C : 25.0f;

    if (!valid_initial_conditions(initial_soc, initial_temperature_C))
    {
        return false;
    }

    if (s_initialized)
    {
        PLANT_GENERATED_TERMINATE(&s_model);
    }

    memset(&s_model, 0, sizeof(s_model));
    memset(&s_dwork, 0, sizeof(s_dwork));
    memset(&s_last_output, 0, sizeof(s_last_output));
    s_model.dwork = &s_dwork;
    PLANT_GENERATED_INITIALIZE(&s_model);

    PLANT_DWORK_SOC(s_dwork) = initial_soc;
    PLANT_DWORK_T_CORE(s_dwork) = initial_temperature_C;
    PLANT_DWORK_T_SURF(s_dwork) = initial_temperature_C;
    PLANT_DWORK_VP1(s_dwork) = 0.0f;
    PLANT_DWORK_VP2(s_dwork) = 0.0f;
    s_last_output.SoC_true = initial_soc;
    s_last_output.T_core = initial_temperature_C;
    s_last_output.T_surf = initial_temperature_C;
    s_last_output.T_max = initial_temperature_C;
    s_last_output.T_avg = initial_temperature_C;
    s_initialized = true;
    return true;
}

bool plant_reset(float initial_soc, float initial_temperature_C)
{
    const plant_configuration_t configuration = {
        .initial_soc = initial_soc,
        .initial_temperature_C = initial_temperature_C
    };
    return plant_init(&configuration);
}

bool plant_step(float pack_current_A, float ambient_temperature_C)
{
    if (!s_initialized ||
        !isfinite(pack_current_A) || !isfinite(ambient_temperature_C))
    {
        return false;
    }

    PLANT_GENERATED_STEP(
        &s_model,
        pack_current_A,
        ambient_temperature_C,
        &s_last_output.V_pack,
        &s_last_output.T_core,
        &s_last_output.T_surf,
        &s_last_output.SoC_true,
        s_last_output.V_group,
        s_last_output.V_segment,
        s_last_output.T_sensor,
        s_last_output.SoC_group,
        &s_last_output.V_min,
        &s_last_output.V_max,
        &s_last_output.T_max,
        &s_last_output.T_avg);

    return output_is_finite(&s_last_output);
}

bool plant_get_outputs(plant_output_t *output)
{
    if (!s_initialized || (output == NULL))
    {
        return false;
    }
    *output = s_last_output;
    return true;
}

bool plant_get_state(plant_state_t *state)
{
    if (!s_initialized || (state == NULL))
    {
        return false;
    }

    state->soc = PLANT_DWORK_SOC(s_dwork);
    state->core_temperature_C = PLANT_DWORK_T_CORE(s_dwork);
    state->surface_temperature_C = PLANT_DWORK_T_SURF(s_dwork);
    state->fast_polarization_V = PLANT_DWORK_VP1(s_dwork);
    state->slow_polarization_V = PLANT_DWORK_VP2(s_dwork);
    return true;
}

void plant_terminate(void)
{
    if (s_initialized)
    {
        PLANT_GENERATED_TERMINATE(&s_model);
        s_initialized = false;
    }
}
