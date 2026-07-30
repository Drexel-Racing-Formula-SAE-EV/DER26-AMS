#ifndef PLANT_MODEL_ADAPTER_H_
#define PLANT_MODEL_ADAPTER_H_

#include <stdbool.h>
#include <stdint.h>

#include "plant_model_manifest.h"

typedef struct {
    float initial_soc;
    float initial_temperature_C;
} plant_configuration_t;

typedef struct {
    float V_pack;
    float T_core;
    float T_surf;
    float SoC_true;
    float V_group[PLANT_NUM_GROUPS];
    float V_segment[PLANT_NUM_SEGMENTS];
    float T_sensor[PLANT_NUM_THERMISTORS];
    float SoC_group[PLANT_NUM_GROUPS];
    float V_min;
    float V_max;
    float T_max;
    float T_avg;
} plant_output_t;

typedef struct {
    float soc;
    float core_temperature_C;
    float surface_temperature_C;
    float fast_polarization_V;
    float slow_polarization_V;
} plant_state_t;

bool plant_init(const plant_configuration_t *configuration);
bool plant_reset(float initial_soc, float initial_temperature_C);
bool plant_step(float pack_current_A, float ambient_temperature_C);
bool plant_get_outputs(plant_output_t *output);
bool plant_get_state(plant_state_t *state);
void plant_terminate(void);

#endif /* PLANT_MODEL_ADAPTER_H_ */
