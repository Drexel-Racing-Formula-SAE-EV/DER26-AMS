#ifndef PLANT_MODEL_BINDING_H_
#define PLANT_MODEL_BINDING_H_

/*
 * This is the only file that knows the generated Simulink identifiers.
 * hil.generate_code rewrites it for each generated model.
 */
#include "drev_75s6p_p42a_accumulator_plant_v3_ams_outputs_validated.h"

typedef RT_MODEL_drev_75s6p_p42a_accu_T plant_generated_model_t;
typedef DW_drev_75s6p_p42a_accumulato_T plant_generated_dwork_t;

#define PLANT_GENERATED_INITIALIZE \
    drev_75s6p_p42a_accumulator_plant_v3_ams_outputs_validated_initialize
#define PLANT_GENERATED_STEP \
    drev_75s6p_p42a_accumulator_plant_v3_ams_outputs_validated_step
#define PLANT_GENERATED_TERMINATE \
    drev_75s6p_p42a_accumulator_plant_v3_ams_outputs_validated_terminate

#define PLANT_DWORK_SOC(dw) ((dw).SoC_Integrator_DSTATE)
#define PLANT_DWORK_T_CORE(dw) ((dw).T_core_int_DSTATE)
#define PLANT_DWORK_T_SURF(dw) ((dw).T_surf_int_DSTATE)
#define PLANT_DWORK_VP1(dw) ((dw).Vp1_Integrator1_DSTATE)
#define PLANT_DWORK_VP2(dw) ((dw).Vp2_Integrator_DSTATE)

#endif /* PLANT_MODEL_BINDING_H_ */
