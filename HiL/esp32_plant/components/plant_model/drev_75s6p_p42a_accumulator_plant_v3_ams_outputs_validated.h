/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: drev_75s6p_p42a_accumulator_plant_v3_ams_outputs_validated.h
 *
 * Code generated for Simulink model 'drev_75s6p_p42a_accumulator_plant_v3_ams_outputs_validated'.
 *
 * Model version                  : 1.67
 * Simulink Coder version         : 25.2 (R2025b) 28-Jul-2025
 * C/C++ source code generated on : Sat Jun 27 23:54:03 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Custom Processor->Custom Processor
 * Emulation hardware selection:
 *    Differs from embedded hardware (Custom Processor->MATLAB Host Computer)
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. ROM efficiency
 * Validation result: All passed
 */

#ifndef drev_75s6p_p42a_accumulator_plant_v3_ams_outputs_validated_h_
#define drev_75s6p_p42a_accumulator_plant_v3_ams_outputs_validated_h_
#ifndef drev_75s6p_p42a_accumulator_plant_v3_ams_outputs_validated_COMMON_INCLUDES_
#define drev_75s6p_p42a_accumulator_plant_v3_ams_outputs_validated_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif
/* drev_75s6p_p42a_accumulator_plant_v3_ams_outputs_validated_COMMON_INCLUDES_ */

#include "drev_75s6p_p42a_accumulator_plant_v3_ams_outputs_validated_types.h"
#include "rt_defines.h"

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  real32_T SoC_Integrator_DSTATE;      /* '<Root>/SoC_Integrator' */
  real32_T T_core_int_DSTATE;          /* '<S2>/T_core_int' */
  real32_T Vp1_Integrator1_DSTATE;     /* '<Root>/Vp1_Integrator1' */
  real32_T Vp2_Integrator_DSTATE;      /* '<Root>/Vp2_Integrator' */
  real32_T T_surf_int_DSTATE;          /* '<S2>/T_surf_int' */
  uint32_T m_bpIndex[2];               /* '<Root>/OCV_LUT' */
} DW_drev_75s6p_p42a_accumulato_T;

/* Real-time Model Data Structure */
struct tag_RTM_drev_75s6p_p42a_accum_T {
  DW_drev_75s6p_p42a_accumulato_T *dwork;
};

/* Model entry point functions */
extern void
  drev_75s6p_p42a_accumulator_plant_v3_ams_outputs_validated_initialize
  (RT_MODEL_drev_75s6p_p42a_accu_T *const drev_75s6p_p42a_accumulator__M);
extern void drev_75s6p_p42a_accumulator_plant_v3_ams_outputs_validated_step
  (RT_MODEL_drev_75s6p_p42a_accu_T *const drev_75s6p_p42a_accumulator__M,
   real32_T drev_75s6p_p42a_accumulator_p_U_I_pack, real32_T
   drev_75s6p_p42a_accumulator_p_U_T_amb, real32_T
   *drev_75s6p_p42a_accumulator_p_Y_V_pack, real32_T
   *drev_75s6p_p42a_accumulator_p_Y_T_core, real32_T
   *drev_75s6p_p42a_accumulator_p_Y_T_surf, real32_T
   *drev_75s6p_p42a_accumulator_p_Y_SoC_true, real32_T
   drev_75s6p_p42a_accumulator_p_Y_V_group[75], real32_T
   drev_75s6p_p42a_accumulator_p_Y_V_segment[5], real32_T
   drev_75s6p_p42a_accumulator_p_Y_T_sensor[120], real32_T
   drev_75s6p_p42a_accumulator_p_Y_SoC_group[75], real32_T
   *drev_75s6p_p42a_accumulator_p_Y_V_min, real32_T
   *drev_75s6p_p42a_accumulator_p_Y_V_max, real32_T
   *drev_75s6p_p42a_accumulator_p_Y_T_max, real32_T
   *drev_75s6p_p42a_accumulator_p_Y_T_avg);
extern void drev_75s6p_p42a_accumulator_plant_v3_ams_outputs_validated_terminate
  (RT_MODEL_drev_75s6p_p42a_accu_T *const drev_75s6p_p42a_accumulator__M);

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'drev_75s6p_p42a_accumulator_plant_v3_ams_outputs_validated'
 * '<S1>'   : 'drev_75s6p_p42a_accumulator_plant_v3_ams_outputs_validated/Accumulator_Output_Expansion'
 * '<S2>'   : 'drev_75s6p_p42a_accumulator_plant_v3_ams_outputs_validated/Thermal_2Node'
 */
#endif       /* drev_75s6p_p42a_accumulator_plant_v3_ams_outputs_validated_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
