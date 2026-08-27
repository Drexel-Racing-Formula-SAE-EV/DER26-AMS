/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: drev_75s6p_p42a_accumulator_plant_v3_ams_outputs_validated.c
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

#include "drev_75s6p_p42a_accumulator_plant_v3_ams_outputs_validated.h"
#include <math.h>
#include <string.h>
#include "rtwtypes.h"
#include "look2_iflf_pbinlc.h"
#include "look2_iflf_binlc.h"
#include "drev_75s6p_p42a_accumulator_plant_v3_ams_outputs_validated_private.h"

/* Model step function */
void drev_75s6p_p42a_accumulator_plant_v3_ams_outputs_validated_step
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
   *drev_75s6p_p42a_accumulator_p_Y_T_avg)
{
  DW_drev_75s6p_p42a_accumulato_T *drev_75s6p_p42a_accumulator__DW =
    drev_75s6p_p42a_accumulator__M->dwork;
  int32_T base;
  int32_T i;
  int32_T j;
  real32_T rtb_T_sensor[120];
  real32_T rtb_SoC_group[75];
  real32_T rtb_V_group[75];
  real32_T rtb_V_segment[5];
  real32_T rtb_C1_LUT;
  real32_T rtb_I_pack_to_cell;
  real32_T rtb_R0_LUT;
  real32_T rtb_SoC_Limit;
  real32_T rtb_T_sensor_0;
  real32_T rtb_V_cell_to_pack;
  real32_T rtb_V_group_0;
  real32_T rtb_inv_Rcs;
  real32_T sum_t;
  real32_T sum_v;

  /* Saturate: '<Root>/SoC_Limit' incorporates:
   *  DiscreteIntegrator: '<Root>/SoC_Integrator'
   */
  if (drev_75s6p_p42a_accumulator__DW->SoC_Integrator_DSTATE > 1.0F) {
    rtb_SoC_Limit = 1.0F;
  } else if (drev_75s6p_p42a_accumulator__DW->SoC_Integrator_DSTATE < 0.0F) {
    rtb_SoC_Limit = 0.0F;
  } else {
    rtb_SoC_Limit = drev_75s6p_p42a_accumulator__DW->SoC_Integrator_DSTATE;
  }

  /* End of Saturate: '<Root>/SoC_Limit' */

  /* Saturate: '<Root>/ T_clamp' incorporates:
   *  DiscreteIntegrator: '<S2>/T_core_int'
   */
  if (drev_75s6p_p42a_accumulator__DW->T_core_int_DSTATE > 40.0F) {
    rtb_C1_LUT = 40.0F;
  } else if (drev_75s6p_p42a_accumulator__DW->T_core_int_DSTATE < 5.0F) {
    rtb_C1_LUT = 5.0F;
  } else {
    rtb_C1_LUT = drev_75s6p_p42a_accumulator__DW->T_core_int_DSTATE;
  }

  /* End of Saturate: '<Root>/ T_clamp' */

  /* Lookup_n-D: '<Root>/OCV_LUT' incorporates:
   *  Gain: '<S2>/inv_Rcs'
   *  Lookup_n-D: '<Root>/C1_LUT'
   *  Saturate: '<Root>/SoC_Limit'
   */
  rtb_inv_Rcs = look2_iflf_pbinlc(rtb_SoC_Limit, rtb_C1_LUT,
    rtCP_OCV_LUT_bp01Data, rtCP_OCV_LUT_bp02Data, rtCP_OCV_LUT_tableData,
    drev_75s6p_p42a_accumulator__DW->m_bpIndex, rtCP_OCV_LUT_maxIndex, 101U);

  /* Lookup_n-D: '<Root>/R0_LUT' incorporates:
   *  DiscreteIntegrator: '<Root>/SoC_Integrator'
   *  Lookup_n-D: '<Root>/C1_LUT'
   */
  rtb_R0_LUT = look2_iflf_binlc
    (drev_75s6p_p42a_accumulator__DW->SoC_Integrator_DSTATE, rtb_C1_LUT,
     rtCP_R0_LUT_bp01Data, rtCP_R0_LUT_bp02Data, rtCP_R0_LUT_tableData,
     rtCP_R0_LUT_maxIndex, 12U);

  /* Gain: '<Root>/I_pack_to_cell' incorporates:
   *  Inport: '<Root>/I_pack'
   */
  rtb_I_pack_to_cell = 0.166666672F * drev_75s6p_p42a_accumulator_p_U_I_pack;

  /* Gain: '<Root>/V_cell_to_pack' incorporates:
   *  DiscreteIntegrator: '<Root>/Vp1_Integrator1'
   *  DiscreteIntegrator: '<Root>/Vp2_Integrator'
   *  Gain: '<S2>/inv_Rcs'
   *  Lookup_n-D: '<Root>/R0_LUT'
   *  Product: '<Root>/Product2'
   *  Sum: '<Root>/Vcell_Calc'
   */
  rtb_V_cell_to_pack = (((rtb_inv_Rcs - rtb_R0_LUT * rtb_I_pack_to_cell) -
    drev_75s6p_p42a_accumulator__DW->Vp1_Integrator1_DSTATE) -
                        drev_75s6p_p42a_accumulator__DW->Vp2_Integrator_DSTATE) *
    75.0F;

  /* MATLAB Function: '<Root>/Accumulator_Output_Expansion' incorporates:
   *  DiscreteIntegrator: '<S2>/T_core_int'
   *  DiscreteIntegrator: '<S2>/T_surf_int'
   */
  rtb_inv_Rcs = rtb_V_cell_to_pack / 75.0F;
  sum_v = 0.0F;
  for (i = 0; i < 75; i++) {
    rtb_V_group_0 = ((real32_T)fmod(((real_T)i + 1.0) * 37.0, 75.0) / 74.0F -
                     0.5F) * 0.008F + rtb_inv_Rcs;
    rtb_V_group[i] = rtb_V_group_0;
    sum_v += rtb_V_group_0;
  }

  rtb_inv_Rcs = (rtb_V_cell_to_pack - sum_v) / 75.0F;
  for (i = 0; i < 75; i++) {
    rtb_V_group[i] += rtb_inv_Rcs;
  }

  for (i = 0; i < 5; i++) {
    rtb_inv_Rcs = 0.0F;
    base = i * 15;
    for (j = 0; j < 15; j++) {
      rtb_inv_Rcs += rtb_V_group[base + j];
    }

    rtb_V_segment[i] = rtb_inv_Rcs;
  }

  for (i = 0; i < 75; i++) {
    rtb_inv_Rcs = ((real32_T)fmod(((real_T)i + 1.0) * 29.0, 75.0) / 74.0F - 0.5F)
      * 0.01F + rtb_SoC_Limit;
    if (rtb_inv_Rcs > 1.0F) {
      rtb_inv_Rcs = 1.0F;
    } else if (rtb_inv_Rcs < 0.0F) {
      rtb_inv_Rcs = 0.0F;
    }

    rtb_SoC_group[i] = rtb_inv_Rcs;
  }

  for (i = 0; i < 120; i++) {
    if (fmod((real_T)i + 1.0, 6.0) == 0.0) {
      rtb_T_sensor[i] = ((real32_T)fmod(((real_T)i + 1.0) * 17.0, 41.0) - 20.0F)
        / 20.0F + (0.65F * drev_75s6p_p42a_accumulator__DW->T_core_int_DSTATE +
                   0.35F * drev_75s6p_p42a_accumulator__DW->T_surf_int_DSTATE);
    } else {
      rtb_T_sensor[i] = ((real32_T)fmod(((real_T)i + 1.0) * 17.0, 41.0) - 20.0F)
        / 20.0F + drev_75s6p_p42a_accumulator__DW->T_surf_int_DSTATE;
    }
  }

  rtb_inv_Rcs = rtb_V_group[0];
  sum_v = rtb_V_group[0];
  for (i = 0; i < 74; i++) {
    rtb_V_group_0 = rtb_V_group[i + 1];
    if (rtb_V_group_0 < rtb_inv_Rcs) {
      rtb_inv_Rcs = rtb_V_group_0;
    }

    if (rtb_V_group_0 > sum_v) {
      sum_v = rtb_V_group_0;
    }
  }

  rtb_V_group_0 = rtb_T_sensor[0];
  sum_t = 0.0F;
  for (i = 0; i < 120; i++) {
    rtb_T_sensor_0 = rtb_T_sensor[i];
    if (rtb_T_sensor_0 > rtb_V_group_0) {
      rtb_V_group_0 = rtb_T_sensor_0;
    }

    sum_t += rtb_T_sensor_0;

    /* Outport: '<Root>/T_sensor' */
    drev_75s6p_p42a_accumulator_p_Y_T_sensor[i] = rtb_T_sensor_0;
  }

  /* Outport: '<Root>/V_group' */
  memcpy(&drev_75s6p_p42a_accumulator_p_Y_V_group[0], &rtb_V_group[0], 75U *
         sizeof(real32_T));

  /* Outport: '<Root>/SoC_group' */
  memcpy(&drev_75s6p_p42a_accumulator_p_Y_SoC_group[0], &rtb_SoC_group[0], 75U *
         sizeof(real32_T));

  /* Outport: '<Root>/V_segment' */
  for (i = 0; i < 5; i++) {
    drev_75s6p_p42a_accumulator_p_Y_V_segment[i] = rtb_V_segment[i];
  }

  /* End of Outport: '<Root>/V_segment' */

  /* Outport: '<Root>/V_min' incorporates:
   *  MATLAB Function: '<Root>/Accumulator_Output_Expansion'
   */
  *drev_75s6p_p42a_accumulator_p_Y_V_min = rtb_inv_Rcs;

  /* Outport: '<Root>/V_max' incorporates:
   *  MATLAB Function: '<Root>/Accumulator_Output_Expansion'
   */
  *drev_75s6p_p42a_accumulator_p_Y_V_max = sum_v;

  /* Outport: '<Root>/T_max' incorporates:
   *  MATLAB Function: '<Root>/Accumulator_Output_Expansion'
   */
  *drev_75s6p_p42a_accumulator_p_Y_T_max = rtb_V_group_0;

  /* Outport: '<Root>/T_avg' incorporates:
   *  MATLAB Function: '<Root>/Accumulator_Output_Expansion'
   */
  *drev_75s6p_p42a_accumulator_p_Y_T_avg = sum_t / 120.0F;

  /* Outport: '<Root>/T_surf' incorporates:
   *  DiscreteIntegrator: '<S2>/T_surf_int'
   */
  *drev_75s6p_p42a_accumulator_p_Y_T_surf =
    drev_75s6p_p42a_accumulator__DW->T_surf_int_DSTATE;

  /* Gain: '<S2>/inv_Rcs' incorporates:
   *  DiscreteIntegrator: '<S2>/T_core_int'
   *  DiscreteIntegrator: '<S2>/T_surf_int'
   *  Sum: '<S2>/dT_cs'
   */
  rtb_inv_Rcs = (drev_75s6p_p42a_accumulator__DW->T_core_int_DSTATE -
                 drev_75s6p_p42a_accumulator__DW->T_surf_int_DSTATE) *
    0.666666687F;

  /* Product: '<S2>/Divide' incorporates:
   *  DiscreteIntegrator: '<Root>/SoC_Integrator'
   *  DiscreteIntegrator: '<Root>/Vp1_Integrator1'
   *  Lookup_n-D: '<Root>/C1_LUT'
   *  Lookup_n-D: '<Root>/n-D Lookup Table'
   *  Product: '<S2>/Vp1_sq'
   */
  sum_v = drev_75s6p_p42a_accumulator__DW->Vp1_Integrator1_DSTATE *
    drev_75s6p_p42a_accumulator__DW->Vp1_Integrator1_DSTATE / look2_iflf_binlc
    (drev_75s6p_p42a_accumulator__DW->SoC_Integrator_DSTATE, rtb_C1_LUT,
     rtCP_nDLookupTable_bp01Data, rtCP_nDLookupTable_bp02Data,
     rtCP_nDLookupTable_tableData, rtCP_nDLookupTable_maxIndex, 12U);

  /* Outport: '<Root>/V_pack' */
  *drev_75s6p_p42a_accumulator_p_Y_V_pack = rtb_V_cell_to_pack;

  /* Product: '<Root>/Product1' incorporates:
   *  DiscreteIntegrator: '<Root>/SoC_Integrator'
   *  DiscreteIntegrator: '<Root>/Vp1_Integrator1'
   *  Lookup_n-D: '<Root>/C1_LUT'
   *  Lookup_n-D: '<Root>/R1C1_LUT'
   */
  rtb_V_cell_to_pack = look2_iflf_binlc
    (drev_75s6p_p42a_accumulator__DW->SoC_Integrator_DSTATE, rtb_C1_LUT,
     rtCP_R1C1_LUT_bp01Data, rtCP_R1C1_LUT_bp02Data, rtCP_R1C1_LUT_tableData,
     rtCP_R1C1_LUT_maxIndex, 12U) *
    drev_75s6p_p42a_accumulator__DW->Vp1_Integrator1_DSTATE;

  /* Product: '<Root>/Product' incorporates:
   *  DiscreteIntegrator: '<Root>/SoC_Integrator'
   *  Lookup_n-D: '<Root>/C1_LUT'
   */
  rtb_C1_LUT = look2_iflf_binlc
    (drev_75s6p_p42a_accumulator__DW->SoC_Integrator_DSTATE, rtb_C1_LUT,
     rtCP_C1_LUT_bp01Data, rtCP_C1_LUT_bp02Data, rtCP_C1_LUT_tableData,
     rtCP_C1_LUT_maxIndex, 12U) * rtb_I_pack_to_cell;

  /* Outport: '<Root>/T_core' incorporates:
   *  DiscreteIntegrator: '<S2>/T_core_int'
   */
  *drev_75s6p_p42a_accumulator_p_Y_T_core =
    drev_75s6p_p42a_accumulator__DW->T_core_int_DSTATE;

  /* Outport: '<Root>/SoC_true' */
  *drev_75s6p_p42a_accumulator_p_Y_SoC_true = rtb_SoC_Limit;

  /* Update for DiscreteIntegrator: '<Root>/SoC_Integrator' incorporates:
   *  Gain: '<Root>/dSoC_per_step'
   */
  drev_75s6p_p42a_accumulator__DW->SoC_Integrator_DSTATE += -6.6137567E-5F *
    rtb_I_pack_to_cell * 0.1F;

  /* Update for DiscreteIntegrator: '<S2>/T_core_int' incorporates:
   *  DiscreteIntegrator: '<Root>/Vp2_Integrator'
   *  Gain: '<S2>/Gain'
   *  Gain: '<S2>/inv_Cc'
   *  Lookup_n-D: '<Root>/R0_LUT'
   *  Product: '<S2>/I_squared'
   *  Product: '<S2>/Vp2_sq'
   *  Product: '<S2>/q_gen_calc'
   *  Sum: '<S2>/Core_Heat_Balance'
   *  Sum: '<S2>/Sum'
   */
  drev_75s6p_p42a_accumulator__DW->T_core_int_DSTATE +=
    (((drev_75s6p_p42a_accumulator__DW->Vp2_Integrator_DSTATE *
       drev_75s6p_p42a_accumulator__DW->Vp2_Integrator_DSTATE * 249.999985F +
       sum_v) + rtb_I_pack_to_cell * rtb_I_pack_to_cell * rtb_R0_LUT) -
     rtb_inv_Rcs) * 0.0181818176F * 0.1F;

  /* Update for DiscreteIntegrator: '<Root>/Vp1_Integrator1' incorporates:
   *  Sum: '<Root>/Vp1_dot'
   */
  drev_75s6p_p42a_accumulator__DW->Vp1_Integrator1_DSTATE += (rtb_C1_LUT +
    rtb_V_cell_to_pack) * 0.1F;

  /* Update for DiscreteIntegrator: '<Root>/Vp2_Integrator' incorporates:
   *  Gain: '<Root>/I_to_Vp2'
   *  Gain: '<Root>/Vp2_feedback'
   *  Sum: '<Root>/Vp2_dot'
   */
  drev_75s6p_p42a_accumulator__DW->Vp2_Integrator_DSTATE += (8.33333324E-5F *
    rtb_I_pack_to_cell + -0.0208333321F *
    drev_75s6p_p42a_accumulator__DW->Vp2_Integrator_DSTATE) * 0.1F;

  /* Update for DiscreteIntegrator: '<S2>/T_surf_int' incorporates:
   *  Gain: '<S2>/inv_Cs'
   *  Gain: '<S2>/inv_Rsa'
   *  Inport: '<Root>/T_amb'
   *  Sum: '<S2>/Surf_Heat_Balance'
   *  Sum: '<S2>/dT_sa'
   */
  drev_75s6p_p42a_accumulator__DW->T_surf_int_DSTATE += (rtb_inv_Rcs -
    (drev_75s6p_p42a_accumulator__DW->T_surf_int_DSTATE -
     drev_75s6p_p42a_accumulator_p_U_T_amb) * 0.125F) * 0.0666666701F * 0.1F;
}

/* Model initialize function */
void drev_75s6p_p42a_accumulator_plant_v3_ams_outputs_validated_initialize
  (RT_MODEL_drev_75s6p_p42a_accu_T *const drev_75s6p_p42a_accumulator__M)
{
  DW_drev_75s6p_p42a_accumulato_T *drev_75s6p_p42a_accumulator__DW =
    drev_75s6p_p42a_accumulator__M->dwork;

  /* InitializeConditions for DiscreteIntegrator: '<Root>/SoC_Integrator' */
  drev_75s6p_p42a_accumulator__DW->SoC_Integrator_DSTATE = 1.0F;

  /* InitializeConditions for DiscreteIntegrator: '<S2>/T_core_int' */
  drev_75s6p_p42a_accumulator__DW->T_core_int_DSTATE = 25.0F;

  /* InitializeConditions for DiscreteIntegrator: '<S2>/T_surf_int' */
  drev_75s6p_p42a_accumulator__DW->T_surf_int_DSTATE = 25.0F;
}

/* Model terminate function */
void drev_75s6p_p42a_accumulator_plant_v3_ams_outputs_validated_terminate
  (RT_MODEL_drev_75s6p_p42a_accu_T *const drev_75s6p_p42a_accumulator__M)
{
  /* (no terminate code required) */
  UNUSED_PARAMETER(drev_75s6p_p42a_accumulator__M);
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
