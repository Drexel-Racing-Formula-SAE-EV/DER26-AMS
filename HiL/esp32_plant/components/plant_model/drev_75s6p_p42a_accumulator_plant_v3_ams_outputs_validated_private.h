/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: drev_75s6p_p42a_accumulator_plant_v3_ams_outputs_validated_private.h
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

#ifndef drev_75s6p_p42a_accumulator_plant_v3_ams_outputs_validated_private_h_
#define drev_75s6p_p42a_accumulator_plant_v3_ams_outputs_validated_private_h_
#include "rtwtypes.h"
#include "drev_75s6p_p42a_accumulator_plant_v3_ams_outputs_validated_types.h"

extern const real32_T rtCP_pooled_2VS2GaLG7tXs[303];
extern const real32_T rtCP_pooled_LAE26UlapTkv[101];
extern const real32_T rtCP_pooled_aZWafQUW4ySs[3];
extern const real32_T rtCP_pooled_MJJiVyV6u4B7[36];
extern const real32_T rtCP_pooled_oSJNQ9HBXQTR[12];
extern const real32_T rtCP_pooled_3i7E1u0kL9f2[36];
extern const real32_T rtCP_pooled_x01aWAzB59fl[36];
extern const real32_T rtCP_pooled_wGkYb7XWb2i2[36];
extern const uint32_T rtCP_pooled_npkPQKZ7Jhkk[2];
extern const uint32_T rtCP_pooled_0bbcgCIiHXCR[2];

#define rtCP_OCV_LUT_tableData         rtCP_pooled_2VS2GaLG7tXs  /* Expression: OCV_2d
                                                                  * Referenced by: '<Root>/OCV_LUT'
                                                                  */
#define rtCP_OCV_LUT_bp01Data          rtCP_pooled_LAE26UlapTkv  /* Expression: soc_ocv_common
                                                                  * Referenced by: '<Root>/OCV_LUT'
                                                                  */
#define rtCP_OCV_LUT_bp02Data          rtCP_pooled_aZWafQUW4ySs  /* Expression: temp_bp_ocv
                                                                  * Referenced by: '<Root>/OCV_LUT'
                                                                  */
#define rtCP_R0_LUT_tableData          rtCP_pooled_MJJiVyV6u4B7  /* Expression: R0_2d_fix
                                                                  * Referenced by: '<Root>/R0_LUT'
                                                                  */
#define rtCP_R0_LUT_bp01Data           rtCP_pooled_oSJNQ9HBXQTR  /* Expression: soc_common
                                                                  * Referenced by: '<Root>/R0_LUT'
                                                                  */
#define rtCP_R0_LUT_bp02Data           rtCP_pooled_aZWafQUW4ySs  /* Expression: temp_bp
                                                                  * Referenced by: '<Root>/R0_LUT'
                                                                  */
#define rtCP_nDLookupTable_tableData   rtCP_pooled_3i7E1u0kL9f2  /* Expression: R1_2d
                                                                  * Referenced by: '<Root>/n-D Lookup Table'
                                                                  */
#define rtCP_nDLookupTable_bp01Data    rtCP_pooled_oSJNQ9HBXQTR  /* Expression: soc_common
                                                                  * Referenced by: '<Root>/n-D Lookup Table'
                                                                  */
#define rtCP_nDLookupTable_bp02Data    rtCP_pooled_aZWafQUW4ySs  /* Expression: temp_bp
                                                                  * Referenced by: '<Root>/n-D Lookup Table'
                                                                  */
#define rtCP_R1C1_LUT_tableData        rtCP_pooled_x01aWAzB59fl  /* Expression: neg_inv_R1C1_2d_fixed
                                                                  * Referenced by: '<Root>/R1C1_LUT'
                                                                  */
#define rtCP_R1C1_LUT_bp01Data         rtCP_pooled_oSJNQ9HBXQTR  /* Expression: soc_common
                                                                  * Referenced by: '<Root>/R1C1_LUT'
                                                                  */
#define rtCP_R1C1_LUT_bp02Data         rtCP_pooled_aZWafQUW4ySs  /* Expression: temp_bp
                                                                  * Referenced by: '<Root>/R1C1_LUT'
                                                                  */
#define rtCP_C1_LUT_tableData          rtCP_pooled_wGkYb7XWb2i2  /* Expression: inv_C1_2d_fixed
                                                                  * Referenced by: '<Root>/C1_LUT'
                                                                  */
#define rtCP_C1_LUT_bp01Data           rtCP_pooled_oSJNQ9HBXQTR  /* Expression: soc_common
                                                                  * Referenced by: '<Root>/C1_LUT'
                                                                  */
#define rtCP_C1_LUT_bp02Data           rtCP_pooled_aZWafQUW4ySs  /* Expression: temp_bp
                                                                  * Referenced by: '<Root>/C1_LUT'
                                                                  */
#define rtCP_OCV_LUT_maxIndex          rtCP_pooled_npkPQKZ7Jhkk  /* Computed Parameter: rtCP_OCV_LUT_maxIndex
                                                                  * Referenced by: '<Root>/OCV_LUT'
                                                                  */
#define rtCP_R0_LUT_maxIndex           rtCP_pooled_0bbcgCIiHXCR  /* Computed Parameter: rtCP_R0_LUT_maxIndex
                                                                  * Referenced by: '<Root>/R0_LUT'
                                                                  */
#define rtCP_nDLookupTable_maxIndex    rtCP_pooled_0bbcgCIiHXCR  /* Computed Parameter: rtCP_nDLookupTable_maxIndex
                                                                  * Referenced by: '<Root>/n-D Lookup Table'
                                                                  */
#define rtCP_R1C1_LUT_maxIndex         rtCP_pooled_0bbcgCIiHXCR  /* Computed Parameter: rtCP_R1C1_LUT_maxIndex
                                                                  * Referenced by: '<Root>/R1C1_LUT'
                                                                  */
#define rtCP_C1_LUT_maxIndex           rtCP_pooled_0bbcgCIiHXCR  /* Computed Parameter: rtCP_C1_LUT_maxIndex
                                                                  * Referenced by: '<Root>/C1_LUT'
                                                                  */
#endif
     /* drev_75s6p_p42a_accumulator_plant_v3_ams_outputs_validated_private_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
