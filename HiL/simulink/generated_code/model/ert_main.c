/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: ert_main.c
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

#include <stddef.h>
#include <stdio.h>            /* This example main program uses printf/fflush */
#include "drev_75s6p_p42a_accumulator_plant_v3_ams_outputs_validated.h" /* Model header file */

static RT_MODEL_drev_75s6p_p42a_accu_T drev_75s6p_p42a_accumulator__M_;
static RT_MODEL_drev_75s6p_p42a_accu_T *const drev_75s6p_p42a_accumulato_MPtr =
  &drev_75s6p_p42a_accumulator__M_;    /* Real-time model */
static DW_drev_75s6p_p42a_accumulato_T drev_75s6p_p42a_accumulator__DW;/* Observable states */

/* '<Root>/I_pack' */
static real32_T drev_75s6p_p42a_accumulator_p_U_I_pack;

/* '<Root>/T_amb' */
static real32_T drev_75s6p_p42a_accumulator_p_U_T_amb;

/* '<Root>/V_pack' */
static real32_T drev_75s6p_p42a_accumulator_p_Y_V_pack;

/* '<Root>/T_core' */
static real32_T drev_75s6p_p42a_accumulator_p_Y_T_core;

/* '<Root>/T_surf' */
static real32_T drev_75s6p_p42a_accumulator_p_Y_T_surf;

/* '<Root>/SoC_true' */
static real32_T drev_75s6p_p42a_accumulator_p_Y_SoC_true;

/* '<Root>/V_group' */
static real32_T drev_75s6p_p42a_accumulator_p_Y_V_group[75];

/* '<Root>/V_segment' */
static real32_T drev_75s6p_p42a_accumulator_p_Y_V_segment[5];

/* '<Root>/T_sensor' */
static real32_T drev_75s6p_p42a_accumulator_p_Y_T_sensor[120];

/* '<Root>/SoC_group' */
static real32_T drev_75s6p_p42a_accumulator_p_Y_SoC_group[75];

/* '<Root>/V_min' */
static real32_T drev_75s6p_p42a_accumulator_p_Y_V_min;

/* '<Root>/V_max' */
static real32_T drev_75s6p_p42a_accumulator_p_Y_V_max;

/* '<Root>/T_max' */
static real32_T drev_75s6p_p42a_accumulator_p_Y_T_max;

/* '<Root>/T_avg' */
static real32_T drev_75s6p_p42a_accumulator_p_Y_T_avg;

/*
 * Associating rt_OneStep with a real-time clock or interrupt service routine
 * is what makes the generated code "real-time".  The function rt_OneStep is
 * always associated with the base rate of the model.  Subrates are managed
 * by the base rate from inside the generated code.  Enabling/disabling
 * interrupts and floating point context switches are target specific.  This
 * example code indicates where these should take place relative to executing
 * the generated code step function.  Overrun behavior should be tailored to
 * your application needs.  This example simply sets an error status in the
 * real-time model and returns from rt_OneStep.
 */
void rt_OneStep(RT_MODEL_drev_75s6p_p42a_accu_T *const
                drev_75s6p_p42a_accumulator__M);
void rt_OneStep(RT_MODEL_drev_75s6p_p42a_accu_T *const
                drev_75s6p_p42a_accumulator__M)
{
  static boolean_T OverrunFlag = false;

  /* Disable interrupts here */

  /* Check for overrun */
  if (OverrunFlag) {
    return;
  }

  OverrunFlag = true;

  /* Save FPU context here (if necessary) */
  /* Re-enable timer or interrupt here */
  /* Set model inputs here */

  /* Step the model */
  drev_75s6p_p42a_accumulator_plant_v3_ams_outputs_validated_step
    (drev_75s6p_p42a_accumulator__M, drev_75s6p_p42a_accumulator_p_U_I_pack,
     drev_75s6p_p42a_accumulator_p_U_T_amb,
     &drev_75s6p_p42a_accumulator_p_Y_V_pack,
     &drev_75s6p_p42a_accumulator_p_Y_T_core,
     &drev_75s6p_p42a_accumulator_p_Y_T_surf,
     &drev_75s6p_p42a_accumulator_p_Y_SoC_true,
     drev_75s6p_p42a_accumulator_p_Y_V_group,
     drev_75s6p_p42a_accumulator_p_Y_V_segment,
     drev_75s6p_p42a_accumulator_p_Y_T_sensor,
     drev_75s6p_p42a_accumulator_p_Y_SoC_group,
     &drev_75s6p_p42a_accumulator_p_Y_V_min,
     &drev_75s6p_p42a_accumulator_p_Y_V_max,
     &drev_75s6p_p42a_accumulator_p_Y_T_max,
     &drev_75s6p_p42a_accumulator_p_Y_T_avg);

  /* Get model outputs here */

  /* Indicate task complete */
  OverrunFlag = false;

  /* Disable interrupts here */
  /* Restore FPU context here (if necessary) */
  /* Enable interrupts here */
}

/*
 * The example main function illustrates what is required by your
 * application code to initialize, execute, and terminate the generated code.
 * Attaching rt_OneStep to a real-time clock is target specific. This example
 * illustrates how you do this relative to initializing the model.
 */
int_T main(int_T argc, const char *argv[])
{
  RT_MODEL_drev_75s6p_p42a_accu_T *const drev_75s6p_p42a_accumulator__M =
    drev_75s6p_p42a_accumulato_MPtr;

  /* Unused arguments */
  (void)(argc);
  (void)(argv);

  /* Pack model data into RTM */
  drev_75s6p_p42a_accumulator__M->dwork = &drev_75s6p_p42a_accumulator__DW;

  /* Initialize model */
  drev_75s6p_p42a_accumulator_plant_v3_ams_outputs_validated_initialize
    (drev_75s6p_p42a_accumulator__M);

  /* Attach rt_OneStep to a timer or interrupt service routine with
   * period 0.1 seconds (base rate of the model) here.
   * The call syntax for rt_OneStep is
   *
   *  rt_OneStep(drev_75s6p_p42a_accumulator__M);
   */
  printf("Warning: The simulation will run forever. "
         "Generated ERT main won't simulate model step behavior. "
         "To change this behavior select the 'MAT-file logging' option.\n");
  fflush((NULL));
  while (1) {
    /*  Perform application tasks here */
  }

  /* The option 'Remove error status field in real-time model data structure'
   * is selected, therefore the following code does not need to execute.
   */

  /* Terminate model */
  drev_75s6p_p42a_accumulator_plant_v3_ams_outputs_validated_terminate
    (drev_75s6p_p42a_accumulator__M);
  return 0;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
