/*
 * stepvel.cpp
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "stepvel".
 *
 * Model version              : 1.33
 * Simulink Coder version : 9.0 (R2018b) 24-May-2018
 * C++ source code generated on : Tue May 28 18:30:11 2019
 *
 * Target selection: ert.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "stepvel.h"
#include "stepvel_private.h"

/* Block states (default storage) */
DW_stepvel_T stepvel_DW;

/* Real-time model */
RT_MODEL_stepvel_T stepvel_M_;
RT_MODEL_stepvel_T *const stepvel_M = &stepvel_M_;

/* Forward declaration for local functions */
static void matlabCodegenHandle_matlabCodeg(robotics_slros_internal_block_T *obj);
static void matlabCodegenHandle_matlabCo_dc(robotics_slros_internal_blo_d_T *obj);
static void matlabCodegenHandle_matlabCodeg(robotics_slros_internal_block_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void matlabCodegenHandle_matlabCo_dc(robotics_slros_internal_blo_d_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

/* Model step function */
void stepvel_step(void)
{
  real_T value;
  real_T value_0;
  SL_Bus_stepvel_geometry_msgs_Twist rtb_BusAssignment;

  /* MATLABSystem: '<Root>/Connstant Velocity' */
  ParamGet_stepvel_93.get_parameter(&value);

  /* MATLABSystem: '<Root>/Steering angle' */
  ParamGet_stepvel_92.get_parameter(&value_0);

  /* BusAssignment: '<Root>/Bus Assignment' incorporates:
   *  Constant: '<S1>/Constant'
   *  MATLABSystem: '<Root>/Connstant Velocity'
   *  MATLABSystem: '<Root>/Steering angle'
   */
  rtb_BusAssignment = stepvel_P.Constant_Value;
  rtb_BusAssignment.Linear.X = value;
  rtb_BusAssignment.Angular.Z = value_0;

  /* Outputs for Atomic SubSystem: '<Root>/Publish for Leader Vel' */
  /* MATLABSystem: '<S2>/SinkBlock' */
  Pub_stepvel_27.publish(&rtb_BusAssignment);

  /* End of Outputs for SubSystem: '<Root>/Publish for Leader Vel' */
}

/* Model initialize function */
void stepvel_initialize(void)
{
  /* Registration code */

  /* initialize error status */
  rtmSetErrorStatus(stepvel_M, (NULL));

  /* states (dwork) */
  (void) memset((void *)&stepvel_DW, 0,
                sizeof(DW_stepvel_T));

  {
    static const char_T tmp[7] = { 'c', 'm', 'd', '_', 'v', 'e', 'l' };

    static const char_T tmp_0[8] = { 's', 't', 'r', 'A', 'n', 'g', 'l', 'e' };

    static const char_T tmp_1[8] = { 'c', 'o', 'n', 's', 't', 'V', 'e', 'l' };

    char_T tmp_2[8];
    char_T tmp_3[9];
    int32_T i;

    /* Start for MATLABSystem: '<Root>/Connstant Velocity' */
    stepvel_DW.obj.matlabCodegenIsDeleted = true;
    stepvel_DW.obj.isInitialized = 0;
    stepvel_DW.obj.ticksUntilNextHit = 0.0;
    stepvel_DW.obj.matlabCodegenIsDeleted = false;
    stepvel_DW.objisempty_p = true;
    stepvel_DW.obj.isSetupComplete = false;
    stepvel_DW.obj.isInitialized = 1;
    for (i = 0; i < 8; i++) {
      tmp_3[i] = tmp_1[i];
    }

    tmp_3[8] = '\x00';
    ParamGet_stepvel_93.initialize(tmp_3);
    ParamGet_stepvel_93.initialize_error_codes(0, 1, 2, 3);
    ParamGet_stepvel_93.set_initial_value(3.0);
    stepvel_DW.obj.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<Root>/Connstant Velocity' */

    /* Start for MATLABSystem: '<Root>/Steering angle' */
    stepvel_DW.obj_n.matlabCodegenIsDeleted = true;
    stepvel_DW.obj_n.isInitialized = 0;
    stepvel_DW.obj_n.ticksUntilNextHit = 0.0;
    stepvel_DW.obj_n.matlabCodegenIsDeleted = false;
    stepvel_DW.objisempty_h = true;
    stepvel_DW.obj_n.isSetupComplete = false;
    stepvel_DW.obj_n.isInitialized = 1;
    for (i = 0; i < 8; i++) {
      tmp_3[i] = tmp_0[i];
    }

    tmp_3[8] = '\x00';
    ParamGet_stepvel_92.initialize(tmp_3);
    ParamGet_stepvel_92.initialize_error_codes(0, 1, 2, 3);
    ParamGet_stepvel_92.set_initial_value(0.0);
    stepvel_DW.obj_n.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<Root>/Steering angle' */

    /* Start for Atomic SubSystem: '<Root>/Publish for Leader Vel' */
    /* Start for MATLABSystem: '<S2>/SinkBlock' */
    stepvel_DW.obj_m.matlabCodegenIsDeleted = true;
    stepvel_DW.obj_m.isInitialized = 0;
    stepvel_DW.obj_m.matlabCodegenIsDeleted = false;
    stepvel_DW.objisempty = true;
    stepvel_DW.obj_m.isSetupComplete = false;
    stepvel_DW.obj_m.isInitialized = 1;
    for (i = 0; i < 7; i++) {
      tmp_2[i] = tmp[i];
    }

    tmp_2[7] = '\x00';
    Pub_stepvel_27.createPublisher(tmp_2, 1);
    stepvel_DW.obj_m.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S2>/SinkBlock' */
    /* End of Start for SubSystem: '<Root>/Publish for Leader Vel' */
  }
}

/* Model terminate function */
void stepvel_terminate(void)
{
  /* Terminate for MATLABSystem: '<Root>/Connstant Velocity' */
  matlabCodegenHandle_matlabCodeg(&stepvel_DW.obj);

  /* Terminate for MATLABSystem: '<Root>/Steering angle' */
  matlabCodegenHandle_matlabCodeg(&stepvel_DW.obj_n);

  /* Terminate for Atomic SubSystem: '<Root>/Publish for Leader Vel' */
  /* Terminate for MATLABSystem: '<S2>/SinkBlock' */
  matlabCodegenHandle_matlabCo_dc(&stepvel_DW.obj_m);

  /* End of Terminate for SubSystem: '<Root>/Publish for Leader Vel' */
}
