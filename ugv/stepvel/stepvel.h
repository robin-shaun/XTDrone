/*
 * stepvel.h
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

#ifndef RTW_HEADER_stepvel_h_
#define RTW_HEADER_stepvel_h_
#include <stddef.h>
#include <string.h>
#ifndef stepvel_COMMON_INCLUDES_
# define stepvel_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "slros_initialize.h"
#endif                                 /* stepvel_COMMON_INCLUDES_ */

#include "stepvel_types.h"

/* Shared type includes */
#include "multiword_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

/* Block states (default storage) for system '<Root>' */
typedef struct {
  robotics_slros_internal_block_T obj; /* '<Root>/Connstant Velocity' */
  robotics_slros_internal_block_T obj_n;/* '<Root>/Steering angle' */
  robotics_slros_internal_blo_d_T obj_m;/* '<S2>/SinkBlock' */
  boolean_T objisempty;                /* '<S2>/SinkBlock' */
  boolean_T objisempty_p;              /* '<Root>/Connstant Velocity' */
  boolean_T objisempty_h;              /* '<Root>/Steering angle' */
} DW_stepvel_T;

/* Parameters (default storage) */
struct P_stepvel_T_ {
  SL_Bus_stepvel_geometry_msgs_Twist Constant_Value;/* Computed Parameter: Constant_Value
                                                     * Referenced by: '<S1>/Constant'
                                                     */
};

/* Real-time Model Data Structure */
struct tag_RTM_stepvel_T {
  const char_T *errorStatus;
};

/* Block parameters (default storage) */
#ifdef __cplusplus

extern "C" {

#endif

  extern P_stepvel_T stepvel_P;

#ifdef __cplusplus

}
#endif

/* Block states (default storage) */
extern DW_stepvel_T stepvel_DW;

#ifdef __cplusplus

extern "C" {

#endif

  /* Model entry point functions */
  extern void stepvel_initialize(void);
  extern void stepvel_step(void);
  extern void stepvel_terminate(void);

#ifdef __cplusplus

}
#endif

/* Real-time Model object */
#ifdef __cplusplus

extern "C" {

#endif

  extern RT_MODEL_stepvel_T *const stepvel_M;

#ifdef __cplusplus

}
#endif

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
 * '<Root>' : 'stepvel'
 * '<S1>'   : 'stepvel/Blank Message Leader'
 * '<S2>'   : 'stepvel/Publish for Leader Vel'
 */
#endif                                 /* RTW_HEADER_stepvel_h_ */
