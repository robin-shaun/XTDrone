/*
 * stepvel_types.h
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

#ifndef RTW_HEADER_stepvel_types_h_
#define RTW_HEADER_stepvel_types_h_
#include "rtwtypes.h"
#include "multiword_types.h"
#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_stepvel_geometry_msgs_Vector3_
#define DEFINED_TYPEDEF_FOR_SL_Bus_stepvel_geometry_msgs_Vector3_

typedef struct {
  real_T X;
  real_T Y;
  real_T Z;
} SL_Bus_stepvel_geometry_msgs_Vector3;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_stepvel_geometry_msgs_Twist_
#define DEFINED_TYPEDEF_FOR_SL_Bus_stepvel_geometry_msgs_Twist_

typedef struct {
  SL_Bus_stepvel_geometry_msgs_Vector3 Linear;
  SL_Bus_stepvel_geometry_msgs_Vector3 Angular;
} SL_Bus_stepvel_geometry_msgs_Twist;

#endif

#ifndef typedef_robotics_slcore_internal_bloc_T
#define typedef_robotics_slcore_internal_bloc_T

typedef struct {
  int32_T __dummy;
} robotics_slcore_internal_bloc_T;

#endif                                 /*typedef_robotics_slcore_internal_bloc_T*/

#ifndef typedef_robotics_slros_internal_block_T
#define typedef_robotics_slros_internal_block_T

typedef struct {
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
  real_T ticksUntilNextHit;
  robotics_slcore_internal_bloc_T SampleTimeHandler;
} robotics_slros_internal_block_T;

#endif                                 /*typedef_robotics_slros_internal_block_T*/

#ifndef typedef_robotics_slros_internal_blo_d_T
#define typedef_robotics_slros_internal_blo_d_T

typedef struct {
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
} robotics_slros_internal_blo_d_T;

#endif                                 /*typedef_robotics_slros_internal_blo_d_T*/

#ifndef typedef_struct_T_stepvel_T
#define typedef_struct_T_stepvel_T

typedef struct {
  char_T Value[4];
} struct_T_stepvel_T;

#endif                                 /*typedef_struct_T_stepvel_T*/

#ifndef typedef_struct_T_stepvel_d_T
#define typedef_struct_T_stepvel_d_T

typedef struct {
  char_T Value[9];
} struct_T_stepvel_d_T;

#endif                                 /*typedef_struct_T_stepvel_d_T*/

#ifndef struct_tag_smnSVBYMKVOFO5RozNZjEpF
#define struct_tag_smnSVBYMKVOFO5RozNZjEpF

struct tag_smnSVBYMKVOFO5RozNZjEpF
{
  char_T Disallow[9];
  char_T Type[9];
};

#endif                                 /*struct_tag_smnSVBYMKVOFO5RozNZjEpF*/

#ifndef typedef_smnSVBYMKVOFO5RozNZjEpF_stepv_T
#define typedef_smnSVBYMKVOFO5RozNZjEpF_stepv_T

typedef struct tag_smnSVBYMKVOFO5RozNZjEpF smnSVBYMKVOFO5RozNZjEpF_stepv_T;

#endif                                 /*typedef_smnSVBYMKVOFO5RozNZjEpF_stepv_T*/

/* Parameters (default storage) */
typedef struct P_stepvel_T_ P_stepvel_T;

/* Forward declaration for rtModel */
typedef struct tag_RTM_stepvel_T RT_MODEL_stepvel_T;

#endif                                 /* RTW_HEADER_stepvel_types_h_ */
