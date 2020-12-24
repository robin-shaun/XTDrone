//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: obstacleStopper_types.h
//
// Code generated for Simulink model 'obstacleStopper'.
//
// Model version                  : 1.86
// Simulink Coder version         : 9.0 (R2018b) 24-May-2018
// C/C++ source code generated on : Fri May 24 15:32:42 2019
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_obstacleStopper_types_h_
#define RTW_HEADER_obstacleStopper_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_obstacleStopper_geometry_msgs_Vector3_
#define DEFINED_TYPEDEF_FOR_SL_Bus_obstacleStopper_geometry_msgs_Vector3_

// MsgType=geometry_msgs/Vector3
typedef struct {
  real_T X;
  real_T Y;
  real_T Z;
} SL_Bus_obstacleStopper_geometry_msgs_Vector3;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_obstacleStopper_geometry_msgs_Twist_
#define DEFINED_TYPEDEF_FOR_SL_Bus_obstacleStopper_geometry_msgs_Twist_

// MsgType=geometry_msgs/Twist
typedef struct {
  // MsgType=geometry_msgs/Vector3
  SL_Bus_obstacleStopper_geometry_msgs_Vector3 Linear;

  // MsgType=geometry_msgs/Vector3
  SL_Bus_obstacleStopper_geometry_msgs_Vector3 Angular;
} SL_Bus_obstacleStopper_geometry_msgs_Twist;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_obstacleStopper_std_msgs_Float64_
#define DEFINED_TYPEDEF_FOR_SL_Bus_obstacleStopper_std_msgs_Float64_

// MsgType=std_msgs/Float64
typedef struct {
  real_T Data;
} SL_Bus_obstacleStopper_std_msgs_Float64;

#endif

#ifndef typedef_robotics_slros_internal_block_T
#define typedef_robotics_slros_internal_block_T

typedef struct {
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
} robotics_slros_internal_block_T;

#endif                                 //typedef_robotics_slros_internal_block_T

#ifndef typedef_robotics_slros_internal_blo_f_T
#define typedef_robotics_slros_internal_blo_f_T

typedef struct {
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
} robotics_slros_internal_blo_f_T;

#endif                                 //typedef_robotics_slros_internal_blo_f_T

// Parameters (default storage)
typedef struct P_obstacleStopper_T_ P_obstacleStopper_T;

// Forward declaration for rtModel
typedef struct tag_RTM_obstacleStopper_T RT_MODEL_obstacleStopper_T;

#endif                                 // RTW_HEADER_obstacleStopper_types_h_

//
// File trailer for generated code.
//
// [EOF]
//
