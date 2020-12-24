//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: obstacleStopper_data.cpp
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
#include "obstacleStopper.h"
#include "obstacleStopper_private.h"

// Block parameters (default storage)
P_obstacleStopper_T obstacleStopper_P = {
  // Mask Parameter: DeadMansSwitch_stepSize
  //  Referenced by: '<S2>/Simulate step size'

  0.02,

  // Mask Parameter: DeadMansSwitch_timeout
  //  Referenced by: '<S2>/Timeout in seconds'

  0.2,

  // Computed Parameter: Out1_Y0
  //  Referenced by: '<S12>/Out1'

  {
    {
      0.0,                             // X
      0.0,                             // Y
      0.0                              // Z
    },                                 // Linear

    {
      0.0,                             // X
      0.0,                             // Y
      0.0                              // Z
    }                                  // Angular
  },

  // Computed Parameter: Constant_Value
  //  Referenced by: '<S7>/Constant'

  {
    {
      0.0,                             // X
      0.0,                             // Y
      0.0                              // Z
    },                                 // Linear

    {
      0.0,                             // X
      0.0,                             // Y
      0.0                              // Z
    }                                  // Angular
  },

  // Computed Parameter: Out1_Y0_d
  //  Referenced by: '<S13>/Out1'

  {
    {
      0.0,                             // X
      0.0,                             // Y
      0.0                              // Z
    },                                 // Linear

    {
      0.0,                             // X
      0.0,                             // Y
      0.0                              // Z
    }                                  // Angular
  },

  // Computed Parameter: Constant_Value_p
  //  Referenced by: '<S8>/Constant'

  {
    {
      0.0,                             // X
      0.0,                             // Y
      0.0                              // Z
    },                                 // Linear

    {
      0.0,                             // X
      0.0,                             // Y
      0.0                              // Z
    }                                  // Angular
  },

  // Computed Parameter: Constant_Value_h
  //  Referenced by: '<S1>/Constant'

  {
    {
      0.0,                             // X
      0.0,                             // Y
      0.0                              // Z
    },                                 // Linear

    {
      0.0,                             // X
      0.0,                             // Y
      0.0                              // Z
    }                                  // Angular
  },

  // Computed Parameter: Out1_Y0_h
  //  Referenced by: '<S11>/Out1'

  {
    0.0                                // Data
  },

  // Computed Parameter: Constant_Value_o
  //  Referenced by: '<S6>/Constant'

  {
    0.0                                // Data
  },

  // Expression: 5
  //  Referenced by: '<S3>/Constant'

  5.0,

  // Expression: 1
  //  Referenced by: '<S3>/Saturation'

  1.0,

  // Expression: 0
  //  Referenced by: '<S3>/Saturation'

  0.0,

  // Expression: 1/20
  //  Referenced by: '<S4>/Sample Time in Seconds'

  0.05,

  // Expression: 3
  //  Referenced by: '<S4>/Gain: Safe Distance At HighSpeed follows 3 second rules'

  3.0
};

//
// File trailer for generated code.
//
// [EOF]
//
