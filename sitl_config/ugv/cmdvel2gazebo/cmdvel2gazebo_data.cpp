//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: cmdvel2gazebo_data.cpp
//
// Code generated for Simulink model 'cmdvel2gazebo'.
//
// Model version                  : 1.73
// Simulink Coder version         : 8.12 (R2017a) 16-Feb-2017
// C/C++ source code generated on : Tue May 22 14:50:02 2018
//
// Target selection: ert.tlc
// Embedded hardware selection: Custom Processor->Custom
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "cmdvel2gazebo.h"
#include "cmdvel2gazebo_private.h"

// Block parameters (auto storage)
P_cmdvel2gazebo_T cmdvel2gazebo_P = {
  0.01,                                // Mask Parameter: DeadMansSwitch_stepSize
                                       //  Referenced by: '<S5>/Simulate step size'

  0.2,                                 // Mask Parameter: DeadMansSwitch_timeout
                                       //  Referenced by: '<S5>/Timeout in seconds'


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
  },                                   // Computed Parameter: Out1_Y0
                                       //  Referenced by: '<S14>/Out1'


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
  },                                   // Computed Parameter: Constant_Value
                                       //  Referenced by: '<S11>/Constant'

  -0.75,                               // Computed Parameter: TransferFcn1_A
                                       //  Referenced by: '<S12>/Transfer Fcn1'

  0.75,                                // Computed Parameter: TransferFcn1_C
                                       //  Referenced by: '<S12>/Transfer Fcn1'

  -1.5,                                // Computed Parameter: TransferFcn2_A
                                       //  Referenced by: '<S12>/Transfer Fcn2'

  1.5,                                 // Computed Parameter: TransferFcn2_C
                                       //  Referenced by: '<S12>/Transfer Fcn2'

  0.0,                                 // Expression: 0
                                       //  Referenced by: '<S12>/Switch'

  2.8101,                              // Expression: 2.8101
                                       //  Referenced by: '<Root>/Gazebo Multiplier'

  2.62,                                // Expression: 2.62
                                       //  Referenced by: '<Root>/Wheelbase, L'

  1.301,                               // Expression: 1.301
                                       //  Referenced by: '<Root>/Car tread, T'

  0.6,                                 // Expression: 0.6
                                       //  Referenced by: '<Root>/maxsteerInside'


  {
    0.0                                // Data
  },                                   // Computed Parameter: Constant_Value_a
                                       //  Referenced by: '<S1>/Constant'


  {
    0.0                                // Data
  },                                   // Computed Parameter: Constant_Value_h
                                       //  Referenced by: '<S2>/Constant'


  {
    0.0                                // Data
  },                                   // Computed Parameter: Constant_Value_j
                                       //  Referenced by: '<S3>/Constant'


  {
    0.0                                // Data
  }                                    // Computed Parameter: Constant_Value_h5
                                       //  Referenced by: '<S4>/Constant'

};

//
// File trailer for generated code.
//
// [EOF]
//
