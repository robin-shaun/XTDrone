//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: obstacleStopper.cpp
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

// Named constants for Chart: '<S4>/Stateflow block for slowing dow or accelerating vehicles at high speed in case of obstacle avoidance' 
#define IN_Check_initial_Safe_distance ((uint8_T)2U)
#define obstacleStop_IN_NO_ACTIVE_CHILD ((uint8_T)0U)
#define obstacleStopper_IN_Accelerate  ((uint8_T)1U)
#define obstacleStopper_IN_Decelerate  ((uint8_T)3U)
#define obstacleStopper_IN_NewCommand  ((uint8_T)4U)
#define obstacleStopper_IN_NormalDrive ((uint8_T)5U)

// Block signals (default storage)
B_obstacleStopper_T obstacleStopper_B;

// Block states (default storage)
DW_obstacleStopper_T obstacleStopper_DW;

// Real-time model
RT_MODEL_obstacleStopper_T obstacleStopper_M_;
RT_MODEL_obstacleStopper_T *const obstacleStopper_M = &obstacleStopper_M_;

// Forward declaration for local functions
static void matlabCodegenHandle_matlabCod_f(robotics_slros_internal_blo_f_T *obj);
static void matlabCodegenHandle_matlabCodeg(robotics_slros_internal_block_T *obj);
static void matlabCodegenHandle_matlabCod_f(robotics_slros_internal_blo_f_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void matlabCodegenHandle_matlabCodeg(robotics_slros_internal_block_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

// Model step function
void obstacleStopper_step(void)
{
  boolean_T timedOut;
  SL_Bus_obstacleStopper_std_msgs_Float64 b_varargout_2;
  boolean_T b_varargout_1;
  real_T rtb_GainSafeDistanceAtHighSpeed;
  real_T rtb_safeValue;

  // Outputs for Atomic SubSystem: '<Root>/Subscribe'
  // MATLABSystem: '<S6>/SourceBlock' incorporates:
  //   Inport: '<S11>/In1'

  b_varargout_1 = Sub_obstacleStopper_12.getLatestMessage(&b_varargout_2);

  // Outputs for Enabled SubSystem: '<S6>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S11>/Enable'

  if (b_varargout_1) {
    obstacleStopper_B.In1_d = b_varargout_2;
  }

  // End of MATLABSystem: '<S6>/SourceBlock'
  // End of Outputs for SubSystem: '<S6>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<Root>/Subscribe'

  // Outputs for Atomic SubSystem: '<Root>/Subscribe2'
  // MATLABSystem: '<S8>/SourceBlock' incorporates:
  //   Inport: '<S13>/In1'

  b_varargout_1 = Sub_obstacleStopper_39.getLatestMessage
    (&obstacleStopper_B.BusAssignment);

  // Outputs for Enabled SubSystem: '<S8>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S13>/Enable'

  if (b_varargout_1) {
    obstacleStopper_B.In1 = obstacleStopper_B.BusAssignment;
  }

  // End of MATLABSystem: '<S8>/SourceBlock'
  // End of Outputs for SubSystem: '<S8>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<Root>/Subscribe2'

  // Outputs for Atomic SubSystem: '<Root>/Subscribe1'
  // MATLABSystem: '<S7>/SourceBlock' incorporates:
  //   Inport: '<S12>/In1'

  b_varargout_1 = Sub_obstacleStopper_13.getLatestMessage
    (&obstacleStopper_B.BusAssignment);

  // Outputs for Enabled SubSystem: '<S7>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S12>/Enable'

  if (b_varargout_1) {
    obstacleStopper_B.In1_l = obstacleStopper_B.BusAssignment;
  }

  // End of Outputs for SubSystem: '<S7>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<Root>/Subscribe1'

  // Outputs for Atomic SubSystem: '<Root>/Dead Man's Switch'
  // MATLAB Function: '<S2>/timeout set to 0 output' incorporates:
  //   MATLABSystem: '<S7>/SourceBlock'

  // MATLAB Function 'Dead Man\'s Switch/timeout set to 0 output': '<S9>:1'
  // '<S9>:1:4' timedOut=true;
  timedOut = true;

  // '<S9>:1:5' if isempty(sinceLastMsg)
  if (!obstacleStopper_DW.sinceLastMsg_not_empty) {
    // '<S9>:1:6' sinceLastMsg=(timeout/stepSize)+1;
    obstacleStopper_DW.sinceLastMsg = obstacleStopper_P.DeadMansSwitch_timeout /
      obstacleStopper_P.DeadMansSwitch_stepSize + 1.0;
    obstacleStopper_DW.sinceLastMsg_not_empty = true;
  }

  // '<S9>:1:9' safeValue=0;
  rtb_safeValue = 0.0;

  // Outputs for Atomic SubSystem: '<Root>/Subscribe1'
  //  if no new message
  //  after timeout, we output 0
  // '<S9>:1:12' if( isNew == true )
  if (b_varargout_1) {
    // '<S9>:1:13' sinceLastMsg = 0;
    obstacleStopper_DW.sinceLastMsg = 0.0;
  } else {
    // '<S9>:1:14' else
    // '<S9>:1:15' sinceLastMsg = sinceLastMsg+1;
    obstacleStopper_DW.sinceLastMsg++;
  }

  // End of Outputs for SubSystem: '<Root>/Subscribe1'
  //  Note: we require step size as an input here, and depend on our
  //  system being executed at a regular rate, or bad things will happen
  // '<S9>:1:20' if( sinceLastMsg < timeout/stepSize )
  if (obstacleStopper_DW.sinceLastMsg < obstacleStopper_P.DeadMansSwitch_timeout
      / obstacleStopper_P.DeadMansSwitch_stepSize) {
    // '<S9>:1:21' timedOut = false;
    timedOut = false;
  }

  // '<S9>:1:24' if( timedOut == false )
  if (!timedOut) {
    // '<S9>:1:25' safeValue = value;
    rtb_safeValue = obstacleStopper_B.In1_l.Linear.X;
  }

  // End of MATLAB Function: '<S2>/timeout set to 0 output'
  // End of Outputs for SubSystem: '<Root>/Dead Man's Switch'

  // Gain: '<S4>/Gain: Safe Distance At HighSpeed follows 3 second rules'
  rtb_GainSafeDistanceAtHighSpeed =
    obstacleStopper_P.GainSafeDistanceAtHighSpeedfoll *
    obstacleStopper_B.In1.Linear.X;

  // Chart: '<S4>/Stateflow block for slowing dow or accelerating vehicles at high speed in case of obstacle avoidance' incorporates:
  //   Constant: '<S4>/Sample Time in Seconds'
  //   SignalConversion: '<Root>/SigConversion_InsertedFor_Bus Selector1_at_outport_0'

  // Gateway: ObstacleStopper V2.0/Stateflow block for slowing dow or accelerating vehicles at high
  // speed in case of obstacle avoidance
  // During: ObstacleStopper V2.0/Stateflow block for slowing dow or accelerating vehicles at high
  // speed in case of obstacle avoidance
  if (obstacleStopper_DW.is_active_c1_obstacleStopper == 0U) {
    // Entry: ObstacleStopper V2.0/Stateflow block for slowing dow or accelerating vehicles at high
    // speed in case of obstacle avoidance
    obstacleStopper_DW.is_active_c1_obstacleStopper = 1U;

    // Entry Internal: ObstacleStopper V2.0/Stateflow block for slowing dow or accelerating vehicles at high
    // speed in case of obstacle avoidance
    // Transition: '<S10>:15'
    obstacleStopper_DW.is_c1_obstacleStopper = IN_Check_initial_Safe_distance;

    // Entry 'Check_initial_Safe_distance': '<S10>:33'
    // '<S10>:33:1' distance = dist;
    obstacleStopper_DW.distance = obstacleStopper_B.In1_d.Data;
  } else {
    switch (obstacleStopper_DW.is_c1_obstacleStopper) {
     case obstacleStopper_IN_Accelerate:
      // During 'Accelerate': '<S10>:14'
      // '<S10>:21:1' sf_internal_predicateOutput = count <(1/Ts) && vOut < v_cmd; 
      if (obstacleStopper_DW.count < 1.0 /
          obstacleStopper_P.SampleTimeinSeconds_Value) {
        b_varargout_1 = (obstacleStopper_B.vOut < rtb_safeValue);
      } else {
        b_varargout_1 = false;
      }

      if (b_varargout_1) {
        // Transition: '<S10>:21'
        obstacleStopper_DW.is_c1_obstacleStopper = obstacleStopper_IN_Accelerate;

        // Entry 'Accelerate': '<S10>:14'
        // '<S10>:14:1' count = 1
        obstacleStopper_DW.count = 1.0;
      } else {
        // '<S10>:17:1' sf_internal_predicateOutput = count >=(1/Ts) || vOut >= v_cmd; 
        if (obstacleStopper_DW.count >= 1.0 /
            obstacleStopper_P.SampleTimeinSeconds_Value) {
          b_varargout_1 = true;
        } else {
          b_varargout_1 = (obstacleStopper_B.vOut >= rtb_safeValue);
        }

        if (b_varargout_1) {
          // Transition: '<S10>:17'
          obstacleStopper_DW.is_c1_obstacleStopper =
            obstacleStopper_IN_NormalDrive;

          // Entry 'NormalDrive': '<S10>:12'
          // '<S10>:12:1' vOut = v_cmd;
          obstacleStopper_B.vOut = rtb_safeValue;

          // '<S10>:12:1' distance = dist;
          obstacleStopper_DW.distance = obstacleStopper_B.In1_d.Data;
        } else {
          // '<S10>:14:1' vOut = vel + 0.05*(Ts);
          obstacleStopper_B.vOut = 0.05 *
            obstacleStopper_P.SampleTimeinSeconds_Value +
            obstacleStopper_B.In1.Linear.X;

          // '<S10>:14:2' count = count + 1;
          obstacleStopper_DW.count++;
        }
      }
      break;

     case IN_Check_initial_Safe_distance:
      // During 'Check_initial_Safe_distance': '<S10>:33'
      // '<S10>:34:1' sf_internal_predicateOutput = distance > (3*v_cmd);
      if (obstacleStopper_DW.distance > 3.0 * rtb_safeValue) {
        // Transition: '<S10>:34'
        obstacleStopper_DW.is_c1_obstacleStopper =
          obstacleStopper_IN_NormalDrive;

        // Entry 'NormalDrive': '<S10>:12'
        // '<S10>:12:1' vOut = v_cmd;
        obstacleStopper_B.vOut = rtb_safeValue;

        // '<S10>:12:1' distance = dist;
        obstacleStopper_DW.distance = obstacleStopper_B.In1_d.Data;
      } else {
        // '<S10>:37:1' sf_internal_predicateOutput = distance  <= 3*v_cmd;
        if (obstacleStopper_DW.distance <= 3.0 * rtb_safeValue) {
          // Transition: '<S10>:37'
          obstacleStopper_DW.is_c1_obstacleStopper =
            obstacleStopper_IN_NewCommand;

          // Entry 'NewCommand': '<S10>:38'
          // '<S10>:38:1' vOut = v_cmd/3;
          obstacleStopper_B.vOut = rtb_safeValue / 3.0;

          // '<S10>:38:1' distance = dist;
          obstacleStopper_DW.distance = obstacleStopper_B.In1_d.Data;
        } else {
          // '<S10>:33:1' vOut = 0;
          obstacleStopper_B.vOut = 0.0;
        }
      }
      break;

     case obstacleStopper_IN_Decelerate:
      // During 'Decelerate': '<S10>:13'
      // '<S10>:20:1' sf_internal_predicateOutput = distance >= safe_distance_highspeed; 
      if (obstacleStopper_DW.distance >= rtb_GainSafeDistanceAtHighSpeed) {
        // Transition: '<S10>:20'
        obstacleStopper_DW.is_c1_obstacleStopper = obstacleStopper_IN_Accelerate;

        // Entry 'Accelerate': '<S10>:14'
        // '<S10>:14:1' count = 1
        obstacleStopper_DW.count = 1.0;
      } else {
        // '<S10>:18:1' sf_internal_predicateOutput = distance < safe_distance_highspeed; 
        if (obstacleStopper_DW.distance < rtb_GainSafeDistanceAtHighSpeed) {
          // Transition: '<S10>:18'
          obstacleStopper_DW.is_c1_obstacleStopper =
            obstacleStopper_IN_Decelerate;

          // Entry 'Decelerate': '<S10>:13'
          // '<S10>:13:1' vOut = vel/10;
          obstacleStopper_B.vOut = obstacleStopper_B.In1.Linear.X / 10.0;

          // '<S10>:13:1' distance = dist;
          obstacleStopper_DW.distance = obstacleStopper_B.In1_d.Data;
        } else {
          // '<S10>:13:2' vOut = vel/10;
          obstacleStopper_B.vOut = obstacleStopper_B.In1.Linear.X / 10.0;

          // '<S10>:13:2' distance = dist;
          obstacleStopper_DW.distance = obstacleStopper_B.In1_d.Data;
        }
      }
      break;

     case obstacleStopper_IN_NewCommand:
      // During 'NewCommand': '<S10>:38'
      // '<S10>:41:1' sf_internal_predicateOutput = distance >= safe_distance_highspeed; 
      if (obstacleStopper_DW.distance >= rtb_GainSafeDistanceAtHighSpeed) {
        // Transition: '<S10>:41'
        obstacleStopper_DW.is_c1_obstacleStopper =
          obstacleStopper_IN_NormalDrive;

        // Entry 'NormalDrive': '<S10>:12'
        // '<S10>:12:1' vOut = v_cmd;
        obstacleStopper_B.vOut = rtb_safeValue;

        // '<S10>:12:1' distance = dist;
        obstacleStopper_DW.distance = obstacleStopper_B.In1_d.Data;
      } else {
        // '<S10>:38:2' vOut = v_cmd/3;
        obstacleStopper_B.vOut = rtb_safeValue / 3.0;

        // '<S10>:38:2' distance = dist;
        obstacleStopper_DW.distance = obstacleStopper_B.In1_d.Data;
      }
      break;

     default:
      // During 'NormalDrive': '<S10>:12'
      // '<S10>:16:1' sf_internal_predicateOutput = distance < safe_distance_highspeed && vel >= 9; 
      if ((obstacleStopper_DW.distance < rtb_GainSafeDistanceAtHighSpeed) &&
          (obstacleStopper_B.In1.Linear.X >= 9.0)) {
        // Transition: '<S10>:16'
        obstacleStopper_DW.is_c1_obstacleStopper = obstacleStopper_IN_Decelerate;

        // Entry 'Decelerate': '<S10>:13'
        // '<S10>:13:1' vOut = vel/10;
        obstacleStopper_B.vOut = obstacleStopper_B.In1.Linear.X / 10.0;

        // '<S10>:13:1' distance = dist;
        obstacleStopper_DW.distance = obstacleStopper_B.In1_d.Data;
      } else {
        // '<S10>:19:1' sf_internal_predicateOutput = distance >= safe_distance_highspeed ||  vel < 9; 
        if ((obstacleStopper_DW.distance >= rtb_GainSafeDistanceAtHighSpeed) ||
            (obstacleStopper_B.In1.Linear.X < 9.0)) {
          // Transition: '<S10>:19'
          obstacleStopper_DW.is_c1_obstacleStopper =
            obstacleStopper_IN_NormalDrive;

          // Entry 'NormalDrive': '<S10>:12'
          // '<S10>:12:1' vOut = v_cmd;
          obstacleStopper_B.vOut = rtb_safeValue;

          // '<S10>:12:1' distance = dist;
          obstacleStopper_DW.distance = obstacleStopper_B.In1_d.Data;
        } else {
          // '<S10>:12:2' vOut = v_cmd;
          obstacleStopper_B.vOut = rtb_safeValue;

          // '<S10>:12:2' distance = dist;
          obstacleStopper_DW.distance = obstacleStopper_B.In1_d.Data;
        }
      }
      break;
    }
  }

  // End of Chart: '<S4>/Stateflow block for slowing dow or accelerating vehicles at high speed in case of obstacle avoidance' 

  // BusAssignment: '<Root>/Bus Assignment' incorporates:
  //   Constant: '<S1>/Constant'

  obstacleStopper_B.BusAssignment = obstacleStopper_P.Constant_Value_h;

  // Sum: '<S3>/Subtract' incorporates:
  //   Constant: '<S3>/Constant'
  //   SignalConversion: '<Root>/SigConversion_InsertedFor_Bus Selector1_at_outport_0'

  rtb_safeValue = obstacleStopper_B.In1_d.Data -
    obstacleStopper_P.Constant_Value_e;

  // Saturate: '<S3>/Saturation'
  if (rtb_safeValue > obstacleStopper_P.Saturation_UpperSat) {
    rtb_safeValue = obstacleStopper_P.Saturation_UpperSat;
  } else {
    if (rtb_safeValue < obstacleStopper_P.Saturation_LowerSat) {
      rtb_safeValue = obstacleStopper_P.Saturation_LowerSat;
    }
  }

  // End of Saturate: '<S3>/Saturation'

  // Signum: '<S3>/Sign'
  if (rtb_safeValue < 0.0) {
    rtb_safeValue = -1.0;
  } else if (rtb_safeValue > 0.0) {
    rtb_safeValue = 1.0;
  } else if (rtb_safeValue == 0.0) {
    rtb_safeValue = 0.0;
  } else {
    rtb_safeValue = (rtNaN);
  }

  // End of Signum: '<S3>/Sign'

  // Product: '<S3>/Product'
  rtb_safeValue *= obstacleStopper_B.vOut;

  // MinMax: '<Root>/MinMax'
  if ((rtb_safeValue < obstacleStopper_B.vOut) || rtIsNaN(obstacleStopper_B.vOut))
  {
    // BusAssignment: '<Root>/Bus Assignment'
    obstacleStopper_B.BusAssignment.Linear.X = rtb_safeValue;
  } else {
    // BusAssignment: '<Root>/Bus Assignment'
    obstacleStopper_B.BusAssignment.Linear.X = obstacleStopper_B.vOut;
  }

  // End of MinMax: '<Root>/MinMax'

  // BusAssignment: '<Root>/Bus Assignment'
  obstacleStopper_B.BusAssignment.Angular.Z = obstacleStopper_B.In1_l.Angular.Z;

  // Outputs for Atomic SubSystem: '<Root>/Publish'
  // MATLABSystem: '<S5>/SinkBlock'
  Pub_obstacleStopper_17.publish(&obstacleStopper_B.BusAssignment);

  // End of Outputs for SubSystem: '<Root>/Publish'
}

// Model initialize function
void obstacleStopper_initialize(void)
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  // initialize error status
  rtmSetErrorStatus(obstacleStopper_M, (NULL));

  // block I/O
  (void) memset(((void *) &obstacleStopper_B), 0,
                sizeof(B_obstacleStopper_T));

  // states (dwork)
  (void) memset((void *)&obstacleStopper_DW, 0,
                sizeof(DW_obstacleStopper_T));

  {
    static const char_T tmp[12] = { 'c', 'm', 'd', '_', 'v', 'e', 'l', '_', 's',
      'a', 'f', 'e' };

    static const char_T tmp_0[7] = { 'c', 'm', 'd', '_', 'v', 'e', 'l' };

    static const char_T tmp_1[35] = { 'd', 'i', 's', 't', 'a', 'n', 'c', 'e',
      'E', 's', 't', 'i', 'm', 'a', 't', 'o', 'r', 'S', 't', 'e', 'e', 'r', 'i',
      'n', 'g', 'B', 'a', 's', 'e', 'd', '/', 'd', 'i', 's', 't' };

    char_T tmp_2[13];
    char_T tmp_3[8];
    char_T tmp_4[4];
    int32_T i;

    // Start for Atomic SubSystem: '<Root>/Subscribe'
    // Start for MATLABSystem: '<S6>/SourceBlock'
    obstacleStopper_DW.obj_e.matlabCodegenIsDeleted = true;
    obstacleStopper_DW.obj_e.isInitialized = 0;
    obstacleStopper_DW.obj_e.matlabCodegenIsDeleted = false;
    obstacleStopper_DW.obj_e.isSetupComplete = false;
    obstacleStopper_DW.obj_e.isInitialized = 1;
    for (i = 0; i < 35; i++) {
      obstacleStopper_B.cv0[i] = tmp_1[i];
    }

    obstacleStopper_B.cv0[35] = '\x00';
    Sub_obstacleStopper_12.createSubscriber(obstacleStopper_B.cv0, 1);
    obstacleStopper_DW.obj_e.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S6>/SourceBlock'
    // End of Start for SubSystem: '<Root>/Subscribe'

    // Start for Atomic SubSystem: '<Root>/Subscribe2'
    // Start for MATLABSystem: '<S8>/SourceBlock'
    obstacleStopper_DW.obj_k.matlabCodegenIsDeleted = true;
    obstacleStopper_DW.obj_k.isInitialized = 0;
    obstacleStopper_DW.obj_k.matlabCodegenIsDeleted = false;
    obstacleStopper_DW.obj_k.isSetupComplete = false;
    obstacleStopper_DW.obj_k.isInitialized = 1;
    tmp_4[0] = 'v';
    tmp_4[1] = 'e';
    tmp_4[2] = 'l';
    tmp_4[3] = '\x00';
    Sub_obstacleStopper_39.createSubscriber(tmp_4, 1);
    obstacleStopper_DW.obj_k.isSetupComplete = true;

    // End of Start for SubSystem: '<Root>/Subscribe2'

    // Start for Atomic SubSystem: '<Root>/Subscribe1'
    // Start for MATLABSystem: '<S7>/SourceBlock'
    obstacleStopper_DW.obj_l.matlabCodegenIsDeleted = true;
    obstacleStopper_DW.obj_l.isInitialized = 0;
    obstacleStopper_DW.obj_l.matlabCodegenIsDeleted = false;
    obstacleStopper_DW.obj_l.isSetupComplete = false;
    obstacleStopper_DW.obj_l.isInitialized = 1;
    for (i = 0; i < 7; i++) {
      tmp_3[i] = tmp_0[i];
    }

    tmp_3[7] = '\x00';
    Sub_obstacleStopper_13.createSubscriber(tmp_3, 1);
    obstacleStopper_DW.obj_l.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S7>/SourceBlock'
    // End of Start for SubSystem: '<Root>/Subscribe1'

    // Start for Atomic SubSystem: '<Root>/Publish'
    // Start for MATLABSystem: '<S5>/SinkBlock'
    obstacleStopper_DW.obj.matlabCodegenIsDeleted = true;
    obstacleStopper_DW.obj.isInitialized = 0;
    obstacleStopper_DW.obj.matlabCodegenIsDeleted = false;
    obstacleStopper_DW.obj.isSetupComplete = false;
    obstacleStopper_DW.obj.isInitialized = 1;
    for (i = 0; i < 12; i++) {
      tmp_2[i] = tmp[i];
    }

    tmp_2[12] = '\x00';
    Pub_obstacleStopper_17.createPublisher(tmp_2, 1);
    obstacleStopper_DW.obj.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S5>/SinkBlock'
    // End of Start for SubSystem: '<Root>/Publish'

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe'
    // SystemInitialize for Enabled SubSystem: '<S6>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S11>/Out1'
    obstacleStopper_B.In1_d = obstacleStopper_P.Out1_Y0_h;

    // End of SystemInitialize for SubSystem: '<S6>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe'

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe2'
    // SystemInitialize for Enabled SubSystem: '<S8>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S13>/Out1'
    obstacleStopper_B.In1 = obstacleStopper_P.Out1_Y0_d;

    // End of SystemInitialize for SubSystem: '<S8>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe2'

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe1'
    // SystemInitialize for Enabled SubSystem: '<S7>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S12>/Out1'
    obstacleStopper_B.In1_l = obstacleStopper_P.Out1_Y0;

    // End of SystemInitialize for SubSystem: '<S7>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe1'

    // SystemInitialize for Atomic SubSystem: '<Root>/Dead Man's Switch'
    // SystemInitialize for MATLAB Function: '<S2>/timeout set to 0 output'
    obstacleStopper_DW.sinceLastMsg_not_empty = false;

    // End of SystemInitialize for SubSystem: '<Root>/Dead Man's Switch'

    // SystemInitialize for Chart: '<S4>/Stateflow block for slowing dow or accelerating vehicles at high speed in case of obstacle avoidance' 
    obstacleStopper_DW.is_active_c1_obstacleStopper = 0U;
    obstacleStopper_DW.is_c1_obstacleStopper = obstacleStop_IN_NO_ACTIVE_CHILD;
  }
}

// Model terminate function
void obstacleStopper_terminate(void)
{
  // Terminate for Atomic SubSystem: '<Root>/Subscribe'
  // Terminate for MATLABSystem: '<S6>/SourceBlock'
  matlabCodegenHandle_matlabCod_f(&obstacleStopper_DW.obj_e);

  // End of Terminate for SubSystem: '<Root>/Subscribe'

  // Terminate for Atomic SubSystem: '<Root>/Subscribe2'
  // Terminate for MATLABSystem: '<S8>/SourceBlock'
  matlabCodegenHandle_matlabCod_f(&obstacleStopper_DW.obj_k);

  // End of Terminate for SubSystem: '<Root>/Subscribe2'

  // Terminate for Atomic SubSystem: '<Root>/Subscribe1'
  // Terminate for MATLABSystem: '<S7>/SourceBlock'
  matlabCodegenHandle_matlabCod_f(&obstacleStopper_DW.obj_l);

  // End of Terminate for SubSystem: '<Root>/Subscribe1'

  // Terminate for Atomic SubSystem: '<Root>/Publish'
  // Terminate for MATLABSystem: '<S5>/SinkBlock'
  matlabCodegenHandle_matlabCodeg(&obstacleStopper_DW.obj);

  // End of Terminate for SubSystem: '<Root>/Publish'
}

//
// File trailer for generated code.
//
// [EOF]
//
