//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: interceptModel.cpp
//
// Code generated for Simulink model 'interceptModel'.
//
// Model version                  : 1.41
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Sun Jul 20 21:25:48 2025
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "interceptModel.h"
#include <math.h>
#include "rt_assert.h"

extern "C"
{

#include "rt_nonfinite.h"

}

#include "rtwtypes.h"
#include "zero_crossing_types.h"

// Block signals (default storage)
B_interceptModel_T interceptModel_B;

// Block states (default storage)
DW_interceptModel_T interceptModel_DW;

// Previous zero-crossings (trigger) states
PrevZCX_interceptModel_T interceptModel_PrevZCX;

// Real-time model
RT_MODEL_interceptModel_T interceptModel_M_ = RT_MODEL_interceptModel_T();
RT_MODEL_interceptModel_T *const interceptModel_M = &interceptModel_M_;
static void rate_monotonic_scheduler(void);

//
// Set which subrates need to run this base step (base rate always runs).
// This function must be called prior to calling the model step function
// in order to remember which rates need to run this base step.  The
// buffering of events allows for overlapping preemption.
//
void interceptModel_SetEventsForThisBaseStep(boolean_T *eventFlags)
{
  // Task runs when its counter is zero, computed via rtmStepTask macro
  eventFlags[1] = ((boolean_T)interceptModel_M->StepTask(1));
}

//
//         This function updates active task flag for each subrate
//         and rate transition flags for tasks that exchange data.
//         The function assumes rate-monotonic multitasking scheduler.
//         The function must be called at model base rate so that
//         the generated code self-manages all its subrates and rate
//         transition flags.
//
static void rate_monotonic_scheduler(void)
{
  // To ensure a deterministic data transfer between two rates,
  //  data is transferred at the priority of a fast task and the frequency
  //  of the slow task.  The following flags indicate when the data transfer
  //  happens.  That is, a rate interaction flag is set true when both rates
  //  will run, and false otherwise.


  // tid 0 shares data with slower tid rate: 1
  interceptModel_M->Timing.RateInteraction.TID0_1 =
    (interceptModel_M->Timing.TaskCounters.TID[1] == 0);

  // Compute which subrates run during the next base time step.  Subrates
  //  are an integer multiple of the base rate counter.  Therefore, the subtask
  //  counter is reset when it reaches its limit (zero means run).

  (interceptModel_M->Timing.TaskCounters.TID[1])++;
  if ((interceptModel_M->Timing.TaskCounters.TID[1]) > 49) {// Sample time: [0.5s, 0.0s] 
    interceptModel_M->Timing.TaskCounters.TID[1] = 0;
  }
}

// Model step function for TID0
void interceptModel_step0(void)        // Sample time: [0.01s, 0.0s]
{
  real_T rtb_Sign_b;
  real_T rtb_VectorConcatenate_tmp;
  real_T rtb_VectorConcatenate_tmp_0;
  real_T rtb_VectorConcatenate_tmp_1;
  real_T rtb_VectorConcatenate_tmp_2;
  real_T rtb_VectorConcatenate_tmp_3;
  int32_T i;
  int32_T i_0;
  int32_T rtb_VectorConcatenate_tmp_4;
  boolean_T rtb_LogicalOperator_tmp;
  boolean_T tmp;
  boolean_T tmp_0;

  {                                    // Sample time: [0.01s, 0.0s]
    rate_monotonic_scheduler();
  }

  // MATLABSystem: '<S18>/SourceBlock'
  tmp = uORB_read_step(interceptModel_DW.obj_o.orbMetadataObj,
                       &interceptModel_DW.obj_o.eventStructObj,
                       &interceptModel_B.r1, false, 1.0);

  // Outputs for Enabled SubSystem: '<S18>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S64>/Enable'

  // Start for MATLABSystem: '<S18>/SourceBlock'
  if (tmp) {
    // SignalConversion generated from: '<S64>/In1'
    interceptModel_B.In1 = interceptModel_B.r1;
  }

  // End of Outputs for SubSystem: '<S18>/Enabled Subsystem'

  // Sqrt: '<S62>/sqrt' incorporates:
  //   DataTypeConversion: '<S6>/Data Type Conversion'
  //   Product: '<S63>/Product'
  //   Product: '<S63>/Product1'
  //   Product: '<S63>/Product2'
  //   Product: '<S63>/Product3'
  //   Sum: '<S63>/Sum'

  interceptModel_B.Sum_a = sqrt(((static_cast<real_T>(interceptModel_B.In1.q[0])
    * interceptModel_B.In1.q[0] + static_cast<real_T>(interceptModel_B.In1.q[1])
    * interceptModel_B.In1.q[1]) + static_cast<real_T>(interceptModel_B.In1.q[2])
    * interceptModel_B.In1.q[2]) + static_cast<real_T>(interceptModel_B.In1.q[3])
    * interceptModel_B.In1.q[3]);

  // Product: '<S61>/Product' incorporates:
  //   DataTypeConversion: '<S6>/Data Type Conversion'

  interceptModel_B.Sum_b = interceptModel_B.In1.q[0] / interceptModel_B.Sum_a;

  // Product: '<S61>/Product1' incorporates:
  //   DataTypeConversion: '<S6>/Data Type Conversion'

  interceptModel_B.Add = interceptModel_B.In1.q[1] / interceptModel_B.Sum_a;

  // Product: '<S61>/Product2' incorporates:
  //   DataTypeConversion: '<S6>/Data Type Conversion'

  interceptModel_B.Product1_pd = interceptModel_B.In1.q[2] /
    interceptModel_B.Sum_a;

  // Product: '<S61>/Product3' incorporates:
  //   DataTypeConversion: '<S6>/Data Type Conversion'

  interceptModel_B.Sum_a = interceptModel_B.In1.q[3] / interceptModel_B.Sum_a;

  // Product: '<S51>/Product3' incorporates:
  //   Product: '<S55>/Product3'

  interceptModel_B.Switch3 = interceptModel_B.Sum_b * interceptModel_B.Sum_b;

  // Product: '<S51>/Product2' incorporates:
  //   Product: '<S55>/Product2'

  rtb_Sign_b = interceptModel_B.Add * interceptModel_B.Add;

  // Product: '<S51>/Product1' incorporates:
  //   Product: '<S55>/Product1'
  //   Product: '<S59>/Product1'

  interceptModel_B.MinMax1_e0 = interceptModel_B.Product1_pd *
    interceptModel_B.Product1_pd;

  // Product: '<S51>/Product' incorporates:
  //   Product: '<S55>/Product'
  //   Product: '<S59>/Product'

  rtb_VectorConcatenate_tmp_1 = interceptModel_B.Sum_a * interceptModel_B.Sum_a;

  // Sum: '<S51>/Sum' incorporates:
  //   Product: '<S51>/Product'
  //   Product: '<S51>/Product1'
  //   Product: '<S51>/Product2'
  //   Product: '<S51>/Product3'

  interceptModel_B.VectorConcatenate[0] = ((interceptModel_B.Switch3 +
    rtb_Sign_b) - interceptModel_B.MinMax1_e0) - rtb_VectorConcatenate_tmp_1;

  // Product: '<S54>/Product3' incorporates:
  //   Product: '<S52>/Product3'

  rtb_VectorConcatenate_tmp = interceptModel_B.Sum_a * interceptModel_B.Sum_b;

  // Product: '<S54>/Product2' incorporates:
  //   Product: '<S52>/Product2'

  rtb_VectorConcatenate_tmp_0 = interceptModel_B.Add *
    interceptModel_B.Product1_pd;

  // Gain: '<S54>/Gain' incorporates:
  //   Product: '<S54>/Product2'
  //   Product: '<S54>/Product3'
  //   Sum: '<S54>/Sum'

  interceptModel_B.VectorConcatenate[1] = (rtb_VectorConcatenate_tmp_0 -
    rtb_VectorConcatenate_tmp) * interceptModel_P.Gain_Gain;

  // Product: '<S57>/Product2' incorporates:
  //   Product: '<S53>/Product2'

  rtb_VectorConcatenate_tmp_2 = interceptModel_B.Add * interceptModel_B.Sum_a;

  // Product: '<S57>/Product1' incorporates:
  //   Product: '<S53>/Product1'

  rtb_VectorConcatenate_tmp_3 = interceptModel_B.Sum_b *
    interceptModel_B.Product1_pd;

  // Gain: '<S57>/Gain' incorporates:
  //   Product: '<S57>/Product1'
  //   Product: '<S57>/Product2'
  //   Sum: '<S57>/Sum'

  interceptModel_B.VectorConcatenate[2] = (rtb_VectorConcatenate_tmp_3 +
    rtb_VectorConcatenate_tmp_2) * interceptModel_P.Gain_Gain_d;

  // Gain: '<S52>/Gain' incorporates:
  //   Sum: '<S52>/Sum'

  interceptModel_B.VectorConcatenate[3] = (rtb_VectorConcatenate_tmp +
    rtb_VectorConcatenate_tmp_0) * interceptModel_P.Gain_Gain_e;

  // Sum: '<S55>/Sum' incorporates:
  //   Sum: '<S59>/Sum'

  interceptModel_B.Switch3 -= rtb_Sign_b;
  interceptModel_B.VectorConcatenate[4] = (interceptModel_B.Switch3 +
    interceptModel_B.MinMax1_e0) - rtb_VectorConcatenate_tmp_1;

  // Product: '<S58>/Product1' incorporates:
  //   Product: '<S56>/Product1'

  rtb_Sign_b = interceptModel_B.Sum_b * interceptModel_B.Add;

  // Product: '<S58>/Product2' incorporates:
  //   Product: '<S56>/Product2'

  rtb_VectorConcatenate_tmp = interceptModel_B.Product1_pd *
    interceptModel_B.Sum_a;

  // Gain: '<S58>/Gain' incorporates:
  //   Product: '<S58>/Product1'
  //   Product: '<S58>/Product2'
  //   Sum: '<S58>/Sum'

  interceptModel_B.VectorConcatenate[5] = (rtb_VectorConcatenate_tmp -
    rtb_Sign_b) * interceptModel_P.Gain_Gain_j;

  // Gain: '<S53>/Gain' incorporates:
  //   Sum: '<S53>/Sum'

  interceptModel_B.VectorConcatenate[6] = (rtb_VectorConcatenate_tmp_2 -
    rtb_VectorConcatenate_tmp_3) * interceptModel_P.Gain_Gain_ju;

  // Gain: '<S56>/Gain' incorporates:
  //   Sum: '<S56>/Sum'

  interceptModel_B.VectorConcatenate[7] = (rtb_Sign_b +
    rtb_VectorConcatenate_tmp) * interceptModel_P.Gain_Gain_b;

  // Sum: '<S59>/Sum'
  interceptModel_B.VectorConcatenate[8] = (interceptModel_B.Switch3 -
    interceptModel_B.MinMax1_e0) + rtb_VectorConcatenate_tmp_1;

  // If: '<S27>/If1' incorporates:
  //   Constant: '<S27>/Constant'

  if (interceptModel_P.DirectionCosineMatrixtoRotati_c > 1) {
    // Outputs for IfAction SubSystem: '<S27>/If Warning//Error' incorporates:
    //   ActionPort: '<S36>/if'

    // Bias: '<S40>/Bias1' incorporates:
    //   Concatenate: '<S60>/Vector Concatenate'
    //   Math: '<S40>/Math Function'
    //   Product: '<S40>/Product'

    for (i = 0; i < 3; i++) {
      for (i_0 = 0; i_0 < 3; i_0++) {
        rtb_VectorConcatenate_tmp_4 = 3 * i_0 + i;
        interceptModel_B.rtb_VectorConcatenate_m[rtb_VectorConcatenate_tmp_4] =
          ((interceptModel_B.VectorConcatenate[3 * i + 1] *
            interceptModel_B.VectorConcatenate[3 * i_0 + 1] +
            interceptModel_B.VectorConcatenate[3 * i] *
            interceptModel_B.VectorConcatenate[3 * i_0]) +
           interceptModel_B.VectorConcatenate[3 * i + 2] *
           interceptModel_B.VectorConcatenate[3 * i_0 + 2]) +
          interceptModel_P.Bias1_Bias[rtb_VectorConcatenate_tmp_4];
      }
    }

    // End of Bias: '<S40>/Bias1'

    // RelationalOperator: '<S48>/Compare' incorporates:
    //   Abs: '<S40>/Abs2'
    //   Constant: '<S48>/Constant'

    for (i = 0; i < 9; i++) {
      interceptModel_B.Compare[i] = (fabs
        (interceptModel_B.rtb_VectorConcatenate_m[i]) >
        interceptModel_P.DirectionCosineMatrixtoRotation);
    }

    // End of RelationalOperator: '<S48>/Compare'

    // Logic: '<S40>/Logical Operator1' incorporates:
    //   RelationalOperator: '<S48>/Compare'

    tmp = interceptModel_B.Compare[0];
    for (i = 0; i < 8; i++) {
      tmp = (tmp || interceptModel_B.Compare[i + 1]);
    }

    // If: '<S36>/If' incorporates:
    //   Abs: '<S41>/Abs1'
    //   Bias: '<S41>/Bias'
    //   Constant: '<S50>/Constant'
    //   Logic: '<S40>/Logical Operator1'
    //   Product: '<S49>/Product'
    //   Product: '<S49>/Product1'
    //   Product: '<S49>/Product2'
    //   Product: '<S49>/Product3'
    //   Product: '<S49>/Product4'
    //   Product: '<S49>/Product5'
    //   RelationalOperator: '<S50>/Compare'
    //   Sum: '<S49>/Sum'

    if (fabs((((((interceptModel_B.VectorConcatenate[0] *
                  interceptModel_B.VectorConcatenate[4] *
                  interceptModel_B.VectorConcatenate[8] -
                  interceptModel_B.VectorConcatenate[0] *
                  interceptModel_B.VectorConcatenate[5] *
                  interceptModel_B.VectorConcatenate[7]) -
                 interceptModel_B.VectorConcatenate[1] *
                 interceptModel_B.VectorConcatenate[3] *
                 interceptModel_B.VectorConcatenate[8]) +
                interceptModel_B.VectorConcatenate[2] *
                interceptModel_B.VectorConcatenate[3] *
                interceptModel_B.VectorConcatenate[7]) +
               interceptModel_B.VectorConcatenate[1] *
               interceptModel_B.VectorConcatenate[5] *
               interceptModel_B.VectorConcatenate[6]) -
              interceptModel_B.VectorConcatenate[2] *
              interceptModel_B.VectorConcatenate[4] *
              interceptModel_B.VectorConcatenate[6]) +
             interceptModel_P.Bias_Bias) >
        interceptModel_P.DirectionCosineMatrixtoRotation) {
      // Outputs for IfAction SubSystem: '<S36>/If Not Proper' incorporates:
      //   ActionPort: '<S39>/Action Port'

      // If: '<S39>/If' incorporates:
      //   Constant: '<S39>/Constant'

      if (interceptModel_P.DirectionCosineMatrixtoRotati_c == 2) {
        // Outputs for IfAction SubSystem: '<S39>/Warning' incorporates:
        //   ActionPort: '<S47>/Action Port'

        // Assertion: '<S47>/Assertion' incorporates:
        //   Constant: '<S39>/Constant1'

        utAssert(interceptModel_P.Constant1_Value != 0.0);

        // End of Outputs for SubSystem: '<S39>/Warning'
      } else if (interceptModel_P.DirectionCosineMatrixtoRotati_c == 3) {
        // Outputs for IfAction SubSystem: '<S39>/Error' incorporates:
        //   ActionPort: '<S45>/Action Port'

        // Assertion: '<S45>/Assertion' incorporates:
        //   Constant: '<S39>/Constant1'

        utAssert(interceptModel_P.Constant1_Value != 0.0);

        // End of Outputs for SubSystem: '<S39>/Error'
      }

      // End of If: '<S39>/If'
      // End of Outputs for SubSystem: '<S36>/If Not Proper'
    } else if (tmp) {
      // Outputs for IfAction SubSystem: '<S36>/Else If Not Orthogonal' incorporates:
      //   ActionPort: '<S37>/Action Port'

      // If: '<S37>/If' incorporates:
      //   Constant: '<S37>/Constant'

      if (interceptModel_P.DirectionCosineMatrixtoRotati_c == 2) {
        // Outputs for IfAction SubSystem: '<S37>/Warning' incorporates:
        //   ActionPort: '<S44>/Action Port'

        // Assertion: '<S44>/Assertion' incorporates:
        //   Constant: '<S37>/Constant1'

        utAssert(interceptModel_P.Constant1_Value_e != 0.0);

        // End of Outputs for SubSystem: '<S37>/Warning'
      } else if (interceptModel_P.DirectionCosineMatrixtoRotati_c == 3) {
        // Outputs for IfAction SubSystem: '<S37>/Error' incorporates:
        //   ActionPort: '<S42>/Action Port'

        // Assertion: '<S42>/Assertion' incorporates:
        //   Constant: '<S37>/Constant1'

        utAssert(interceptModel_P.Constant1_Value_e != 0.0);

        // End of Outputs for SubSystem: '<S37>/Error'
      }

      // End of If: '<S37>/If'
      // End of Outputs for SubSystem: '<S36>/Else If Not Orthogonal'
    }

    // End of If: '<S36>/If'
    // End of Outputs for SubSystem: '<S27>/If Warning//Error'
  }

  // End of If: '<S27>/If1'

  // MATLABSystem: '<S15>/SourceBlock'
  tmp = uORB_read_step(interceptModel_DW.obj_e.orbMetadataObj,
                       &interceptModel_DW.obj_e.eventStructObj,
                       &interceptModel_B.r3, false, 1.0);

  // Outputs for Enabled SubSystem: '<S15>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S20>/Enable'

  // Start for MATLABSystem: '<S15>/SourceBlock'
  if (tmp) {
    // SignalConversion generated from: '<S20>/In1'
    interceptModel_B.In1_f = interceptModel_B.r3;
  }

  // End of Outputs for SubSystem: '<S15>/Enabled Subsystem'

  // Sum: '<S4>/Add1' incorporates:
  //   Constant: '<S4>/Constant'
  //   DataTypeConversion: '<S6>/Data Type Conversion5'

  interceptModel_B.Sum_b = interceptModel_P.Constant_Value_k -
    interceptModel_B.In1_f.target_x;

  // Saturate: '<S4>/Saturation'
  if (interceptModel_B.Sum_b > interceptModel_P.Saturation_UpperSat) {
    interceptModel_B.Sum_b = interceptModel_P.Saturation_UpperSat;
  } else if (interceptModel_B.Sum_b < interceptModel_P.Saturation_LowerSat) {
    interceptModel_B.Sum_b = interceptModel_P.Saturation_LowerSat;
  }

  // End of Saturate: '<S4>/Saturation'

  // RateTransition generated from: '<Root>/Rate Transition'
  tmp = interceptModel_M->Timing.RateInteraction.TID0_1;
  if (tmp) {
    // RateTransition generated from: '<Root>/Rate Transition'
    interceptModel_B.x_error_kp = interceptModel_DW.RateTransition_1_Buffer0;

    // RateTransition generated from: '<Root>/Rate Transition'
    interceptModel_B.int_reset_x = interceptModel_DW.RateTransition_4_Buffer0;
  }

  // Logic: '<S4>/Logical Operator' incorporates:
  //   DataTypeConversion: '<S4>/Data Type Conversion5'
  //   Logic: '<S5>/Logical Operator'

  rtb_LogicalOperator_tmp = ((interceptModel_B.int_reset_x != 0.0F) ||
    interceptModel_B.In1_f.tracker_status);

  // Delay: '<S9>/Delay_Integration' incorporates:
  //   Constant: '<S4>/Constant1'
  //   Logic: '<S4>/Logical Operator'

  interceptModel_DW.icLoad_p = ((rtb_LogicalOperator_tmp &&
    (interceptModel_PrevZCX.Delay_Integration_Reset_ZCE != POS_ZCSIG)) ||
    interceptModel_DW.icLoad_p);
  interceptModel_PrevZCX.Delay_Integration_Reset_ZCE = rtb_LogicalOperator_tmp;
  if (interceptModel_DW.icLoad_p) {
    interceptModel_DW.Delay_Integration_DSTATE =
      interceptModel_P.Constant1_Value_m;
  }

  // Sum: '<S4>/Add' incorporates:
  //   DataTypeConversion: '<S4>/Data Type Conversion2'
  //   Delay: '<S9>/Delay_Integration'
  //   Product: '<S4>/Product'

  interceptModel_B.Add = interceptModel_B.Sum_b * interceptModel_B.x_error_kp +
    interceptModel_DW.Delay_Integration_DSTATE;

  // RateTransition generated from: '<Root>/Rate Transition'
  if (tmp) {
    // RateTransition generated from: '<Root>/Rate Transition'
    interceptModel_B.roll_lim_x = interceptModel_DW.RateTransition_5_Buffer0;
  }

  // MinMax: '<S8>/MinMax1' incorporates:
  //   DataTypeConversion: '<S4>/Data Type Conversion6'
  //   MinMax: '<S12>/MinMax1'

  tmp_0 = rtIsNaN(static_cast<real_T>(interceptModel_B.roll_lim_x));
  if ((interceptModel_B.Add <= interceptModel_B.roll_lim_x) || tmp_0) {
    rtb_Sign_b = interceptModel_B.Add;
  } else {
    rtb_Sign_b = interceptModel_B.roll_lim_x;
  }

  // End of MinMax: '<S8>/MinMax1'

  // Gain: '<S4>/Gain1' incorporates:
  //   DataTypeConversion: '<S4>/Data Type Conversion6'

  interceptModel_B.Sum_a = interceptModel_P.Gain1_Gain_n *
    interceptModel_B.roll_lim_x;

  // MinMax: '<S8>/MinMax2'
  if ((rtb_Sign_b >= interceptModel_B.Sum_a) || rtIsNaN(interceptModel_B.Sum_a))
  {
    interceptModel_B.Sum_a = rtb_Sign_b;
  }

  // End of MinMax: '<S8>/MinMax2'

  // Sum: '<S5>/Add1' incorporates:
  //   Constant: '<S5>/Constant'
  //   DataTypeConversion: '<S6>/Data Type Conversion5'

  interceptModel_B.Product1_pd = interceptModel_P.Constant_Value_o5 -
    interceptModel_B.In1_f.target_y;

  // Saturate: '<S5>/Saturation'
  if (interceptModel_B.Product1_pd > interceptModel_P.Saturation_UpperSat_e) {
    interceptModel_B.Product1_pd = interceptModel_P.Saturation_UpperSat_e;
  } else if (interceptModel_B.Product1_pd <
             interceptModel_P.Saturation_LowerSat_j) {
    interceptModel_B.Product1_pd = interceptModel_P.Saturation_LowerSat_j;
  }

  // End of Saturate: '<S5>/Saturation'

  // Delay: '<S13>/Delay_Integration' incorporates:
  //   Constant: '<S5>/Constant1'
  //   Logic: '<S4>/Logical Operator'

  interceptModel_DW.icLoad_i = ((rtb_LogicalOperator_tmp &&
    (interceptModel_PrevZCX.Delay_Integration_Reset_ZCE_p != POS_ZCSIG)) ||
    interceptModel_DW.icLoad_i);
  interceptModel_PrevZCX.Delay_Integration_Reset_ZCE_p = rtb_LogicalOperator_tmp;
  if (interceptModel_DW.icLoad_i) {
    interceptModel_DW.Delay_Integration_DSTATE_h =
      interceptModel_P.Constant1_Value_a;
  }

  // Sum: '<S5>/Add' incorporates:
  //   DataTypeConversion: '<S5>/Data Type Conversion2'
  //   Delay: '<S13>/Delay_Integration'
  //   Product: '<S5>/Product'

  interceptModel_B.MinMax1_e0 = interceptModel_B.Product1_pd *
    interceptModel_B.x_error_kp + interceptModel_DW.Delay_Integration_DSTATE_h;

  // MinMax: '<S12>/MinMax1' incorporates:
  //   DataTypeConversion: '<S5>/Data Type Conversion6'

  if ((interceptModel_B.MinMax1_e0 <= interceptModel_B.roll_lim_x) || tmp_0) {
    rtb_Sign_b = interceptModel_B.MinMax1_e0;
  } else {
    rtb_Sign_b = interceptModel_B.roll_lim_x;
  }

  // Gain: '<S5>/Gain1' incorporates:
  //   DataTypeConversion: '<S5>/Data Type Conversion6'

  interceptModel_B.Switch3 = interceptModel_P.Gain1_Gain_o *
    interceptModel_B.roll_lim_x;

  // MinMax: '<S12>/MinMax2'
  if ((rtb_Sign_b >= interceptModel_B.Switch3) || rtIsNaN
      (interceptModel_B.Switch3)) {
    interceptModel_B.Switch3 = rtb_Sign_b;
  }

  // End of MinMax: '<S12>/MinMax2'

  // BusAssignment: '<Root>/Bus Assignment' incorporates:
  //   Constant: '<S1>/Constant'

  interceptModel_B.BusAssignment = interceptModel_P.Constant_Value_d;

  // BusAssignment: '<Root>/Bus Assignment' incorporates:
  //   DataTypeConversion: '<Root>/Data Type Conversion2'
  //   DataTypeConversion: '<Root>/Data Type Conversion4'

  interceptModel_B.BusAssignment.target_roll = static_cast<real32_T>
    (interceptModel_B.Sum_a);
  interceptModel_B.BusAssignment.target_pitch = static_cast<real32_T>
    (interceptModel_B.Switch3);

  // MATLABSystem: '<S2>/SinkBlock' incorporates:
  //   BusAssignment: '<Root>/Bus Assignment'

  uORB_write_step(interceptModel_DW.obj_ei.orbMetadataObj,
                  &interceptModel_DW.obj_ei.orbAdvertiseObj,
                  &interceptModel_B.BusAssignment);

  // Sum: '<S12>/Sum'
  rtb_Sign_b = interceptModel_B.MinMax1_e0 - interceptModel_B.Switch3;

  // Signum: '<S12>/Sign'
  if (rtIsNaN(rtb_Sign_b)) {
    interceptModel_B.Switch3 = (rtNaN);
  } else if (rtb_Sign_b < 0.0) {
    interceptModel_B.Switch3 = -1.0;
  } else {
    interceptModel_B.Switch3 = (rtb_Sign_b > 0.0);
  }

  // End of Signum: '<S12>/Sign'

  // RateTransition generated from: '<Root>/Rate Transition'
  if (tmp) {
    // RateTransition generated from: '<Root>/Rate Transition'
    interceptModel_B.x_int_lim = interceptModel_DW.RateTransition_3_Buffer0;
  }

  // MinMax: '<S11>/MinMax1' incorporates:
  //   DataTypeConversion: '<S5>/Data Type Conversion4'
  //   MinMax: '<S7>/MinMax1'

  tmp_0 = rtIsNaN(static_cast<real_T>(interceptModel_B.x_int_lim));
  if ((interceptModel_B.Product1_pd <= interceptModel_B.x_int_lim) || tmp_0) {
    rtb_Sign_b = interceptModel_B.Product1_pd;
  } else {
    rtb_Sign_b = interceptModel_B.x_int_lim;
  }

  // End of MinMax: '<S11>/MinMax1'

  // Gain: '<S5>/Gain' incorporates:
  //   DataTypeConversion: '<S5>/Data Type Conversion4'

  interceptModel_B.MinMax1_e0 = interceptModel_P.Gain_Gain_g *
    interceptModel_B.x_int_lim;

  // MinMax: '<S11>/MinMax2'
  if ((rtb_Sign_b >= interceptModel_B.MinMax1_e0) || rtIsNaN
      (interceptModel_B.MinMax1_e0)) {
    interceptModel_B.MinMax1_e0 = rtb_Sign_b;
  }

  // Sum: '<S11>/Sum' incorporates:
  //   MinMax: '<S11>/MinMax2'

  rtb_Sign_b = interceptModel_B.Product1_pd - interceptModel_B.MinMax1_e0;

  // Signum: '<S11>/Sign'
  if (rtIsNaN(rtb_Sign_b)) {
    rtb_Sign_b = (rtNaN);
  } else if (rtb_Sign_b < 0.0) {
    rtb_Sign_b = -1.0;
  } else {
    rtb_Sign_b = (rtb_Sign_b > 0.0);
  }

  // End of Signum: '<S11>/Sign'

  // RateTransition generated from: '<Root>/Rate Transition'
  if (tmp) {
    // RateTransition generated from: '<Root>/Rate Transition'
    interceptModel_B.x_error_ki = interceptModel_DW.RateTransition_2_Buffer0;
  }

  // Product: '<S5>/Product1' incorporates:
  //   DataTypeConversion: '<S5>/Data Type Conversion3'

  interceptModel_B.Product1_pd *= interceptModel_B.x_error_ki;

  // Switch: '<S14>/Switch3' incorporates:
  //   Constant: '<S14>/Constant1'
  //   Constant: '<S14>/Constant2'
  //   Constant: '<S14>/Constant3'
  //   Logic: '<S14>/Logical Operator1'
  //   Logic: '<S14>/Logical Operator2'
  //   MinMax: '<S14>/MinMax1'
  //   MinMax: '<S14>/MinMax2'
  //   RelationalOperator: '<S14>/Relational Operator1'
  //   RelationalOperator: '<S14>/Relational Operator2'
  //   RelationalOperator: '<S14>/Relational Operator3'
  //   RelationalOperator: '<S14>/Relational Operator4'
  //   Sum: '<S14>/Add1'
  //   Sum: '<S14>/Add2'
  //   Switch: '<S14>/Switch2'

  if ((interceptModel_B.Switch3 == interceptModel_P.Constant1_Value_ez) ||
      (rtb_Sign_b == interceptModel_P.Constant1_Value_ez)) {
    // MinMax: '<S14>/MinMax1' incorporates:
    //   Constant: '<S14>/Constant3'

    if ((!(interceptModel_B.Product1_pd >= interceptModel_P.Constant3_Value_a)) &&
        (!rtIsNaN(interceptModel_P.Constant3_Value_a))) {
      interceptModel_B.Product1_pd = interceptModel_P.Constant3_Value_a;
    }

    interceptModel_B.Switch3 = interceptModel_P.Constant3_Value_a +
      interceptModel_B.Product1_pd;
  } else {
    if ((interceptModel_B.Switch3 == interceptModel_P.Constant2_Value_d) ||
        (rtb_Sign_b == interceptModel_P.Constant2_Value_d)) {
      // Switch: '<S14>/Switch2' incorporates:
      //   Constant: '<S14>/Constant3'

      rtb_Sign_b = interceptModel_P.Constant3_Value_a;
    } else if ((interceptModel_B.Product1_pd >=
                interceptModel_P.Constant3_Value_a) || rtIsNaN
               (interceptModel_P.Constant3_Value_a)) {
      // MinMax: '<S14>/MinMax1' incorporates:
      //   Switch: '<S14>/Switch2'

      rtb_Sign_b = interceptModel_B.Product1_pd;
    } else {
      // Switch: '<S14>/Switch2' incorporates:
      //   Constant: '<S14>/Constant3'
      //   MinMax: '<S14>/MinMax1'

      rtb_Sign_b = interceptModel_P.Constant3_Value_a;
    }

    // MinMax: '<S14>/MinMax2' incorporates:
    //   Constant: '<S14>/Constant3'
    //   MinMax: '<S14>/MinMax1'

    if ((!(interceptModel_B.Product1_pd <= interceptModel_P.Constant3_Value_a)) &&
        (!rtIsNaN(interceptModel_P.Constant3_Value_a))) {
      interceptModel_B.Product1_pd = interceptModel_P.Constant3_Value_a;
    }

    interceptModel_B.Switch3 = interceptModel_B.Product1_pd + rtb_Sign_b;
  }

  // End of Switch: '<S14>/Switch3'

  // Product: '<S13>/Product1' incorporates:
  //   Constant: '<S13>/Constant'

  interceptModel_B.Product1_pd = interceptModel_B.Switch3 *
    interceptModel_P.Constant_Value_ox;

  // Product: '<S4>/Product1' incorporates:
  //   DataTypeConversion: '<S4>/Data Type Conversion3'

  interceptModel_B.Switch3 = interceptModel_B.Sum_b *
    interceptModel_B.x_error_ki;

  // Sum: '<S8>/Sum'
  rtb_Sign_b = interceptModel_B.Add - interceptModel_B.Sum_a;

  // Signum: '<S8>/Sign'
  if (rtIsNaN(rtb_Sign_b)) {
    interceptModel_B.Sum_a = (rtNaN);
  } else if (rtb_Sign_b < 0.0) {
    interceptModel_B.Sum_a = -1.0;
  } else {
    interceptModel_B.Sum_a = (rtb_Sign_b > 0.0);
  }

  // End of Signum: '<S8>/Sign'

  // MinMax: '<S7>/MinMax1' incorporates:
  //   DataTypeConversion: '<S4>/Data Type Conversion4'

  if ((interceptModel_B.Sum_b <= interceptModel_B.x_int_lim) || tmp_0) {
    rtb_Sign_b = interceptModel_B.Sum_b;
  } else {
    rtb_Sign_b = interceptModel_B.x_int_lim;
  }

  // Gain: '<S4>/Gain' incorporates:
  //   DataTypeConversion: '<S4>/Data Type Conversion4'

  interceptModel_B.MinMax1_e0 = interceptModel_P.Gain_Gain_jy *
    interceptModel_B.x_int_lim;

  // MinMax: '<S7>/MinMax2'
  if ((rtb_Sign_b >= interceptModel_B.MinMax1_e0) || rtIsNaN
      (interceptModel_B.MinMax1_e0)) {
    interceptModel_B.MinMax1_e0 = rtb_Sign_b;
  }

  // Sum: '<S7>/Sum' incorporates:
  //   MinMax: '<S7>/MinMax2'

  rtb_Sign_b = interceptModel_B.Sum_b - interceptModel_B.MinMax1_e0;

  // Signum: '<S7>/Sign'
  if (rtIsNaN(rtb_Sign_b)) {
    interceptModel_B.Sum_b = (rtNaN);
  } else if (rtb_Sign_b < 0.0) {
    interceptModel_B.Sum_b = -1.0;
  } else {
    interceptModel_B.Sum_b = (rtb_Sign_b > 0.0);
  }

  // End of Signum: '<S7>/Sign'

  // Switch: '<S10>/Switch3' incorporates:
  //   Constant: '<S10>/Constant1'
  //   Constant: '<S10>/Constant2'
  //   Constant: '<S10>/Constant3'
  //   Logic: '<S10>/Logical Operator1'
  //   Logic: '<S10>/Logical Operator2'
  //   MinMax: '<S10>/MinMax1'
  //   MinMax: '<S10>/MinMax2'
  //   RelationalOperator: '<S10>/Relational Operator1'
  //   RelationalOperator: '<S10>/Relational Operator2'
  //   RelationalOperator: '<S10>/Relational Operator3'
  //   RelationalOperator: '<S10>/Relational Operator4'
  //   Sum: '<S10>/Add1'
  //   Sum: '<S10>/Add2'
  //   Switch: '<S10>/Switch2'

  if ((interceptModel_B.Sum_a == interceptModel_P.Constant1_Value_ev) ||
      (interceptModel_B.Sum_b == interceptModel_P.Constant1_Value_ev)) {
    // MinMax: '<S10>/MinMax1' incorporates:
    //   Constant: '<S10>/Constant3'

    if ((!(interceptModel_B.Switch3 >= interceptModel_P.Constant3_Value_i)) && (
         !rtIsNaN(interceptModel_P.Constant3_Value_i))) {
      interceptModel_B.Switch3 = interceptModel_P.Constant3_Value_i;
    }

    interceptModel_B.Switch3 += interceptModel_P.Constant3_Value_i;
  } else {
    if ((interceptModel_B.Sum_a == interceptModel_P.Constant2_Value) ||
        (interceptModel_B.Sum_b == interceptModel_P.Constant2_Value)) {
      // Switch: '<S10>/Switch2' incorporates:
      //   Constant: '<S10>/Constant3'

      rtb_Sign_b = interceptModel_P.Constant3_Value_i;
    } else if ((interceptModel_B.Switch3 >= interceptModel_P.Constant3_Value_i) ||
               rtIsNaN(interceptModel_P.Constant3_Value_i)) {
      // MinMax: '<S10>/MinMax1' incorporates:
      //   Switch: '<S10>/Switch2'

      rtb_Sign_b = interceptModel_B.Switch3;
    } else {
      // Switch: '<S10>/Switch2' incorporates:
      //   Constant: '<S10>/Constant3'
      //   MinMax: '<S10>/MinMax1'

      rtb_Sign_b = interceptModel_P.Constant3_Value_i;
    }

    // MinMax: '<S10>/MinMax2' incorporates:
    //   Constant: '<S10>/Constant3'
    //   MinMax: '<S10>/MinMax1'

    if ((!(interceptModel_B.Switch3 <= interceptModel_P.Constant3_Value_i)) && (
         !rtIsNaN(interceptModel_P.Constant3_Value_i))) {
      interceptModel_B.Switch3 = interceptModel_P.Constant3_Value_i;
    }

    interceptModel_B.Switch3 += rtb_Sign_b;
  }

  // End of Switch: '<S10>/Switch3'

  // MATLABSystem: '<S19>/SourceBlock'
  uORB_read_step(interceptModel_DW.obj.orbMetadataObj,
                 &interceptModel_DW.obj.eventStructObj, &interceptModel_B.r,
                 false, 1.0);

  // MATLABSystem: '<S16>/SourceBlock'
  uORB_read_step(interceptModel_DW.obj_h.orbMetadataObj,
                 &interceptModel_DW.obj_h.eventStructObj, &interceptModel_B.r2,
                 false, 1.0);

  // Update for Delay: '<S9>/Delay_Integration' incorporates:
  //   Constant: '<S9>/Constant'
  //   Product: '<S9>/Product1'
  //   Sum: '<S9>/Add1'

  interceptModel_DW.icLoad_p = false;
  interceptModel_DW.Delay_Integration_DSTATE += interceptModel_B.Switch3 *
    interceptModel_P.Constant_Value_fl;

  // Update for Delay: '<S13>/Delay_Integration' incorporates:
  //   Sum: '<S13>/Add1'

  interceptModel_DW.icLoad_i = false;
  interceptModel_DW.Delay_Integration_DSTATE_h += interceptModel_B.Product1_pd;
}

// Model step function for TID1
void interceptModel_step1(void)        // Sample time: [0.5s, 0.0s]
{
  real32_T rtb_int_reset_x;
  real32_T rtb_roll_lim_x;
  real32_T rtb_x_error_ki;
  real32_T rtb_x_error_kp;
  real32_T rtb_x_int_lim;
  boolean_T b_varargout_2;

  // MATLABSystem: '<S3>/Read Parameter'
  b_varargout_2 = MW_Param_Step(interceptModel_DW.obj_oq.MW_PARAMHANDLE,
    MW_SINGLE, &rtb_x_error_kp);
  if (b_varargout_2) {
    rtb_x_error_kp = 0.0F;
  }

  // End of MATLABSystem: '<S3>/Read Parameter'

  // RateTransition generated from: '<Root>/Rate Transition'
  interceptModel_DW.RateTransition_1_Buffer0 = rtb_x_error_kp;

  // MATLABSystem: '<S3>/Read Parameter1'
  b_varargout_2 = MW_Param_Step(interceptModel_DW.obj_d.MW_PARAMHANDLE,
    MW_SINGLE, &rtb_x_error_ki);
  if (b_varargout_2) {
    rtb_x_error_ki = 0.0F;
  }

  // End of MATLABSystem: '<S3>/Read Parameter1'

  // RateTransition generated from: '<Root>/Rate Transition'
  interceptModel_DW.RateTransition_2_Buffer0 = rtb_x_error_ki;

  // MATLABSystem: '<S3>/Read Parameter2'
  b_varargout_2 = MW_Param_Step(interceptModel_DW.obj_lm.MW_PARAMHANDLE,
    MW_SINGLE, &rtb_x_int_lim);
  if (b_varargout_2) {
    rtb_x_int_lim = 0.0F;
  }

  // End of MATLABSystem: '<S3>/Read Parameter2'

  // RateTransition generated from: '<Root>/Rate Transition'
  interceptModel_DW.RateTransition_3_Buffer0 = rtb_x_int_lim;

  // MATLABSystem: '<S3>/Read Parameter3'
  b_varargout_2 = MW_Param_Step(interceptModel_DW.obj_n.MW_PARAMHANDLE,
    MW_SINGLE, &rtb_int_reset_x);
  if (b_varargout_2) {
    rtb_int_reset_x = 0.0F;
  }

  // End of MATLABSystem: '<S3>/Read Parameter3'

  // RateTransition generated from: '<Root>/Rate Transition'
  interceptModel_DW.RateTransition_4_Buffer0 = rtb_int_reset_x;

  // MATLABSystem: '<S3>/Read Parameter4'
  b_varargout_2 = MW_Param_Step(interceptModel_DW.obj_p.MW_PARAMHANDLE,
    MW_SINGLE, &rtb_roll_lim_x);
  if (b_varargout_2) {
    rtb_roll_lim_x = 0.0F;
  }

  // End of MATLABSystem: '<S3>/Read Parameter4'

  // RateTransition generated from: '<Root>/Rate Transition'
  interceptModel_DW.RateTransition_5_Buffer0 = rtb_roll_lim_x;

  // MATLABSystem: '<S3>/Read Parameter5'
  MW_Param_Step(interceptModel_DW.obj_m.MW_PARAMHANDLE, MW_SINGLE,
                &rtb_roll_lim_x);

  // MATLABSystem: '<S3>/Read Parameter6'
  MW_Param_Step(interceptModel_DW.obj_l.MW_PARAMHANDLE, MW_SINGLE,
                &rtb_roll_lim_x);

  // MATLABSystem: '<S3>/Read Parameter7'
  MW_Param_Step(interceptModel_DW.obj_k.MW_PARAMHANDLE, MW_SINGLE,
                &rtb_roll_lim_x);

  // MATLABSystem: '<S3>/Read Parameter8'
  MW_Param_Step(interceptModel_DW.obj_ov.MW_PARAMHANDLE, MW_SINGLE,
                &rtb_roll_lim_x);

  // MATLABSystem: '<S3>/Read Parameter9'
  MW_Param_Step(interceptModel_DW.obj_c.MW_PARAMHANDLE, MW_SINGLE,
                &rtb_roll_lim_x);
}

// Use this function only if you need to maintain compatibility with an existing static main program.
void interceptModel_step(int_T tid)
{
  switch (tid) {
   case 0 :
    interceptModel_step0();
    break;

   case 1 :
    interceptModel_step1();
    break;

   default :
    // do nothing
    break;
  }
}

// Model initialize function
void interceptModel_initialize(void)
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  {
    static const char_T ParameterNameStr[11] = "X_ERROR_KP";
    static const char_T ParameterNameStr_0[11] = "X_ERROR_KI";
    static const char_T ParameterNameStr_1[10] = "X_INT_LIM";
    static const char_T ParameterNameStr_2[12] = "INT_RESET_X";
    static const char_T ParameterNameStr_3[11] = "ROLL_LIM_X";
    static const char_T ParameterNameStr_4[11] = "Y_ERROR_KP";
    static const char_T ParameterNameStr_5[11] = "Y_ERROR_KI";
    static const char_T ParameterNameStr_6[10] = "Y_INT_LIM";
    static const char_T ParameterNameStr_7[12] = "INT_RESET_Y";
    static const char_T ParameterNameStr_8[11] = "ROLL_LIM_Y";

    // Start for RateTransition generated from: '<Root>/Rate Transition'
    interceptModel_B.x_error_kp =
      interceptModel_P.RateTransition_1_InitialConditi;

    // Start for RateTransition generated from: '<Root>/Rate Transition'
    interceptModel_B.int_reset_x =
      interceptModel_P.RateTransition_4_InitialConditi;

    // Start for RateTransition generated from: '<Root>/Rate Transition'
    interceptModel_B.roll_lim_x =
      interceptModel_P.RateTransition_5_InitialConditi;

    // Start for RateTransition generated from: '<Root>/Rate Transition'
    interceptModel_B.x_int_lim =
      interceptModel_P.RateTransition_3_InitialConditi;

    // Start for RateTransition generated from: '<Root>/Rate Transition'
    interceptModel_B.x_error_ki =
      interceptModel_P.RateTransition_2_InitialConditi;
    interceptModel_PrevZCX.Delay_Integration_Reset_ZCE = POS_ZCSIG;
    interceptModel_PrevZCX.Delay_Integration_Reset_ZCE_p = POS_ZCSIG;

    // InitializeConditions for RateTransition generated from: '<Root>/Rate Transition' 
    interceptModel_DW.RateTransition_1_Buffer0 =
      interceptModel_P.RateTransition_1_InitialConditi;

    // InitializeConditions for RateTransition generated from: '<Root>/Rate Transition' 
    interceptModel_DW.RateTransition_4_Buffer0 =
      interceptModel_P.RateTransition_4_InitialConditi;

    // InitializeConditions for Delay: '<S9>/Delay_Integration'
    interceptModel_DW.icLoad_p = true;

    // InitializeConditions for RateTransition generated from: '<Root>/Rate Transition' 
    interceptModel_DW.RateTransition_5_Buffer0 =
      interceptModel_P.RateTransition_5_InitialConditi;

    // InitializeConditions for Delay: '<S13>/Delay_Integration'
    interceptModel_DW.icLoad_i = true;

    // InitializeConditions for RateTransition generated from: '<Root>/Rate Transition' 
    interceptModel_DW.RateTransition_3_Buffer0 =
      interceptModel_P.RateTransition_3_InitialConditi;

    // InitializeConditions for RateTransition generated from: '<Root>/Rate Transition' 
    interceptModel_DW.RateTransition_2_Buffer0 =
      interceptModel_P.RateTransition_2_InitialConditi;

    // SystemInitialize for Enabled SubSystem: '<S15>/Enabled Subsystem'
    // SystemInitialize for SignalConversion generated from: '<S20>/In1' incorporates:
    //   Outport: '<S20>/Out1'

    interceptModel_B.In1_f = interceptModel_P.Out1_Y0_n;

    // End of SystemInitialize for SubSystem: '<S15>/Enabled Subsystem'

    // SystemInitialize for Enabled SubSystem: '<S18>/Enabled Subsystem'
    // SystemInitialize for SignalConversion generated from: '<S64>/In1' incorporates:
    //   Outport: '<S64>/Out1'

    interceptModel_B.In1 = interceptModel_P.Out1_Y0_p;

    // End of SystemInitialize for SubSystem: '<S18>/Enabled Subsystem'

    // Start for MATLABSystem: '<S18>/SourceBlock'
    interceptModel_DW.obj_o.matlabCodegenIsDeleted = false;
    interceptModel_DW.obj_o.isInitialized = 1;
    interceptModel_DW.obj_o.orbMetadataObj = ORB_ID(vehicle_attitude);
    uORB_read_initialize(interceptModel_DW.obj_o.orbMetadataObj,
                         &interceptModel_DW.obj_o.eventStructObj);
    interceptModel_DW.obj_o.isSetupComplete = true;

    // Start for MATLABSystem: '<S15>/SourceBlock'
    interceptModel_DW.obj_e.matlabCodegenIsDeleted = false;
    interceptModel_DW.obj_e.isInitialized = 1;
    interceptModel_DW.obj_e.orbMetadataObj = ORB_ID(target_location);
    uORB_read_initialize(interceptModel_DW.obj_e.orbMetadataObj,
                         &interceptModel_DW.obj_e.eventStructObj);
    interceptModel_DW.obj_e.isSetupComplete = true;

    // Start for MATLABSystem: '<S2>/SinkBlock' incorporates:
    //   BusAssignment: '<Root>/Bus Assignment'

    interceptModel_DW.obj_ei.matlabCodegenIsDeleted = false;
    interceptModel_DW.obj_ei.isInitialized = 1;
    interceptModel_DW.obj_ei.orbMetadataObj = ORB_ID(intercept_roll_pitch);
    uORB_write_initialize(interceptModel_DW.obj_ei.orbMetadataObj,
                          &interceptModel_DW.obj_ei.orbAdvertiseObj,
                          &interceptModel_B.BusAssignment, 1);
    interceptModel_DW.obj_ei.isSetupComplete = true;

    // Start for MATLABSystem: '<S19>/SourceBlock'
    interceptModel_DW.obj.matlabCodegenIsDeleted = false;
    interceptModel_DW.obj.isInitialized = 1;
    interceptModel_DW.obj.orbMetadataObj = ORB_ID(vehicle_local_position);
    uORB_read_initialize(interceptModel_DW.obj.orbMetadataObj,
                         &interceptModel_DW.obj.eventStructObj);
    interceptModel_DW.obj.isSetupComplete = true;

    // Start for MATLABSystem: '<S16>/SourceBlock'
    interceptModel_DW.obj_h.matlabCodegenIsDeleted = false;
    interceptModel_DW.obj_h.isInitialized = 1;
    interceptModel_DW.obj_h.orbMetadataObj = ORB_ID(airspeed_validated);
    uORB_read_initialize(interceptModel_DW.obj_h.orbMetadataObj,
                         &interceptModel_DW.obj_h.eventStructObj);
    interceptModel_DW.obj_h.isSetupComplete = true;

    // Start for MATLABSystem: '<S3>/Read Parameter'
    interceptModel_DW.obj_oq.matlabCodegenIsDeleted = false;
    interceptModel_DW.obj_oq.isInitialized = 1;
    interceptModel_DW.obj_oq.MW_PARAMHANDLE = MW_Init_Param(&ParameterNameStr[0],
      true, 500.0);
    interceptModel_DW.obj_oq.isSetupComplete = true;

    // Start for MATLABSystem: '<S3>/Read Parameter1'
    interceptModel_DW.obj_d.matlabCodegenIsDeleted = false;
    interceptModel_DW.obj_d.isInitialized = 1;
    interceptModel_DW.obj_d.MW_PARAMHANDLE = MW_Init_Param(&ParameterNameStr_0[0],
      true, 500.0);
    interceptModel_DW.obj_d.isSetupComplete = true;

    // Start for MATLABSystem: '<S3>/Read Parameter2'
    interceptModel_DW.obj_lm.matlabCodegenIsDeleted = false;
    interceptModel_DW.obj_lm.isInitialized = 1;
    interceptModel_DW.obj_lm.MW_PARAMHANDLE = MW_Init_Param(&ParameterNameStr_1
      [0], true, 500.0);
    interceptModel_DW.obj_lm.isSetupComplete = true;

    // Start for MATLABSystem: '<S3>/Read Parameter3'
    interceptModel_DW.obj_n.matlabCodegenIsDeleted = false;
    interceptModel_DW.obj_n.isInitialized = 1;
    interceptModel_DW.obj_n.MW_PARAMHANDLE = MW_Init_Param(&ParameterNameStr_2[0],
      true, 500.0);
    interceptModel_DW.obj_n.isSetupComplete = true;

    // Start for MATLABSystem: '<S3>/Read Parameter4'
    interceptModel_DW.obj_p.matlabCodegenIsDeleted = false;
    interceptModel_DW.obj_p.isInitialized = 1;
    interceptModel_DW.obj_p.MW_PARAMHANDLE = MW_Init_Param(&ParameterNameStr_3[0],
      true, 500.0);
    interceptModel_DW.obj_p.isSetupComplete = true;

    // Start for MATLABSystem: '<S3>/Read Parameter5'
    interceptModel_DW.obj_m.matlabCodegenIsDeleted = false;
    interceptModel_DW.obj_m.isInitialized = 1;
    interceptModel_DW.obj_m.MW_PARAMHANDLE = MW_Init_Param(&ParameterNameStr_4[0],
      true, 500.0);
    interceptModel_DW.obj_m.isSetupComplete = true;

    // Start for MATLABSystem: '<S3>/Read Parameter6'
    interceptModel_DW.obj_l.matlabCodegenIsDeleted = false;
    interceptModel_DW.obj_l.isInitialized = 1;
    interceptModel_DW.obj_l.MW_PARAMHANDLE = MW_Init_Param(&ParameterNameStr_5[0],
      true, 500.0);
    interceptModel_DW.obj_l.isSetupComplete = true;

    // Start for MATLABSystem: '<S3>/Read Parameter7'
    interceptModel_DW.obj_k.matlabCodegenIsDeleted = false;
    interceptModel_DW.obj_k.isInitialized = 1;
    interceptModel_DW.obj_k.MW_PARAMHANDLE = MW_Init_Param(&ParameterNameStr_6[0],
      true, 500.0);
    interceptModel_DW.obj_k.isSetupComplete = true;

    // Start for MATLABSystem: '<S3>/Read Parameter8'
    interceptModel_DW.obj_ov.matlabCodegenIsDeleted = false;
    interceptModel_DW.obj_ov.isInitialized = 1;
    interceptModel_DW.obj_ov.MW_PARAMHANDLE = MW_Init_Param(&ParameterNameStr_7
      [0], true, 500.0);
    interceptModel_DW.obj_ov.isSetupComplete = true;

    // Start for MATLABSystem: '<S3>/Read Parameter9'
    interceptModel_DW.obj_c.matlabCodegenIsDeleted = false;
    interceptModel_DW.obj_c.isInitialized = 1;
    interceptModel_DW.obj_c.MW_PARAMHANDLE = MW_Init_Param(&ParameterNameStr_8[0],
      true, 500.0);
    interceptModel_DW.obj_c.isSetupComplete = true;
  }
}

// Model terminate function
void interceptModel_terminate(void)
{
  // Terminate for MATLABSystem: '<S18>/SourceBlock'
  if (!interceptModel_DW.obj_o.matlabCodegenIsDeleted) {
    interceptModel_DW.obj_o.matlabCodegenIsDeleted = true;
    if ((interceptModel_DW.obj_o.isInitialized == 1) &&
        interceptModel_DW.obj_o.isSetupComplete) {
      uORB_read_terminate(&interceptModel_DW.obj_o.eventStructObj);
    }
  }

  // End of Terminate for MATLABSystem: '<S18>/SourceBlock'

  // Terminate for MATLABSystem: '<S15>/SourceBlock'
  if (!interceptModel_DW.obj_e.matlabCodegenIsDeleted) {
    interceptModel_DW.obj_e.matlabCodegenIsDeleted = true;
    if ((interceptModel_DW.obj_e.isInitialized == 1) &&
        interceptModel_DW.obj_e.isSetupComplete) {
      uORB_read_terminate(&interceptModel_DW.obj_e.eventStructObj);
    }
  }

  // End of Terminate for MATLABSystem: '<S15>/SourceBlock'

  // Terminate for MATLABSystem: '<S2>/SinkBlock'
  if (!interceptModel_DW.obj_ei.matlabCodegenIsDeleted) {
    interceptModel_DW.obj_ei.matlabCodegenIsDeleted = true;
    if ((interceptModel_DW.obj_ei.isInitialized == 1) &&
        interceptModel_DW.obj_ei.isSetupComplete) {
      uORB_write_terminate(&interceptModel_DW.obj_ei.orbAdvertiseObj);
    }
  }

  // End of Terminate for MATLABSystem: '<S2>/SinkBlock'

  // Terminate for MATLABSystem: '<S19>/SourceBlock'
  if (!interceptModel_DW.obj.matlabCodegenIsDeleted) {
    interceptModel_DW.obj.matlabCodegenIsDeleted = true;
    if ((interceptModel_DW.obj.isInitialized == 1) &&
        interceptModel_DW.obj.isSetupComplete) {
      uORB_read_terminate(&interceptModel_DW.obj.eventStructObj);
    }
  }

  // End of Terminate for MATLABSystem: '<S19>/SourceBlock'

  // Terminate for MATLABSystem: '<S16>/SourceBlock'
  if (!interceptModel_DW.obj_h.matlabCodegenIsDeleted) {
    interceptModel_DW.obj_h.matlabCodegenIsDeleted = true;
    if ((interceptModel_DW.obj_h.isInitialized == 1) &&
        interceptModel_DW.obj_h.isSetupComplete) {
      uORB_read_terminate(&interceptModel_DW.obj_h.eventStructObj);
    }
  }

  // End of Terminate for MATLABSystem: '<S16>/SourceBlock'

  // Terminate for MATLABSystem: '<S3>/Read Parameter'
  if (!interceptModel_DW.obj_oq.matlabCodegenIsDeleted) {
    interceptModel_DW.obj_oq.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S3>/Read Parameter'

  // Terminate for MATLABSystem: '<S3>/Read Parameter1'
  if (!interceptModel_DW.obj_d.matlabCodegenIsDeleted) {
    interceptModel_DW.obj_d.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S3>/Read Parameter1'

  // Terminate for MATLABSystem: '<S3>/Read Parameter2'
  if (!interceptModel_DW.obj_lm.matlabCodegenIsDeleted) {
    interceptModel_DW.obj_lm.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S3>/Read Parameter2'

  // Terminate for MATLABSystem: '<S3>/Read Parameter3'
  if (!interceptModel_DW.obj_n.matlabCodegenIsDeleted) {
    interceptModel_DW.obj_n.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S3>/Read Parameter3'

  // Terminate for MATLABSystem: '<S3>/Read Parameter4'
  if (!interceptModel_DW.obj_p.matlabCodegenIsDeleted) {
    interceptModel_DW.obj_p.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S3>/Read Parameter4'

  // Terminate for MATLABSystem: '<S3>/Read Parameter5'
  if (!interceptModel_DW.obj_m.matlabCodegenIsDeleted) {
    interceptModel_DW.obj_m.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S3>/Read Parameter5'

  // Terminate for MATLABSystem: '<S3>/Read Parameter6'
  if (!interceptModel_DW.obj_l.matlabCodegenIsDeleted) {
    interceptModel_DW.obj_l.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S3>/Read Parameter6'

  // Terminate for MATLABSystem: '<S3>/Read Parameter7'
  if (!interceptModel_DW.obj_k.matlabCodegenIsDeleted) {
    interceptModel_DW.obj_k.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S3>/Read Parameter7'

  // Terminate for MATLABSystem: '<S3>/Read Parameter8'
  if (!interceptModel_DW.obj_ov.matlabCodegenIsDeleted) {
    interceptModel_DW.obj_ov.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S3>/Read Parameter8'

  // Terminate for MATLABSystem: '<S3>/Read Parameter9'
  if (!interceptModel_DW.obj_c.matlabCodegenIsDeleted) {
    interceptModel_DW.obj_c.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S3>/Read Parameter9'
}

uint8_T &RT_MODEL_interceptModel_T::TaskCounter(int32_T idx)
{
  return (Timing.TaskCounters.TID[(idx)]);
}

boolean_T RT_MODEL_interceptModel_T::StepTask(int32_T idx) const
{
  return (Timing.TaskCounters.TID[(idx)] == 0);
}

const char_T* RT_MODEL_interceptModel_T::getErrorStatus() const
{
  return (errorStatus);
}

void RT_MODEL_interceptModel_T::setErrorStatus(const char_T* const volatile
  aErrorStatus)
{
  (errorStatus = aErrorStatus);
}

//
// File trailer for generated code.
//
// [EOF]
//
