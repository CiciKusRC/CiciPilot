//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: interceptModel.cpp
//
// Code generated for Simulink model 'interceptModel'.
//
// Model version                  : 1.95
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Wed Aug  6 21:21:32 2025
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "interceptModel.h"
#include "rtwtypes.h"
#include "interceptModel_types.h"
#include "interceptModel_private.h"
#include <math.h>
#include "rt_assert.h"

extern "C"
{

#include "rt_nonfinite.h"

}

#include <string.h>
#include "zero_crossing_types.h"

const real_T interceptModel_period = 0.01;

// Block signals (default storage)
B_interceptModel_T interceptModel_B;

// Block states (default storage)
DW_interceptModel_T interceptModel_DW;

// Previous zero-crossings (trigger) states
PrevZCX_interceptModel_T interceptModel_PrevZCX;

// Real-time model
RT_MODEL_interceptModel_T interceptModel_M_ = RT_MODEL_interceptModel_T();
RT_MODEL_interceptModel_T *const interceptModel_M = &interceptModel_M_;

// Forward declaration for local functions
static void interceptModel_SystemCore_setup(dsp_simulink_MovingAverage_in_T *obj);

// Forward declaration for local functions
static void interceptMod_SystemCore_setup_d(dsp_simulink_MovingAverage_d_T *obj);
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

// System initialize for atomic system:
void intercept_PX4Timestamp_Init(DW_PX4Timestamp_interceptMode_T *localDW)
{
  // Start for MATLABSystem: '<Root>/PX4 Timestamp'
  localDW->obj.matlabCodegenIsDeleted = false;
  localDW->objisempty = true;
  localDW->obj.isInitialized = 1;
  localDW->obj.isSetupComplete = true;
}

// Output and update for atomic system:
void interceptModel_PX4Timestamp(B_PX4Timestamp_interceptModel_T *localB)
{
  // MATLABSystem: '<Root>/PX4 Timestamp'
  localB->PX4Timestamp = hrt_absolute_time();
}

// Termination for atomic system:
void intercept_PX4Timestamp_Term(DW_PX4Timestamp_interceptMode_T *localDW)
{
  // Terminate for MATLABSystem: '<Root>/PX4 Timestamp'
  if (!localDW->obj.matlabCodegenIsDeleted) {
    localDW->obj.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<Root>/PX4 Timestamp'
}

static void interceptModel_SystemCore_setup(dsp_simulink_MovingAverage_in_T *obj)
{
  obj->isInitialized = 1;

  // Start for MATLABSystem: '<S6>/Moving Average'
  obj->NumChannels = 1;
  obj->FrameLength = 1;
  obj->pCumSum = 0.0F;
  obj->pCumSumRev[0] = 0.0F;
  obj->pCumSumRev[1] = 0.0F;
  obj->pCumSumRev[2] = 0.0F;
  obj->pCumRevIndex = 1.0F;
  obj->pModValueRev = 0.0F;
  obj->isSetupComplete = true;
  obj->TunablePropsChanged = false;
}

// System initialize for atomic system:
void intercep_MovingAverage_Init(DW_MovingAverage_interceptMod_T *localDW)
{
  // Start for MATLABSystem: '<S6>/Moving Average'
  localDW->obj.isInitialized = 0;
  localDW->obj.NumChannels = -1;
  localDW->obj.FrameLength = -1;
  localDW->obj.matlabCodegenIsDeleted = false;
  localDW->objisempty = true;
  interceptModel_SystemCore_setup(&localDW->obj);

  // InitializeConditions for MATLABSystem: '<S6>/Moving Average'
  localDW->obj.pCumSum = 0.0F;
  localDW->obj.pCumSumRev[0] = 0.0F;
  localDW->obj.pCumSumRev[1] = 0.0F;
  localDW->obj.pCumSumRev[2] = 0.0F;
  localDW->obj.pCumRevIndex = 1.0F;
  localDW->obj.pModValueRev = 0.0F;
}

// Output and update for atomic system:
void interceptMode_MovingAverage(real32_T rtu_0, B_MovingAverage_interceptMode_T
  *localB, DW_MovingAverage_interceptMod_T *localDW)
{
  real32_T csum;
  real32_T cumRevIndex;
  real32_T z;

  // MATLABSystem: '<S6>/Moving Average'
  if (localDW->obj.TunablePropsChanged) {
    localDW->obj.TunablePropsChanged = false;
  }

  z = 0.0F;

  // MATLABSystem: '<S6>/Moving Average'
  localB->MovingAverage = 0.0F;

  // MATLABSystem: '<S6>/Moving Average'
  csum = localDW->obj.pCumSum + rtu_0;
  if (localDW->obj.pModValueRev == 0.0F) {
    z = localDW->obj.pCumSumRev[static_cast<int32_T>(localDW->obj.pCumRevIndex)
      - 1] + csum;
  }

  localDW->obj.pCumSumRev[static_cast<int32_T>(localDW->obj.pCumRevIndex) - 1] =
    rtu_0;
  if (localDW->obj.pCumRevIndex != 3.0F) {
    cumRevIndex = localDW->obj.pCumRevIndex + 1.0F;
  } else {
    cumRevIndex = 1.0F;
    csum = 0.0F;
    localDW->obj.pCumSumRev[1] += localDW->obj.pCumSumRev[2];
    localDW->obj.pCumSumRev[0] += localDW->obj.pCumSumRev[1];
  }

  if (localDW->obj.pModValueRev == 0.0F) {
    // MATLABSystem: '<S6>/Moving Average'
    localB->MovingAverage = z / 4.0F;
  }

  localDW->obj.pCumSum = csum;
  localDW->obj.pCumRevIndex = cumRevIndex;
  if (localDW->obj.pModValueRev > 0.0F) {
    localDW->obj.pModValueRev--;
  } else {
    localDW->obj.pModValueRev = 0.0F;
  }
}

// Termination for atomic system:
void intercep_MovingAverage_Term(DW_MovingAverage_interceptMod_T *localDW)
{
  // Terminate for MATLABSystem: '<S6>/Moving Average'
  if (!localDW->obj.matlabCodegenIsDeleted) {
    localDW->obj.matlabCodegenIsDeleted = true;
    if ((localDW->obj.isInitialized == 1) && localDW->obj.isSetupComplete) {
      localDW->obj.NumChannels = -1;
      localDW->obj.FrameLength = -1;
    }
  }

  // End of Terminate for MATLABSystem: '<S6>/Moving Average'
}

static void interceptMod_SystemCore_setup_d(dsp_simulink_MovingAverage_d_T *obj)
{
  obj->isInitialized = 1;

  // Start for MATLABSystem: '<Root>/Moving Average1'
  obj->NumChannels = 1;
  obj->FrameLength = 1;
  obj->pCumSum = 0.0F;
  for (int32_T i = 0; i < 6; i++) {
    // Start for MATLABSystem: '<Root>/Moving Average1'
    obj->pCumSumRev[i] = 0.0F;
  }

  // Start for MATLABSystem: '<Root>/Moving Average1'
  obj->pCumRevIndex = 1.0F;
  obj->pModValueRev = 0.0F;
  obj->isSetupComplete = true;
  obj->TunablePropsChanged = false;
}

// Model step function for TID0
void interceptModel_step0(void)        // Sample time: [0.01s, 0.0s]
{
  b_dspcodegen_FIRFilter_inte_d_T *obj_0;
  b_dspcodegen_FIRFilter_interc_T *obj;
  real_T rtb_VectorConcatenate_tmp;
  int32_T ParamStep;
  int32_T k;
  int32_T rtb_VectorConcatenate_tmp_0;
  real32_T ParamStep_0;
  real32_T csum;
  real32_T cumRevIndex;
  boolean_T rtb_LogicalOperator;
  boolean_T tmp;

  {                                    // Sample time: [0.01s, 0.0s]
    rate_monotonic_scheduler();
  }

  // MATLABSystem: '<S22>/SourceBlock'
  tmp = uORB_read_step(interceptModel_DW.obj_o.orbMetadataObj,
                       &interceptModel_DW.obj_o.eventStructObj,
                       &interceptModel_B.r1, false, 1.0);

  // Outputs for Enabled SubSystem: '<S22>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S68>/Enable'

  // Start for MATLABSystem: '<S22>/SourceBlock'
  if (tmp) {
    // SignalConversion generated from: '<S68>/In1'
    interceptModel_B.In1 = interceptModel_B.r1;
  }

  // End of Outputs for SubSystem: '<S22>/Enabled Subsystem'

  // Sqrt: '<S66>/sqrt' incorporates:
  //   DataTypeConversion: '<S6>/Data Type Conversion'
  //   Product: '<S67>/Product'
  //   Product: '<S67>/Product1'
  //   Product: '<S67>/Product2'
  //   Product: '<S67>/Product3'
  //   Sum: '<S67>/Sum'

  interceptModel_B.Product1_pd = sqrt(((static_cast<real_T>
    (interceptModel_B.In1.q[0]) * interceptModel_B.In1.q[0] + static_cast<real_T>
    (interceptModel_B.In1.q[1]) * interceptModel_B.In1.q[1]) +
    static_cast<real_T>(interceptModel_B.In1.q[2]) * interceptModel_B.In1.q[2])
    + static_cast<real_T>(interceptModel_B.In1.q[3]) * interceptModel_B.In1.q[3]);

  // Product: '<S65>/Product' incorporates:
  //   DataTypeConversion: '<S6>/Data Type Conversion'

  interceptModel_B.Product1_c = interceptModel_B.In1.q[0] /
    interceptModel_B.Product1_pd;

  // Product: '<S65>/Product1' incorporates:
  //   DataTypeConversion: '<S6>/Data Type Conversion'

  interceptModel_B.DataTypeConversion5 = interceptModel_B.In1.q[1] /
    interceptModel_B.Product1_pd;

  // Product: '<S65>/Product2' incorporates:
  //   DataTypeConversion: '<S6>/Data Type Conversion'

  interceptModel_B.Add_h = interceptModel_B.In1.q[2] /
    interceptModel_B.Product1_pd;

  // Product: '<S65>/Product3' incorporates:
  //   DataTypeConversion: '<S6>/Data Type Conversion'

  interceptModel_B.Product1_pd = interceptModel_B.In1.q[3] /
    interceptModel_B.Product1_pd;

  // Product: '<S55>/Product3' incorporates:
  //   Product: '<S59>/Product3'

  interceptModel_B.DataTypeConversion3 = interceptModel_B.Product1_c *
    interceptModel_B.Product1_c;

  // Product: '<S55>/Product2' incorporates:
  //   Product: '<S59>/Product2'

  interceptModel_B.DataTypeConversion2 = interceptModel_B.DataTypeConversion5 *
    interceptModel_B.DataTypeConversion5;

  // Product: '<S55>/Product1' incorporates:
  //   Product: '<S59>/Product1'
  //   Product: '<S63>/Product1'

  interceptModel_B.Add = interceptModel_B.Add_h * interceptModel_B.Add_h;

  // Product: '<S55>/Product' incorporates:
  //   Product: '<S59>/Product'
  //   Product: '<S63>/Product'

  interceptModel_B.rtb_VectorConcatenate_tmp_k = interceptModel_B.Product1_pd *
    interceptModel_B.Product1_pd;

  // Sum: '<S55>/Sum' incorporates:
  //   Product: '<S55>/Product'
  //   Product: '<S55>/Product1'
  //   Product: '<S55>/Product2'
  //   Product: '<S55>/Product3'

  interceptModel_B.VectorConcatenate[0] = ((interceptModel_B.DataTypeConversion3
    + interceptModel_B.DataTypeConversion2) - interceptModel_B.Add) -
    interceptModel_B.rtb_VectorConcatenate_tmp_k;

  // Product: '<S58>/Product3' incorporates:
  //   Product: '<S56>/Product3'

  interceptModel_B.rtb_VectorConcatenate_tmp = interceptModel_B.Product1_pd *
    interceptModel_B.Product1_c;

  // Product: '<S58>/Product2' incorporates:
  //   Product: '<S56>/Product2'

  interceptModel_B.rtb_VectorConcatenate_tmp_c =
    interceptModel_B.DataTypeConversion5 * interceptModel_B.Add_h;

  // Gain: '<S58>/Gain' incorporates:
  //   Product: '<S58>/Product2'
  //   Product: '<S58>/Product3'
  //   Sum: '<S58>/Sum'

  interceptModel_B.VectorConcatenate[1] =
    (interceptModel_B.rtb_VectorConcatenate_tmp_c -
     interceptModel_B.rtb_VectorConcatenate_tmp) * interceptModel_P.Gain_Gain;

  // Product: '<S61>/Product2' incorporates:
  //   Product: '<S57>/Product2'

  interceptModel_B.rtb_VectorConcatenate_tmp_cx =
    interceptModel_B.DataTypeConversion5 * interceptModel_B.Product1_pd;

  // Product: '<S61>/Product1' incorporates:
  //   Product: '<S57>/Product1'

  rtb_VectorConcatenate_tmp = interceptModel_B.Product1_c *
    interceptModel_B.Add_h;

  // Gain: '<S61>/Gain' incorporates:
  //   Product: '<S61>/Product1'
  //   Product: '<S61>/Product2'
  //   Sum: '<S61>/Sum'

  interceptModel_B.VectorConcatenate[2] = (rtb_VectorConcatenate_tmp +
    interceptModel_B.rtb_VectorConcatenate_tmp_cx) *
    interceptModel_P.Gain_Gain_d;

  // Gain: '<S56>/Gain' incorporates:
  //   Sum: '<S56>/Sum'

  interceptModel_B.VectorConcatenate[3] =
    (interceptModel_B.rtb_VectorConcatenate_tmp +
     interceptModel_B.rtb_VectorConcatenate_tmp_c) *
    interceptModel_P.Gain_Gain_e;

  // Sum: '<S59>/Sum' incorporates:
  //   Sum: '<S63>/Sum'

  interceptModel_B.DataTypeConversion3 -= interceptModel_B.DataTypeConversion2;
  interceptModel_B.VectorConcatenate[4] = (interceptModel_B.DataTypeConversion3
    + interceptModel_B.Add) - interceptModel_B.rtb_VectorConcatenate_tmp_k;

  // Product: '<S62>/Product1' incorporates:
  //   Product: '<S60>/Product1'

  interceptModel_B.DataTypeConversion2 = interceptModel_B.Product1_c *
    interceptModel_B.DataTypeConversion5;

  // Product: '<S62>/Product2' incorporates:
  //   Product: '<S60>/Product2'

  interceptModel_B.rtb_VectorConcatenate_tmp = interceptModel_B.Add_h *
    interceptModel_B.Product1_pd;

  // Gain: '<S62>/Gain' incorporates:
  //   Product: '<S62>/Product1'
  //   Product: '<S62>/Product2'
  //   Sum: '<S62>/Sum'

  interceptModel_B.VectorConcatenate[5] =
    (interceptModel_B.rtb_VectorConcatenate_tmp -
     interceptModel_B.DataTypeConversion2) * interceptModel_P.Gain_Gain_j;

  // Gain: '<S57>/Gain' incorporates:
  //   Sum: '<S57>/Sum'

  interceptModel_B.VectorConcatenate[6] =
    (interceptModel_B.rtb_VectorConcatenate_tmp_cx - rtb_VectorConcatenate_tmp) *
    interceptModel_P.Gain_Gain_ju;

  // Gain: '<S60>/Gain' incorporates:
  //   Sum: '<S60>/Sum'

  interceptModel_B.VectorConcatenate[7] = (interceptModel_B.DataTypeConversion2
    + interceptModel_B.rtb_VectorConcatenate_tmp) * interceptModel_P.Gain_Gain_b;

  // Sum: '<S63>/Sum'
  interceptModel_B.VectorConcatenate[8] = (interceptModel_B.DataTypeConversion3
    - interceptModel_B.Add) + interceptModel_B.rtb_VectorConcatenate_tmp_k;

  // If: '<S31>/If1' incorporates:
  //   Constant: '<S31>/Constant'

  if (interceptModel_P.DirectionCosineMatrixtoRotati_c > 1) {
    // Outputs for IfAction SubSystem: '<S31>/If Warning//Error' incorporates:
    //   ActionPort: '<S40>/if'

    // Bias: '<S44>/Bias1' incorporates:
    //   Concatenate: '<S64>/Vector Concatenate'
    //   Math: '<S44>/Math Function'
    //   Product: '<S44>/Product'

    for (k = 0; k < 3; k++) {
      for (ParamStep = 0; ParamStep < 3; ParamStep++) {
        rtb_VectorConcatenate_tmp_0 = 3 * ParamStep + k;
        interceptModel_B.rtb_VectorConcatenate_m[rtb_VectorConcatenate_tmp_0] =
          ((interceptModel_B.VectorConcatenate[3 * k + 1] *
            interceptModel_B.VectorConcatenate[3 * ParamStep + 1] +
            interceptModel_B.VectorConcatenate[3 * k] *
            interceptModel_B.VectorConcatenate[3 * ParamStep]) +
           interceptModel_B.VectorConcatenate[3 * k + 2] *
           interceptModel_B.VectorConcatenate[3 * ParamStep + 2]) +
          interceptModel_P.Bias1_Bias[rtb_VectorConcatenate_tmp_0];
      }
    }

    // End of Bias: '<S44>/Bias1'

    // RelationalOperator: '<S52>/Compare' incorporates:
    //   Abs: '<S44>/Abs2'
    //   Constant: '<S52>/Constant'

    for (k = 0; k < 9; k++) {
      interceptModel_B.Compare[k] = (fabs
        (interceptModel_B.rtb_VectorConcatenate_m[k]) >
        interceptModel_P.DirectionCosineMatrixtoRotation);
    }

    // End of RelationalOperator: '<S52>/Compare'

    // Logic: '<S44>/Logical Operator1' incorporates:
    //   RelationalOperator: '<S52>/Compare'

    tmp = interceptModel_B.Compare[0];
    for (k = 0; k < 8; k++) {
      tmp = (tmp || interceptModel_B.Compare[k + 1]);
    }

    // If: '<S40>/If' incorporates:
    //   Abs: '<S45>/Abs1'
    //   Bias: '<S45>/Bias'
    //   Constant: '<S54>/Constant'
    //   Logic: '<S44>/Logical Operator1'
    //   Product: '<S53>/Product'
    //   Product: '<S53>/Product1'
    //   Product: '<S53>/Product2'
    //   Product: '<S53>/Product3'
    //   Product: '<S53>/Product4'
    //   Product: '<S53>/Product5'
    //   RelationalOperator: '<S54>/Compare'
    //   Sum: '<S53>/Sum'

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
      // Outputs for IfAction SubSystem: '<S40>/If Not Proper' incorporates:
      //   ActionPort: '<S43>/Action Port'

      // If: '<S43>/If' incorporates:
      //   Constant: '<S43>/Constant'

      if (interceptModel_P.DirectionCosineMatrixtoRotati_c == 2) {
        // Outputs for IfAction SubSystem: '<S43>/Warning' incorporates:
        //   ActionPort: '<S51>/Action Port'

        // Assertion: '<S51>/Assertion' incorporates:
        //   Constant: '<S43>/Constant1'

        utAssert(interceptModel_P.Constant1_Value != 0.0);

        // End of Outputs for SubSystem: '<S43>/Warning'
      } else if (interceptModel_P.DirectionCosineMatrixtoRotati_c == 3) {
        // Outputs for IfAction SubSystem: '<S43>/Error' incorporates:
        //   ActionPort: '<S49>/Action Port'

        // Assertion: '<S49>/Assertion' incorporates:
        //   Constant: '<S43>/Constant1'

        utAssert(interceptModel_P.Constant1_Value != 0.0);

        // End of Outputs for SubSystem: '<S43>/Error'
      }

      // End of If: '<S43>/If'
      // End of Outputs for SubSystem: '<S40>/If Not Proper'
    } else if (tmp) {
      // Outputs for IfAction SubSystem: '<S40>/Else If Not Orthogonal' incorporates:
      //   ActionPort: '<S41>/Action Port'

      // If: '<S41>/If' incorporates:
      //   Constant: '<S41>/Constant'

      if (interceptModel_P.DirectionCosineMatrixtoRotati_c == 2) {
        // Outputs for IfAction SubSystem: '<S41>/Warning' incorporates:
        //   ActionPort: '<S48>/Action Port'

        // Assertion: '<S48>/Assertion' incorporates:
        //   Constant: '<S41>/Constant1'

        utAssert(interceptModel_P.Constant1_Value_e != 0.0);

        // End of Outputs for SubSystem: '<S41>/Warning'
      } else if (interceptModel_P.DirectionCosineMatrixtoRotati_c == 3) {
        // Outputs for IfAction SubSystem: '<S41>/Error' incorporates:
        //   ActionPort: '<S46>/Action Port'

        // Assertion: '<S46>/Assertion' incorporates:
        //   Constant: '<S41>/Constant1'

        utAssert(interceptModel_P.Constant1_Value_e != 0.0);

        // End of Outputs for SubSystem: '<S41>/Error'
      }

      // End of If: '<S41>/If'
      // End of Outputs for SubSystem: '<S40>/Else If Not Orthogonal'
    }

    // End of If: '<S40>/If'
    // End of Outputs for SubSystem: '<S31>/If Warning//Error'
  }

  // End of If: '<S31>/If1'

  // MATLABSystem: '<S18>/SourceBlock'
  tmp = uORB_read_step(interceptModel_DW.obj_e.orbMetadataObj,
                       &interceptModel_DW.obj_e.eventStructObj,
                       &interceptModel_B.r3, false, 1.0);

  // Outputs for Enabled SubSystem: '<S18>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S24>/Enable'

  // Start for MATLABSystem: '<S18>/SourceBlock'
  if (tmp) {
    // SignalConversion generated from: '<S24>/In1'
    interceptModel_B.In1_f = interceptModel_B.r3;
  }

  // End of Outputs for SubSystem: '<S18>/Enabled Subsystem'

  // Sum: '<S5>/Add1' incorporates:
  //   Constant: '<S5>/Constant'
  //   DataTypeConversion: '<S6>/Data Type Conversion5'

  interceptModel_B.Product1_c = interceptModel_P.Constant_Value_kj -
    interceptModel_B.In1_f.target_x;

  // RateTransition generated from: '<Root>/Rate Transition'
  tmp = interceptModel_M->Timing.RateInteraction.TID0_1;
  if (tmp) {
    // RateTransition generated from: '<Root>/Rate Transition'
    interceptModel_B.x_error_kp = interceptModel_DW.RateTransition_1_Buffer0;
  }

  // MATLABSystem: '<S3>/Read Parameter3'
  rtb_LogicalOperator = MW_Param_Step(interceptModel_DW.obj_n.MW_PARAMHANDLE,
    MW_INT32, &ParamStep);
  if (rtb_LogicalOperator) {
    ParamStep = 0;
  }

  // MATLABSystem: '<S3>/Read Parameter10'
  rtb_LogicalOperator = MW_Param_Step(interceptModel_DW.obj_a5.MW_PARAMHANDLE,
    MW_SINGLE, &ParamStep_0);
  if (rtb_LogicalOperator) {
    ParamStep_0 = 0.0F;
  }

  // Logic: '<S5>/Logical Operator' incorporates:
  //   Constant: '<S6>/Constant'
  //   DataTypeConversion: '<S5>/Data Type Conversion5'
  //   DataTypeConversion: '<S6>/Data Type Conversion1'
  //   MATLABSystem: '<S3>/Read Parameter10'
  //   MATLABSystem: '<S3>/Read Parameter3'
  //
  rtb_LogicalOperator = ((ParamStep != 0) || (interceptModel_P.Constant_Value_eh
    != 0.0) || (ParamStep_0 != 0.0F));

  // Delay: '<S13>/Delay_Integration' incorporates:
  //   Constant: '<S5>/Constant1'

  interceptModel_DW.icLoad_p = ((rtb_LogicalOperator &&
    (interceptModel_PrevZCX.Delay_Integration_Reset_ZCE != POS_ZCSIG)) ||
    interceptModel_DW.icLoad_p);
  interceptModel_PrevZCX.Delay_Integration_Reset_ZCE = rtb_LogicalOperator;
  if (interceptModel_DW.icLoad_p) {
    interceptModel_DW.Delay_Integration_DSTATE =
      interceptModel_P.Constant1_Value_m;
  }

  // Sum: '<S5>/Add' incorporates:
  //   DataTypeConversion: '<S5>/Data Type Conversion2'
  //   Delay: '<S13>/Delay_Integration'
  //   Product: '<S5>/Product'

  interceptModel_B.Add_h = interceptModel_B.Product1_c *
    interceptModel_B.x_error_kp + interceptModel_DW.Delay_Integration_DSTATE;

  // RateTransition generated from: '<Root>/Rate Transition'
  if (tmp) {
    // RateTransition generated from: '<Root>/Rate Transition'
    interceptModel_B.roll_lim_x = interceptModel_DW.RateTransition_5_Buffer0;
  }

  // MinMax: '<S12>/MinMax1' incorporates:
  //   DataTypeConversion: '<S5>/Data Type Conversion6'

  if ((interceptModel_B.Add_h <= interceptModel_B.roll_lim_x) || rtIsNaN(
       static_cast<real_T>(interceptModel_B.roll_lim_x))) {
    interceptModel_B.DataTypeConversion3 = interceptModel_B.Add_h;
  } else {
    interceptModel_B.DataTypeConversion3 = interceptModel_B.roll_lim_x;
  }

  // End of MinMax: '<S12>/MinMax1'

  // Gain: '<S5>/Gain1' incorporates:
  //   DataTypeConversion: '<S5>/Data Type Conversion6'

  interceptModel_B.DataTypeConversion5 = interceptModel_P.Gain1_Gain_n *
    interceptModel_B.roll_lim_x;

  // MinMax: '<S12>/MinMax2'
  if ((interceptModel_B.DataTypeConversion3 >=
       interceptModel_B.DataTypeConversion5) || rtIsNaN
      (interceptModel_B.DataTypeConversion5)) {
    interceptModel_B.DataTypeConversion5 = interceptModel_B.DataTypeConversion3;
  }

  // End of MinMax: '<S12>/MinMax2'

  // Saturate: '<S5>/Saturation1'
  if (interceptModel_B.DataTypeConversion5 >
      interceptModel_P.Saturation1_UpperSat) {
    interceptModel_B.Product1_pd = interceptModel_P.Saturation1_UpperSat;
  } else if (interceptModel_B.DataTypeConversion5 <
             interceptModel_P.Saturation1_LowerSat) {
    interceptModel_B.Product1_pd = interceptModel_P.Saturation1_LowerSat;
  } else {
    interceptModel_B.Product1_pd = interceptModel_B.DataTypeConversion5;
  }

  // End of Saturate: '<S5>/Saturation1'

  // MATLABSystem: '<Root>/Moving Average1'
  if (interceptModel_DW.obj_d.TunablePropsChanged) {
    interceptModel_DW.obj_d.TunablePropsChanged = false;
  }

  ParamStep_0 = 0.0F;

  // BusAssignment: '<Root>/Bus Assignment' incorporates:
  //   MATLABSystem: '<Root>/Moving Average1'

  interceptModel_B.BusAssignment_o.target_roll = 0.0F;

  // MATLABSystem: '<Root>/Moving Average1' incorporates:
  //   DataTypeConversion: '<Root>/Data Type Conversion2'

  csum = interceptModel_DW.obj_d.pCumSum + static_cast<real32_T>
    (interceptModel_B.Product1_pd);
  if (interceptModel_DW.obj_d.pModValueRev == 0.0F) {
    ParamStep_0 = interceptModel_DW.obj_d.pCumSumRev[static_cast<int32_T>
      (interceptModel_DW.obj_d.pCumRevIndex) - 1] + csum;
  }

  interceptModel_DW.obj_d.pCumSumRev[static_cast<int32_T>
    (interceptModel_DW.obj_d.pCumRevIndex) - 1] = static_cast<real32_T>
    (interceptModel_B.Product1_pd);
  if (interceptModel_DW.obj_d.pCumRevIndex != 6.0F) {
    cumRevIndex = interceptModel_DW.obj_d.pCumRevIndex + 1.0F;
  } else {
    cumRevIndex = 1.0F;
    csum = 0.0F;
    for (k = 4; k >= 0; k--) {
      interceptModel_DW.obj_d.pCumSumRev[k] +=
        interceptModel_DW.obj_d.pCumSumRev[k + 1];
    }
  }

  if (interceptModel_DW.obj_d.pModValueRev == 0.0F) {
    // BusAssignment: '<Root>/Bus Assignment'
    interceptModel_B.BusAssignment_o.target_roll = ParamStep_0 / 7.0F;
  }

  interceptModel_DW.obj_d.pCumSum = csum;
  interceptModel_DW.obj_d.pCumRevIndex = cumRevIndex;
  if (interceptModel_DW.obj_d.pModValueRev > 0.0F) {
    interceptModel_DW.obj_d.pModValueRev--;
  } else {
    interceptModel_DW.obj_d.pModValueRev = 0.0F;
  }

  interceptModel_PX4Timestamp(&interceptModel_B.PX4Timestamp);

  // Sum: '<S4>/Add1' incorporates:
  //   Constant: '<S4>/Constant'
  //   DataTypeConversion: '<S6>/Data Type Conversion5'

  interceptModel_B.Product1_pd = interceptModel_P.Constant_Value_o5 -
    interceptModel_B.In1_f.target_y;

  // RateTransition generated from: '<Root>/Rate Transition'
  if (tmp) {
    // RateTransition generated from: '<Root>/Rate Transition'
    interceptModel_B.y_error_kp = interceptModel_DW.RateTransition_6_Buffer0;

    // RateTransition generated from: '<Root>/Rate Transition'
    interceptModel_B.int_reset_y = interceptModel_DW.RateTransition_9_Buffer0;
  }

  // Logic: '<S4>/Logical Operator' incorporates:
  //   Constant: '<S6>/Constant'
  //   DataTypeConversion: '<S4>/Data Type Conversion5'
  //   DataTypeConversion: '<S6>/Data Type Conversion1'
  //   Logic: '<S4>/Logical Operator1'

  rtb_LogicalOperator = ((interceptModel_B.int_reset_y != 0.0F) ||
    (!(interceptModel_P.Constant_Value_eh != 0.0)));

  // Delay: '<S9>/Delay_Integration' incorporates:
  //   Constant: '<S4>/Constant1'

  interceptModel_DW.icLoad_i = ((rtb_LogicalOperator &&
    (interceptModel_PrevZCX.Delay_Integration_Reset_ZCE_p != POS_ZCSIG)) ||
    interceptModel_DW.icLoad_i);
  interceptModel_PrevZCX.Delay_Integration_Reset_ZCE_p = rtb_LogicalOperator;
  if (interceptModel_DW.icLoad_i) {
    interceptModel_DW.Delay_Integration_DSTATE_h =
      interceptModel_P.Constant1_Value_a;
  }

  // Sum: '<S4>/Add' incorporates:
  //   DataTypeConversion: '<S4>/Data Type Conversion2'
  //   Delay: '<S9>/Delay_Integration'
  //   Product: '<S4>/Product'

  interceptModel_B.Add = interceptModel_B.Product1_pd *
    interceptModel_B.y_error_kp + interceptModel_DW.Delay_Integration_DSTATE_h;

  // RateTransition generated from: '<Root>/Rate Transition'
  if (tmp) {
    // RateTransition generated from: '<Root>/Rate Transition'
    interceptModel_B.roll_lim_y = interceptModel_DW.RateTransition_10_Buffer0;
  }

  // MinMax: '<S8>/MinMax1' incorporates:
  //   DataTypeConversion: '<S4>/Data Type Conversion6'

  if ((interceptModel_B.Add <= interceptModel_B.roll_lim_y) || rtIsNaN(
       static_cast<real_T>(interceptModel_B.roll_lim_y))) {
    interceptModel_B.DataTypeConversion3 = interceptModel_B.Add;
  } else {
    interceptModel_B.DataTypeConversion3 = interceptModel_B.roll_lim_y;
  }

  // End of MinMax: '<S8>/MinMax1'

  // Gain: '<S4>/Gain1' incorporates:
  //   DataTypeConversion: '<S4>/Data Type Conversion6'

  interceptModel_B.DataTypeConversion2 = interceptModel_P.Gain1_Gain_o *
    interceptModel_B.roll_lim_y;

  // MinMax: '<S8>/MinMax2'
  if ((interceptModel_B.DataTypeConversion3 >=
       interceptModel_B.DataTypeConversion2) || rtIsNaN
      (interceptModel_B.DataTypeConversion2)) {
    interceptModel_B.DataTypeConversion2 = interceptModel_B.DataTypeConversion3;
  }

  // End of MinMax: '<S8>/MinMax2'

  // Saturate: '<S4>/Saturation1'
  if (interceptModel_B.DataTypeConversion2 >
      interceptModel_P.Saturation1_UpperSat_d) {
    interceptModel_B.DataTypeConversion3 =
      interceptModel_P.Saturation1_UpperSat_d;
  } else if (interceptModel_B.DataTypeConversion2 <
             interceptModel_P.Saturation1_LowerSat_h) {
    interceptModel_B.DataTypeConversion3 =
      interceptModel_P.Saturation1_LowerSat_h;
  } else {
    interceptModel_B.DataTypeConversion3 = interceptModel_B.DataTypeConversion2;
  }

  // End of Saturate: '<S4>/Saturation1'

  // RateLimiter: '<S4>/Rate Limiter'
  interceptModel_B.rtb_VectorConcatenate_tmp_k =
    interceptModel_B.DataTypeConversion3 - interceptModel_DW.PrevY;
  if (interceptModel_B.rtb_VectorConcatenate_tmp_k >
      interceptModel_P.RateLimiter_RisingLim * interceptModel_period) {
    interceptModel_B.DataTypeConversion3 =
      interceptModel_P.RateLimiter_RisingLim * interceptModel_period +
      interceptModel_DW.PrevY;
  } else if (interceptModel_B.rtb_VectorConcatenate_tmp_k <
             interceptModel_P.RateLimiter_FallingLim * interceptModel_period) {
    interceptModel_B.DataTypeConversion3 =
      interceptModel_P.RateLimiter_FallingLim * interceptModel_period +
      interceptModel_DW.PrevY;
  }

  interceptModel_DW.PrevY = interceptModel_B.DataTypeConversion3;

  // End of RateLimiter: '<S4>/Rate Limiter'

  // BusAssignment: '<Root>/Bus Assignment' incorporates:
  //   DataTypeConversion: '<Root>/Data Type Conversion4'
  //   MATLABSystem: '<Root>/PX4 Timestamp'

  interceptModel_B.BusAssignment_o.timestamp =
    interceptModel_B.PX4Timestamp.PX4Timestamp;
  interceptModel_B.BusAssignment_o.target_pitch = static_cast<real32_T>
    (interceptModel_B.DataTypeConversion3);

  // MATLABSystem: '<S2>/SinkBlock' incorporates:
  //   BusAssignment: '<Root>/Bus Assignment'

  uORB_write_step(interceptModel_DW.obj_ei.orbMetadataObj,
                  &interceptModel_DW.obj_ei.orbAdvertiseObj,
                  &interceptModel_B.BusAssignment_o);

  // Sum: '<S8>/Sum'
  interceptModel_B.DataTypeConversion3 = interceptModel_B.Add -
    interceptModel_B.DataTypeConversion2;

  // Signum: '<S8>/Sign'
  if (rtIsNaN(interceptModel_B.DataTypeConversion3)) {
    interceptModel_B.DataTypeConversion2 = (rtNaN);
  } else if (interceptModel_B.DataTypeConversion3 < 0.0) {
    interceptModel_B.DataTypeConversion2 = -1.0;
  } else {
    interceptModel_B.DataTypeConversion2 = (interceptModel_B.DataTypeConversion3
      > 0.0);
  }

  // End of Signum: '<S8>/Sign'

  // RateTransition generated from: '<Root>/Rate Transition'
  if (tmp) {
    // RateTransition generated from: '<Root>/Rate Transition'
    interceptModel_B.y_int_lim = interceptModel_DW.RateTransition_8_Buffer0;
  }

  // MinMax: '<S7>/MinMax1' incorporates:
  //   DataTypeConversion: '<S4>/Data Type Conversion4'

  if ((interceptModel_B.Product1_pd <= interceptModel_B.y_int_lim) || rtIsNaN(
       static_cast<real_T>(interceptModel_B.y_int_lim))) {
    interceptModel_B.DataTypeConversion3 = interceptModel_B.Product1_pd;
  } else {
    interceptModel_B.DataTypeConversion3 = interceptModel_B.y_int_lim;
  }

  // End of MinMax: '<S7>/MinMax1'

  // Gain: '<S4>/Gain' incorporates:
  //   DataTypeConversion: '<S4>/Data Type Conversion4'

  interceptModel_B.Add = interceptModel_P.Gain_Gain_g *
    interceptModel_B.y_int_lim;

  // MinMax: '<S7>/MinMax2'
  if ((interceptModel_B.DataTypeConversion3 >= interceptModel_B.Add) || rtIsNaN
      (interceptModel_B.Add)) {
    interceptModel_B.Add = interceptModel_B.DataTypeConversion3;
  }

  // Sum: '<S7>/Sum' incorporates:
  //   MinMax: '<S7>/MinMax2'

  interceptModel_B.DataTypeConversion3 = interceptModel_B.Product1_pd -
    interceptModel_B.Add;

  // Signum: '<S7>/Sign'
  if (rtIsNaN(interceptModel_B.DataTypeConversion3)) {
    interceptModel_B.DataTypeConversion3 = (rtNaN);
  } else if (interceptModel_B.DataTypeConversion3 < 0.0) {
    interceptModel_B.DataTypeConversion3 = -1.0;
  } else {
    interceptModel_B.DataTypeConversion3 = (interceptModel_B.DataTypeConversion3
      > 0.0);
  }

  // End of Signum: '<S7>/Sign'

  // RateTransition generated from: '<Root>/Rate Transition'
  if (tmp) {
    // RateTransition generated from: '<Root>/Rate Transition'
    interceptModel_B.y_error_ki = interceptModel_DW.RateTransition_7_Buffer0;
  }

  // Product: '<S4>/Product1' incorporates:
  //   DataTypeConversion: '<S4>/Data Type Conversion3'

  interceptModel_B.Product1_pd *= interceptModel_B.y_error_ki;

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

  if ((interceptModel_B.DataTypeConversion2 ==
       interceptModel_P.Constant1_Value_ez) ||
      (interceptModel_B.DataTypeConversion3 ==
       interceptModel_P.Constant1_Value_ez)) {
    // MinMax: '<S10>/MinMax1' incorporates:
    //   Constant: '<S10>/Constant3'

    if ((!(interceptModel_B.Product1_pd >= interceptModel_P.Constant3_Value_a)) &&
        (!rtIsNaN(interceptModel_P.Constant3_Value_a))) {
      interceptModel_B.Product1_pd = interceptModel_P.Constant3_Value_a;
    }

    interceptModel_B.DataTypeConversion2 = interceptModel_P.Constant3_Value_a +
      interceptModel_B.Product1_pd;
  } else {
    if ((interceptModel_B.DataTypeConversion2 ==
         interceptModel_P.Constant2_Value) ||
        (interceptModel_B.DataTypeConversion3 ==
         interceptModel_P.Constant2_Value)) {
      // Switch: '<S10>/Switch2' incorporates:
      //   Constant: '<S10>/Constant3'

      interceptModel_B.DataTypeConversion3 = interceptModel_P.Constant3_Value_a;
    } else if ((interceptModel_B.Product1_pd >=
                interceptModel_P.Constant3_Value_a) || rtIsNaN
               (interceptModel_P.Constant3_Value_a)) {
      // MinMax: '<S10>/MinMax1' incorporates:
      //   Switch: '<S10>/Switch2'

      interceptModel_B.DataTypeConversion3 = interceptModel_B.Product1_pd;
    } else {
      // Switch: '<S10>/Switch2' incorporates:
      //   Constant: '<S10>/Constant3'
      //   MinMax: '<S10>/MinMax1'

      interceptModel_B.DataTypeConversion3 = interceptModel_P.Constant3_Value_a;
    }

    // MinMax: '<S10>/MinMax2' incorporates:
    //   Constant: '<S10>/Constant3'
    //   MinMax: '<S10>/MinMax1'

    if ((!(interceptModel_B.Product1_pd <= interceptModel_P.Constant3_Value_a)) &&
        (!rtIsNaN(interceptModel_P.Constant3_Value_a))) {
      interceptModel_B.Product1_pd = interceptModel_P.Constant3_Value_a;
    }

    interceptModel_B.DataTypeConversion2 = interceptModel_B.Product1_pd +
      interceptModel_B.DataTypeConversion3;
  }

  // End of Switch: '<S10>/Switch3'

  // Product: '<S9>/Product1' incorporates:
  //   Constant: '<S9>/Constant'

  interceptModel_B.Product1_pd = interceptModel_B.DataTypeConversion2 *
    interceptModel_P.Constant_Value_ox;

  // Sum: '<S12>/Sum'
  interceptModel_B.DataTypeConversion3 = interceptModel_B.Add_h -
    interceptModel_B.DataTypeConversion5;

  // Signum: '<S12>/Sign'
  if (rtIsNaN(interceptModel_B.DataTypeConversion3)) {
    interceptModel_B.Add_h = (rtNaN);
  } else if (interceptModel_B.DataTypeConversion3 < 0.0) {
    interceptModel_B.Add_h = -1.0;
  } else {
    interceptModel_B.Add_h = (interceptModel_B.DataTypeConversion3 > 0.0);
  }

  // End of Signum: '<S12>/Sign'
  interceptModel_PX4Timestamp(&interceptModel_B.PX4Timestamp_p);

  // BusAssignment: '<S5>/Bus Assignment1' incorporates:
  //   Constant: '<S14>/Constant'

  interceptModel_B.BusAssignment1 = interceptModel_P.Constant_Value_i;

  // BusAssignment: '<S5>/Bus Assignment1' incorporates:
  //   DataTypeConversion: '<S5>/Data Type Conversion7'
  //   DataTypeConversion: '<S5>/Data Type Conversion8'
  //   DataTypeConversion: '<S5>/Data Type Conversion9'
  //   Delay: '<S13>/Delay_Integration'
  //   MATLABSystem: '<S3>/Read Parameter3'
  //   MATLABSystem: '<S5>/PX4 Timestamp'
  //
  interceptModel_B.BusAssignment1.timestamp =
    interceptModel_B.PX4Timestamp_p.PX4Timestamp;
  interceptModel_B.BusAssignment1.x_sat_flag_bit = static_cast<real32_T>
    (interceptModel_B.Add_h);
  interceptModel_B.BusAssignment1.x_integral_effort = static_cast<real32_T>
    (interceptModel_DW.Delay_Integration_DSTATE);
  interceptModel_B.BusAssignment1.proportional_effort = static_cast<real32_T>
    (ParamStep);

  // MATLABSystem: '<S15>/SinkBlock' incorporates:
  //   BusAssignment: '<S5>/Bus Assignment1'

  uORB_write_step(interceptModel_DW.obj_k.orbMetadataObj,
                  &interceptModel_DW.obj_k.orbAdvertiseObj,
                  &interceptModel_B.BusAssignment1);

  // RateTransition generated from: '<Root>/Rate Transition'
  if (tmp) {
    // RateTransition generated from: '<Root>/Rate Transition'
    interceptModel_B.x_int_lim = interceptModel_DW.RateTransition_3_Buffer0;
  }

  // MinMax: '<S11>/MinMax1' incorporates:
  //   DataTypeConversion: '<S5>/Data Type Conversion4'

  if ((interceptModel_B.Product1_c <= interceptModel_B.x_int_lim) || rtIsNaN(
       static_cast<real_T>(interceptModel_B.x_int_lim))) {
    interceptModel_B.DataTypeConversion3 = interceptModel_B.Product1_c;
  } else {
    interceptModel_B.DataTypeConversion3 = interceptModel_B.x_int_lim;
  }

  // End of MinMax: '<S11>/MinMax1'

  // Gain: '<S5>/Gain' incorporates:
  //   DataTypeConversion: '<S5>/Data Type Conversion4'

  interceptModel_B.Add = interceptModel_P.Gain_Gain_jy *
    interceptModel_B.x_int_lim;

  // MinMax: '<S11>/MinMax2'
  if ((interceptModel_B.DataTypeConversion3 >= interceptModel_B.Add) || rtIsNaN
      (interceptModel_B.Add)) {
    interceptModel_B.Add = interceptModel_B.DataTypeConversion3;
  }

  // Sum: '<S11>/Sum' incorporates:
  //   MinMax: '<S11>/MinMax2'

  interceptModel_B.DataTypeConversion3 = interceptModel_B.Product1_c -
    interceptModel_B.Add;

  // Signum: '<S11>/Sign'
  if (rtIsNaN(interceptModel_B.DataTypeConversion3)) {
    interceptModel_B.DataTypeConversion5 = (rtNaN);
  } else if (interceptModel_B.DataTypeConversion3 < 0.0) {
    interceptModel_B.DataTypeConversion5 = -1.0;
  } else {
    interceptModel_B.DataTypeConversion5 = (interceptModel_B.DataTypeConversion3
      > 0.0);
  }

  // End of Signum: '<S11>/Sign'

  // RateTransition generated from: '<Root>/Rate Transition'
  if (tmp) {
    // RateTransition generated from: '<Root>/Rate Transition'
    interceptModel_B.x_error_ki = interceptModel_DW.RateTransition_2_Buffer0;
  }

  // Product: '<S5>/Product1' incorporates:
  //   DataTypeConversion: '<S5>/Data Type Conversion3'

  interceptModel_B.Product1_c *= interceptModel_B.x_error_ki;

  // Switch: '<S16>/Switch3' incorporates:
  //   Constant: '<S16>/Constant1'
  //   Constant: '<S16>/Constant2'
  //   Constant: '<S16>/Constant3'
  //   Logic: '<S16>/Logical Operator1'
  //   Logic: '<S16>/Logical Operator2'
  //   MinMax: '<S16>/MinMax1'
  //   MinMax: '<S16>/MinMax2'
  //   RelationalOperator: '<S16>/Relational Operator1'
  //   RelationalOperator: '<S16>/Relational Operator2'
  //   RelationalOperator: '<S16>/Relational Operator3'
  //   RelationalOperator: '<S16>/Relational Operator4'
  //   Sum: '<S16>/Add1'
  //   Sum: '<S16>/Add2'
  //   Switch: '<S16>/Switch2'

  if ((interceptModel_B.Add_h == interceptModel_P.Constant1_Value_ev) ||
      (interceptModel_B.DataTypeConversion5 ==
       interceptModel_P.Constant1_Value_ev)) {
    // MinMax: '<S16>/MinMax1' incorporates:
    //   Constant: '<S16>/Constant3'

    if ((!(interceptModel_B.Product1_c >= interceptModel_P.Constant3_Value_i)) &&
        (!rtIsNaN(interceptModel_P.Constant3_Value_i))) {
      interceptModel_B.Product1_c = interceptModel_P.Constant3_Value_i;
    }

    interceptModel_B.DataTypeConversion2 = interceptModel_P.Constant3_Value_i +
      interceptModel_B.Product1_c;
  } else {
    if ((interceptModel_B.Add_h == interceptModel_P.Constant2_Value_j) ||
        (interceptModel_B.DataTypeConversion5 ==
         interceptModel_P.Constant2_Value_j)) {
      // Switch: '<S16>/Switch2' incorporates:
      //   Constant: '<S16>/Constant3'

      interceptModel_B.DataTypeConversion3 = interceptModel_P.Constant3_Value_i;
    } else if ((interceptModel_B.Product1_c >=
                interceptModel_P.Constant3_Value_i) || rtIsNaN
               (interceptModel_P.Constant3_Value_i)) {
      // MinMax: '<S16>/MinMax1' incorporates:
      //   Switch: '<S16>/Switch2'

      interceptModel_B.DataTypeConversion3 = interceptModel_B.Product1_c;
    } else {
      // Switch: '<S16>/Switch2' incorporates:
      //   Constant: '<S16>/Constant3'
      //   MinMax: '<S16>/MinMax1'

      interceptModel_B.DataTypeConversion3 = interceptModel_P.Constant3_Value_i;
    }

    // MinMax: '<S16>/MinMax2' incorporates:
    //   Constant: '<S16>/Constant3'
    //   MinMax: '<S16>/MinMax1'

    if ((!(interceptModel_B.Product1_c <= interceptModel_P.Constant3_Value_i)) &&
        (!rtIsNaN(interceptModel_P.Constant3_Value_i))) {
      interceptModel_B.Product1_c = interceptModel_P.Constant3_Value_i;
    }

    interceptModel_B.DataTypeConversion2 = interceptModel_B.Product1_c +
      interceptModel_B.DataTypeConversion3;
  }

  // End of Switch: '<S16>/Switch3'
  interceptMode_MovingAverage(interceptModel_B.In1_f.target_x,
    &interceptModel_B.MovingAverage, &interceptModel_DW.MovingAverage);
  interceptModel_PX4Timestamp(&interceptModel_B.PX4Timestamp_pn);

  // MATLABSystem: '<S6>/Lowpass Filter'
  obj = interceptModel_DW.obj_a.FilterObj;
  if (interceptModel_DW.obj_a.FilterObj->isInitialized != 1) {
    interceptModel_DW.obj_a.FilterObj->isSetupComplete = false;
    interceptModel_DW.obj_a.FilterObj->isInitialized = 1;
    interceptModel_DW.obj_a.FilterObj->isSetupComplete = true;

    // System object Initialization function: dsp.FIRFilter
    for (k = 0; k < 15; k++) {
      interceptModel_DW.obj_a.FilterObj->cSFunObject.W0_states[k] =
        interceptModel_DW.obj_a.FilterObj->cSFunObject.P0_InitialStates;
    }
  }

  // System object Outputs function: dsp.FIRFilter
  // Reverse the coefficients
  for (k = 0; k < 16; k++) {
    obj->cSFunObject.W2_simRevCoeff[15 - k] = obj->cSFunObject.P1_Coefficients[k];
  }

  // Reverse copy the states from States_Dwork to ContextBuff_Dwork
  // Copy the initial part of input to ContextBuff_Dwork
  for (k = 0; k < 15; k++) {
    obj->cSFunObject.W1_simContextBuf[14 - k] = obj->cSFunObject.W0_states[k];
  }

  obj->cSFunObject.W1_simContextBuf[15] = interceptModel_B.In1_f.target_x;
  ParamStep_0 = 0.0F;
  for (k = 0; k < 16; k++) {
    ParamStep_0 += obj->cSFunObject.W2_simRevCoeff[k] *
      obj->cSFunObject.W1_simContextBuf[k];
  }

  // store output sample
  // Shift state buffer when input buffer is shorter than state buffer
  for (k = 13; k >= 0; k--) {
    obj->cSFunObject.W0_states[k + 1] = obj->cSFunObject.W0_states[k];
  }

  // Reverse copy the states from input to States_Dwork
  obj->cSFunObject.W0_states[0] = interceptModel_B.In1_f.target_x;
  interceptMode_MovingAverage(interceptModel_B.In1_f.target_x,
    &interceptModel_B.MovingAverage1_p, &interceptModel_DW.MovingAverage1_p);

  // MATLABSystem: '<S6>/Lowpass Filter1'
  obj_0 = interceptModel_DW.obj.FilterObj;
  if (interceptModel_DW.obj.FilterObj->isInitialized != 1) {
    interceptModel_DW.obj.FilterObj->isSetupComplete = false;
    interceptModel_DW.obj.FilterObj->isInitialized = 1;
    interceptModel_DW.obj.FilterObj->isSetupComplete = true;

    // System object Initialization function: dsp.FIRFilter
    for (k = 0; k < 38; k++) {
      interceptModel_DW.obj.FilterObj->cSFunObject.W0_states[k] =
        interceptModel_DW.obj.FilterObj->cSFunObject.P0_InitialStates;
    }
  }

  // System object Outputs function: dsp.FIRFilter
  // Reverse the coefficients
  for (k = 0; k < 39; k++) {
    obj_0->cSFunObject.W2_simRevCoeff[38 - k] =
      obj_0->cSFunObject.P1_Coefficients[k];
  }

  // Reverse copy the states from States_Dwork to ContextBuff_Dwork
  // Copy the initial part of input to ContextBuff_Dwork
  for (k = 0; k < 38; k++) {
    obj_0->cSFunObject.W1_simContextBuf[37 - k] = obj_0->cSFunObject.W0_states[k];
  }

  obj_0->cSFunObject.W1_simContextBuf[38] =
    interceptModel_B.MovingAverage1_p.MovingAverage;
  csum = 0.0F;
  for (k = 0; k < 39; k++) {
    csum += obj_0->cSFunObject.W2_simRevCoeff[k] *
      obj_0->cSFunObject.W1_simContextBuf[k];
  }

  // store output sample
  // Shift state buffer when input buffer is shorter than state buffer
  for (k = 36; k >= 0; k--) {
    obj_0->cSFunObject.W0_states[k + 1] = obj_0->cSFunObject.W0_states[k];
  }

  // Reverse copy the states from input to States_Dwork
  obj_0->cSFunObject.W0_states[0] =
    interceptModel_B.MovingAverage1_p.MovingAverage;

  // BusAssignment: '<S6>/Bus Assignment' incorporates:
  //   Constant: '<S17>/Constant'

  interceptModel_B.BusAssignment = interceptModel_P.Constant_Value_k;

  // BusAssignment: '<S6>/Bus Assignment' incorporates:
  //   DataTypeConversion: '<S6>/Data Type Conversion2'
  //   MATLABSystem: '<S6>/Lowpass Filter'
  //   MATLABSystem: '<S6>/Lowpass Filter1'
  //   MATLABSystem: '<S6>/PX4 Timestamp'

  interceptModel_B.BusAssignment.timestamp =
    interceptModel_B.PX4Timestamp_pn.PX4Timestamp;
  interceptModel_B.BusAssignment.double_a =
    interceptModel_B.MovingAverage.MovingAverage;
  interceptModel_B.BusAssignment.double_b = ParamStep_0;
  interceptModel_B.BusAssignment.single_a = csum;

  // MATLABSystem: '<S20>/SinkBlock' incorporates:
  //   BusAssignment: '<S6>/Bus Assignment'

  uORB_write_step(interceptModel_DW.obj_g.orbMetadataObj,
                  &interceptModel_DW.obj_g.orbAdvertiseObj,
                  &interceptModel_B.BusAssignment);

  // MATLABSystem: '<S23>/SourceBlock'
  uORB_read_step(interceptModel_DW.obj_f.orbMetadataObj,
                 &interceptModel_DW.obj_f.eventStructObj, &interceptModel_B.r,
                 false, 1.0);

  // MATLABSystem: '<S19>/SourceBlock'
  uORB_read_step(interceptModel_DW.obj_h.orbMetadataObj,
                 &interceptModel_DW.obj_h.eventStructObj, &interceptModel_B.r2,
                 false, 1.0);

  // Update for Delay: '<S13>/Delay_Integration' incorporates:
  //   Constant: '<S13>/Constant'
  //   Product: '<S13>/Product1'
  //   Sum: '<S13>/Add1'

  interceptModel_DW.icLoad_p = false;
  interceptModel_DW.Delay_Integration_DSTATE +=
    interceptModel_B.DataTypeConversion2 * interceptModel_P.Constant_Value_fl;

  // Update for Delay: '<S9>/Delay_Integration' incorporates:
  //   Sum: '<S9>/Add1'

  interceptModel_DW.icLoad_i = false;
  interceptModel_DW.Delay_Integration_DSTATE_h += interceptModel_B.Product1_pd;
}

// Model step function for TID1
void interceptModel_step1(void)        // Sample time: [0.5s, 0.0s]
{
  real32_T rtb_int_reset_y;
  real32_T rtb_roll_lim_x;
  real32_T rtb_roll_lim_y;
  real32_T rtb_x_error_ki;
  real32_T rtb_x_error_kp;
  real32_T rtb_x_int_lim;
  real32_T rtb_y_error_ki;
  real32_T rtb_y_error_kp;
  real32_T rtb_y_int_lim;
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
  b_varargout_2 = MW_Param_Step(interceptModel_DW.obj_d5.MW_PARAMHANDLE,
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
  b_varargout_2 = MW_Param_Step(interceptModel_DW.obj_m.MW_PARAMHANDLE,
    MW_SINGLE, &rtb_y_error_kp);
  if (b_varargout_2) {
    rtb_y_error_kp = 0.0F;
  }

  // End of MATLABSystem: '<S3>/Read Parameter5'

  // RateTransition generated from: '<Root>/Rate Transition'
  interceptModel_DW.RateTransition_6_Buffer0 = rtb_y_error_kp;

  // MATLABSystem: '<S3>/Read Parameter6'
  b_varargout_2 = MW_Param_Step(interceptModel_DW.obj_l.MW_PARAMHANDLE,
    MW_SINGLE, &rtb_y_error_ki);
  if (b_varargout_2) {
    rtb_y_error_ki = 0.0F;
  }

  // End of MATLABSystem: '<S3>/Read Parameter6'

  // RateTransition generated from: '<Root>/Rate Transition'
  interceptModel_DW.RateTransition_7_Buffer0 = rtb_y_error_ki;

  // MATLABSystem: '<S3>/Read Parameter7'
  b_varargout_2 = MW_Param_Step(interceptModel_DW.obj_km.MW_PARAMHANDLE,
    MW_SINGLE, &rtb_y_int_lim);
  if (b_varargout_2) {
    rtb_y_int_lim = 0.0F;
  }

  // End of MATLABSystem: '<S3>/Read Parameter7'

  // RateTransition generated from: '<Root>/Rate Transition'
  interceptModel_DW.RateTransition_8_Buffer0 = rtb_y_int_lim;

  // MATLABSystem: '<S3>/Read Parameter8'
  b_varargout_2 = MW_Param_Step(interceptModel_DW.obj_ov.MW_PARAMHANDLE,
    MW_SINGLE, &rtb_int_reset_y);
  if (b_varargout_2) {
    rtb_int_reset_y = 0.0F;
  }

  // End of MATLABSystem: '<S3>/Read Parameter8'

  // RateTransition generated from: '<Root>/Rate Transition'
  interceptModel_DW.RateTransition_9_Buffer0 = rtb_int_reset_y;

  // MATLABSystem: '<S3>/Read Parameter9'
  b_varargout_2 = MW_Param_Step(interceptModel_DW.obj_c.MW_PARAMHANDLE,
    MW_SINGLE, &rtb_roll_lim_y);
  if (b_varargout_2) {
    rtb_roll_lim_y = 0.0F;
  }

  // End of MATLABSystem: '<S3>/Read Parameter9'

  // RateTransition generated from: '<Root>/Rate Transition'
  interceptModel_DW.RateTransition_10_Buffer0 = rtb_roll_lim_y;
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
    static const char_T ParameterNameStr[12] = "INT_RESET_X";
    static const char_T ParameterNameStr_0[11] = "VIS_NAV_EN";
    static const char_T ParameterNameStr_1[11] = "X_ERROR_KP";
    static const char_T ParameterNameStr_2[11] = "X_ERROR_KI";
    static const char_T ParameterNameStr_3[10] = "X_INT_LIM";
    static const char_T ParameterNameStr_4[11] = "ROLL_LIM_X";
    static const char_T ParameterNameStr_5[11] = "Y_ERROR_KP";
    static const char_T ParameterNameStr_6[11] = "Y_ERROR_KI";
    static const char_T ParameterNameStr_7[10] = "Y_INT_LIM";
    static const char_T ParameterNameStr_8[12] = "INT_RESET_Y";
    static const char_T ParameterNameStr_9[11] = "ROLL_LIM_Y";
    int32_T i;
    static const real32_T tmp[16] = { -0.0042155534F, 0.00929741841F,
      -0.0188495331F, 0.0344127603F, -0.0595634282F, 0.103044719F, -0.196835205F,
      0.63135618F, 0.63135618F, -0.196835205F, 0.103044719F, -0.0595634282F,
      0.0344127603F, -0.0188495331F, 0.00929741841F, -0.0042155534F };

    static const real32_T tmp_0[39] = { -0.000592497177F, -0.00202371506F,
      -0.00210517156F, 0.00125815463F, 0.00458381372F, 0.000810182071F,
      -0.00749678F, -0.00598164694F, 0.00892258622F, 0.0147602707F,
      -0.00612160703F, -0.0266663674F, -0.00450636586F, 0.0400735252F,
      0.0286152121F, -0.0525242575F, -0.0823807865F, 0.0613648221F, 0.309911311F,
      0.435434699F, 0.309911311F, 0.0613648221F, -0.0823807865F, -0.0525242575F,
      0.0286152121F, 0.0400735252F, -0.00450636586F, -0.0266663674F,
      -0.00612160703F, 0.0147602707F, 0.00892258622F, -0.00598164694F,
      -0.00749678F, 0.000810182071F, 0.00458381372F, 0.00125815463F,
      -0.00210517156F, -0.00202371506F, -0.000592497177F };

    // Start for RateTransition generated from: '<Root>/Rate Transition'
    interceptModel_B.x_error_kp =
      interceptModel_P.RateTransition_1_InitialConditi;

    // Start for RateTransition generated from: '<Root>/Rate Transition'
    interceptModel_B.roll_lim_x =
      interceptModel_P.RateTransition_5_InitialConditi;

    // Start for RateTransition generated from: '<Root>/Rate Transition'
    interceptModel_B.y_error_kp =
      interceptModel_P.RateTransition_6_InitialConditi;

    // Start for RateTransition generated from: '<Root>/Rate Transition'
    interceptModel_B.int_reset_y =
      interceptModel_P.RateTransition_9_InitialConditi;

    // Start for RateTransition generated from: '<Root>/Rate Transition'
    interceptModel_B.roll_lim_y =
      interceptModel_P.RateTransition_10_InitialCondit;

    // Start for RateTransition generated from: '<Root>/Rate Transition'
    interceptModel_B.y_int_lim =
      interceptModel_P.RateTransition_8_InitialConditi;

    // Start for RateTransition generated from: '<Root>/Rate Transition'
    interceptModel_B.y_error_ki =
      interceptModel_P.RateTransition_7_InitialConditi;

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

    // InitializeConditions for Delay: '<S13>/Delay_Integration'
    interceptModel_DW.icLoad_p = true;

    // InitializeConditions for RateTransition generated from: '<Root>/Rate Transition' 
    interceptModel_DW.RateTransition_5_Buffer0 =
      interceptModel_P.RateTransition_5_InitialConditi;

    // InitializeConditions for RateTransition generated from: '<Root>/Rate Transition' 
    interceptModel_DW.RateTransition_6_Buffer0 =
      interceptModel_P.RateTransition_6_InitialConditi;

    // InitializeConditions for RateTransition generated from: '<Root>/Rate Transition' 
    interceptModel_DW.RateTransition_9_Buffer0 =
      interceptModel_P.RateTransition_9_InitialConditi;

    // InitializeConditions for Delay: '<S9>/Delay_Integration'
    interceptModel_DW.icLoad_i = true;

    // InitializeConditions for RateTransition generated from: '<Root>/Rate Transition' 
    interceptModel_DW.RateTransition_10_Buffer0 =
      interceptModel_P.RateTransition_10_InitialCondit;

    // InitializeConditions for RateLimiter: '<S4>/Rate Limiter'
    interceptModel_DW.PrevY = interceptModel_P.RateLimiter_IC;

    // InitializeConditions for RateTransition generated from: '<Root>/Rate Transition' 
    interceptModel_DW.RateTransition_8_Buffer0 =
      interceptModel_P.RateTransition_8_InitialConditi;

    // InitializeConditions for RateTransition generated from: '<Root>/Rate Transition' 
    interceptModel_DW.RateTransition_7_Buffer0 =
      interceptModel_P.RateTransition_7_InitialConditi;

    // InitializeConditions for RateTransition generated from: '<Root>/Rate Transition' 
    interceptModel_DW.RateTransition_3_Buffer0 =
      interceptModel_P.RateTransition_3_InitialConditi;

    // InitializeConditions for RateTransition generated from: '<Root>/Rate Transition' 
    interceptModel_DW.RateTransition_2_Buffer0 =
      interceptModel_P.RateTransition_2_InitialConditi;

    // SystemInitialize for Enabled SubSystem: '<S18>/Enabled Subsystem'
    // SystemInitialize for SignalConversion generated from: '<S24>/In1' incorporates:
    //   Outport: '<S24>/Out1'

    interceptModel_B.In1_f = interceptModel_P.Out1_Y0_n;

    // End of SystemInitialize for SubSystem: '<S18>/Enabled Subsystem'

    // SystemInitialize for Enabled SubSystem: '<S22>/Enabled Subsystem'
    // SystemInitialize for SignalConversion generated from: '<S68>/In1' incorporates:
    //   Outport: '<S68>/Out1'

    interceptModel_B.In1 = interceptModel_P.Out1_Y0_p;

    // End of SystemInitialize for SubSystem: '<S22>/Enabled Subsystem'

    // Start for MATLABSystem: '<S22>/SourceBlock'
    interceptModel_DW.obj_o.matlabCodegenIsDeleted = false;
    interceptModel_DW.obj_o.isInitialized = 1;
    interceptModel_DW.obj_o.orbMetadataObj = ORB_ID(vehicle_attitude);
    uORB_read_initialize(interceptModel_DW.obj_o.orbMetadataObj,
                         &interceptModel_DW.obj_o.eventStructObj);
    interceptModel_DW.obj_o.isSetupComplete = true;

    // Start for MATLABSystem: '<S18>/SourceBlock'
    interceptModel_DW.obj_e.matlabCodegenIsDeleted = false;
    interceptModel_DW.obj_e.isInitialized = 1;
    interceptModel_DW.obj_e.orbMetadataObj = ORB_ID(target_location);
    uORB_read_initialize(interceptModel_DW.obj_e.orbMetadataObj,
                         &interceptModel_DW.obj_e.eventStructObj);
    interceptModel_DW.obj_e.isSetupComplete = true;

    // Start for MATLABSystem: '<S3>/Read Parameter3'
    interceptModel_DW.obj_n.matlabCodegenIsDeleted = false;
    interceptModel_DW.obj_n.isInitialized = 1;
    interceptModel_DW.obj_n.MW_PARAMHANDLE = MW_Init_Param(&ParameterNameStr[0],
      true, 200.0);
    interceptModel_DW.obj_n.isSetupComplete = true;

    // Start for MATLABSystem: '<S3>/Read Parameter10'
    interceptModel_DW.obj_a5.matlabCodegenIsDeleted = false;
    interceptModel_DW.obj_a5.isInitialized = 1;
    interceptModel_DW.obj_a5.MW_PARAMHANDLE = MW_Init_Param(&ParameterNameStr_0
      [0], true, 200.0);
    interceptModel_DW.obj_a5.isSetupComplete = true;

    // Start for MATLABSystem: '<Root>/Moving Average1'
    interceptModel_DW.obj_d.isInitialized = 0;
    interceptModel_DW.obj_d.NumChannels = -1;
    interceptModel_DW.obj_d.FrameLength = -1;
    interceptModel_DW.obj_d.matlabCodegenIsDeleted = false;
    interceptMod_SystemCore_setup_d(&interceptModel_DW.obj_d);

    // InitializeConditions for MATLABSystem: '<Root>/Moving Average1'
    interceptModel_DW.obj_d.pCumSum = 0.0F;
    for (i = 0; i < 6; i++) {
      interceptModel_DW.obj_d.pCumSumRev[i] = 0.0F;
    }

    interceptModel_DW.obj_d.pCumRevIndex = 1.0F;
    interceptModel_DW.obj_d.pModValueRev = 0.0F;

    // End of InitializeConditions for MATLABSystem: '<Root>/Moving Average1'
    intercept_PX4Timestamp_Init(&interceptModel_DW.PX4Timestamp);

    // Start for MATLABSystem: '<S2>/SinkBlock' incorporates:
    //   BusAssignment: '<Root>/Bus Assignment'

    interceptModel_DW.obj_ei.matlabCodegenIsDeleted = false;
    interceptModel_DW.obj_ei.isInitialized = 1;
    interceptModel_DW.obj_ei.orbMetadataObj = ORB_ID(intercept_roll_pitch);
    uORB_write_initialize(interceptModel_DW.obj_ei.orbMetadataObj,
                          &interceptModel_DW.obj_ei.orbAdvertiseObj,
                          &interceptModel_B.BusAssignment_o, 1);
    interceptModel_DW.obj_ei.isSetupComplete = true;
    intercept_PX4Timestamp_Init(&interceptModel_DW.PX4Timestamp_p);

    // Start for MATLABSystem: '<S15>/SinkBlock' incorporates:
    //   BusAssignment: '<S5>/Bus Assignment1'

    interceptModel_DW.obj_k.matlabCodegenIsDeleted = false;
    interceptModel_DW.obj_k.isInitialized = 1;
    interceptModel_DW.obj_k.orbMetadataObj = ORB_ID(intercept_debug);
    uORB_write_initialize(interceptModel_DW.obj_k.orbMetadataObj,
                          &interceptModel_DW.obj_k.orbAdvertiseObj,
                          &interceptModel_B.BusAssignment1, 1);
    interceptModel_DW.obj_k.isSetupComplete = true;
    intercep_MovingAverage_Init(&interceptModel_DW.MovingAverage);
    intercept_PX4Timestamp_Init(&interceptModel_DW.PX4Timestamp_pn);

    // Start for MATLABSystem: '<S6>/Lowpass Filter'
    interceptModel_DW.obj_a.NumChannels = -1;
    interceptModel_DW.obj_a.matlabCodegenIsDeleted = false;
    interceptModel_DW.obj_a.isInitialized = 1;
    if (interceptModel_DW.obj_a.NumChannels == -1) {
      interceptModel_DW.obj_a.NumChannels = 1;
    }

    interceptModel_DW.obj_a._pobj0.isInitialized = 0;

    // System object Constructor function: dsp.FIRFilter
    interceptModel_DW.obj_a._pobj0.cSFunObject.P0_InitialStates = 0.0F;
    memcpy(&interceptModel_DW.obj_a._pobj0.cSFunObject.P1_Coefficients[0], &tmp
           [0], sizeof(real32_T) << 4U);
    interceptModel_DW.obj_a.FilterObj = &interceptModel_DW.obj_a._pobj0;
    interceptModel_DW.obj_a.isSetupComplete = true;

    // InitializeConditions for MATLABSystem: '<S6>/Lowpass Filter'
    if (interceptModel_DW.obj_a.FilterObj->isInitialized == 1) {
      // System object Initialization function: dsp.FIRFilter
      for (i = 0; i < 15; i++) {
        interceptModel_DW.obj_a.FilterObj->cSFunObject.W0_states[i] =
          interceptModel_DW.obj_a.FilterObj->cSFunObject.P0_InitialStates;
      }
    }

    // End of InitializeConditions for MATLABSystem: '<S6>/Lowpass Filter'
    intercep_MovingAverage_Init(&interceptModel_DW.MovingAverage1_p);

    // Start for MATLABSystem: '<S6>/Lowpass Filter1'
    interceptModel_DW.obj.NumChannels = -1;
    interceptModel_DW.obj.matlabCodegenIsDeleted = false;
    interceptModel_DW.obj.isInitialized = 1;
    if (interceptModel_DW.obj.NumChannels == -1) {
      interceptModel_DW.obj.NumChannels = 1;
    }

    interceptModel_DW.obj._pobj0.isInitialized = 0;

    // System object Constructor function: dsp.FIRFilter
    interceptModel_DW.obj._pobj0.cSFunObject.P0_InitialStates = 0.0F;
    memcpy(&interceptModel_DW.obj._pobj0.cSFunObject.P1_Coefficients[0], &tmp_0
           [0], 39U * sizeof(real32_T));
    interceptModel_DW.obj.FilterObj = &interceptModel_DW.obj._pobj0;
    interceptModel_DW.obj.isSetupComplete = true;

    // InitializeConditions for MATLABSystem: '<S6>/Lowpass Filter1'
    if (interceptModel_DW.obj.FilterObj->isInitialized == 1) {
      // System object Initialization function: dsp.FIRFilter
      for (i = 0; i < 38; i++) {
        interceptModel_DW.obj.FilterObj->cSFunObject.W0_states[i] =
          interceptModel_DW.obj.FilterObj->cSFunObject.P0_InitialStates;
      }
    }

    // End of InitializeConditions for MATLABSystem: '<S6>/Lowpass Filter1'

    // Start for MATLABSystem: '<S20>/SinkBlock' incorporates:
    //   BusAssignment: '<S6>/Bus Assignment'

    interceptModel_DW.obj_g.matlabCodegenIsDeleted = false;
    interceptModel_DW.obj_g.isInitialized = 1;
    interceptModel_DW.obj_g.orbMetadataObj = ORB_ID(simulink_custom_message);
    uORB_write_initialize(interceptModel_DW.obj_g.orbMetadataObj,
                          &interceptModel_DW.obj_g.orbAdvertiseObj,
                          &interceptModel_B.BusAssignment, 1);
    interceptModel_DW.obj_g.isSetupComplete = true;

    // Start for MATLABSystem: '<S23>/SourceBlock'
    interceptModel_DW.obj_f.matlabCodegenIsDeleted = false;
    interceptModel_DW.obj_f.isInitialized = 1;
    interceptModel_DW.obj_f.orbMetadataObj = ORB_ID(vehicle_local_position);
    uORB_read_initialize(interceptModel_DW.obj_f.orbMetadataObj,
                         &interceptModel_DW.obj_f.eventStructObj);
    interceptModel_DW.obj_f.isSetupComplete = true;

    // Start for MATLABSystem: '<S19>/SourceBlock'
    interceptModel_DW.obj_h.matlabCodegenIsDeleted = false;
    interceptModel_DW.obj_h.isInitialized = 1;
    interceptModel_DW.obj_h.orbMetadataObj = ORB_ID(airspeed_validated);
    uORB_read_initialize(interceptModel_DW.obj_h.orbMetadataObj,
                         &interceptModel_DW.obj_h.eventStructObj);
    interceptModel_DW.obj_h.isSetupComplete = true;

    // Start for MATLABSystem: '<S3>/Read Parameter'
    interceptModel_DW.obj_oq.matlabCodegenIsDeleted = false;
    interceptModel_DW.obj_oq.isInitialized = 1;
    interceptModel_DW.obj_oq.MW_PARAMHANDLE = MW_Init_Param(&ParameterNameStr_1
      [0], true, 500.0);
    interceptModel_DW.obj_oq.isSetupComplete = true;

    // Start for MATLABSystem: '<S3>/Read Parameter1'
    interceptModel_DW.obj_d5.matlabCodegenIsDeleted = false;
    interceptModel_DW.obj_d5.isInitialized = 1;
    interceptModel_DW.obj_d5.MW_PARAMHANDLE = MW_Init_Param(&ParameterNameStr_2
      [0], true, 500.0);
    interceptModel_DW.obj_d5.isSetupComplete = true;

    // Start for MATLABSystem: '<S3>/Read Parameter2'
    interceptModel_DW.obj_lm.matlabCodegenIsDeleted = false;
    interceptModel_DW.obj_lm.isInitialized = 1;
    interceptModel_DW.obj_lm.MW_PARAMHANDLE = MW_Init_Param(&ParameterNameStr_3
      [0], true, 500.0);
    interceptModel_DW.obj_lm.isSetupComplete = true;

    // Start for MATLABSystem: '<S3>/Read Parameter4'
    interceptModel_DW.obj_p.matlabCodegenIsDeleted = false;
    interceptModel_DW.obj_p.isInitialized = 1;
    interceptModel_DW.obj_p.MW_PARAMHANDLE = MW_Init_Param(&ParameterNameStr_4[0],
      true, 500.0);
    interceptModel_DW.obj_p.isSetupComplete = true;

    // Start for MATLABSystem: '<S3>/Read Parameter5'
    interceptModel_DW.obj_m.matlabCodegenIsDeleted = false;
    interceptModel_DW.obj_m.isInitialized = 1;
    interceptModel_DW.obj_m.MW_PARAMHANDLE = MW_Init_Param(&ParameterNameStr_5[0],
      true, 500.0);
    interceptModel_DW.obj_m.isSetupComplete = true;

    // Start for MATLABSystem: '<S3>/Read Parameter6'
    interceptModel_DW.obj_l.matlabCodegenIsDeleted = false;
    interceptModel_DW.obj_l.isInitialized = 1;
    interceptModel_DW.obj_l.MW_PARAMHANDLE = MW_Init_Param(&ParameterNameStr_6[0],
      true, 500.0);
    interceptModel_DW.obj_l.isSetupComplete = true;

    // Start for MATLABSystem: '<S3>/Read Parameter7'
    interceptModel_DW.obj_km.matlabCodegenIsDeleted = false;
    interceptModel_DW.obj_km.isInitialized = 1;
    interceptModel_DW.obj_km.MW_PARAMHANDLE = MW_Init_Param(&ParameterNameStr_7
      [0], true, 500.0);
    interceptModel_DW.obj_km.isSetupComplete = true;

    // Start for MATLABSystem: '<S3>/Read Parameter8'
    interceptModel_DW.obj_ov.matlabCodegenIsDeleted = false;
    interceptModel_DW.obj_ov.isInitialized = 1;
    interceptModel_DW.obj_ov.MW_PARAMHANDLE = MW_Init_Param(&ParameterNameStr_8
      [0], true, 500.0);
    interceptModel_DW.obj_ov.isSetupComplete = true;

    // Start for MATLABSystem: '<S3>/Read Parameter9'
    interceptModel_DW.obj_c.matlabCodegenIsDeleted = false;
    interceptModel_DW.obj_c.isInitialized = 1;
    interceptModel_DW.obj_c.MW_PARAMHANDLE = MW_Init_Param(&ParameterNameStr_9[0],
      true, 500.0);
    interceptModel_DW.obj_c.isSetupComplete = true;
  }
}

// Model terminate function
void interceptModel_terminate(void)
{
  // Terminate for MATLABSystem: '<S22>/SourceBlock'
  if (!interceptModel_DW.obj_o.matlabCodegenIsDeleted) {
    interceptModel_DW.obj_o.matlabCodegenIsDeleted = true;
    if ((interceptModel_DW.obj_o.isInitialized == 1) &&
        interceptModel_DW.obj_o.isSetupComplete) {
      uORB_read_terminate(&interceptModel_DW.obj_o.eventStructObj);
    }
  }

  // End of Terminate for MATLABSystem: '<S22>/SourceBlock'

  // Terminate for MATLABSystem: '<S18>/SourceBlock'
  if (!interceptModel_DW.obj_e.matlabCodegenIsDeleted) {
    interceptModel_DW.obj_e.matlabCodegenIsDeleted = true;
    if ((interceptModel_DW.obj_e.isInitialized == 1) &&
        interceptModel_DW.obj_e.isSetupComplete) {
      uORB_read_terminate(&interceptModel_DW.obj_e.eventStructObj);
    }
  }

  // End of Terminate for MATLABSystem: '<S18>/SourceBlock'

  // Terminate for MATLABSystem: '<S3>/Read Parameter3'
  if (!interceptModel_DW.obj_n.matlabCodegenIsDeleted) {
    interceptModel_DW.obj_n.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S3>/Read Parameter3'

  // Terminate for MATLABSystem: '<S3>/Read Parameter10'
  if (!interceptModel_DW.obj_a5.matlabCodegenIsDeleted) {
    interceptModel_DW.obj_a5.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S3>/Read Parameter10'

  // Terminate for MATLABSystem: '<Root>/Moving Average1'
  if (!interceptModel_DW.obj_d.matlabCodegenIsDeleted) {
    interceptModel_DW.obj_d.matlabCodegenIsDeleted = true;
    if ((interceptModel_DW.obj_d.isInitialized == 1) &&
        interceptModel_DW.obj_d.isSetupComplete) {
      interceptModel_DW.obj_d.NumChannels = -1;
      interceptModel_DW.obj_d.FrameLength = -1;
    }
  }

  // End of Terminate for MATLABSystem: '<Root>/Moving Average1'
  intercept_PX4Timestamp_Term(&interceptModel_DW.PX4Timestamp);

  // Terminate for MATLABSystem: '<S2>/SinkBlock'
  if (!interceptModel_DW.obj_ei.matlabCodegenIsDeleted) {
    interceptModel_DW.obj_ei.matlabCodegenIsDeleted = true;
    if ((interceptModel_DW.obj_ei.isInitialized == 1) &&
        interceptModel_DW.obj_ei.isSetupComplete) {
      uORB_write_terminate(&interceptModel_DW.obj_ei.orbAdvertiseObj);
    }
  }

  // End of Terminate for MATLABSystem: '<S2>/SinkBlock'
  intercept_PX4Timestamp_Term(&interceptModel_DW.PX4Timestamp_p);

  // Terminate for MATLABSystem: '<S15>/SinkBlock'
  if (!interceptModel_DW.obj_k.matlabCodegenIsDeleted) {
    interceptModel_DW.obj_k.matlabCodegenIsDeleted = true;
    if ((interceptModel_DW.obj_k.isInitialized == 1) &&
        interceptModel_DW.obj_k.isSetupComplete) {
      uORB_write_terminate(&interceptModel_DW.obj_k.orbAdvertiseObj);
    }
  }

  // End of Terminate for MATLABSystem: '<S15>/SinkBlock'
  intercep_MovingAverage_Term(&interceptModel_DW.MovingAverage);
  intercept_PX4Timestamp_Term(&interceptModel_DW.PX4Timestamp_pn);

  // Terminate for MATLABSystem: '<S6>/Lowpass Filter'
  if (!interceptModel_DW.obj_a.matlabCodegenIsDeleted) {
    interceptModel_DW.obj_a.matlabCodegenIsDeleted = true;
    if ((interceptModel_DW.obj_a.isInitialized == 1) &&
        interceptModel_DW.obj_a.isSetupComplete) {
      if (interceptModel_DW.obj_a.FilterObj->isInitialized == 1) {
        interceptModel_DW.obj_a.FilterObj->isInitialized = 2;
      }

      interceptModel_DW.obj_a.NumChannels = -1;
    }
  }

  // End of Terminate for MATLABSystem: '<S6>/Lowpass Filter'
  intercep_MovingAverage_Term(&interceptModel_DW.MovingAverage1_p);

  // Terminate for MATLABSystem: '<S6>/Lowpass Filter1'
  if (!interceptModel_DW.obj.matlabCodegenIsDeleted) {
    interceptModel_DW.obj.matlabCodegenIsDeleted = true;
    if ((interceptModel_DW.obj.isInitialized == 1) &&
        interceptModel_DW.obj.isSetupComplete) {
      if (interceptModel_DW.obj.FilterObj->isInitialized == 1) {
        interceptModel_DW.obj.FilterObj->isInitialized = 2;
      }

      interceptModel_DW.obj.NumChannels = -1;
    }
  }

  // End of Terminate for MATLABSystem: '<S6>/Lowpass Filter1'

  // Terminate for MATLABSystem: '<S20>/SinkBlock'
  if (!interceptModel_DW.obj_g.matlabCodegenIsDeleted) {
    interceptModel_DW.obj_g.matlabCodegenIsDeleted = true;
    if ((interceptModel_DW.obj_g.isInitialized == 1) &&
        interceptModel_DW.obj_g.isSetupComplete) {
      uORB_write_terminate(&interceptModel_DW.obj_g.orbAdvertiseObj);
    }
  }

  // End of Terminate for MATLABSystem: '<S20>/SinkBlock'

  // Terminate for MATLABSystem: '<S23>/SourceBlock'
  if (!interceptModel_DW.obj_f.matlabCodegenIsDeleted) {
    interceptModel_DW.obj_f.matlabCodegenIsDeleted = true;
    if ((interceptModel_DW.obj_f.isInitialized == 1) &&
        interceptModel_DW.obj_f.isSetupComplete) {
      uORB_read_terminate(&interceptModel_DW.obj_f.eventStructObj);
    }
  }

  // End of Terminate for MATLABSystem: '<S23>/SourceBlock'

  // Terminate for MATLABSystem: '<S19>/SourceBlock'
  if (!interceptModel_DW.obj_h.matlabCodegenIsDeleted) {
    interceptModel_DW.obj_h.matlabCodegenIsDeleted = true;
    if ((interceptModel_DW.obj_h.isInitialized == 1) &&
        interceptModel_DW.obj_h.isSetupComplete) {
      uORB_read_terminate(&interceptModel_DW.obj_h.eventStructObj);
    }
  }

  // End of Terminate for MATLABSystem: '<S19>/SourceBlock'

  // Terminate for MATLABSystem: '<S3>/Read Parameter'
  if (!interceptModel_DW.obj_oq.matlabCodegenIsDeleted) {
    interceptModel_DW.obj_oq.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S3>/Read Parameter'

  // Terminate for MATLABSystem: '<S3>/Read Parameter1'
  if (!interceptModel_DW.obj_d5.matlabCodegenIsDeleted) {
    interceptModel_DW.obj_d5.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S3>/Read Parameter1'

  // Terminate for MATLABSystem: '<S3>/Read Parameter2'
  if (!interceptModel_DW.obj_lm.matlabCodegenIsDeleted) {
    interceptModel_DW.obj_lm.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S3>/Read Parameter2'

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
  if (!interceptModel_DW.obj_km.matlabCodegenIsDeleted) {
    interceptModel_DW.obj_km.matlabCodegenIsDeleted = true;
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
