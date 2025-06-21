/* Copyright 2022-2023 The MathWorks, Inc. */
//
// Prerelease License - for engineering feedback and testing purposes
// only. Not for sale.
//
// File: TestConnectionModel.h
//
// Code generated for Simulink model 'TestConnectionModel'.
//
// Model version                  : 1.3
// Simulink Coder version         : 23.2 (R2023b) 19-Apr-2023
// C/C++ source code generated on : Wed Jun  7 20:25:59 2023
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_TestConnectionModel_h_
#define RTW_HEADER_TestConnectionModel_h_
#include "rtwtypes.h"
#include "TestConnectionModel_types.h"
#include <stddef.h>

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm) ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val) ((rtm)->errorStatus = (val))
#endif

// External outputs (root outports fed by signals with default storage)
struct ExtY_TestConnectionModel_T {
    real_T Out1; // '<Root>/Out1'
};

// Parameters (default storage)
struct P_TestConnectionModel_T_ {
    real_T Constant_Value; // Expression: 1
                           //  Referenced by: '<Root>/Constant'

    real_T Gain_Gain; // Expression: 1
                      //  Referenced by: '<Root>/Gain'
};

// Real-time Model Data Structure
struct tag_RTM_TestConnectionModel_T {
    const char_T* volatile errorStatus;
};

// Block parameters (default storage)
#ifdef __cplusplus

extern "C" {

#endif

extern P_TestConnectionModel_T TestConnectionModel_P;

#ifdef __cplusplus
}

#endif

#ifdef __cplusplus

extern "C" {

#endif

// External outputs (root outports fed by signals with default storage)
extern struct ExtY_TestConnectionModel_T TestConnectionModel_Y;

#ifdef __cplusplus
}

#endif

#ifdef __cplusplus

extern "C" {

#endif

// Model entry point functions
extern void TestConnectionModel_initialize(void);
extern void TestConnectionModel_step(void);
extern void TestConnectionModel_terminate(void);

#ifdef __cplusplus
}

#endif

// Real-time Model object
#ifdef __cplusplus

extern "C" {

#endif

extern RT_MODEL_TestConnectionModel_T* const TestConnectionModel_M;

#ifdef __cplusplus
}

#endif

extern volatile boolean_T stopRequested;
extern volatile boolean_T runModel;

//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'TestConnectionModel'

#endif // RTW_HEADER_TestConnectionModel_h_

//
// File trailer for generated code.
//
// [EOF]
//
