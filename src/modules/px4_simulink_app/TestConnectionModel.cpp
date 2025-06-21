/* Copyright 2022-2023 The MathWorks, Inc. */
//
// Prerelease License - for engineering feedback and testing purposes
// only. Not for sale.
//
// File: TestConnectionModel.cpp
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
#include "TestConnectionModel.h"

// External outputs (root outports fed by signals with default storage)
ExtY_TestConnectionModel_T TestConnectionModel_Y;

// Real-time model
RT_MODEL_TestConnectionModel_T TestConnectionModel_M_ = RT_MODEL_TestConnectionModel_T();
RT_MODEL_TestConnectionModel_T* const TestConnectionModel_M = &TestConnectionModel_M_;

// Model step function
void TestConnectionModel_step(void) {
    // Outport: '<Root>/Out1' incorporates:
    //   Constant: '<Root>/Constant'
    //   Gain: '<Root>/Gain'

    TestConnectionModel_Y.Out1 =
        TestConnectionModel_P.Gain_Gain * TestConnectionModel_P.Constant_Value;
}

// Model initialize function
void TestConnectionModel_initialize(void) {
    // (no initialization code required)
}

// Model terminate function
void TestConnectionModel_terminate(void) {
    // (no terminate code required)
}

//
// File trailer for generated code.
//
// [EOF]
//
