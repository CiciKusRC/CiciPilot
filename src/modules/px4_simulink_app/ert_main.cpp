/* Copyright 2018-2023 The MathWorks, Inc. */
//
// Prerelease License - for engineering feedback and testing purposes
// only. Not for sale.
//
// File: ert_main.cpp
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
#include <stdio.h>
#include <stdlib.h>
#include "TestConnectionModel.h"
#include "TestConnectionModel_private.h"
#include "rtwtypes.h"
#include "limits.h"
#include "MW_PX4_TaskControl.h"
#include "nuttxinitialize.h"
#define UNUSED(x) x = x
#define NAMELEN 16

// Function prototype declaration
void exitFcn(int sig);
void* terminateTask(void* arg);
void* baseRateTask(void* arg);
void* subrateTask(void* arg);
volatile boolean_T stopRequested = false;
volatile boolean_T runModel = true;
px4_sem_t stopSem;
px4_sem_t baserateTaskSem;
pthread_t schedulerThread;
pthread_t baseRateThread;
void* threadJoinStatus;
int terminatingmodel = 0;
void* baseRateTask(void* arg) {
    runModel = (rtmGetErrorStatus(TestConnectionModel_M) == (NULL));
    while (runModel) {
        px4_sem_wait(&baserateTaskSem);
        TestConnectionModel_step();

        // Get model outputs here
        stopRequested = !((rtmGetErrorStatus(TestConnectionModel_M) == (NULL)));
    }

    runModel = 0;
    terminateTask(arg);
    pthread_exit((void*)0);
    return NULL;
}

void exitFcn(int sig) {
    UNUSED(sig);
    rtmSetErrorStatus(TestConnectionModel_M, "stopping the model");
    runModel = 0;
}

void* terminateTask(void* arg) {
    UNUSED(arg);
    terminatingmodel = 1;

    { runModel = 0; }

    MW_PX4_Terminate();

    // Terminate model
    TestConnectionModel_terminate();
    px4_sem_post(&stopSem);
    return NULL;
}

int px4_simulink_app_task_main(int argc, char* argv[]) {
    px4_simulink_app_control_MAVLink();
    rtmSetErrorStatus(TestConnectionModel_M, 0);

    // Initialize model
    TestConnectionModel_initialize();

    // Call RTOS Initialization function
    nuttxRTOSInit(0.2, 0);

    // Wait for stop semaphore
    px4_sem_wait(&stopSem);

#if (MW_NUMBER_TIMER_DRIVEN_TASKS > 0)

    {
        int i;
        for (i = 0; i < MW_NUMBER_TIMER_DRIVEN_TASKS; i++) {
            CHECK_STATUS(px4_sem_destroy(&timerTaskSem[i]), 0, "px4_sem_destroy");
        }
    }

#endif

    return 0;
}

//
// File trailer for generated code.
//
// [EOF]
//
