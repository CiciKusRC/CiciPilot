/* Copyright 2019-2022 The MathWorks, Inc. */


#include "xil_interface_lib.h"
#include "nuttxinitialize.h"
#include "MW_PX4_TaskControl.h"

void* baseRateTask(void* arg);

px4_sem_t baserateTaskSem;
px4_sem_t stopSem;
pthread_t baseRateThread;
int terminatingmodel = 0;
extern struct hrt_call BaseRate_HRT;

XIL_INTERFACE_LIB_ERROR_CODE errorCode = XIL_INTERFACE_LIB_SUCCESS;
int errorOccurred = 0;

void* baseRateTask(void* arg) {

    /* XIL initialization */
    const int argc = 0;
    void* argv = (void*)0;
    errorCode = xilInit(argc, argv);
    errorOccurred = (errorCode != XIL_INTERFACE_LIB_SUCCESS);

    /* main XIL loop */
    while (!errorOccurred) {

        px4_sem_wait(&baserateTaskSem);
        errorCode = xilRun();
        if (errorCode != XIL_INTERFACE_LIB_SUCCESS) {
            if (errorCode == XIL_INTERFACE_LIB_TERMINATE) {
                /* orderly shutdown of rtiostream */
                xilTerminateComms();
            }
            errorOccurred = 1;
        }
    }
    // terminating
    terminatingmodel = 1;
    px4_sem_post(&stopSem);
}

int px4_simulink_app_task_main(int argc, char* argv[]) {
    // stop MAVLink if enabled
    px4_simulink_app_control_MAVLink();

    // Call RTOS Initialization function
    // nuttxRTOSInit(MW_BASERATE_PERIOD,MW_NUMBER_SUBRATES);
    nuttxRTOSInit(0.001, 0);
    px4_sem_wait(&stopSem);
    hrt_cancel(&BaseRate_HRT);
    return 0;
}



// LocalWords:  XIL rtiostream MAV RTOS nuttx BASERATE SUBRATES
