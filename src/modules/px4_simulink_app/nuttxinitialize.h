/* Copyright 2018-2024 The MathWorks, Inc. */

#ifndef NUTTXINITIALIZE_H_
#define NUTTXINITIALIZE_H_

#if defined(MW_PX4_NUTTX_BUILD)
#define _GNU_SOURCE
#include <nuttx/config.h>
#include <nuttx/sched.h>
#endif

#include <include/visibility.h>
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <sys/types.h>
#include <unistd.h>
#include <pthread.h>
#include <sched.h>
#include <semaphore.h>
#include <errno.h>
#include <signal.h>
#include <time.h>

/*Other common includes*/
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>

#include <string.h>
#include "drivers/drv_hrt.h"

/* Required Coder Target header */
#ifndef PIL
#include "MW_custom_RTOS_header.h"
#else
#include "MW_PIL_custom_RTOS_header.h"
#endif
#include "MW_target_hardware_resources.h"

#define BASERATE_TIMER_SIGNAL 17
// #define BASERATE_TIMER_SIGNAL SIGRTMIN
#define SIGVALUE_INT 42
#define TERMTASK_STACK_SIZE 2048
#define SCHEDTASK_STACK_SIZE 2048

#define CHECK_STATUS(status, expStatus, fcn)                                \
    {                                                                       \
        int var = status;                                                   \
        if (var != expStatus) {                                             \
            PX4_INFO("Call to %s returned error status (%d).\n", fcn, var); \
            perror(fcn);                                                    \
            fflush(stderr);                                                 \
        }                                                                   \
    }

#define CHECK_STATUS_NOT(status, errStatus, fcn)                          \
    {                                                                     \
        int var = status;                                                 \
        if (var == errStatus) {                                           \
            PX4_INFO("Call to %s returned error status (%d).", fcn, var); \
            perror(fcn);                                                  \
            fflush(stderr);                                               \
        }                                                                 \
    }

typedef struct {
    int sigNo;
    sigset_t sigMask;
    double period;
} baseRateInfo_t; // this struct is used only to create a background task for external mode

#ifdef __cplusplus

extern "C" {

#endif

// extern void backgroundTask(void* arg);

/* Declare function prototypes here */
void nuttxRTOSInit(double baseRatePeriod, int numSubrates);
void myWaitForThisEvent(int sigNo);
void myAddBlockForThisEvent(int sigNo);
void setTaskPeriod(double periodInSeconds, int sigNo);
void schedulerTask(void* arg);
#ifdef __cplusplus
}
#endif

#endif /* NUTTXINITIALIZE_H_ */
