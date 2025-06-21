/*
 * Copyright 2018-2024 The MathWorks, Inc.
 *
 * File: MW_PX4_TaskControl.c
 *
 * Abstract:
 *  This file contains the main application for Simulink that is launched
 *  by PX4 stack at boot up. The main function spawns a new task and assigns
 *  the main function in ert_main.c as callback.
 *
 */

#include "nuttxinitialize.h"
#include "MW_PX4_TaskControl.h"
/*Simulink model generated code specific headers*/

#define MW_StringifyDefine(x) MW_StringifyDefineExpanded(x)
#define MW_StringifyDefineExpanded(x) #x

#define MW_StringifyDefineFunction(x, y) MW_StringifyDefineExpandedFunction(x, y)
#define MW_StringifyDefineExpandedFunction(x, y) x##y

#define MW_StringifyDefineX(x) MW_StringifyDefineExpandedX(x)
#define MW_StringifyDefineExpandedX(x) x.##h

#define MW_StringifyDefineTypesX(x) MW_StringifyDefineExpandedTypesX(x)
#define MW_StringifyDefineExpandedTypesX(x) x##_types.h

#define MW_StringifyDefinePrivateX(x) MW_StringifyDefineExpandedPrivateX(x)
#define MW_StringifyDefineExpandedPrivateX(x) x##_private.h

#include MW_StringifyDefine(MODEL.h)
#include MW_StringifyDefine(MW_StringifyDefineTypesX(MODEL))
#include MW_StringifyDefine(MW_StringifyDefinePrivateX(MODEL))

#ifndef MW_PX4_DISABLE_MAVLINK
#include "mavlink_main.h"
#endif

#define DEBUG 0

#ifdef PIL
extern int errorOccurred;
#endif

extern "C" __EXPORT int px4_simulink_app_main(int argc,
                                              char* argv[]); /* sbcheck:ok:extern_c needed */

extern struct hrt_call
    BaseRate_HRT; /* Base-rate HRT-callback is used to post base-rate semaphore */
extern int terminatingmodel;
extern int baserate_sem_copy; /* used for checking HRT semaphore water-mark*/
static bool g_baseRateLife =
    false;                        /* global storage to contain the status of the px4_simulink_app */
static int px4_simulink_app_task; /* Handle of daemon thread */
char mavlink_shell_started = 0;

/* Print the correct usage. */
void px4_app_usage(const char* reason) {
    if (reason) {
        PX4_INFO("px4_simulink_app : %s\n", reason);
    }
    errx(1, "usage: px4_simulink_app {start|stop|status} [-p <additional params>]\n\n");
}

#if defined(MW_PX4_DISABLE_MAVLINK)
uint8_t buffer[100];
MavlinkShell shell;
void shellWait(MavlinkShell* shell_ptr, uint8_t buf[]) {
    /*The shell wait here is to be used for commands which are expected to
      return a few characters (a max. of 100 characters.)*/
    int read = 0, readTotal = 0;
    for (int i = 0; i < 100; i++) {
        buf[i] = 0;
    }
    while (strstr((char*)buf, "nsh>") == NULL) {
        read = shell_ptr->read(buf + readTotal, 100 - readTotal);
        readTotal = readTotal + read;
    }
}
#endif

void px4_simulink_app_control_MAVLink() {
    if (mavlink_shell_started == 0) {
#ifdef MW_PX4_DISABLE_MAVLINK
        shell.start();
        shellWait(&shell, buffer);
#endif
    }

#ifdef MW_PX4_DISABLE_MAVLINK
    shell.write((uint8_t*)"mavlink stop-all\n", sizeof("mavlink stop-all\n"));
    usleep(100000);
#endif
}

int px4_simulink_app_main(int argc, char* argv[]) {
    if (argc < 2) {
        px4_app_usage("missing command");
    }

    if (!strcmp(argv[1], "start")) {
        if (g_baseRateLife == false) {
            /* Start the Simulink Tasks here */
#if DEBUG
            printf("px4_simulink_app : Starting the Simulink model\n");
            fflush(stdout);
#endif

            /* Reset semaphore */
            g_baseRateLife = true;
            terminatingmodel =
                0; // reset global variable. For more info on the rational behind this, please see:
            // http://nuttx.org/doku.php?id=wiki:nxinternal:tasks-vs-processes#nuttx_flat-build_behavior

            px4_simulink_app_task = px4_task_spawn_cmd(
                "px4_simulink_app_task", /* Definition of px4_task_spawn_cmd :
                                            C:\px4\Firmware\src\platforms\nuttx\px4_layer\px4_nuttx_tasks.c
                                          */
                SCHED_DEFAULT,           /* For STM32 F4, NuttX has SCHED_DEFAULT as SCHED_RR */
                SCHED_PRIORITY_MAX - 15, /* SCHED_PRIORITY_MAX: 255 */
                2048, px4_simulink_app_task_main, (char* const*)NULL);

        } else {
            warnx("px4_simulink_app : Simulink model is already running\n");
            fflush(stdout);
        }
#if defined(MW_PX4_NUTTX_BUILD)
        exit(0);
#else
        return 0;
#endif
    }

    if (!strcmp(argv[1], "stop")) {
        if (g_baseRateLife == true) {
#ifdef PIL
            errorOccurred = 1;
#else

#if defined(rtmSetErrorStatus)
            rtmSetErrorStatus(MW_StringifyDefineFunction(MODEL, _M), "Module stopped by user");
#else
            MW_StringifyDefineFunction(MODEL, _M)->setErrorStatus("Module stopped by user");
#endif
            g_baseRateLife = false;
            px4_sem_post(&baserateTaskSem);
#endif
#if DEBUG
            PX4_INFO("px4_simulink_app : Exiting the Simulink model\n");
#endif
        } else {
            warnx("px4_simulink_app : No Simulink model is running\n");
        }

        fflush(stdout);
#if defined(MW_PX4_NUTTX_BUILD)
        exit(0);
#else
        return 0;
#endif
    }

    if (!strcmp(argv[1], "status")) {
        if (g_baseRateLife) {
            PX4_INFO("px4_simulink_app : model is running\n");
        } else {
            PX4_INFO("px4_simulink_app : model is not started\n");
        }
        fflush(stdout);
#if defined(MW_PX4_NUTTX_BUILD)
        exit(0);
#else
        return 0;
#endif
    }

    px4_app_usage("unrecognized command");
#if defined(MW_PX4_NUTTX_BUILD)
    exit(1);
#else
    return 1;
#endif
}

void MW_PX4_Terminate() {
    hrt_cancel(&BaseRate_HRT);
    g_baseRateLife = false; // set status of PX4 Simulink App to stop

#if DEBUG
    PX4_INFO("px4_simulink_app : Received command to end the Simulink task \n");
    fflush(stdout);
#endif
}

// LocalWords:  HRT px nsh mavlink nuttx doku nxinternal Nutt SCHED fs microsd
// LocalWords:  dev ACM tty
