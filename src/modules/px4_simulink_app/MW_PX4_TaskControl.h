/* Copyright 2018-2022 The MathWorks, Inc. */

#ifndef MW_PX4_TASKCONTROL_H_
#define MW_PX4_TASKCONTROL_H_

/* Nuttx Specific Includes */
#if defined(MW_PX4_NUTTX_BUILD)
#define _GNU_SOURCE

#include <nuttx/config.h>
#include <nuttx/sched.h>
#include "debug.h"
#endif

/* Common Includes */
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
#include "stdbool.h"
#include "fcntl.h"
#include "math.h"
#include "poll.h"
#include "string.h"

/* PX4 includes*/
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include "sys/mount.h"
#include "sys/ioctl.h"
#include "sys/stat.h"
#include "perf/perf_counter.h"
#include "systemlib/err.h"
#include "parameters/param.h"


#ifdef __cplusplus
#include "mavlink_shell.h"
#endif

/* Required Coder Target header */
#include "rtwtypes.h"

/* Required for Ext-Mode */
#ifdef EXT_MODE
#include "rtw_extmode.h"
#include "ext_work.h"
#endif

#ifdef __cplusplus

extern "C" { /* sbcheck:ok:extern_c needed because of C compatibility*/

void shellWait(MavlinkShell* shell, uint8_t buffer[]);
void px4_simulink_app_control_MAVLink();
#endif

/*Function prototype declarations*/
void MW_PX4_Init(int argc, char* argv[]);
void MW_PX4_Terminate(void);

/*Main PX4 Simulink App entry */
int px4_simulink_app_main(int argc, char* argv[]);
int px4_simulink_app_task_main(int argc, char* argv[]);
void px4_app_usage(const char* reason);

#ifdef __cplusplus
}
#endif

/*The following undef is because Nuttx has a macro with its own definition that conflicts
 * with the Simulink definition. Without this it throws a warning in build */
#undef UNUSED

#endif /* MW_PX4_TASKCONTROL_H_ */
