/* Copyright 2018-2024 The MathWorks, Inc. */
#ifndef MW_PX4_PWM_H
#define MW_PX4_PWM_H

#if (defined(MATLAB_MEX_FILE) || defined(RSIM_PARAMETER_LOADING) || defined(RSIM_WITH_SL_SOLVER))

#define pwm_open(a, b) (0)
#define pwm_arm(a) (0)
#define pwm_disarm(a) (0)
#define pwm_setMinPWM(a, b, c, d) (0)
#define pwm_setMaxPWM(a, b, c, d) (0)
#define pwm_forceFailsafe(a, b) (0)
#define pwm_setServo(a, b, c, d) (0)
#define pwm_resetServo(a, b, c, d) (0)
#define pwm_close(a) (0)
#define getMinPWMValues(a, b) (0)
#define getMaxPWMValues(a, b) (0)

#else

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <uORB/uORB.h>

#ifdef __PX4_NUTTX
#include <nuttx/fs/ioctl.h>
#endif

#if defined(MW_PX4_POSIX_BUILD)
#define USE_ACTUATOR_TEST 0
#elif defined(MW_PX4_NUTTX_BUILD) && defined(MW_ENABLE_HITL) && (MW_ENABLE_HITL == 1) && \
    !defined(PX4_CONNECTEDIO)
#define USE_ACTUATOR_TEST 0
#elif defined(MW_PX4_NUTTX_BUILD)
#define USE_ACTUATOR_TEST 1
#endif

#if USE_ACTUATOR_TEST == 0
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/vehicle_status.h>
#else
#include <uORB/topics/actuator_test.h>
#endif

#include <drivers/drv_hrt.h>
#include "systemlib/err.h"
#include <parameters/param.h>
#include "drivers/drv_pwm_output.h"

#if USE_ACTUATOR_TEST
static int32_t mainMinPWMValue[8];
static int32_t mainMaxPWMValue[8];
static int32_t mainDAPWMValue[8];
static int32_t auxMinPWMValue[8];
static int32_t auxMaxPWMValue[8];
static int32_t auxDAPWMValue[8];
#endif

#ifdef __cplusplus

extern "C" {

#endif

void pwm_open(orb_advert_t* actuatorObj, orb_advert_t* armObj);

void pwm_arm(orb_advert_t* armObj);

void pwm_disarm(orb_advert_t* armObj);

void pwm_forceFailsafe(orb_advert_t* armObj);

void pwm_setServo(unsigned servo_count,
                  int channelMask,
                  uint16_t* pwm_value,
                  boolean_T isMain,
                  orb_advert_t* actuatorObj);

void pwm_resetServo(unsigned servo_count, boolean_T isMain, orb_advert_t* actuatorObj);

void pwm_close(unsigned servo_count, orb_advert_t* actuatorObj, orb_advert_t* armObj);

void send_actuator_test_output(int motorNum, float value, bool isArmed);

void getMainDAPWMValues(int32_t* daPWMvalues);

void getAUXDAPWMValues(int32_t* daPWMvalues);

void getMainMinValues(int32_t* minPWMvalues);

void getAUXMinValues(int32_t* minPWMvalues);

void getMainMaxValues(int32_t* maxPWMvalues);

void getAUXMaxValues(int32_t* maxPWMvalues);

bool getIntegerParamValue(const char* parameterName, int32_t* paramValue_int);

bool getSingleParamValue(const char* parameterName, float* paramValue_float);


#ifdef __cplusplus
}
#endif

#endif // ( defined(MATLAB_MEX_FILE) || defined(RSIM_PARAMETER_LOADING) ||
       // defined(RSIM_WITH_SL_SOLVER) )

#endif // MW_PX4_PWM_H
