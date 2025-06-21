/* Copyright 2018-2024 The MathWorks, Inc. */

#include "nuttxinitialize.h"
#include "MW_PX4_TaskControl.h"
#include "MW_PX4_PWM.h"
#include "MW_Parameter.h"
#include <uORB/Publication.hpp>

#define DEBUG 0

static constexpr uint16_t PWM_SIM_DISARMED = 900;
static constexpr uint16_t PWM_SIM_FAILSAFE = 600;
static constexpr uint16_t PWM_SIM_PWM_MIN = 1000;
static constexpr uint16_t PWM_SIM_PWM_MAX = 2000;
#if USE_ACTUATOR_TEST
uORB::Publication<actuator_test_s> actuator_test_pub{ORB_ID(actuator_test)};
#endif

void pwm_open(orb_advert_t* actuatorObj, orb_advert_t* armObj) {
#if (defined(PX4_CONNECTEDIO) && (DEBUG == 1))
    PX4_INFO("*     actuatorObj: %d * ", actuatorObj);
    PX4_INFO("*     armObj: %d *\n", armObj);
#endif

#if USE_ACTUATOR_TEST
    getMainDAPWMValues(mainDAPWMValue);
    getAUXDAPWMValues(auxDAPWMValue);
    getMainMinValues(mainMinPWMValue);
    getAUXMinValues(auxMinPWMValue);
    getMainMaxValues(mainMaxPWMValue);
    getAUXMaxValues(auxMaxPWMValue);
#else
    /* advertise actuator_outputs topic */
    struct actuator_outputs_s outputs;
    memset(&outputs, 0, sizeof(outputs));
    *actuatorObj = orb_advertise_queue(ORB_ID(actuator_outputs_sim), &outputs, 1);

    /* advertise actuator_armed topic */
    struct actuator_armed_s armPWM;
    memset(&armPWM, 0, sizeof(armPWM));
    *armObj = orb_advertise_queue(ORB_ID(actuator_armed), &armPWM, 1);
#endif
}

void pwm_arm(orb_advert_t* armObj) {
#if (defined(PX4_CONNECTEDIO) && (DEBUG == 1))
    PX4_INFO("* pwm_arm: *isArmed (status = %d)* ", *isArmed);
    PX4_INFO("*     armObj: %d *\n", armObj);
#endif

#if USE_ACTUATOR_TEST

#else
    struct actuator_armed_s armPWM;
    memset(&armPWM, 0, sizeof(armPWM));
    armPWM.armed = true;
    orb_publish(ORB_ID(actuator_armed), *armObj, &armPWM);
#endif
}

void pwm_disarm(orb_advert_t* armObj) {
#if (defined(PX4_CONNECTEDIO) && (DEBUG == 1))
    PX4_INFO("* pwm_disarm: *isArmed (status = %d)* ", *isArmed);
    PX4_INFO("*     armObj: %d *\n", armObj);
#endif

#if USE_ACTUATOR_TEST
#else
    struct actuator_armed_s armPWM;
    memset(&armPWM, 0, sizeof(armPWM));
    armPWM.armed = false;
    orb_publish(ORB_ID(actuator_armed), *armObj, &armPWM);

#if defined(MW_PX4_POSIX_BUILD)
    struct vehicle_status_s arm_vehicle_stat;
    /* Subscribe to current vehicle_status*/
    int vehicle_status_fd = orb_subscribe(ORB_ID(vehicle_status));
    orb_copy(ORB_ID(vehicle_status), vehicle_status_fd, &arm_vehicle_stat);
    /* Arm vehicle status to ARMING_STATE_SHUTDOWN = 4*/
    arm_vehicle_stat.arming_state = 4;
    static bool isAdvertised_disarm = false;
    static orb_advert_t vehicle_status_pub;
    if (!isAdvertised_disarm) {
        isAdvertised_disarm = true;
        vehicle_status_pub = orb_advertise_queue(ORB_ID(vehicle_status), &arm_vehicle_stat, 1);
    }
    orb_publish(ORB_ID(vehicle_status), vehicle_status_pub, &arm_vehicle_stat);

    orb_unsubscribe(vehicle_status_fd);
#endif

#endif
}

void pwm_forceFailsafe(orb_advert_t* armObj) {

    pwm_disarm(armObj);
}

void pwm_setServo(unsigned servo_count,
                  int channelMask,
                  uint16_t* pwm_value,
                  boolean_T isMain,
                  orb_advert_t* actuatorObj) {
#if (defined(PX4_CONNECTEDIO) && (DEBUG == 1))
    PX4_INFO("* pwm_setServo: servo_count %d* ", servo_count);
    PX4_INFO("*     isMain %d* ", isMain);
    PX4_INFO("*     actuatorObj %d* \n", actuatorObj);
    uint32_T iPWM;
    printf("* pwm_setServo: PWM values :");
    for (iPWM = 0; iPWM < 8; iPWM++) {
        printf("[%d]", pwm_value[iPWM]);
    }
    printf("\n");
#endif
#if USE_ACTUATOR_TEST
    float output = 0;
    if (isMain) {
        for (unsigned i = 0; i < servo_count; ++i) {
            output = (float)pwm_value[i];
            output = (output - mainMinPWMValue[i]) / (mainMaxPWMValue[i] - mainMinPWMValue[i]);
            send_actuator_test_output(actuator_test_s::FUNCTION_MOTOR1 + i, output, false);
        }
    } else {
        for (unsigned i = 0; i < servo_count; ++i) {
            output = (float)pwm_value[i];
            output = (output - ((auxMaxPWMValue[i] + auxMinPWMValue[i]) / 2)) /
                     ((auxMaxPWMValue[i] - auxMinPWMValue[i]) / 2);
            send_actuator_test_output(actuator_test_s::FUNCTION_SERVO1 + i, output, false);
        }
    }
#else
    if (isMain) {
        struct actuator_outputs_s outputs;
        memset(&outputs, 0, sizeof(outputs));
        outputs.noutputs = servo_count;
        outputs.timestamp = hrt_absolute_time();
        for (unsigned i = 0; i < servo_count; i++) {

            if (pwm_value[i] != PWM_SIM_DISARMED && channelMask & 1 << i) {

                float output = (float)pwm_value[i];
                /* Scale the motors that are non - reversible to[0, 1]*/
                outputs.output[i] =
                    (output - PWM_SIM_PWM_MIN) / (PWM_SIM_PWM_MAX - PWM_SIM_PWM_MIN);
            }
        }
        orb_publish(ORB_ID(actuator_outputs_sim), *actuatorObj, &outputs);

#if defined(MW_PX4_POSIX_BUILD)
        /*Only for Host target vehicle_status needs to be armed, not for HITL*/
        struct vehicle_status_s arm_vehicle_stat;
        /* Subscribe to current vehicle_status*/
        int vehicle_status_fd = orb_subscribe(ORB_ID(vehicle_status));
        orb_copy(ORB_ID(vehicle_status), vehicle_status_fd, &arm_vehicle_stat);
        /* Arm vehicle status to ARMING_STATE_ARMED = 2*/
        arm_vehicle_stat.arming_state = 2;
        orb_advert_t vehicle_status_pub =
            orb_advertise_queue(ORB_ID(vehicle_status), &arm_vehicle_stat, 1);
        orb_publish(ORB_ID(vehicle_status), vehicle_status_pub, &arm_vehicle_stat);
        orb_unsubscribe(vehicle_status_fd);
#endif
    }
#endif
}

void pwm_resetServo(unsigned servo_count, boolean_T isMain, orb_advert_t* actuatorObj) {
#if (defined(PX4_CONNECTEDIO) && (DEBUG == 1))
    PX4_INFO("* pwm_setServo: servo_count %d* ", servo_count);
    PX4_INFO("*     isMain %d* ", isMain);
    PX4_INFO("*     actuatorObj %d* \n", actuatorObj);
#endif

#if USE_ACTUATOR_TEST
    if (isMain) {
        for (unsigned i = 0; i < servo_count; ++i) {
            send_actuator_test_output(actuator_test_s::FUNCTION_MOTOR1 + i, 0, true);
        }
    } else {
        for (unsigned i = 0; i < servo_count; ++i) {
            send_actuator_test_output(actuator_test_s::FUNCTION_SERVO1 + i, 0, true);
        }
    }
#else
    if (isMain) {
        struct actuator_outputs_s outputs;
        memset(&outputs, 0, sizeof(outputs));
        outputs.noutputs = servo_count;
        outputs.timestamp = hrt_absolute_time();
        orb_publish(ORB_ID(actuator_outputs_sim), *actuatorObj, &outputs);
    }

#endif
}

void pwm_close(unsigned servo_count, orb_advert_t* actuatorObj, orb_advert_t* armObj) {
#if (defined(PX4_CONNECTEDIO) && (DEBUG == 1))
    PX4_INFO("* pwm_close: armObj %d* ", armObj);
    PX4_INFO("*     actuatorObj %d*\n", actuatorObj);
#endif

#if USE_ACTUATOR_TEST
    for (unsigned i = 0; i < servo_count; ++i) {
        send_actuator_test_output(actuator_test_s::FUNCTION_MOTOR1 + i, 0, true);
    }

    for (unsigned i = 0; i < servo_count; ++i) {
        send_actuator_test_output(actuator_test_s::FUNCTION_SERVO1 + i, 0, true);
    }
#else
    orb_unadvertise(*actuatorObj);
    orb_unadvertise(*armObj);
#endif
}

void send_actuator_test_output(int motorNum, float value, bool isArmed) {
#if USE_ACTUATOR_TEST
    actuator_test_s actuator_test{};
    actuator_test.timestamp = hrt_absolute_time();
    actuator_test.function = motorNum;
    actuator_test.value = value;
    actuator_test.action =
        isArmed ? actuator_test_s::ACTION_RELEASE_CONTROL : actuator_test_s::ACTION_DO_CONTROL;
    actuator_test.timeout_ms = 0;
    actuator_test_pub.publish(actuator_test);
#endif
}

void getMainDAPWMValues(int32_t* daPWMvalues) {
    bool success;
    int32_t paramIntValue;
    for (int index = 0; index < 8; index++) {
        switch (index) {
        case 0: {
            success = getIntegerParamValue("PWM_MAIN_DIS1", &paramIntValue);
            if (success) {
                daPWMvalues[index] = paramIntValue;
            } else {
                daPWMvalues[index] = 1000;
            }
        } break;
        case 1: {
            success = getIntegerParamValue("PWM_MAIN_DIS2", &paramIntValue);
            if (success) {
                daPWMvalues[index] = paramIntValue;
            } else {
                daPWMvalues[index] = 1000;
            }
        } break;
        case 2: {
            success = getIntegerParamValue("PWM_MAIN_DIS3", &paramIntValue);
            if (success) {
                daPWMvalues[index] = paramIntValue;
            } else {
                daPWMvalues[index] = 1000;
            }
        } break;
        case 3: {
            success = getIntegerParamValue("PWM_MAIN_DIS4", &paramIntValue);
            if (success) {
                daPWMvalues[index] = paramIntValue;
            } else {
                daPWMvalues[index] = 1000;
            }
        } break;
        case 4: {
            success = getIntegerParamValue("PWM_MAIN_DIS5", &paramIntValue);
            if (success) {
                daPWMvalues[index] = paramIntValue;
            } else {
                daPWMvalues[index] = 1000;
            }
        } break;
        case 5: {
            success = getIntegerParamValue("PWM_MAIN_DIS6", &paramIntValue);
            if (success) {
                daPWMvalues[index] = paramIntValue;
            } else {
                daPWMvalues[index] = 1000;
            }
        } break;
        case 6: {
            success = getIntegerParamValue("PWM_MAIN_DIS7", &paramIntValue);
            if (success) {
                daPWMvalues[index] = paramIntValue;
            } else {
                daPWMvalues[index] = 1000;
            }
        } break;
        case 7: {
            success = getIntegerParamValue("PWM_MAIN_DIS8", &paramIntValue);
            if (success) {
                daPWMvalues[index] = paramIntValue;
            } else {
                daPWMvalues[index] = 1000;
            }
        } break;
        }
    }
}

void getAUXDAPWMValues(int32_t* daPWMvalues) {
    bool success;
    int32_t paramIntValue;
    for (int index = 0; index < 8; index++) {
        switch (index) {
        case 0: {
            success = getIntegerParamValue("PWM_AUX_DIS1", &paramIntValue);
            if (success) {
                daPWMvalues[index] = paramIntValue;
            } else {
                daPWMvalues[index] = 1000;
            }
        } break;
        case 1: {
            success = getIntegerParamValue("PWM_AUX_DIS2", &paramIntValue);
            if (success) {
                daPWMvalues[index] = paramIntValue;
            } else {
                daPWMvalues[index] = 1000;
            }
        } break;
        case 2: {
            success = getIntegerParamValue("PWM_AUX_DIS3", &paramIntValue);
            if (success) {
                daPWMvalues[index] = paramIntValue;
            } else {
                daPWMvalues[index] = 1000;
            }
        } break;
        case 3: {
            success = getIntegerParamValue("PWM_AUX_DIS4", &paramIntValue);
            if (success) {
                daPWMvalues[index] = paramIntValue;
            } else {
                daPWMvalues[index] = 1000;
            }
        } break;
        case 4: {
            success = getIntegerParamValue("PWM_AUX_DIS5", &paramIntValue);
            if (success) {
                daPWMvalues[index] = paramIntValue;
            } else {
                daPWMvalues[index] = 1000;
            }
        } break;
        case 5: {
            success = getIntegerParamValue("PWM_AUX_DIS6", &paramIntValue);
            if (success) {
                daPWMvalues[index] = paramIntValue;
            } else {
                daPWMvalues[index] = 1000;
            }
        } break;
        case 6: {
            success = getIntegerParamValue("PWM_AUX_DIS7", &paramIntValue);
            if (success) {
                daPWMvalues[index] = paramIntValue;
            } else {
                daPWMvalues[index] = 1000;
            }
        } break;
        case 7: {
            success = getIntegerParamValue("PWM_AUX_DIS8", &paramIntValue);
            if (success) {
                daPWMvalues[index] = paramIntValue;
            } else {
                daPWMvalues[index] = 1000;
            }
        } break;
        }
    }
}

void getMainMinValues(int32_t* minPWMvalues) {
    bool success;
    int32_t paramIntValue;
    for (int index = 0; index < 8; index++) {
        switch (index) {
        case 0: {
            success = getIntegerParamValue("PWM_MAIN_MIN1", &paramIntValue);
            if (success) {
                minPWMvalues[index] = paramIntValue;
            } else {
                minPWMvalues[index] = 1000;
            }
        } break;
        case 1: {
            success = getIntegerParamValue("PWM_MAIN_MIN2", &paramIntValue);
            if (success) {
                minPWMvalues[index] = paramIntValue;
            } else {
                minPWMvalues[index] = 1000;
            }
        } break;
        case 2: {
            success = getIntegerParamValue("PWM_MAIN_MIN3", &paramIntValue);
            if (success) {
                minPWMvalues[index] = paramIntValue;
            } else {
                minPWMvalues[index] = 1000;
            }
        } break;
        case 3: {
            success = getIntegerParamValue("PWM_MAIN_MIN4", &paramIntValue);
            if (success) {
                minPWMvalues[index] = paramIntValue;
            } else {
                minPWMvalues[index] = 1000;
            }
        } break;
        case 4: {
            success = getIntegerParamValue("PWM_MAIN_MIN5", &paramIntValue);
            if (success) {
                minPWMvalues[index] = paramIntValue;
            } else {
                minPWMvalues[index] = 1000;
            }
        } break;
        case 5: {
            success = getIntegerParamValue("PWM_MAIN_MIN6", &paramIntValue);
            if (success) {
                minPWMvalues[index] = paramIntValue;
            } else {
                minPWMvalues[index] = 1000;
            }
        } break;
        case 6: {
            success = getIntegerParamValue("PWM_MAIN_MIN7", &paramIntValue);
            if (success) {
                minPWMvalues[index] = paramIntValue;
            } else {
                minPWMvalues[index] = 1000;
            }
        } break;
        case 7: {
            success = getIntegerParamValue("PWM_MAIN_MIN8", &paramIntValue);
            if (success) {
                minPWMvalues[index] = paramIntValue;
            } else {
                minPWMvalues[index] = 1000;
            }
        } break;
        }
    }
}

void getAUXMinValues(int32_t* minPWMvalues) {
    bool success;
    int32_t paramIntValue;
    for (int index = 0; index < 8; index++) {
        switch (index) {
        case 0: {
            success = getIntegerParamValue("PWM_AUX_MIN1", &paramIntValue);
            if (success) {
                minPWMvalues[index] = paramIntValue;
            } else {
                minPWMvalues[index] = 1000;
            }
        } break;
        case 1: {
            success = getIntegerParamValue("PWM_AUX_MIN2", &paramIntValue);
            if (success) {
                minPWMvalues[index] = paramIntValue;
            } else {
                minPWMvalues[index] = 1000;
            }
        } break;
        case 2: {
            success = getIntegerParamValue("PWM_AUX_MIN3", &paramIntValue);
            if (success) {
                minPWMvalues[index] = paramIntValue;
            } else {
                minPWMvalues[index] = 1000;
            }
        } break;
        case 3: {
            success = getIntegerParamValue("PWM_AUX_MIN4", &paramIntValue);
            if (success) {
                minPWMvalues[index] = paramIntValue;
            } else {
                minPWMvalues[index] = 1000;
            }
        } break;
        case 4: {
            success = getIntegerParamValue("PWM_AUX_MIN5", &paramIntValue);
            if (success) {
                minPWMvalues[index] = paramIntValue;
            } else {
                minPWMvalues[index] = 1000;
            }
        } break;
        case 5: {
            success = getIntegerParamValue("PWM_AUX_MIN6", &paramIntValue);
            if (success) {
                minPWMvalues[index] = paramIntValue;
            } else {
                minPWMvalues[index] = 1000;
            }
        } break;
        case 6: {
            success = getIntegerParamValue("PWM_AUX_MIN7", &paramIntValue);
            if (success) {
                minPWMvalues[index] = paramIntValue;
            } else {
                minPWMvalues[index] = 1000;
            }
        } break;
        case 7: {
            success = getIntegerParamValue("PWM_AUX_MIN8", &paramIntValue);
            if (success) {
                minPWMvalues[index] = paramIntValue;
            } else {
                minPWMvalues[index] = 1000;
            }
        } break;
        }
    }
}

void getMainMaxValues(int32_t* maxPWMvalues) {
    bool success;
    int32_t paramIntValue;
    for (int index = 0; index < 8; index++) {
        switch (index) {
        case 0: {
            success = getIntegerParamValue("PWM_MAIN_MAX1", &paramIntValue);
            if (success) {
                maxPWMvalues[index] = paramIntValue;
            } else {
                maxPWMvalues[index] = 2000;
            }
        } break;
        case 1: {
            success = getIntegerParamValue("PWM_MAIN_MAX2", &paramIntValue);
            if (success) {
                maxPWMvalues[index] = paramIntValue;
            } else {
                maxPWMvalues[index] = 2000;
            }
        } break;
        case 2: {
            success = getIntegerParamValue("PWM_MAIN_MAX3", &paramIntValue);
            if (success) {
                maxPWMvalues[index] = paramIntValue;
            } else {
                maxPWMvalues[index] = 2000;
            }
        } break;
        case 3: {
            success = getIntegerParamValue("PWM_MAIN_MAX4", &paramIntValue);
            if (success) {
                maxPWMvalues[index] = paramIntValue;
            } else {
                maxPWMvalues[index] = 2000;
            }
        } break;
        case 4: {
            success = getIntegerParamValue("PWM_MAIN_MAX5", &paramIntValue);
            if (success) {
                maxPWMvalues[index] = paramIntValue;
            } else {
                maxPWMvalues[index] = 2000;
            }
        } break;
        case 5: {
            success = getIntegerParamValue("PWM_MAIN_MAX6", &paramIntValue);
            if (success) {
                maxPWMvalues[index] = paramIntValue;
            } else {
                maxPWMvalues[index] = 2000;
            }
        } break;
        case 6: {
            success = getIntegerParamValue("PWM_MAIN_MAX7", &paramIntValue);
            if (success) {
                maxPWMvalues[index] = paramIntValue;
            } else {
                maxPWMvalues[index] = 2000;
            }
        } break;
        case 7: {
            success = getIntegerParamValue("PWM_MAIN_MAX8", &paramIntValue);
            if (success) {
                maxPWMvalues[index] = paramIntValue;
            } else {
                maxPWMvalues[index] = 2000;
            }
        } break;
        }
    }
}

void getAUXMaxValues(int32_t* maxPWMvalues) {
    bool success;
    int32_t paramIntValue;
    for (int index = 0; index < 8; index++) {
        switch (index) {
        case 0: {
            success = getIntegerParamValue("PWM_AUX_MAX1", &paramIntValue);
            if (success) {
                maxPWMvalues[index] = paramIntValue;
            } else {
                maxPWMvalues[index] = 2000;
            }
        } break;
        case 1: {
            success = getIntegerParamValue("PWM_AUX_MAX2", &paramIntValue);
            if (success) {
                maxPWMvalues[index] = paramIntValue;
            } else {
                maxPWMvalues[index] = 2000;
            }
        } break;
        case 2: {
            success = getIntegerParamValue("PWM_AUX_MAX3", &paramIntValue);
            if (success) {
                maxPWMvalues[index] = paramIntValue;
            } else {
                maxPWMvalues[index] = 2000;
            }
        } break;
        case 3: {
            success = getIntegerParamValue("PWM_AUX_MAX4", &paramIntValue);
            if (success) {
                maxPWMvalues[index] = paramIntValue;
            } else {
                maxPWMvalues[index] = 2000;
            }
        } break;
        case 4: {
            success = getIntegerParamValue("PWM_AUX_MAX5", &paramIntValue);
            if (success) {
                maxPWMvalues[index] = paramIntValue;
            } else {
                maxPWMvalues[index] = 2000;
            }
        } break;
        case 5: {
            success = getIntegerParamValue("PWM_AUX_MAX6", &paramIntValue);
            if (success) {
                maxPWMvalues[index] = paramIntValue;
            } else {
                maxPWMvalues[index] = 2000;
            }
        } break;
        case 6: {
            success = getIntegerParamValue("PWM_AUX_MAX7", &paramIntValue);
            if (success) {
                maxPWMvalues[index] = paramIntValue;
            } else {
                maxPWMvalues[index] = 2000;
            }
        } break;
        case 7: {
            success = getIntegerParamValue("PWM_AUX_MAX8", &paramIntValue);
            if (success) {
                maxPWMvalues[index] = paramIntValue;
            } else {
                maxPWMvalues[index] = 2000;
            }
        } break;
        }
    }
}

bool getSingleParamValue(const char* parameterName, float* paramValue_float) {

    bool success = false;
    MW_Param_Handle param_handle = PARAM_INVALID;
    param_handle = param_find(parameterName);

#if DEBUG
    printf("Parameter name = %s.\n", parameterName);
    if (param_handle != PARAM_INVALID) {
        printf("Valid Parameter Handle = %u.\n", param_handle);
    } else {
        printf("Invalid Parameter Handle. \n");
    }
#endif

    if (param_handle != PARAM_INVALID) {
        if (!param_get(param_handle, paramValue_float)) {
            success = true;
#if DEBUG
            printf("MW_SINGLE. Value = %4.4f\n", *paramValue_float);
#endif
        }
    }

    return success;
}

bool getIntegerParamValue(const char* parameterName, int32_t* paramValue_int) {

    bool success = false;
    MW_Param_Handle param_handle = PARAM_INVALID;
    param_handle = param_find(parameterName);

#if DEBUG
    printf("Parameter name = %s.\n", parameterName);
    if (param_handle != PARAM_INVALID) {
        printf("Valid Parameter Handle = %u.\n", param_handle);
    } else {
        printf("Invalid Parameter Handle. \n");
    }
#endif

    if (param_handle != PARAM_INVALID) {
        if (!param_get(param_handle, paramValue_int)) {
            success = true;
#if DEBUG
            printf("Value = %ld\n", *paramValue_int);
#endif
        }
    }

    return success;
}
