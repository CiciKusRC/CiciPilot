/* Copyright 2024 The MathWorks, Inc. */

#include "MW_PX4_Actuators.h"
#include<math.h>

/* Handle to actuator device class*/
MW_ACTUATOR_DEVICE mwActuatorDevice = MW_ACTUATOR_DEVICE();
#ifdef __cplusplus
	extern "C" { /* sbcheck:ok:extern_c */
#endif

void MW_actuators_init(uint8_t QSIZE)
{
	mwActuatorDevice.init(QSIZE);
	// Reset all Motors and Servos in init to ensure qsize is set properly 
	float motorValues[actuator_test_s::MAX_NUM_MOTORS] = {0};
    float servoValues[actuator_test_s::MAX_NUM_SERVOS] = {0};
    mwActuatorDevice.setActuatorValues(false, &motorValues[0], &servoValues[0]);
 return;
}
void MW_actuators_set(bool isArmed, float* motorValues, float* servoValues)
{
    mwActuatorDevice.setActuatorValues(isArmed, &motorValues[0], &servoValues[0]);
    return;
}
void MW_actuators_terminate()
{
    mwActuatorDevice.terminate();
    return;
}
#ifdef __cplusplus
}
#endif 