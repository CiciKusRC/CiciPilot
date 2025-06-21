/* Actuator Device class for PX4 Autopilots based on actuator_test uORB message*/
/* Copyright 2024 The MathWorks, Inc. */

#include<math.h>
#include <drivers/drv_hrt.h>
#include "MW_PX4_ActuatorDevice.h"

void MW_ACTUATOR_DEVICE::init(uint8_t QSIZE){

	actuator_test_pub = orb_advertise_queue(ORB_ID(actuator_test),NULL,QSIZE);
#if defined(MW_PX4_POSIX_BUILD)
    // PX4 SITL needs 'vehicle_thrust_setpoint' to be published for ActuatorTest uORB to work properly (v1.14.0).
    // 'mc_rate_control' should publish this. Since we are removing controller module we need to publish this

    //Publish only if no one else already publishing vehicle_thrust_setpoint
    if (!(orb_exists(ORB_ID(vehicle_thrust_setpoint), 0) == PX4_OK)) {
        vehicle_thrust_pub = orb_advertise_queue(ORB_ID(vehicle_thrust_setpoint),NULL,1);
        publishThrust = true;
    }
#endif
}

void MW_ACTUATOR_DEVICE::publishActuatorTest(int functionNum, float value, bool isArmed){
    actuator_test_s actuator_test_data{};
    actuator_test_data.timestamp = hrt_absolute_time();
    actuator_test_data.function = functionNum;
    actuator_test_data.value = value;
    actuator_test_data.action =
        isArmed ? actuator_test_s::ACTION_DO_CONTROL : actuator_test_s::ACTION_RELEASE_CONTROL;
    actuator_test_data.timeout_ms = 0;
	orb_publish(ORB_ID(actuator_test),actuator_test_pub,&actuator_test_data);
}

void MW_ACTUATOR_DEVICE::setActuatorValues(bool isArmed, float* motorValues, float* servoValues){

#if defined(MW_PX4_POSIX_BUILD)
    // Publish 'vehicle_thrust_setpoint' uOPRB for SITL
    if (publishThrust) {
        vehicle_thrust_setpoint_s vehicleThrust;
        vehicleThrust.timestamp = hrt_absolute_time();
        vehicleThrust.timestamp_sample = vehicleThrust.timestamp ;
        orb_publish(ORB_ID(vehicle_thrust_setpoint),vehicle_thrust_pub,&vehicleThrust);
    }
#endif    
    for (unsigned mIdx = 0; mIdx < actuator_test_s::MAX_NUM_MOTORS; ++mIdx) {
        if (!isnan(motorValues[mIdx])){
            publishActuatorTest(actuator_test_s::FUNCTION_MOTOR1 + mIdx, motorValues[mIdx], isArmed);
        }
    }
    for (unsigned sIdx = 0; sIdx < actuator_test_s::MAX_NUM_SERVOS; ++sIdx) {
        if (!isnan(servoValues[sIdx])){
			publishActuatorTest(actuator_test_s::FUNCTION_SERVO1 + sIdx, servoValues[sIdx], isArmed);
        }
    }
}
void MW_ACTUATOR_DEVICE::terminate() {
	orb_unadvertise(actuator_test_pub);
#if defined(MW_PX4_POSIX_BUILD)
    if (publishThrust) {
        orb_unadvertise(vehicle_thrust_pub); 
    }
#endif 
}	 
