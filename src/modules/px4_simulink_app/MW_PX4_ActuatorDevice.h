/* Copyright 2024 The MathWorks, Inc. */

#ifndef __MW_ACTUATOR_DEVICE_H__
#define __MW_ACTUATOR_DEVICE_H__
#include <uORB/uORB.h>
#include <uORB/topics/actuator_test.h>

#if defined(MW_PX4_POSIX_BUILD)
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <px4_platform_common/defines.h>
#endif

#ifdef __cplusplus
extern "C" { /* sbcheck:ok:extern_c */
#endif
class MW_ACTUATOR_DEVICE {
   orb_advert_t actuator_test_pub;
#if defined(MW_PX4_POSIX_BUILD)
   orb_advert_t vehicle_thrust_pub;
   bool publishThrust = false;
#endif
   void publishActuatorTest(int functionNum, float value, bool isArmed);
public:
   MW_ACTUATOR_DEVICE(){};
   ~MW_ACTUATOR_DEVICE(){}; 
   void setActuatorValues(bool isArmed, float* motorValues, float* servoValues);
   void init(uint8_t QSIZE);
   void terminate();
};
#ifdef __cplusplus
}
#endif

#endif /* __MW_ACTUATOR_DEVICE_H__ */