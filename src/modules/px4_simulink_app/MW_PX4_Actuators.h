/* Copyright 2024 The MathWorks, Inc. */
#ifndef MW_PX4_ACTUATOR_H
#define MW_PX4_ACTUATOR_H

#if (defined(MATLAB_MEX_FILE) || defined(RSIM_PARAMETER_LOADING) || defined(RSIM_WITH_SL_SOLVER))

#define MW_actuators_init(a) (0)
#define MW_actuators_step(a, b, c) (0)
#define MW_actuators_terminate() (0)

#else
#include "MW_PX4_ActuatorDevice.h"

#ifdef __cplusplus
extern "C" {
#endif

void MW_actuators_init(uint8_t QSIZE);

void MW_actuators_set(bool isArmed, float* motorValues, float* servoValues);

void MW_actuators_terminate();

#ifdef __cplusplus

}
#endif

#endif // ( defined(MATLAB_MEX_FILE) || defined(RSIM_PARAMETER_LOADING) ||
       // defined(RSIM_WITH_SL_SOLVER) )

#endif // MW_PX4_ACTUATOR_H
