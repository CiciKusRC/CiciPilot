/* Copyright 2018 The MathWorks, Inc. */
#ifndef MW_uORB_READ_H
#define MW_uORB_READ_H

#if (defined(MATLAB_MEX_FILE) || defined(RSIM_PARAMETER_LOADING) || defined(RSIM_WITH_SL_SOLVER))

#define uORB_read_step(a, b, c, d, e) (0)
#define uORB_read_initialize(a, b) (0)
#define uORB_read_terminate(a) (0)

#else

#include "MW_uORB_Init.h"

#ifdef __cplusplus

extern "C" {

#endif

void uORB_read_initialize(
                    orb_metadata_t* orbData, 
                    pollfd_t* eventStructObj);

boolean_T uORB_read_step(
                    orb_metadata_t* orbData, 
                    pollfd_t* eventStructObj, 
                    void* busData, 
                    boolean_T blockingMode,
                    double blockingTimeout);

void uORB_read_terminate(const pollfd_t* eventStructObj);

#ifdef __cplusplus

}
#endif

#endif // ( defined(MATLAB_MEX_FILE) || defined(RSIM_PARAMETER_LOADING) ||
       // defined(RSIM_WITH_SL_SOLVER) )

#endif // MW_uORB_READ_H
