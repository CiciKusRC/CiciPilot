/* Copyright 2018 The MathWorks, Inc. */
#ifndef MW_uORB_WRITE_H
#define MW_uORB_WRITE_H

#if (defined(MATLAB_MEX_FILE) || defined(RSIM_PARAMETER_LOADING) || defined(RSIM_WITH_SL_SOLVER))

#define uORB_write_step(a, b, c) (0)
#define uORB_write_initialize(a, b, c, d) (0)
#define uORB_write_terminate(a) (0)

#else

#include "MW_uORB_Init.h"

#ifdef __cplusplus

extern "C" {

#endif

void uORB_write_initialize(
                    orb_metadata_t* orbData, 
                    orb_advert_t* orbAdvertObj, 
                    void* busData,
                    int queueLen);

void uORB_write_step(
                    orb_metadata_t* orbData, 
                    orb_advert_t* orbAdvertObj, 
                    void* busData);

void uORB_write_terminate(const orb_advert_t* orbAdvertObj);

#ifdef __cplusplus

}
#endif

#endif // ( defined(MATLAB_MEX_FILE) || defined(RSIM_PARAMETER_LOADING) ||
       // defined(RSIM_WITH_SL_SOLVER) )

#endif // MW_uORB_WRITE_H
