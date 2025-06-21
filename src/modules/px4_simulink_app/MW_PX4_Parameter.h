/* Copyright 2024 The MathWorks, Inc. */

#ifndef __MW_PARAMETER_READ_WRITE_H__
#define __MW_PARAMETER_READ_WRITE_H__

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "rtwtypes.h"
#include "limits.h"
#include "perf/perf_counter.h"
#include "systemlib/err.h"
#include "parameters/param.h"
#include "MW_Parameter.h"
#ifdef __cplusplus

extern "C" {
    
#endif
    
    typedef enum param_data_type{
        MW_INT32 = 0,
        MW_SINGLE,
    } MW_PARAM_DATA_TYPE;
    
    MW_Param_Handle MW_Init_Param(void * ParameterName, uint8_T isString , double sampleTime);
    void MW_Param_terminate(void);
    bool MW_ParamRead_Step(MW_Param_Handle param_h, MW_PARAM_DATA_TYPE dataType, void * dataPtr);
    bool MW_ParamWrite_Step(MW_Param_Handle param_h, MW_PARAM_DATA_TYPE dataType, void * dataPtr);
    
#ifdef __cplusplus
    
}
#endif

#endif /* __MW_PARAMETER_READ_WRITE_H__ */