/* Copyright 2018-2020 The MathWorks, Inc. */
/* Reads the Parameter value and outputs the same.

 **/
#include "MW_ParameterRead.h"

//Below Macro can be enabled for debugging purposes
#define DEBUG 0

/*
 ****************************************************************************************************************************************************
 * Wrapper function For Initializing and getting the parameter handle
 * Input Params
 * isString  - Denotes if the Parameter is string or numeric
 * sampleTime - Sample time of the block
 
 * Output Params
 * param_handle  - Handle to the requested parameter.
 ****************************************************************************************************************************************************
 */
MW_Param_Handle MW_Init_Param(void * ParameterName, uint8_T isString , double sampleTime)
{

    MW_Param_Handle param_handle = PARAM_INVALID;
    char * parameterStr;
    /* Check parameters */
    if (0 == isString)
    {
        fprintf(stderr,"Only string as ParameterName name is supported.\n");
#if defined(MW_PX4_NUTTX_BUILD)	
	exit(1);
#else
	return 1;
#endif
    }
    else
    {
        parameterStr = (char*)ParameterName;
    }
    param_handle = param_find(parameterStr);
#if DEBUG
    printf("In MW_Init_Param. Parameter name = %s.\n", parameterStr);
#endif
    if (param_handle != PARAM_INVALID)
    {
#if DEBUG
        printf("In MW_Init_Param. Valid Parameter Handle = %u.\n", param_handle);
#endif    
    }
    else
    {
#if DEBUG
        printf("In MW_Init_Param. Invalid Parameter Handle. \n");
#endif
    }
    return param_handle;
}

/*
 ****************************************************************************************************************************************************
 * Wrapper function For reading the parameter
 * Input Params
 * param_h  - Handle to the parameter requested
 * dataType - Data Type of the Parameter Requested
 * dataPtr - Pointer to the data, into which param_get is written to
 *
 * Output Params
 * status  - 0 for a valid Parameter Handle ; 1 for a valid Parameter Handle
 ****************************************************************************************************************************************************
 */
bool MW_Param_Step(MW_Param_Handle param_h, MW_PARAM_DATA_TYPE dataType, void * dataPtr)
{
    //status - 0 for a valid Parameter Handle
    //status - 1 for an invalid Parameter Handle
    bool status = 1;
    if (param_h != PARAM_INVALID)
    {
#if DEBUG
        printf("In MW_Param_Step.Parameter Handle = %u.\n", param_h);
#endif
        int32_t  ReturnTmpInt;
        float ReturnTmpFlt;
        switch (dataType)
        {
            case MW_INT32:
                if (!param_get(param_h, &ReturnTmpInt))
                {
                    *(int*)dataPtr  = ReturnTmpInt;
#if DEBUG
                    printf("In MW_Param_Step MW_INT32. Value = %ld\n", *(int*)dataPtr);
#endif
                }
                break;
            case MW_SINGLE:
                if (!param_get(param_h, &ReturnTmpFlt))
                {
                    *(float*)dataPtr  = ReturnTmpFlt;
#if DEBUG
                    printf("In MW_Param_Step MW_SINGLE. Value = %4.4f\n", *(float*)dataPtr);
#endif
                }
                break;
            default:
                break;
        }
         status = 0;
    }
    return status;
}

void MW_Param_terminate(void) 
{
}
