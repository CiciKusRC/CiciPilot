/* Copyright 2018 The MathWorks, Inc. */

#ifndef __MW_PX4_SCI_H__
#define __MW_PX4_SCI_H__

#include "MW_SVD.h"
#include "MW_SCI.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Returns data available to read */
    MW_SCI_Status_Type MW_SCI_GetDataBytesAvailable(MW_Handle_Type SCIModuleHandle, bool blockingMode, void * size, int32_T timeout);
struct timespec diff(struct timespec start, struct timespec end);
int Serial_alloc(const char *name);
int getCurrSciDev(const char *name);
#ifdef __cplusplus
}
#endif

#endif /* __MW_PX4_SCI_H__ */