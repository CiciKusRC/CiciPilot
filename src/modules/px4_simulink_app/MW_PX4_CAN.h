/* MW_PX4_CAN.h: Header file for CAN driver implementation*/

/* Copyright 2022-2023 The MathWorks, Inc. */

#ifndef MW_PX4_CAN_H
#define MW_PX4_CAN_H

#if (defined(MATLAB_MEX_FILE) || defined(RSIM_PARAMETER_LOADING) || defined(RSIM_WITH_SL_SOLVER))
#define MW_CAN_Open(a,b,c) (0)
#define MW_CAN_SetBaudRate(a,b) (0)
#define MW_CAN_SetTestMode(a,b) (0)
#define MW_CAN_ConfigureFilter(a,b,c) (0)
#define MW_CAN_TransmitMessage(a,b,c,d,e,f) (0)
#define MW_CAN_ReceiveMessage(a,b,c,d,e,f) (0)
#define MW_CAN_ReceiveMessages_By_ID(a,b,c,d,e,f) (0)
#define MW_CAN_AssignGlobalBufferForID(a,b,c) (0)
#define MW_CAN_Close(a) (0)
#else
#include "MW_PX4_CAN_DEVICE.h"
#include <px4_platform_common/px4_config.h>
#include <nuttx/config.h>
#include "rtwtypes.h"

typedef uint8_T MW_CAN_Status_T;
#ifdef __cplusplus
extern "C" {  /* sbcheck:ok:extern_c */
#endif
MW_CAN_Status_T MW_CAN_Open(uint8_T CANModule,uint32_T baudRate, uint8_T mode);
MW_CAN_Status_T MW_CAN_SetBaudRate(uint8_T CANModule, uint32_T BaudRate);
MW_CAN_Status_T MW_CAN_SetTestMode(uint8_T CANModule, uint8_T mode);
MW_CAN_Status_T MW_CAN_ConfigureFilter(uint8_T CANModule, uint32_T acceptMask, uint32_T acceptID);
MW_CAN_Status_T MW_CAN_TransmitMessage(uint8_T CANModule, uint8_T* txData, uint32_T id, uint8_T idType, uint8_T remote, uint8_T length);
MW_CAN_Status_T MW_CAN_ReceiveMessage(uint8_T CANModule, uint8_T* rxData, uint32_T *id, uint8_T *idType, uint8_T *remote, uint8_T *length);
MW_CAN_Status_T MW_CAN_ReceiveMessages_By_ID(uint8_T CANModule, uint8_T* rxData, uint32_T id, uint8_T idType, uint8_T *remote, uint8_T *length);
void MW_CAN_AssignGlobalBufferForID(uint8_T CANModule, uint32_T id, uint8_T idType);
void MW_CAN_Close(uint8_T CANModule);
#ifdef __cplusplus
}
#endif
#endif // ( defined(MATLAB_MEX_FILE) || defined(RSIM_PARAMETER_LOADING) ||
       // defined(RSIM_WITH_SL_SOLVER) )
#endif