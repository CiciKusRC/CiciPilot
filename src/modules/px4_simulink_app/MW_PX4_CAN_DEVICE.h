/* CAN Device generic header file defining generic base class and hardware specific child classes*/
/* Copyright 2023 The MathWorks, Inc. */


#ifndef __MW_CAN_H__
#define __MW_CAN_H__
#include "rtwtypes.h"
#include <nuttx/config.h>
#ifndef MW_CAN_MODULES_MAX
#define MW_CAN_MODULES_MAX  2U
#endif

#ifdef MW_CAN_UAVCAN
#include <uavcan/driver/can.hpp>
#include <uavcan/driver/system_clock.hpp>
#include <uavcan_stm32h7/uavcan_stm32h7.hpp>
#include <uavcan_stm32h7/clock.hpp>

#elif defined(MW_CAN_NUTTX)
#include <sys/types.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <poll.h>
#endif

#ifdef __cplusplus
extern "C" { /* sbcheck:ok:extern_c */
#endif
class MW_CAN_DEVICE {
  public:
    MW_CAN_DEVICE(){};
    virtual ~MW_CAN_DEVICE() = default;

    uint8_T init(uint8_T CANModule,uint32_T baudRate, uint8_T mode);
    uint8_T receiveMessage(uint8_T CANModule, uint8_T* rxData, uint32_T *id, uint8_T *idType, uint8_T *remote, uint8_T *length);
    uint8_T transmitMessage(uint8_T CANModule, uint8_T* txData, uint32_T id, uint8_T idType, uint8_T remote, uint8_T length);
    void close(uint8_T CANModule);
};
#ifdef MW_CAN_UAVCAN
class MW_H7CAN_DEVICE: public MW_CAN_DEVICE{
public:
	MW_H7CAN_DEVICE (){
	MW_CAN_DEVICE();	
	};
	uavcan::ICanIface* canHandleMap[MW_CAN_MODULES_MAX] = {};
	uint8_T isLoopback;
    int32_t isUavcanEnabled = 3;
    uint8_T init(uint8_T CANModule,uint32_T baudRate, uint8_T mode);
    uint8_T receiveMessage(uint8_T CANModule, uint8_T* rxData, uint32_T *id, uint8_T *idType, uint8_T *remote, uint8_T *length);
    uint8_T transmitMessage(uint8_T CANModule, uint8_T* txData, uint32_T id, uint8_T idType, uint8_T remote, uint8_T length);
    void close(uint8_T CANModule);
 };
#elif defined(MW_CAN_NUTTX)
class MW_NUTTX_CAN_DEVICE: public MW_CAN_DEVICE{
	uint8_T setBaudRate(uint8_T CANModule, uint32_T BaudRate);
	uint8_T setTestMode(uint8_T CANModule, uint8_T mode);
public:
	MW_NUTTX_CAN_DEVICE (){
	MW_CAN_DEVICE();	
	};
	int8_T canHandleMap[MW_CAN_MODULES_MAX] = {0};
    uint8_T init(uint8_T CANModule,uint32_T baudRate, uint8_T mode);
    uint8_T receiveMessage(uint8_T CANModule, uint8_T* rxData, uint32_T *id, uint8_T *idType, uint8_T *remote, uint8_T *length);
    uint8_T transmitMessage(uint8_T CANModule, uint8_T* txData, uint32_T id, uint8_T idType, uint8_T remote, uint8_T length);
	void close(uint8_T CANModule);
	
 };
 #endif
#ifdef __cplusplus
}
#endif

#endif /* __MW_CAN_H__ */