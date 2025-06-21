/* CAN Device class for STM32H7 PX4 Autopilots based on UAVCAN v0 drivers*/
/* Copyright 2023 The MathWorks, Inc. */

#include "MW_PX4_CAN_DEVICE.h"
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/log.h>
#include <nuttx/config.h>
#include "parameters/param.h"


#ifdef MW_CAN_UAVCAN
using namespace uavcan_stm32h7;

uint8_T MW_H7CAN_DEVICE::init(uint8_T CANModule,uint32_T baudRate, uint8_T mode){
    /*Check Whether UAVCAN is enabled or not using UAVCAN_ENABLE parameter*/
    param_t param_handle = param_find("UAVCAN_ENABLE");
    if (param_handle != PARAM_INVALID) {
        param_get(param_handle, &isUavcanEnabled);
    }
    printf(" UAVCAN_ENABLE Param Value = %d \n", isUavcanEnabled);
    /* Open and initialize CAN driver only if UAVCAN is not enabled. Since both UAVCAN and the CAN blocks using the same drivers,
       initializing CAN here while UAVCAN is enabled will result in hardfaults. This is because of uavcan module's implementation. */ 
    if (isUavcanEnabled!=0) {
        return uint8_T(1); /* CAN open error status */
    }
    /*  Declare CanInitHelper class with Rx Queue length as 36 */
    typedef CanInitHelper<36> CanInitHelper;
    static CanInitHelper *can = nullptr;
    /* The CAN driver donot have a deinititialization. So the init need to be called only once */
    if (can == nullptr) {
	    can = new CanInitHelper(board_get_can_interfaces());
	    if (can == nullptr) {                   
		    PX4_ERR("Out of memory");
		    return uint8_T(1);/* CAN open error status */
	    }
	    const int initStatus = can->driver.init(baudRate, (mode == 2)? CanIface::SilentMode : CanIface::NormalMode, board_get_can_interfaces());
	    if (initStatus < 0) {
		    PX4_ERR("CAN driver init failed. Error Code: %i", initStatus);
		    return uint8_T(1); /* CAN open error status */
	    }
    }
    /* Get CAN interface handle */
    uavcan::ICanIface *if0 = can->driver.getIface(CANModule);
    canHandleMap[CANModule] = if0;// store the CAN interface handle in a map
    isLoopback = (mode==1);
    return uint8_T(0);
 }
uint8_T MW_H7CAN_DEVICE::receiveMessage(uint8_T CANModule, uint8_T* rxData, uint32_T *id, uint8_T *idType, uint8_T *remote, uint8_T *length){
    if (isUavcanEnabled!=0) {
        return uint8_T(4); 
     }
     uavcan::CanFrame rxframe;
     uavcan::MonotonicTime ts_monotonic;
     uavcan::UtcTime ts_utc;
     uavcan::CanIOFlags flags;
     int readstatus = canHandleMap[CANModule]->receive(rxframe, ts_monotonic, ts_utc, flags);
     if (readstatus) {
	    *idType = uint8_T((rxframe.id & rxframe.FlagEFF)>>31);
	    idType ? (*id= rxframe.id & rxframe.MaskExtID) : (*id= rxframe.id & rxframe.MaskStdID);
	    *remote = uint8_T((rxframe.id & rxframe.FlagRTR)>>30);
	    *length = uint8_T(rxframe.dlc);
	    memcpy(&rxData[0], &rxframe.data[0], rxframe.dlc);
	    return uint8_T(0);
     } else {
	    return uint8_T(4); 
     }
 }
uint8_T MW_H7CAN_DEVICE::transmitMessage(uint8_T CANModule, uint8_T* txData, uint32_T id, uint8_T idType, uint8_T remote, uint8_T length){
    if (isUavcanEnabled!=0) {
        return uint8_T(4); 
     }
     uavcan::CanFrame txframe;
     idType ? txframe.id = (id | txframe.FlagEFF): txframe.id = id;
     remote ? txframe.id = (txframe.id | txframe.FlagRTR): txframe.id = txframe.id;
     txframe.dlc = length;
     memcpy(&txframe.data[0], &txData[0], length);
     int readstatus = canHandleMap[CANModule]->send(txframe,uavcan::MonotonicTime::fromUSec(1000000), uavcan::CanIOFlags(isLoopback));
     if (readstatus) {
	    return uint8_T(0);
     } else {
	    return uint8_T(4); 
     }
 }
 void MW_H7CAN_DEVICE::close(uint8_T CANModule){
	if(canHandleMap[CANModule] != nullptr){
		canHandleMap[CANModule] = nullptr;
	}
}

#endif