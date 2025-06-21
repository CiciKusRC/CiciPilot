/* CAN Device class for STM32F7 and STM32F4 PX4 Autopilots based on NUTTX CAN drivers*/
/* Copyright 2023 The MathWorks, Inc. */
#include "MW_PX4_CAN_DEVICE.h"
#include "stm32_can.h"
#include <nuttx/can/can.h>
#include <arch/board/board.h>

#ifdef MW_CAN_NUTTX
 uint8_T MW_NUTTX_CAN_DEVICE::setBaudRate(uint8_T CANModule, uint32_T baudRate)
{
	int8_T ret;
	struct canioc_bittiming_s timings;
	/* get CAN bit timings*/
	ret = (int8_T) ioctl(canHandleMap[CANModule], CANIOC_GET_BITTIMING, (unsigned long)&timings);
	if (ret != OK)
	{
		return (uint8_T) 2;/* CAN Baud rate setting failed */
	}

	timings.bt_baud = baudRate;
	/* set CAN bit timings with desired baudrate*/
	ret = (int8_T) ioctl(canHandleMap[CANModule], CANIOC_SET_BITTIMING, (unsigned long)&timings);
	if (ret != OK)
	{
		return (uint8_T) 2;/* CAN Baud rate setting failed */
	}

	return (uint8_T) 0;
	
}
/* set the test mode for CAN bus */
/* 0-Normal 1-Loopback 2-Silent */
uint8_T MW_NUTTX_CAN_DEVICE::setTestMode(uint8_T CANModule, uint8_T mode)
{
	if(mode) // for Normal mode, we dont need to do anything 
	{
		int8_T ret;
		struct canioc_connmodes_s connmodes;
		if (mode==1){ /*Loopback*/
			connmodes.bm_loopback = 1;
			connmodes.bm_silent = 0;
		}else{ /*Silent*/
			connmodes.bm_silent = 1;
			connmodes.bm_loopback = 0;
		}
		ret = ioctl(canHandleMap[CANModule], CANIOC_SET_CONNMODES, (unsigned long)&connmodes);
		if (ret != OK)
		{
			return (uint8_T) 3;/* CAN Test mode setting failed */
		}
	}
	return (uint8_T)0;
}
 
 uint8_T MW_NUTTX_CAN_DEVICE::init(uint8_T CANModule,uint32_T baudRate, uint8_T mode){
    	if(CANModule>0){
		/*Only CAN1 (CANModule=0) is supported*/
		return (uint8_T) 1;/* CAN Open failure */
	}
		
	if(canHandleMap[CANModule] == 0)
	{
		int8_T fd = -1;
		/*Register CAN device at "/dev/can0"*/
		struct can_dev_s *can = stm32_caninitialize(1);
		can_register("/dev/can0", can);
		/*Open the CAN device "/dev/can0"*/
		fd = open("/dev/can0", O_RDWR | O_NONBLOCK);
        printf(" CAN open fd = %d \n",fd);

		if (fd < 0)
		{
			return (uint8_T) 1; /* CAN open failed */
		}else{
            /*Store the CAN handle in global map*/
			canHandleMap[CANModule] = fd;
			uint8_T ret;
			/*set CAN baudrate*/			
			ret =  setBaudRate(CANModule, baudRate);
			if (ret>0){
				return ret; /* CAN Baud rate setting failed */
			}
			/*set CAN test mode*/
			ret =  setTestMode(CANModule, mode);
			if (ret>0){
				return ret; /* CAN Test mode setting failed */
			}				
		}		
	} 
	
	return (uint8_T) 0;/* CAN Open success */ 
 }
 uint8_T MW_NUTTX_CAN_DEVICE::receiveMessage(uint8_T CANModule, uint8_T* rxData, uint32_T *id, uint8_T *idType, uint8_T *remote, uint8_T *length){
	struct can_msg_s rxmsg;
	const ssize_t nbytes = read(canHandleMap[CANModule], &rxmsg, sizeof(rxmsg));
	if (nbytes < 0 || (size_t)nbytes < CAN_MSGLEN(0) || (size_t)nbytes > sizeof(rxmsg)) 
	{
		return 4; /*CAN Receive failure*/
	}else {
		 memcpy(&rxData[0], &rxmsg.cm_data[0], rxmsg.cm_hdr.ch_dlc);
		*id     = (uint32_T)rxmsg.cm_hdr.ch_id;
		*idType = (uint8_T)rxmsg.cm_hdr.ch_extid;
		*remote = (uint8_T)rxmsg.cm_hdr.ch_rtr;
		*length = (uint8_T)rxmsg.cm_hdr.ch_dlc;
	}
	return (uint8_T)0; 
 }
 uint8_T MW_NUTTX_CAN_DEVICE::transmitMessage(uint8_T CANModule, uint8_T* txData, uint32_T id, uint8_T idType, uint8_T remote, uint8_T length){
	/* idType 0: std, 1: extended*/
	/* remote 0: rtr false, 1: rtr true*/
	struct can_msg_s txmsg;
	if(remote){
		txmsg.cm_hdr.ch_rtr    = true;
	}else {
		txmsg.cm_hdr.ch_rtr    = false;
	}
    txmsg.cm_hdr.ch_dlc    = (int)length;
    txmsg.cm_hdr.ch_id     = id;
    txmsg.cm_hdr.ch_extid  = (bool)idType;
    txmsg.cm_hdr.ch_unused = 0;

	memcpy(&txmsg.cm_data[0], &txData[0], length);
	const size_t msg_len = CAN_MSGLEN(txmsg.cm_hdr.ch_dlc);
	const ssize_t nbytes = write(canHandleMap[CANModule], &txmsg, msg_len);
    if ( nbytes<0 || (size_t)nbytes != msg_len)
	{
		return (uint8_T) 4; /*CAN Transmit failure*/
	}
	return (uint8_T) 0;/*CAN Transmit Success*/
 }
void MW_NUTTX_CAN_DEVICE::close(uint8_T CANModule){
	if(canHandleMap[CANModule] != 0){
		close(canHandleMap[CANModule]);
		canHandleMap[CANModule] = 0;
	}
}
#endif

