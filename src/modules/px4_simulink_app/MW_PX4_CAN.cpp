/* Copyright 2022 The MathWorks, Inc. */

#include "MW_PX4_CAN.h"
#include "MW_target_hardware_resources.h"

#ifndef MW_CAN_MODULES_MAX
#define MW_CAN_MODULES_MAX  2U
#endif

typedef struct
{
	uint32_T 	ID;
	uint8_T		Data[8] ={0};
	uint8_T 	CANModule;
    uint8_T 	IDType;
    uint8_T 	Length;
    uint8_T 	Remote;
	uint8_T 	Valid;
	
}CANMsgType;

#ifdef MW_NUM_CAN_RECEIVE_RAW
/*Global receive buffer shared across CAN Rx blocks with 'Raw data' mode.
MW_NUM_CAN_RECEIVE_RAW is the number of CAN Rx blocks with 'Raw data' 
mode in the Simulink model. This will be defined during onAfterCodegen hook*/
volatile CANMsgType globalCANRxBuffer[MW_NUM_CAN_RECEIVE_RAW];
#endif
#ifdef MW_CAN_UAVCAN
	MW_H7CAN_DEVICE canDevice = MW_H7CAN_DEVICE();
#elif defined( MW_CAN_NUTTX)
	MW_NUTTX_CAN_DEVICE canDevice = MW_NUTTX_CAN_DEVICE();
#endif
/*Index used to assign the buffer for each CAN message IDs.
Used in MW_CAN_AssignGlobalBufferForID function*/
static uint8_T canRxIdAssigner = 0;

#ifdef __cplusplus
	extern "C" { /* sbcheck:ok:extern_c */
#endif

MW_CAN_Status_T MW_CAN_Open(uint8_T CANModule, uint32_T baudRate, uint8_T mode)
{
	/* Check if valid CAN handle is present, if not open CAN node and store handle*/
	/* Non-zero handle value in the canHandleMap indicate valid CAN handle*/
        //printf("inside MW CAN Init CANModule= %d baudRate= %d mode= %d \n", CANModule, baudRate, mode);

#ifndef PX4_CONNECTEDIO
    /* Get CAN Module from MW_target_hardware_resources.h */
#ifdef MW_CAN_PORTINDEX
	CANModule = MW_CAN_PORTINDEX;
    printf(" CANModule = %d ",CANModule);
#endif
		/* Get Bauddrate setting from MW_target_hardware_resources.h */
#ifdef MW_CAN_BAUDRATE
	baudRate = MW_CAN_BAUDRATE;
    printf(" baudRate = %d ",baudRate);
#endif
		/* Get CAN test mode setting from MW_target_hardware_resources.h */
#ifdef MW_CAN_TESTMODE
	mode = MW_CAN_TESTMODE;
    printf(" mode = %d \n",mode);
#endif
#endif //ifndef PX4_CONNECTEDIO
return (MW_CAN_Status_T) canDevice.init(CANModule,baudRate,mode);
}
/* set desired Baudrate for CAN bus */
MW_CAN_Status_T MW_CAN_SetBaudRate(uint8_T CANModule, uint32_T baudRate)
{
	// Do nothing. BAudrate is already set inside init
	return (MW_CAN_Status_T) 0;
	
}

MW_CAN_Status_T MW_CAN_SetTestMode(uint8_T CANModule, uint8_T mode)
{
	// do nothing Test mode is already set inside init
	return (MW_CAN_Status_T) 0;
}
/*Configure CAN receive buffer filters */
MW_CAN_Status_T MW_CAN_ConfigureFilter(uint8_T CANModule, uint32_T acceptMask, uint32_T acceptID)
{
	// Not Implemented in stm32 drivers / UAVCAN drivers
	return (MW_CAN_Status_T) 0;
}
/*Assign the buffer for each CAN message IDs*/
void MW_CAN_AssignGlobalBufferForID(uint8_T CANModule, uint32_T id, uint8_T idType)
{
#if !defined(PX4_CONNECTEDIO) && defined(MW_CAN_PORTINDEX)
	CANModule = MW_CAN_PORTINDEX;
#endif
#ifdef MW_NUM_CAN_RECEIVE_RAW
	globalCANRxBuffer[canRxIdAssigner].CANModule = CANModule;
    globalCANRxBuffer[canRxIdAssigner].ID = id;
    globalCANRxBuffer[canRxIdAssigner].IDType = idType;
	globalCANRxBuffer[canRxIdAssigner].Valid = 0;
    canRxIdAssigner = canRxIdAssigner + 1U;
	if (canRxIdAssigner>MW_NUM_CAN_RECEIVE_RAW)
	{
		canRxIdAssigner = MW_NUM_CAN_RECEIVE_RAW;
	}
#endif
}
/* Transmit CAN message*/
MW_CAN_Status_T MW_CAN_TransmitMessage(uint8_T CANModule, uint8_T* txData, uint32_T id, uint8_T idType, uint8_T remote, uint8_T length)
{
#if !defined(PX4_CONNECTEDIO) && defined(MW_CAN_PORTINDEX)
	CANModule = MW_CAN_PORTINDEX;
#endif
return canDevice.transmitMessage(CANModule, &txData[0], id, idType, remote, length );
}
/* Receive CAN message with specified message ID */
MW_CAN_Status_T MW_CAN_ReceiveMessages_By_ID(uint8_T CANModule, uint8_T* rxData, uint32_T id, uint8_T idType, uint8_T *remote, uint8_T *length)
{
	/* idType 0: std, 1: extended*/
	int rxStatus = 0;
#if !defined(PX4_CONNECTEDIO) && defined(MW_CAN_PORTINDEX)
	CANModule = MW_CAN_PORTINDEX;
#endif
#ifdef MW_NUM_CAN_RECEIVE_RAW
	//struct can_msg_s rxmsg;
	int msgIdx,idx;
	uint8_T 	rx_data[8] ={0};
	uint8_T 	rx_idType=0;
    uint32_T 	rx_id=0;
    uint8_T 	rx_length=0;
    uint8_T 	rx_remote=0;
	/* Read all CAN messages available, till CAN read errors*/
	while(rxStatus>=0)
	{
		rxStatus = canDevice.receiveMessage(CANModule, &rx_data[0], &rx_id, &rx_idType, &rx_remote, &rx_length);// call read fn here read(canHandleMap[CANModule], &rxmsg, sizeof(rxmsg));
		if (rxStatus ==4) 
		{
			rxStatus= -1; /*CAN Receive failure*/
		}else {
			/* Update Global Receive Buffer if CAN Receive is successfull*/
			/* Store in global buffer for Raw Data Type CAN Receive block */
			for(msgIdx=0;msgIdx<MW_NUM_CAN_RECEIVE_RAW;msgIdx++)
			{
				if((globalCANRxBuffer[msgIdx].ID == rx_id) && (globalCANRxBuffer[msgIdx].IDType == rx_idType) && (globalCANRxBuffer[msgIdx].CANModule == CANModule))
				{
					globalCANRxBuffer[msgIdx].Length = rx_length;
					globalCANRxBuffer[msgIdx].Remote = rx_remote;
					globalCANRxBuffer[msgIdx].Valid = 1;
					memcpy(&globalCANRxBuffer[msgIdx].Data[0], &rx_data[0], rx_length);
				}
			}
		}
	}		
		
	/* Read from Buffer*/
	for(msgIdx=0;msgIdx<MW_NUM_CAN_RECEIVE_RAW;msgIdx++)
    {
        if((id == globalCANRxBuffer[msgIdx].ID) && (idType == globalCANRxBuffer[msgIdx].IDType) && (CANModule ==globalCANRxBuffer[msgIdx].CANModule) && (globalCANRxBuffer[msgIdx].Valid ==1 ))
        {
            for(idx=0;idx<8;idx++)
            {
                rxData[idx] = globalCANRxBuffer[msgIdx].Data[idx];
				globalCANRxBuffer[msgIdx].Data[idx] =0;
            }
			globalCANRxBuffer[msgIdx].Valid = 0;
            *length = globalCANRxBuffer[msgIdx].Length;
            *remote = globalCANRxBuffer[msgIdx].Remote;
			rxStatus =0; /* Read Sucess */
            break;
        }
    }
		
#endif
    if (rxStatus<0){
		return (MW_CAN_Status_T)4; /*CAN Receive failure*/
	}else{
		return (MW_CAN_Status_T)0; /*CAN Receive Success*/
	}
}
/* Receive Generic CAN message */
MW_CAN_Status_T MW_CAN_ReceiveMessage(uint8_T CANModule, uint8_T* rxData, uint32_T *id, uint8_T *idType, uint8_T *remote, uint8_T *length)
{
#if !defined(PX4_CONNECTEDIO) && defined(MW_CAN_PORTINDEX)
	CANModule = MW_CAN_PORTINDEX;
#endif
	return canDevice.receiveMessage(CANModule, &rxData[0], id, idType, remote, length );
}
/* Close CAN */
void MW_CAN_Close(uint8_T CANModule)
{
#if !defined(PX4_CONNECTEDIO) && defined(MW_CAN_PORTINDEX)
	CANModule = MW_CAN_PORTINDEX;
#endif
	canRxIdAssigner = 0;
	canDevice.close(CANModule);	
}

#ifdef __cplusplus
}
#endif 