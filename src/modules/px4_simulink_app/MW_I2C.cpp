/* Copyright 2020 The MathWorks, Inc. */
#include "MW_I2C.h"
#include "MW_PX4_I2C.h"

#ifdef __cplusplus
	extern "C" { /* sbcheck:ok:extern_c */
#endif

/* Initialize a I2C */
MW_Handle_Type MW_I2C_Open(uint32_T I2CModule, MW_I2C_Mode_Type i2c_mode)
{
    #if ( defined(MATLAB_MEX_FILE) || defined(RSIM_PARAMETER_LOADING) ||  defined(RSIM_WITH_SL_SOLVER) )
		/* This will be run in Rapid Accelerator Mode */
		return NULL;
	#else
        MW_I2C_DEVICE *mw_i2c_dev = MW_I2C_DEVICE_interface(I2CModule);
        mw_i2c_dev->init();
        return (MW_Handle_Type) mw_i2c_dev;
    #endif
}

/* Set the I2C bus speed in Master Mode */
MW_I2C_Status_Type MW_I2C_SetBusSpeed(MW_Handle_Type I2CModuleHandle, uint32_T BusSpeed)
{
    #if ( defined(MATLAB_MEX_FILE) || defined(RSIM_PARAMETER_LOADING) ||  defined(RSIM_WITH_SL_SOLVER) )
		/* This will be run in Rapid Accelerator Mode */
		return MW_I2C_SUCCESS;
	#else
        if (I2CModuleHandle != NULL)
		{  
            uint8_t bus;
            MW_I2C_DEVICE * handle = static_cast<MW_I2C_DEVICE *> (I2CModuleHandle);
            bus = handle->get_device_bus();
#ifndef PX4_CONNECTEDIO        
             /* Override the BusSpeed from configset using macros
             * If macros not defined then use default 100,000 Hz init 
             * value for BusSpeed
             */
            #ifdef MW_I2C_BUS1SPEEDKHZ
                if (bus == 1)
                    BusSpeed = MW_I2C_BUS1SPEEDKHZ*1000; // in Hz
            #endif

            #ifdef MW_I2C_BUS2SPEEDKHZ
                if (bus == 2)
                    BusSpeed = MW_I2C_BUS2SPEEDKHZ*1000; // in Hz
            #endif
            
            #ifdef MW_I2C_BUS3SPEEDKHZ
                if (bus == 3)
                    BusSpeed = MW_I2C_BUS3SPEEDKHZ*1000; // in Hz
            #endif
            
            #ifdef MW_I2C_BUS4SPEEDKHZ
                if (bus == 4)
                    BusSpeed = MW_I2C_BUS4SPEEDKHZ*1000; // in Hz
            #endif
#endif
            handle->setBusSpeed(bus,BusSpeed);
            return MW_I2C_SUCCESS;
        }
        else
            return (MW_I2C_Status_Type) 1;  /*Unable to open the current I2C bus*/
     #endif
}

/* Set the slave address (used only by slave) */
MW_I2C_Status_Type MW_I2C_SetSlaveAddress(MW_Handle_Type I2CModuleHandle, uint32_T SlaveAddress)
{
    return MW_I2C_SUCCESS;
}

/* Receive the data on Master device from a specified slave */
MW_I2C_Status_Type MW_I2C_MasterRead(MW_Handle_Type I2CModuleHandle, uint16_T SlaveAddress, uint8_T * data, uint32_T DataLength, uint8_T RepeatedStart, uint8_T SendNoAck)
{
    int status = 0;
    #if ( defined(MATLAB_MEX_FILE) || defined(RSIM_PARAMETER_LOADING) ||  defined(RSIM_WITH_SL_SOLVER) )
		/* This will be run in Rapid Accelerator Mode */
		return MW_I2C_SUCCESS;
	#else
        if (I2CModuleHandle != NULL)
		{  
            MW_I2C_DEVICE * handle = static_cast<MW_I2C_DEVICE *> (I2CModuleHandle);
            status = handle->read(SlaveAddress, data, DataLength);
            if( status == 0)
                return MW_I2C_SUCCESS; /*I2C read success*/  
            else
                return (MW_I2C_Status_Type) 2; /*I2C read failed*/         
        }
        else
            return (MW_I2C_Status_Type) 1;  /*Unable to open the current I2C bus*/
    #endif    
}

/* Send the data from master to a specified slave */
MW_I2C_Status_Type MW_I2C_MasterWrite(MW_Handle_Type I2CModuleHandle, uint16_T SlaveAddress, uint8_T * data, uint32_T DataLength, uint8_T RepeatedStart, uint8_T SendNoAck)
{   
    int status = 0;
    #if ( defined(MATLAB_MEX_FILE) || defined(RSIM_PARAMETER_LOADING) ||  defined(RSIM_WITH_SL_SOLVER) )
        /* This will be run in Rapid Accelerator Mode */
        return MW_I2C_SUCCESS;
    #else
        if (I2CModuleHandle != NULL)
        {
			/* scanI2CBus function write data with datalength 0 and 
			PX4 returns error when written with datalength 0 - following is a workaround to run the test in BaT.
			better solution need to be investigated - open bug g2324534*/
            if (DataLength == 0)
            {
                DataLength = 1;
            }
            MW_I2C_DEVICE * handle = static_cast<MW_I2C_DEVICE *> (I2CModuleHandle);
            status = handle->write(SlaveAddress, data, DataLength);
            if( status == 0)
                return MW_I2C_SUCCESS; /*I2C write success*/ 
            else
                return (MW_I2C_Status_Type) 2;  /*I2C write failed*/ 
        }
        else
            return (MW_I2C_Status_Type) 1;  /*Unable to open the current I2C bus*/
    #endif 
}

/* Read data on the slave device from a Master */
MW_I2C_Status_Type MW_I2C_SlaveRead(MW_Handle_Type I2CModuleHandle, uint8_T * data, uint32_T DataLength, uint8_T SendNoAck)
{
    return MW_I2C_SUCCESS;
}

/* Send the data to a master from the slave */
MW_I2C_Status_Type MW_I2C_SlaveWrite(MW_Handle_Type I2CModuleHandle, uint8_T * data, uint32_T DataLength, uint8_T SendNoAck)
{
    return MW_I2C_SUCCESS;
}

/* Get the status of I2C device */
MW_I2C_Status_Type MW_I2C_GetStatus(MW_Handle_Type I2CModuleHandle)
{
    return MW_I2C_SUCCESS;
}

/* Release I2C module */
void MW_I2C_Close(MW_Handle_Type I2CModuleHandle)
{
    #if !( defined(MATLAB_MEX_FILE) || defined(RSIM_PARAMETER_LOADING) ||  defined(RSIM_WITH_SL_SOLVER) )
        MW_I2C_DEVICE * handle = static_cast<MW_I2C_DEVICE *> (I2CModuleHandle);
        delete handle;
    #endif
}

#ifdef __cplusplus
}
#endif 