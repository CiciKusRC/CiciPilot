/* Copyright 2018 The MathWorks, Inc. */

#include "nuttxinitialize.h"
#include "MW_PX4_TaskControl.h"
#include "MW_uORB_Write.h"

#define DEBUG 0

void uORB_write_initialize(orb_metadata_t* orbData,
                          orb_advert_t* orbAdvertObj,
                          void* busData,
                          int queueLen) {
    *orbAdvertObj = orb_advertise_queue(orbData, busData, queueLen);    
#if DEBUG
    PX4_INFO("Started advertising topic: %s \n",  orbData->o_name);
#endif    
}

void uORB_write_step(orb_metadata_t* orbData,
                    orb_advert_t* orbAdvertObj,
                    void* busData) {
    orb_publish(orbData, *orbAdvertObj, busData);
}

void uORB_write_terminate(const orb_advert_t* orbAdvertObj) {
    orb_unadvertise(*orbAdvertObj);
}
