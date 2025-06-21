/* Copyright 2018-2020 The MathWorks, Inc. */

#include "nuttxinitialize.h"
#include "MW_PX4_TaskControl.h"
#include "MW_uORB_Read.h"

#define DEBUG 0

void uORB_read_initialize(orb_metadata_t* orbData, pollfd_t* eventStructObj) {
    int fd = orb_subscribe(orbData);
    eventStructObj->fd = fd;
    eventStructObj->events = POLLIN;
#if DEBUG
    PX4_INFO("* Subscribed to topic: %s (fd = %d)*\n", orbData->o_name, fd);
#endif
}

boolean_T uORB_read_step(orb_metadata_t* orbData,
                         pollfd_t* eventStructObj,
                         void* busData,
                         boolean_T blockingMode,
                         double blockingTimeout) {
    boolean_T updated = 0;
    bool isUpdated = false;
    if (blockingMode) {
#if defined(MW_PX4_NUTTX_BUILD)
        int poll_ret = px4_poll(eventStructObj, 1, blockingTimeout);
        static int error_counter = 0;
        if (poll_ret == 0) {
#if DEBUG
            PX4_ERR("Got no data within %.9lf second", blockingTimeout / 1000.0);
#endif
        } else if (poll_ret < 0) {
            if (error_counter < 10 || error_counter % 500 == 0) {
                /* use a counter to prevent flooding and slowing the system down */
#if DEBUG
                PX4_ERR("ERROR return value from poll(): %d", poll_ret);
#endif
            }
            error_counter++;

        } else {
            if (eventStructObj->revents & POLLIN) {
                orb_copy(orbData, eventStructObj->fd, busData);
                updated = 1;
            }
        }
#elif defined(MW_PX4_POSIX_BUILD)
        double count = 0;
        /* The blocking timeout mentioned in the block mask is seconds but
         * system object converts it to milliseconds*/
        while (count < blockingTimeout) {
            // Sleep for 1 ms and then again check if
            // uORB topic is updated
            usleep(1000);
            orb_check(eventStructObj->fd, &isUpdated);
            if (isUpdated) {
                orb_copy(orbData, eventStructObj->fd, busData);
                break;
            }
            count++;
        }

        updated = isUpdated ? 1 : 0;
#endif
    } else {
        orb_check(eventStructObj->fd, &isUpdated);
        if (isUpdated) {
            orb_copy(orbData, eventStructObj->fd, busData);
        }
        updated = isUpdated ? 1 : 0;
    }

    return updated;
}

void uORB_read_terminate(const pollfd_t* eventStructObj) {
    orb_unsubscribe(eventStructObj->fd);
}
