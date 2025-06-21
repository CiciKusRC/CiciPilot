/*
 * Copyright 2020 The MathWorks, Inc.
 *
 * File: xcp_platform_custom.h
 *
 * Abstract:
 *  Custom implementation for XCP Platform Abstraction Layer interface
 *  supporting PX4 Host Target on Linux and Windows OS
 */

#ifndef XCP_PLATFORM_CUSTOM_H
#define XCP_PLATFORM_CUSTOM_H

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <sys/time.h> /* gettimeofday */

#if _POSIX_C_SOURCE >= 199309L
#include <time.h> /* for nanosleep */
#else
#include <stddef.h>
#include <sys/select.h> /* for select */
#endif                  /* _POSIX_C_SOURCE >= 199309L */

#include <pthread.h>

/* The default "printf-like" logging function is the standard printf */
#define XCP_PRINTF printf

/* Define Mutual exclusion APIs */
#define XCP_MUTEX_DEFINE(lock) pthread_mutex_t lock
#define XCP_MUTEX_INIT(lock) pthread_mutex_init(&(lock), NULL)
#define XCP_MUTEX_LOCK(lock) pthread_mutex_lock(&(lock))
#define XCP_MUTEX_UNLOCK(lock) pthread_mutex_unlock(&(lock))

#ifdef __cplusplus

extern "C" {

#endif /*__cplusplus*/

/* Define the API to convert the XCP address and extension into the corresponding
   variable address in the target address space */
extern uint8_T* xcpAddressGet(uint8_T addressExtension, uint32_T address);
#define XCP_ADDRESS_GET(addressExtension, address) xcpAddressGet(addressExtension, address)


#ifndef XCP_TIMESTAMP_BASED_ON_SIMULATION_TIME

/* Define the API to get the current timestamp */
extern uint32_T xcpGetTimestamp(void);
#define XCP_TIMESTAMP_GET xcpGetTimestamp

#endif

/* Define the API to pause the execution (sleep) for a specific amount of time */
extern void xcpSleep(uint32_T seconds, uint32_T microseconds);
#define XCP_SLEEP(seconds, microseconds) xcpSleep(seconds, microseconds)

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* XCP_PLATFORM_CUSTOM_H */
