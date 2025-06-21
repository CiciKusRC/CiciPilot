/*
 * Copyright 2020-2023 The MathWorks, Inc.
 *
 * File: xcp_platform_custom.cpp
 *
 * Abstract:
 *  Implementation of custom XCP platform services for PX4 Host Target. All of the code
 *  apart from xcpAddressGet has been borrowed from 
 *  toolbox/coder/xcp/src/target/server/platform/default/xcp_platform_default.c
 *  The implementation of xcpAddressGet has been customized for PX4 Host Target in Windows and 
 *  Linux platform
 */

#include <stdlib.h>

#include "xcp_common.h"
#include "xcp_platform_custom.h"
#include "xcp_cfg.h"

/* Define the API to pause the execution (sleep) for a specific amount of time */

void xcpSleep(uint32_T seconds, uint32_T microseconds) {
#if _POSIX_C_SOURCE >= 199309L
    /* nanosleep() is the preferred sleep method on Linux */
    struct timespec t;
    t.tv_sec = seconds;
    t.tv_nsec = microseconds * 1000;

    nanosleep(&t, NULL);
#else
    /* if nanosleep() is not available, use the select() instead */
    struct timeval t;
    t.tv_sec = seconds;
    t.tv_usec = (suseconds_t)microseconds; /* tv_usec is signed */

    select(0, NULL, NULL, NULL, &t);
#endif /* _POSIX_C_SOURCE >= 199309L */
}

#ifndef XCP_TIMESTAMP_BASED_ON_SIMULATION_TIME
/* @note The following section provides the default implementation
         of the xcpGetTimestamp() API, based on the 'real' time
         for Linux and Windows platforms.
         If XCP_TIMESTAMP_BASED_ON_SIMULATION_TIME is defined,
         the same function will be implemented in the External Mode
         abstraction layer, based on the model simulation time */

/* Define the XCP_TIMESTAMP_UNITS_PER_SECOND value based on the
   selected XCP_TIMESTAMP_UNIT */
#if (XCP_TIMESTAMP_UNIT == XCP_TIMESTAMP_UNIT_1NS)
#define XCP_TIMESTAMP_UNITS_PER_SECOND 1e9
#elif (XCP_TIMESTAMP_UNIT == XCP_TIMESTAMP_UNIT_10NS)
#define XCP_TIMESTAMP_UNITS_PER_SECOND 1e8
#elif (XCP_TIMESTAMP_UNIT == XCP_TIMESTAMP_UNIT_100NS)
#define XCP_TIMESTAMP_UNITS_PER_SECOND 1e7
#elif (XCP_TIMESTAMP_UNIT == XCP_TIMESTAMP_UNIT_1US)
#define XCP_TIMESTAMP_UNITS_PER_SECOND 1e6
#elif (XCP_TIMESTAMP_UNIT == XCP_TIMESTAMP_UNIT_10US)
#define XCP_TIMESTAMP_UNITS_PER_SECOND 1e5
#elif (XCP_TIMESTAMP_UNIT == XCP_TIMESTAMP_UNIT_100US)
#define XCP_TIMESTAMP_UNITS_PER_SECOND 1e4
#elif (XCP_TIMESTAMP_UNIT == XCP_TIMESTAMP_UNIT_1MS)
#define XCP_TIMESTAMP_UNITS_PER_SECOND 1e3
#elif (XCP_TIMESTAMP_UNIT == XCP_TIMESTAMP_UNIT_10MS)
#define XCP_TIMESTAMP_UNITS_PER_SECOND 1e2
#elif (XCP_TIMESTAMP_UNIT == XCP_TIMESTAMP_UNIT_100MS)
#define XCP_TIMESTAMP_UNITS_PER_SECOND 1e1
#elif (XCP_TIMESTAMP_UNIT == XCP_TIMESTAMP_UNIT_1S)
#define XCP_TIMESTAMP_UNITS_PER_SECOND 1
#elif (XCP_TIMESTAMP_UNIT == XCP_TIMESTAMP_UNIT_1PS)
#define XCP_TIMESTAMP_UNITS_PER_SECOND 1e12
#elif (XCP_TIMESTAMP_UNIT == XCP_TIMESTAMP_UNIT_10PS)
#define XCP_TIMESTAMP_UNITS_PER_SECOND 1e11
#elif (XCP_TIMESTAMP_UNIT == XCP_TIMESTAMP_UNIT_100PS)
#define XCP_TIMESTAMP_UNITS_PER_SECOND 1e10
#else
#error "invalid XCP_TIMESTAMP_UNIT value"
#endif

/* Define the corresponding XCP_TIMESTAMP_UNITS_PER_MICROSECOND value */
#define XCP_TIMESTAMP_UNITS_PER_MICROSECOND (XCP_TIMESTAMP_UNITS_PER_SECOND / 1e6)

static boolean_T xcpTimestampInitialized = false;
static uint64_t xcpTimestampStartTime = 0;

/* Return value in XCP_TIMESTAMP_UNITs
   @note in the current implementation the time starts counting from 0 on the first call */
uint32_T xcpGetTimestamp(void) {
    struct timeval timeVal;
    uint64_t currentTime = 0;
    uint32_T ret = 0;

    gettimeofday(&timeVal, NULL);
    currentTime = timeVal.tv_sec * XCP_TIMESTAMP_UNITS_PER_SECOND +
                  timeVal.tv_usec * XCP_TIMESTAMP_UNITS_PER_MICROSECOND;

    if (!xcpTimestampInitialized) {
        /* Read initial time and store it in xcpTimestampStartTime variable */
        xcpTimestampStartTime = currentTime;

        xcpTimestampInitialized = true;
    }

    if (currentTime >= xcpTimestampStartTime) {
        ret = (uint32_T)(currentTime - xcpTimestampStartTime);
    } else {
        ret = (uint32_T)(currentTime + UINT64_MAX - xcpTimestampStartTime);
    }

    return ret;
}

#endif /* XCP_TIMESTAMP_BASED_ON_SIMULATION_TIME */

/* Define the API to convert the XCP address and extension into the corresponding
   variable address in the target address space */
#if defined(_MINGW_W64)

/* The PX4 Firmware is compiled on Windows using the mingw64 toolchain that provides
   equivalent Windows implementation of POSIX APIs. The final executable px4.exe
   contains the Image Base address as 0x100400000.
   The DwarfParser extracts the symbol address not as Relative Virtual Address defined
   in the executable but including the base address. For ex- the variable xcpModelStatus
   is extracted as a symbol having address 0x1005EB6F0 where 0x001EB6F0 (0x1005EB6F0 - 0x100400000)
   is RVA.
   XCPTargetHandler parses this symbol by extracting the 32 bit information as 
   'address' = 0x005EB6F0 and the higher bit information as 'addressExtension' = 1. 
   The following function recombines the 'address' (0x005EB6F0) and 'addressExtension' (1)  
   to get the actual load address (0x1005EB6F0) back*/

uint8_T* xcpAddressGet(uint8_T addressExtension, uint32_T address) {
    return (uint8_T*)((uintptr_t)address | (((uintptr_t)addressExtension) << 32));
}

#elif defined(_GNU_LINUX)

/* The PX4 Firmware is compiled on Linux using the GNU GCC toolchain.
   The linker script provides a symbol '__executable_start' which is
   the START address of the .text segment of the elf.
   The DwarfParser extracts the symbol address as Relative Virtual Address defined
   in the elf. Because the elf is an Position Independent Executable is (PIE),
   Linux OS is free to load the elf starting at any address (ASLR).
   Adding the text segment start address (__executable_start) with RVA of the symbols
   ensures that we are immune to Address Space Randomization*/

extern char __executable_start;

/* If the binary is compiled as Position Independent Executable, the address sent by the XCP Controller
   is treated as an offset from the __executable_start address */
uint8_T* xcpAddressGet(uint8_T addressExtension, uint32_T address) {
    XCP_UNUSED_PARAM(addressExtension);

    return (uint8_T*)((uintptr_t)&__executable_start + (uintptr_t)address);
}

#else

#error "platform not supported: a custom Platform Abstraction Layer needs to be provided."

#endif
