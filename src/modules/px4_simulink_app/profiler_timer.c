/* Copyright 2019-2021 The MathWorks, Inc. */

#ifdef PIL  // Profiler timestamp configuration for PIL

#define MW_NVIC_SYSTICK_CTRL_CLK_SRC    0x00000004  // Clock Source
#define MW_NVIC_SYSTICK_INT_ENABLE_SRC  0x00000002  // Interrupt enable 
#define MW_NVIC_SYSTICK_CTRL_ENABLE     0x00000001  // Enable

#define MW_MAX_SYSTICK_COUNT            0x00FFFFFF

/* Define SYSTICK core registers */
volatile unsigned long* SYSTICK_RELOAD  = (volatile unsigned long*)0xE000E014;
volatile unsigned long* SYSTICK_CURRENT = (volatile unsigned long*)0xE000E018;
volatile unsigned long* SYSTICK_CTRL    = (volatile unsigned long*)0xE000E010;

/* Flag to initialize the timer */
unsigned char initTimer = 0;
/* Counter to count sysTick reaching to zero */
volatile unsigned long sysTickZeroCount = 0;

unsigned long profileTimerRead(void)
{
    unsigned long timerVal = 0;
    
    /* Removing the sign extension add at the end of this function */
    if(0 == initTimer) {
        /*When initTimer is null, it is xilProfilingSectionStart */
        *SYSTICK_RELOAD  = MW_MAX_SYSTICK_COUNT;
        *SYSTICK_CTRL    = MW_NVIC_SYSTICK_CTRL_CLK_SRC | MW_NVIC_SYSTICK_CTRL_ENABLE | MW_NVIC_SYSTICK_INT_ENABLE_SRC;
        *SYSTICK_CURRENT = MW_MAX_SYSTICK_COUNT;
        
        initTimer = 1;
    } else {
        /*When initTimer is NOT null, it is xilProfilingSectionEnd*/
        /* Up counting */
        timerVal = (MW_MAX_SYSTICK_COUNT - *SYSTICK_CURRENT);
    }
    
    timerVal = sysTickZeroCount + timerVal;

    return(timerVal);
}

void SysTick_Handler(void)
{
    sysTickZeroCount += (MW_MAX_SYSTICK_COUNT + 1);
}

#else  // Profiler timestamp configuration for External Mode Profiling

#include <drivers/drv_hrt.h>

/*
Using high resolution system time to get the timestamp for
profiler. The resolution of this timer is 1us. Using only 
the lower 32-bits as the timestamp.
*/
uint32_t profileTimerRead(void) {
    uint64_t currentTime = hrt_absolute_time();
    uint32_t time32bit = currentTime & 0xFFFFFFFF;

    return time32bit;
}

#endif

/* EOF */
