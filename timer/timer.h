/******************************************************************************
*
* Author: Chris Lawrence
*
* Summary: Header file for timer functions
*
******************************************************************************/

#ifndef TIMER_H
#define TIMER_H

#include <stdint.h>
#include "config/config.h"

// --- Function Prototypes ---
void initMicroTimer(void);
uint32_t micros(void);
void initRTC(void);
void reset_rtc_seconds(void);
void set_rtc_seconds(uint32_t seconds);
uint32_t get_rtc_seconds();

#endif /* TIMERS_H */
