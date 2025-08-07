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
#include "config.h"

// --- Function Prototypes ---
void initMicroTimer(void);
void initMilliTimer(void);
uint32_t millis(void);
uint32_t micros(void);

#endif /* TIMERS_H */
