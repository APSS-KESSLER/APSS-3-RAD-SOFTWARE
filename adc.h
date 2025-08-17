/******************************************************************************
*
* Author: Chris Lawrence
*
* Summary: Header file for ADC functionality
*
******************************************************************************/

#ifndef ADC_H
#define ADC_H

// --- CONSTANTS defined using macros ---
#define maxEventVoltage 2.5 // Todo: figure out what is appropriate
#define minEventVoltage 2   // Todo: figure out what is appropriate
#define PWM_PERIOD 8191
#define DUTY_CYCLE 4095

// --- Function Prototypes ---
void initAdc();

#endif
