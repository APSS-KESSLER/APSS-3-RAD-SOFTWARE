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
// Voltage calculated as (Vin/Vref)*((2^n) - 1)
#define maxEventVoltage 1638 // ~1V --> Todo: figure out what is appropriate
#define minEventVoltage 819   // ~0.5V --> Todo: figure out what is appropriate
#define PWM_PERIOD 8191
#define DUTY_CYCLE 4095

// --- Function Prototypes ---
void initAdc();

#endif
