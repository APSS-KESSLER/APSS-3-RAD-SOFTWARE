/******************************************************************************
*
* Author: Chris Lawrence
*
* Summary: Header file for ADC functionality
*
******************************************************************************/

#ifndef ADC_H
#define ADC_H

#include <stdbool.h>

// --- CONSTANTS defined using macros ---
// Voltage calculated as (Vin/Vref)*((2^n) - 1)
#define maxEventVoltage 655 // ~240mV
#define minEventVoltage 327   // ~120mV
#define PWM_PERIOD 8191
#define DUTY_CYCLE 4095

// --- Function Prototypes ---
void initAdc();
bool adcStillLow();
bool adcStillHigh();


typedef enum{
    noEvent,
    highEvent,
    inEvent,
    lowEvent
} adcEvent;

extern volatile adcEvent currentEvent;
extern volatile unsigned long eventTime;

#endif
