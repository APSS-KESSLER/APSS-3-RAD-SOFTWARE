/******************************************************************************
*
* Author: Chris Lawrence
*
* Summary: Source file for ADC functionality
*
******************************************************************************/

#include <msp430fr2355.h>
#include "adc.h"
#include <stdint.h>

// ADC logging variables
volatile uint8_t consecHigh = 0;
unsigned int SlowToggle_Period = 20000-1;
unsigned int FastToggle_Period = 3000-1;
unsigned int adcResult;                                         // Temporarily stores the ADC value

// Function to initialise ADC
void initAdc() {

    // Configure ADC;
    ADCCTL0 = ADCSHT_2 | ADCON;                                 // sample-and-hold 16 ADCCLK cycles, ADCON
    ADCCTL1 = ADCSHP | ADCSHS_2 | ADCSSEL_1 | ADCCONSEQ_2;      // TB1.1 trigger; ACLK for ADCCLK; Rpt single ch
    ADCCTL2 = ADCRES_2;                                         // 12-bit conversion results
    ADCMCTL0 = ADCINCH_2 | ADCSREF_1;                           // Vref 2.5v, A2
    ADCHI = maxEventVoltage;                                    // Window Comparator Hi-threshold
    ADCLO = minEventVoltage;                                    // Window Comparator Lo-threshold
    ADCIE |= ADCHIIE | ADCLOIE | ADCINIE;                       // Enable ADC conv complete interrupt

    // Configure Internal reference voltage
    PMMCTL0_H = PMMPW_H;                                        // Unlock the PMM registers
    PMMCTL2 |= INTREFEN | REFVSEL_2;                            // Enable internal 2.5V reference
    __delay_cycles(400);                                 // Delay for reference settling

    // Configure TB0 period-timer
    TB0CCTL0 = CCIE;                                            // CCR0 interrupt enabled
    TB0CTL = TBSSEL_2 | TBCLR;                                  // SMCLK, clear TBR

    // Configure ADC timer trigger TB1.1
    TB1CCR0 = PWM_PERIOD;                                       // PWM Period
    TB1CCR1 = DUTY_CYCLE;                                       // Duty cycle TB1.1 (currently every 0.5s)
    TB1CCTL1 = OUTMOD_3;                                        // TB1CCR1 set/reset mode
    TB1CTL = TBSSEL_2 | MC_1 | TBCLR;                           // SMCLK, up mode
    ADCCTL0 |= ADCENC;                                          // Enable conversion

}
