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
#include "timer.h"

// ADC logging variables
volatile uint8_t consecHigh = 0;
unsigned int SlowToggle_Period = 20000-1;
unsigned int FastToggle_Period = 3000-1;
unsigned int adcResult;                                         // Temporarily stores the ADC value

volatile adcEvent currentEvent = noEvent;
volatile unsigned long eventTime;


// Function to initialise ADC
void initAdc() {

    // Configure ADC;
    ADCCTL0 = ADCSHT_2 | ADCON;                                 // sample-and-hold 16 ADCCLK cycles, ADCON
    ADCCTL1 = ADCSHP | ADCSHS_2 | ADCSSEL_1 | ADCCONSEQ_2;      // TB1.1 trigger; ACLK for ADCCLK; Rpt single ch
    ADCCTL2 = ADCRES_2;                                         // 12-bit conversion results
    ADCMCTL0 = ADCINCH_5 | ADCSREF_1;                           // Vref 1.5v, A5
    ADCHI = maxEventVoltage;                                    // Window Comparator Hi-threshold
    ADCLO = minEventVoltage;                                    // Window Comparator Lo-threshold
    ADCIE |= ADCHIIE | ADCLOIE | ADCINIE;                       // Enable ADC conv complete interrupt

    // Configure Internal reference voltage
    PMMCTL0_H = PMMPW_H;                                        // Unlock the PMM registers
    PMMCTL2 |= INTREFEN | REFVSEL_0;                            // Enable internal 1.5V reference
    __delay_cycles(400);                                        // Delay for reference settling

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

bool adcStillLow(){
    ADCCTL0 &= ~ADCENC;          // Disable to allow new start
    ADCCTL0 |= ADCENC;           // Re-enable
    ADCCTL0 |= ADCSC;            // Start conversions
    while (ADCCTL1 & ADCBUSY) {}
    return (ADCMEM0 < ADCLO);              // Read the new result
}

bool adcStillHigh(){
    ADCCTL0 &= ~ADCENC;          // Disable to allow new start
    ADCCTL0 |= ADCENC;           // Re-enable
    ADCCTL0 |= ADCSC;            // Start conversions
    while (ADCCTL1 & ADCBUSY) {}
    return (ADCMEM0 > ADCHI);              // Read the new result
}

// ADC interrupt service routine
#pragma vector=ADC_VECTOR
__interrupt void ADC_ISR(void) {
    switch(__even_in_range(ADCIV,ADCIV_ADCIFG))
    {
        case ADCIV_NONE:
            break;
        case ADCIV_ADCOVIFG:
            break;
        case ADCIV_ADCTOVIFG:
            break;
        case ADCIV_ADCHIIFG:                            // ADCHI; A5 > 240mV
            currentEvent = highEvent;
            ADCIFG &= ~ADCHIIFG;            
            break;
        case ADCIV_ADCLOIFG:                            // ADCLO; A5 < 120mV
            currentEvent = lowEvent;
            ADCIFG &= ~ADCLOIFG;
            break;
        case ADCIV_ADCINIFG:                            // ADCIN; 120mV < A5 < 240mV
            currentEvent = inEvent;
            ADCIFG &= ~ADCINIFG;
            break;
        case ADCIV_ADCIFG:
            break;
        default:
            break;
    }
}

// Timer2 B2 interrupt service routine
#pragma vector=TIMER0_B0_VECTOR
__interrupt void TIMER0_B0_ISR(void) {
  P1OUT ^= BIT0;
}

