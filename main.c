// **********************************************************
// *                                                        *
// *     Author: Chris Lawrence                             *
// *                                                        *
// *     Summary: MSP430 Code Adaptation of CosmicWatch     *
// *                                                        *
// **********************************************************

#include <msp430fr2355.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "pinConfig.h"
#include "spi.h"
#include "config.h"
#include "i2c.h"
#include "timer.h"
#include "adc.h"

//******************************************************************************
// Initialisation Functions ****************************************************
//******************************************************************************

// Function to initialise DCO frequency for MCLK and SMCLK clocks
void initDcoFrequency() {
    
    __bis_SR_register(SCG0);                         // disable FLL
    CSCTL3 |= SELREF__REFOCLK;                       // Set REFO as FLL reference source
    CSCTL0 = 0;                                      // clear DCO and MOD registers
    CSCTL1 &= ~(DCORSEL_7);                          // Clear DCO frequency select bits first
    CSCTL1 |= DCORSEL_2;                             // Set DCO = 4MHz
    CSCTL2 = FLLD_1 + 60;                            // DCODIV = 2MHz
    __delay_cycles(3);                          
    __bic_SR_register(SCG0);                         // enable FLL

    while(CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1));       // Poll until FLL is locked

    CSCTL4 = SELMS__DCOCLKDIV | SELA__REFOCLK;       // set default REFO(~32768Hz) as ACLK source, ACLK = 32768Hz
                                                     // default DCODIV as MCLK and SMCLK source
    CSCTL5 |= DIVM__1 | DIVS__2;                     // SMCLK = 1MHz, MCLK = 2MHz

}

int main(void){
    
    // Initialise system
    WDTCTL = WDTPW | WDTHOLD;                     // Stop watchdog

    configurePin1_4(GPIO_14in);                   // Configure pin 1.4 to GPIO input
    configurePin1_5(A5);                          // Configure pin 1.5 to Analogue
    configurePin1_6(UCA0RX);                      // Confiugre pin 1.6 to RX
    configurePin1_7(UCA0TX);                      // Configure pin 1.7 to TX
    PM5CTL0 &= ~LOCKLPM5;                         // Enable GPIO
    P1IFG = 0;                                    // Clear to avoid erroneous port interrupts

    initDcoFrequency();                           // Clock
    initAdc();                                    // ADC
    initMicroTimer();                             // Timer B1 for microseconds
    initI2C();                                    // I2C setup
    initSPI();                                    // SPI setup
    __enable_interrupt();                         // Enable global interrupts

    P1DIR |= BIT0;
    P1OUT &= ~BIT0;

    P1DIR |= BIT1;
    P1OUT &= ~BIT1;
    
    while (1) {

        switch(currentEvent){

            case noEvent:
            // P1OUT |= BIT1; // Set LED off by default
            break;

            case highEvent:
            eventTime = micros();
            while(micros() - eventTime < 5){}
            if (adcStillHigh()){
                P1OUT &= ~BIT1;
            }
            currentEvent = noEvent;              
            break;

            case inEvent:
            P1OUT |= BIT1;
            // currentEvent = noEvent;
            break;

            case lowEvent:
            eventTime = micros();
            while(micros() - eventTime < 5){}
            if (adcStillLow()){
                P1OUT &= ~BIT1;
            }
            currentEvent = noEvent;              
            break;
        }

    }
}
