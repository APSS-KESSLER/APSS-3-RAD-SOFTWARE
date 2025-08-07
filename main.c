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
#include "pinConfig.h"
#include "spi.h"
#include "config.h"
#include "i2c.h"
#include "timer.h"

//******************************************************************************
// Initialisation Functions ****************************************************
//******************************************************************************

// Function to initialise DCO frequency for MCLK and SMCLK clocks
void initDcoFrequency() {
    
    __bis_SR_register(SCG0);                    // disable FLL
    CSCTL3 |= SELREF__REFOCLK;                       // Set REFO as FLL reference source
    CSCTL0 = 0;                                      // clear DCO and MOD registers
    CSCTL1 &= ~(DCORSEL_7);                          // Clear DCO frequency select bits first
    CSCTL1 |= DCORSEL_2;                             // Set DCO = 4MHz
    CSCTL2 = FLLD_1 + 60;                            // DCODIV = 2MHz
    __delay_cycles(3);                          
    __bic_SR_register(SCG0);                    // enable FLL

    while(CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1));       // Poll until FLL is locked

    CSCTL4 = SELMS__DCOCLKDIV | SELA__REFOCLK;       // set default REFO(~32768Hz) as ACLK source, ACLK = 32768Hz
                                                     // default DCODIV as MCLK and SMCLK source
    CSCTL5 |= DIVM__1 | DIVS__2;                     // SMCLK = 1MHz, MCLK = 2MHz

}

// Function to initialise ADC
void initAdc() {

    // Configure the ADC
    ADCCTL0 &= ~ADCENC;                             // Disable ADC to configure
    ADCCTL0 = ADCON | ADCSHT_2 | ADCMSC;
    ADCCTL1 = ADCSHP | ADCDIV_7;                    // Use sampling timer, set ADC clock to MODCLK / 8
    ADCCTL2 &= ~(BIT5 | BIT4);                      // Clear resolution bits
    ADCCTL2 = ADCRES_2;                             // 12-bit resolution ** TODO: find appropriate clock frequency for bit resolution **
    ADCMCTL0 = ADCINCH_0;                           // Default to channel 0 (A0)

}

//******************************************************************************
// I/O  ************************************************************************
//******************************************************************************

// Function to configure and read from an analogue pin
unsigned int adcRead(unsigned int channel) {
   
    // Clear ADC function bits
    P5SEL0 &= ~(BIT0 | BIT1 | BIT2 | BIT3);
    P5SEL1 &= ~(BIT0 | BIT1 | BIT2 | BIT3);

    // Decide which channel is being utilised
    switch(channel) {
        
        // P5.0
        case 0:
            P5SEL0 |= BIT0;
            P5SEL1 &= ~BIT0;
            break;
       
        // P5.1
        case 1:
            P5SEL0 |= BIT1;
            P5SEL1 &= ~BIT1;
            break;
      
        // P5.2
        case 2:
            P5SEL0 |= BIT2;
            P5SEL1 &= ~BIT2;
            break;
        
        // P5.3
        case 3:
            P5SEL0 |= BIT3;
            P5SEL1 &= ~BIT3;
            break;

        // Invalid input handling
        default:
            return 0;
    }

    // Clear existing channel selection bits and set the new ADC input channel (A0 --> A15)
    // TODO: If ADC issues occur, toggle ADCENC bit off, reconfigure ADC, then re-enable ADCENC
    ADCMCTL0 = (ADCMCTL0 & ~(ADCINCH_15)) | (channel & 0x0F);  
    __delay_cycles(10);  // 10 us delay so ADC can stabilise

    // Begin Conversion
    ADCCTL0 |= ADCENC | ADCSC;
    while (ADCCTL1 & ADCBUSY);              // Wait for conversion
    return ADCMEM0;                         // Return result
}

int gpioReadPin5(uint8_t pin) {
    
    if (pin > 3) return 0; // Prevent out-of-range access

    // Clear analog mode to enable digital function
    P5SEL0 &= ~(1 << pin);
    P5SEL1 &= ~(1 << pin);

    // Set as input
    P5DIR &= ~(1 << pin);

    // Read pin
    return (P5IN & (1 << pin)) ? 1 : 0;
}

int main(void){
    
    // Initialise system
    WDTCTL = WDTPW | WDTHOLD;                     // Stop watchdog

    configurePin1_4(GPIO_14in);            // Configure pin 1.4 to GPIO input
    configurePin1_5(A5);                   // Configure pin 1.5 to Analogue
    configurePin1_6(UCA0RX);               // Confiugre pin 1.6 to RX
    configurePin1_7(UCA0TX);               // Configure pin 1.7 t TX
    PM5CTL0 &= ~LOCKLPM5;                         // Enable GPIO
    P1IFG = 0;                                    // Clear to avoid erroneous port interrupts

    initDcoFrequency();                           // Clock
    initAdc();                                    // ADC
    initMicroTimer();                             // Timer B0 for microseconds
    initMilliTimer();                             // Timer B3 for milliseconds
    initI2C();                                    // I2C setup
    initSPI();                                    // SPI setup
    __enable_interrupt();                         // Enable global interrupts

    // Main program loop (to be implemented)        
    while (1) {

    
    }
    
}
