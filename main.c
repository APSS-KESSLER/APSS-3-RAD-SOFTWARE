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
#include "datalog.h"
#include "uart.h"

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
    WDTCTL = WDTPW | WDTHOLD;                       // Stop watchdog

    configurePin1_4(GPIO_14in);                    // Configure pin 1.4 to GPIO input
    configurePin1_5(A5);                           // Configure pin 1.5 to Analogue
    configurePin1_6(MISO);                         // Confiugre pin 1.6 to RX
    configurePin1_7(MOSI);                         // Configure pin 1.7 to TX
    P1IFG = 0;                                     // Clear to avoid erroneous port interrupts

    initDcoFrequency();                           // Clock setup
    initAdc();                                    // ADC setup
    initMicroTimer();                             // Timer B1 for microseconds
    initI2C();                                    // I2C setup
    // initSPI();                                    // SPI setup
    initUart();                                   // UART setup
    initRTC();                                    // RTC setup
    __enable_interrupt();                         // Enable global interrupts

    P6DIR |= BIT6;                                // P6.6 as output
    P6OUT &= ~BIT6;                               // LED off initially
    PM5CTL0 &= ~LOCKLPM5;                         // Enable GPIO
    uint32_t eventTimeLog = 0;

    while (1) {
        
        // // ADC event handling
        switch(currentEvent){

            case noEvent:
            P6OUT &= ~BIT6;   // LED off initially
            break;

            case highEvent:
            eventTime = micros();
            eventTimeLog = get_rtc_seconds();
            while(micros() - eventTime < 5){}
            if (adcStillHigh()){
                enqueue(eventTimeLog);
                P6OUT &= ~BIT6;
            }
            currentEvent = noEvent; 
            break;

            case inEvent:
            // To be implemented
            break;

            case lowEvent:
            eventTime = micros();
            eventTimeLog = get_rtc_seconds();
            while(micros() - eventTime < 5){}
            if (adcStillLow()){
                enqueue(eventTimeLog);
                P6OUT |= BIT6;
            }
            currentEvent = noEvent; 
            break;
        }

    }
}
