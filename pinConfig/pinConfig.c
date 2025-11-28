/******************************************************************************
*
* Author: Chris Lawrence
*
* Summary: Source file for Pin Configuration
*
******************************************************************************/

#include <msp430fr2355.h>
#include "pinConfig.h"

// Setup Pin 1.2 for I2C data line (SDA) and Pin 1.3 for I2C clock line (SCL)
void configurePinI2C(){

    // Reset all pins in port
    P1SEL0 &= ~(BIT2 | BIT3);
    P1SEL1 &= ~(BIT2 | BIT3);
    P1DIR  &= ~(BIT2 | BIT3);
    P1REN  &= ~(BIT2 | BIT3);
    P1OUT  &= ~(BIT2 | BIT3);

    // Configure P1.2 for UCB0SDA
    P1SEL0 |= BIT2;
    P1SEL1 &= ~BIT2;

    // Configure P1.3 for UCB0SCL
    P1SEL0 |= BIT3;
    P1SEL1 &= ~BIT3;
}

void configurePin1_4(unsigned int config){

    // Reset all pins in the port
    P1SEL0 &= ~BIT4;
    P1SEL1 &= ~BIT4;
    P1DIR &= ~BIT4;
    P1REN &= ~BIT4;
    P1OUT &= ~BIT4;

    switch(config){

        case GPIO_14in:
            P1DIR &= ~BIT4;     // Configured as input
            P1REN |= BIT4;      // Enable internal resistor
            P1OUT |= BIT4;      // Select pullup resistor
            break;

        case GPIO_14out:       
            P1DIR |= BIT4;      // Configured as output
            P1OUT |= BIT4;      // Set output mode to HIGH
            break;

        case A4:
            P1SEL0 |= BIT4;     // Select Bits for analog function
            P1SEL1 |= BIT4;
            break;

        case UCA0STE:
            P1SEL0 |= BIT4;
            P1SEL1 &= ~BIT4;    
            break;

    }

}

void configurePin1_5(unsigned int config){

    // Reset all pins in the port
    P1SEL0 &= ~BIT5;
    P1SEL1 &= ~BIT5;
    P1DIR &= ~BIT5;
    P1REN &= ~BIT5;
    P1OUT &= ~BIT5;

    switch(config){

        case GPIO_15in:
            P1DIR &= ~BIT5;     // Configured as input
            P1REN |= BIT5;      // Enable internal resistor
            P1OUT |= BIT5;      // Select pullup resistor
            break;

        case GPIO_15out:       
            P1DIR |= BIT5;      // Configured as output
            P1OUT |= BIT5;      // Set output mode to HIGH
            break;

        case A5:
            P1SEL0 |= BIT5;     // Select Bits for analog function
            P1SEL1 |= BIT5;
            break;

        case UCA0CLK:
            P1SEL0 |= BIT5;
            P1SEL1 &= ~BIT5;    
            break;
    }

}

void configurePin1_6(unsigned int config){

    // Reset all pins in the port
    P1SEL0 &= ~BIT6;
    P1SEL1 &= ~BIT6;
    P1DIR &= ~BIT6;
    P1REN &= ~BIT6;
    P1OUT &= ~BIT6;

    switch(config){

        case GPIO_16in:
            P1DIR &= ~BIT6;     // Configured as input
            P1REN |= BIT6;      // Enable internal resistor
            P1OUT |= BIT6;      // Select pullup resistor
            break;

        case GPIO_16out:
            P1DIR |= BIT6;      // Configured as output
            P1OUT |= BIT6;      // Set output mode to HIGH
            break;

        case A6:
            P1SEL0 |= BIT6;     // Select Bits for analog function
            P1SEL1 |= BIT6;
            break;

        case UCA0RX:
            P1SEL0 |= BIT6;
            P1SEL1 &= ~BIT6;   
            break;

        case MISO:
            P1SEL0 |= BIT6;
            P1SEL1 &= ~BIT6;   
            break;
    }
}

void configurePin1_7(unsigned int config){

    // Reset all pins in the port
    P1SEL0 &= ~BIT7;
    P1SEL1 &= ~BIT7;
    P1DIR &= ~BIT7;
    P1REN &= ~BIT7;
    P1OUT &= ~BIT7;

    switch(config){

        case GPIO_17in:
            P1DIR &= ~BIT7;     // Configured as input
            P1REN |= BIT7;      // Enable internal resistor
            P1OUT |= BIT7;      // Select pullup resistor
            break;

        case GPIO_17out:
            P1DIR |= BIT7;      // Configured as output
            P1OUT |= BIT7;      // Set output mode to HIGH
            break;

        case A7:
            P1SEL0 |= BIT7;     // Select Bits for analog function
            P1SEL1 |= BIT7;
            break;

        case UCA0TX:
            P1SEL0 |= BIT7;
            P1SEL1 &= ~BIT7;   
            break;

        case MOSI:
            P1SEL0 |= BIT7;
            P1SEL1 &= ~BIT7;  
            break;
    }

}
