/******************************************************************************
*
* Author: Chris Lawrence
*
* Summary: Source file for I2C communication
*
******************************************************************************/

#include "i2c.h"
#include <msp430fr2355.h>

// Function to initialise I2C communication
void initI2C() {

    UCB0CTLW0 = UCSWRST;                           // Software reset enabled
    UCB0CTLW0 |= UCMODE_3 | UCSYNC;                // I2C mode, sync mode
    UCB0I2COA0 = SLAVE_ADDR | UCOAEN;              // Own Address and enable
    UCB0CTLW0 &= ~UCSWRST;                         // clear reset register
    UCB0IE |= UCRXIE + UCSTPIE;
    
}

