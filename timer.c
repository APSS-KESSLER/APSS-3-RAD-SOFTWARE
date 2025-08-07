/******************************************************************************
*
* Author: Chris Lawrence
*
* Summary: Source file for timer functions
*
******************************************************************************/

#include "timer.h"
#include <msp430fr2355.h>

// Initialise timer for microsecond counting
void initMicroTimer() {

    TB0CTL = TBSSEL_2 | MC_2 | TBCLR | TBIE;         // SMCLK, Continuous mode, Clear, Interrupt enable

}

// Initialise timer for millisecond counting
void initMilliTimer() {

    TB3CCR0 = TIMER_1MS_SMCLK - 1;                  // 1ms interval
    TB3CCTL0 = CCIE;                                // Enable interrupt for CCR0
    TB3CTL = TBSSEL_2 | MC_1 | TBCLR;               // use SMCLK, count up to TB3CCR0, and clear timer

}

// Function to return milliseconds since inception
// Will overflow after approximately 49.7 days (2^32ms)
uint32_t millis() {
    return interrupt_timer;
}

// Function to get a 32-bit timer count combining timer and overflow count
// Overflow will occur every 71 minutes @ 1 MHz, Depending on timestamp requirements will need to find way to manage
uint32_t micros() {

    // Variables to hold successive readings of the overflow counter and timer register
    uint16_t t1;
    uint32_t ovf1, ovf2;

    // Loop until consistent readings are obtained, this handles the case where the timer overflows during the read
    do {
        ovf1 = timer_overflows;
        t1 = TB0R;
        ovf2 = timer_overflows;
    } while (ovf1 != ovf2);

    return (ovf2 << 16) | t1; // Combine overflow count and timer value
}

// Microsecond ISR
#pragma vector = TIMER0_B1_VECTOR
__interrupt void Timer_B_ISR(void) {

    // Switch statements to avoid undefined behaviour for other interrupt flags
    switch (TB0IV) {

        // Overflow flag case
        case TBIV__TBIFG:

            timer_overflows++; // Increment microseconds timer

            // Increment system deadtime
            if (waiting_for_interrupt == 1) {
                uint32_t current_time = (timer_overflows << 16) | TB0R;
                total_deadtime += (current_time - waiting_t1);
            }

            // Reset interrupt flag
            waiting_for_interrupt = 0;
            break;

        // Invalid case handling
        default:
            break;
}
}

// Millisecond ISR
#pragma vector = TIMER3_B0_VECTOR
__interrupt void TimerB3ISR(void) {

    // Increment ms counter
    interrupt_timer++;

}
