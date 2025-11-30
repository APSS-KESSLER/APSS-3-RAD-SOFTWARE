/******************************************************************************
*
* Author: Chris Lawrence
*
* Summary: Source file for timer functions
*
******************************************************************************/

#include "timer.h"
#include <msp430fr2355.h>

uint32_t rtc_seconds = 0;

// Initialise timer for microsecond counting
void initMicroTimer() {

    TB3CTL = TBSSEL_2 | MC_2 | TBCLR | TBIE;         // SMCLK, Continuous mode, Clear, Interrupt enable

}

// Function to get a 32-bit timer count combining timer and overflow count (Overflows ~71 minutes)
uint32_t micros() {

    // Variables to hold successive readings of the overflow counter and timer register
    uint16_t t1;
    uint32_t ovf1, ovf2;

    // Loop until consistent readings are obtained, this handles the case where the timer overflows during the read
    do {
        ovf1 = timer_overflows;
        t1 = TB3R;
        ovf2 = timer_overflows;
    } while (ovf1 != ovf2);

    return (ovf2 << 16) | t1; // Combine overflow count and timer value
}

// Initialise RTC for event time
void initRTC() {
    RTCCTL = RTCSS__VLOCLK | RTCPS__10 | RTCSR | RTCIE; // VLO Clock (10kHz), divide by 10, clear counter, enable rtc interrupt
    RTCMOD = 1;                                         // 1 millisecond with 10,000/10=1,000hz clock

}

void reset_rtc_seconds() {
    rtc_seconds = 0;
}

void set_rtc_seconds(uint32_t seconds) {
    rtc_seconds = seconds;
}

uint32_t get_rtc_seconds(){
    return rtc_seconds;
}


// Microsecond ISR
#pragma vector = TIMER3_B1_VECTOR
__interrupt void Timer_B_ISR(void) {

    // Switch statements to avoid undefined behaviour for other interrupt flags
    switch (TB3IV) {

        // Overflow flag case
        case TBIV__TBIFG:

            timer_overflows++; // Increment microseconds timer

            break;

        // Invalid case handling
        default:
            break;
}
}

// RTC ISR
#pragma vector = RTC_VECTOR
__interrupt void RTC_ISR(void) {
    switch (__even_in_range(RTCIV, RTCIV_RTCIF))
    {
    case RTCIV_NONE:
        break;
    case RTCIV_RTCIF:  // RTC Overflow
        rtc_seconds = rtc_seconds + 1;
        break;
    default:
        break;
    }
}
