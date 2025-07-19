// **********************************************************
// *                                                        *
// *     Author: Chris Lawrence                             *
// *     Date:   19/07/25                                   *
// *                                                        *
// *     Summary: MSP430 Code Adaptation of CosmicWatch     *
// *                                                        *
// **********************************************************

#include <msp430fr2355.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>

// Signal and LED constants
const int SIGNAL_THRESHOLD = 50;  // Min threshold to trigger on. See calibration.pdf for conversion to mV
const int RESET_THRESHOLD = 25;
const int LED_BRIGHTNESS = 255;  // Brightness of the LED [0,255]

// Calibration fit data for 10k,10k,249,10pf; 20nF,100k,100k, 0,0,57.6k,  1 point
const long double cal[] = {-9.085681659276021e-27, 4.6790804314609205e-23, -1.0317125207013292e-19,
                           1.2741066484319192e-16, -9.684460759517656e-14, 4.6937937442284284e-11, -1.4553498837275352e-08,
                           2.8216624998078298e-06, -0.000323032620672037, 0.019538631135788468, -0.3774384056850066, 12.324891083404246};

const int cal_max = 1023;

// Macro time constants
#define TIMER_1MS_SMCLK 1000 // At 1 MHz SMCLK, 1 clock cycle = 1 µs → 1000000 cycles = 1 s
#define TIMER0_B3_IVECTOR 0xFFF8 // Timer0_B3 Interrupt Vector Address

// Volatile time variables
volatile unsigned long interrupt_timer   = 0;   
volatile unsigned long total_deadtime    = 0;    
volatile unsigned long waiting_t1        = 0;    // Reserved for future use

// Non-volatile time variables
unsigned long measurement_deadtime       = 0;
unsigned long measurementT1;
unsigned long measurementT2;                     // Reserved for future use
unsigned long time_stamp                 = 0;
unsigned long start_time                 = 0;    // Reference time for all the time measurements

// Muon variables
float sipm_voltage                       = 0;
long int count                           = 0;    // A tally of the number of muon counts observed
float last_sipm_voltage                  = 0;    // Reserved for future use
float temperatureC;

// Flags and mode indicators for interrupt handling and device state
volatile uint8_t waiting_for_interrupt   = 0;
uint8_t SLAVE;
uint8_t MASTER;
uint8_t keep_pulse                       = 0;

unsigned long millis(){
    return interrupt_timer;
}

// Timer A0 ISR
// Tells the compiler that the next function following the pragma is an ISR and needs an entry in the interrupt vector table.
#pragma vector = TIMER0_B3_IVECTOR
__interrupt void Timer0_B3ISR(void) {

    // Increment ms counter
    interrupt_timer++;
    
    // If interrupt state is HIGH, accumulate deadtime and reset waiting state to LOW
    if (waiting_for_interrupt == 1) {
        total_deadtime += (interrupt_timer - waiting_t1);
    }
    waiting_for_interrupt = 0;
    
    // May need to manually clear interrupt --> experiment with board (CCTL0 &= ~CCIFG should work if needed)
}

// Fit Polynomial Model to SIPM Voltage
float getSipmVoltage(float adc_value) {
    
    // Reset voltage variable
    float voltage = 0;

    // Calculate new voltage
    int cal_size = sizeof(cal) / sizeof(cal[0]);

    unsigned int i = 0;
    for (i = 0; i < cal_size; i++) {
        voltage += cal[i] * pow(adc_value, (cal_size - i - 1));
    }

    return voltage;
}

// Analogue reading function
unsigned int analogueRead(unsigned int channel) {
   
    // Decide which channel is being utilised
    switch(channel) {
        case 0:  // P5.0
            P5SEL0 |= BIT0;
            P5SEL1 &= ~BIT0;
            break;
        case 1:  // P5.1
            P5SEL0 |= BIT1;
            P5SEL1 &= ~BIT1;
            break;
        case 2:  // P5.2
            P5SEL0 |= BIT2;
            P5SEL1 &= ~BIT2;
            break;
        case 3:  // P5.3
            P5SEL0 |= BIT3;
            P5SEL1 &= ~BIT3;
            break;
        default: // Invalid case
            return 0;
    }

    // Configure the ADC
    ADCCTL0 &= ~ADCENC;                     // Disable ADC to configure
    ADCCTL0 = ADCON | ADCSHT_2 | ADCMSC;
    ADCCTL1 = ADCSHP;                       // Use sampling timer
    ADCCTL2 = ADCRES_2;                     // 12-bit resolution
    ADCMCTL0 = channel & 0x0F;              // Select input channel A0–A15

    // Begin Conversion
    ADCCTL0 |= ADCENC | ADCSC;
    while (ADCCTL1 & ADCBUSY);              // Wait for conversion
    return ADCMEM0;                         // Return result
}

// Placeholder function
void analogueWrite(unsigned int pin, unsigned int value) {

}

// Delay function in ms
void delay_ms(unsigned int ms) {

    unsigned int i = 0;

    for (i = 0; i < ms; i++) {
        __delay_cycles(1000); // For 1MHz SMCLK, 1000 cycles = 1ms
    }
}


int main(void){
    
    // Stop watchdog timer  
    WDTCTL = WDTPW | WDTHOLD; 

    // Initialise the ADC
    ADCCTL0 = ADCSHT_2 | ADCON;                 // ADC ON, Sample-and-hold time (16 ADCCLK cycles)
    ADCCTL1 = ADCSHP | ADCDIV_3;                // Sampling timer, clock divided by 8 
    ADCCTL2 &= ~(BIT5 | BIT4);                  // Clear ADCRES bits
    ADCCTL2 = ADCRES_2;                         // 12-bit resolution
    ADCMCTL0 = ADCINCH_0;                       // Default to A0

    // Timer0_B3 Setup for system timer
    TB0CCR0 = TIMER_1MS_SMCLK -1;               // 1ms interval
    TB0CCTL0 = CCIE;                            // ENable interrupt for CCR0
    TB0CTL = TBSSEL_2 | MC_1 | TBCLR;           // use SMCLK, count up to TB0CCR0, and clear timer

    __enable_interrupt();                       // Enable global interrupts

    // Main program loop
    while(1) {
        

    }

}
