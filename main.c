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

// Calibration fit data for 10k,10k,249,10pf; 20nF,100k,100k, 0,0,57.6k,  1 point
const long double cal[] = {-9.085681659276021e-27, 4.6790804314609205e-23, -1.0317125207013292e-19,
                           1.2741066484319192e-16, -9.684460759517656e-14, 4.6937937442284284e-11, -1.4553498837275352e-08,
                           2.8216624998078298e-06, -0.000323032620672037, 0.019538631135788468, -0.3774384056850066, 12.324891083404246};

const int cal_max = 1023;

// Macro time constants
#define TIMER_1MS_SMCLK 1000 // At 1 MHz SMCLK, 1 clock cycle = 1 µs → 1000 cycles = 1 ms
#define TIMER0_B3_IVECTOR 0xFFF8 // Timer0_B3 Interrupt Vector Address

// Volatile time variables
volatile unsigned long interrupt_timer   = 0;   
volatile unsigned long total_deadtime    = 0;    
volatile unsigned long waiting_t1        = 0;    // Timestamp for deadtime calculation

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
__interrupt void TimerB0ISR(void) {

    // Increment ms counter
    interrupt_timer++;
    
    // If interrupt state is HIGH, accumulate deadtime and reset waiting state to LOW
    if (waiting_for_interrupt == 1) {
        total_deadtime += (interrupt_timer - waiting_t1);
    }

    waiting_for_interrupt = 0;
    TB0CCTL0 &= ~CCIFG; // Manually clear interrupt flag
}

// Fit Polynomial Model to SIPM Voltage
float getSipmVoltage(float adc_value) {
    
    // Reset voltage variable
    float voltage = 0;

    // Calculate new voltage
    int cal_size = sizeof(cal) / sizeof(cal[0]);

    // Horner's Method for power efficient calculations    
    unsigned int i = 0;
    for (i = 0; i < cal_size; i++) {
        voltage = voltage * adc_value + cal[i];
    }

    return voltage;
}

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

// Function to activate LED to visually confirm detector activity
void ledEnable(uint8_t pin) {
    
    TB1CCR0 = TIMER_1MS_SMCLK-1;        // PWM Period of 1ms

    // Switch to different LED
    switch(pin) {
        
        // P2.0 = TB1.1
        case 0:
            P2DIR |= BIT0;              // Set pin 2.0 as output
            P2SEL0 |= BIT0;             // Assign to primary module function

            TB1CCTL1 = OUTMOD_7;        // CCR1 reset/set
            TB1CCR1 = 750;              // CCR1 PWM duty cycle
            break;

        // P2.1 = TB1.2
        case 1:
            P2DIR |= BIT1;              // Set pin 2.1 as output
            P2SEL0 |= BIT1;             // Assign to primary module function

            TB1CCTL2 = OUTMOD_7;        // CCR2 reset/set
            TB1CCR2 = 250;              // CCR2 PWM duty cycle
            break;

        // Invalid input handling
        default:
            break;
    }
}

// Function to deactivate LED after a detector event has occured
void ledDisable(uint8_t pin) {

    switch(pin) {

        // Disable Timer B1 Capture/Compare 1 output mode
        case 0:
            TB1CCTL1 &= ~OUTMOD_7; // Clear the OUTMOD bits for CCR1

            P2SEL0 &= ~BIT0;       // Revert P2.0 from primary module function to GPIO
            P2DIR &= ~BIT0;        // Set P2.0 as high impedance input

            // Consider adding a pull-up or pull-down resistor to prevent floating input
            // Final decision depends on the PCB design and intended pin behavior
            break;

        // Disable Timer B1 Capture/Compare 2 output mode
        case 1:
            TB1CCTL2 &= ~OUTMOD_7; // Clear the OUTMOD bits for CCR2

            P2SEL0 &= ~BIT1;       // Revert P2.1 from primary module function to GPIO
            P2DIR &= ~BIT1;        // Set P2.1 as high impedance input 

            // Consider adding a pull-up or pull-down resistor to prevent floating input
            // Final decision depends on the PCB design and intended pin behavior
            break;

        // Invalid input handling
        default:
            break;
    }
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
    
    // Disable the GPIO power-on default high-impedance mode to activate
    // previously configured port settings
    PM5CTL0 &= ~LOCKLPM5;

    // Configure the ADC
    ADCCTL0 &= ~ADCENC;                                         // Disable ADC to configure
    ADCCTL0 = ADCON | ADCSHT_2 | ADCMSC;
    ADCCTL1 = ADCSHP;                                           // Use sampling timer
    ADCCTL2 &= ~(BIT5 | BIT4);                                  // Clear resolution bits
    ADCCTL2 = ADCRES_2;                                         // 12-bit resolution
    ADCMCTL0 = ADCINCH_0;                                       // Default to channel 0 (A0)

    // Timer B0 Setup for system timer
    TB0CCR0 = TIMER_1MS_SMCLK -1;               // 1ms interval
    TB0CCTL0 = CCIE;                            // Enable interrupt for CCR0
    TB0CTL = TBSSEL_2 | MC_1 | TBCLR;           // use SMCLK, count up to TB0CCR0, and clear timer

    // Timer B1 Setup for LED PWM
    TB1CCR0 = TIMER_1MS_SMCLK -1;               // 1ms interval
    TB1CTL = TBSSEL_2 | MC_1 | TBCLR;           // SMCLK, Up mode, clear timer


    __enable_interrupt();                       // Enable global interrupts

    // Main program loop (to be implemented)
    while(1) {
        

    }

}
