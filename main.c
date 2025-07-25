// **********************************************************
// *                                                        *
// *     Author: Chris Lawrence                             *
// *     Date:   25/07/25                                   *
// *                                                        *
// *     Summary: MSP430 Code Adaptation of CosmicWatch     *
// *                                                        *
// **********************************************************

#include <msp430fr2355.h>
#include <stdint.h>
#include <stdio.h>

// Time constants
#define TIMER_1MS_SMCLK               1000              // At 1 MHz SMCLK, 1 clock cycle = 1 µs → 1000 cycles = 1 ms
#define TIMER_1US_SMCLK               1                 // At 1 MHz SMCLK, 1 clock cycle = 1 µs → 1000 cycles = 1 ms


// I2C constants
#define SLAVE_ADDR                    0x00              // TODO: Set I2C slave address (placeholder, not yet assigned)

// Command identifiers for Slave device communication
#define CMD_TYPE_0_SLAVE              0
#define CMD_TYPE_1_SLAVE              1
#define CMD_TYPE_2_SLAVE              2

// Command identifiers for Master device communication
#define CMD_TYPE_0_MASTER             3
#define CMD_TYPE_1_MASTER             4
#define CMD_TYPE_2_MASTER             5

// Data lengths for each command type in bytes
#define TYPE_0_LENGTH                 1
#define TYPE_1_LENGTH                 2
#define TYPE_2_LENGTH                 6

#define MAX_BUFFER_SIZE               20                // Will require changing in future once requirements known

// Buffers for data sent between master/slave
uint8_t MasterType2 [TYPE_2_LENGTH] = {0};
uint8_t MasterType1 [TYPE_1_LENGTH] = {0};
uint8_t MasterType0 [TYPE_0_LENGTH] = {0};
uint8_t SlaveType2 [TYPE_2_LENGTH] = {0};
uint8_t SlaveType1 [TYPE_1_LENGTH] = {0};
uint8_t SlaveType0 [TYPE_0_LENGTH] = {0};

// Signal and LED constants
const int SIGNAL_THRESHOLD = 50;                        // Min threshold to trigger on. See calibration.pdf for conversion to mV
const int RESET_THRESHOLD = 25;

// Volatile time variables
volatile unsigned long interrupt_timer   = 0;   
volatile unsigned long total_deadtime    = 0;    
volatile unsigned long waiting_t1        = 0;           // Timestamp for deadtime calculation
volatile uint32_t timer_overflows        = 0;           // Overflow counter for microseconds since program start

// Non-volatile time variables
unsigned long measurement_deadtime       = 0;
unsigned long measurementT1;
unsigned long measurementT2;                            // Reserved for future use
unsigned long time_stamp                 = 0;
unsigned long start_time                 = 0;           // Reference time for all the time measurements

// Muon variables
float sipm_voltage                       = 0;
long int count                           = 0;           // A tally of the number of muon counts observed
float last_sipm_voltage                  = 0;           // Reserved for future use
float temperatureC;

// Flags and mode indicators for interrupt handling and device state
volatile uint8_t waiting_for_interrupt   = 0;
uint8_t SLAVE;
uint8_t MASTER;
uint8_t keep_pulse                       = 0;

// Calibration fit data for 10k,10k,249,10pf; 20nF,100k,100k, 0,0,57.6k,  1 point
const long double cal[] = {-9.085681659276021e-27, 4.6790804314609205e-23, -1.0317125207013292e-19,
                           1.2741066484319192e-16, -9.684460759517656e-14, 4.6937937442284284e-11, -1.4553498837275352e-08,
                           2.8216624998078298e-06, -0.000323032620672037, 0.019538631135788468, -0.3774384056850066, 12.324891083404246};

const int cal_max = 1023;

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

// Initialise timer for LED PWM
void initLedPwmTimer() {

    TB1CCR0 = TIMER_1MS_SMCLK -1;                   // 1ms interval
    TB1CTL = TBSSEL_2 | MC_1 | TBCLR;               // SMCLK, Up mode, clear timer

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

// Function to initialise I2C communication
void initI2C() {

    UCB0CTLW0 = UCSWRST;                           // Software reset enabled
    UCB0CTLW0 |= UCMODE_3 | UCSYNC;                // I2C mode, sync mode
    UCB0I2COA0 = SLAVE_ADDR | UCOAEN;              // Own Address and enable
    UCB0CTLW0 &= ~UCSWRST;                         // clear reset register
    UCB0IE |= UCRXIE + UCSTPIE;
    
}

// Function to initialise SPI communication
void initSPI() 
{
    //Clock Polarity: The inactive state is high
    //MSB First, 8-bit, Master, 4-pin mode, Synchronous
    UCB0CTLW0 = UCSWRST;                                // Put state machine in reset
    UCB0CTLW0 |= UCCKPL | UCMSB | UCSYNC | UCMODE_2;    // 4-pin, 8-bit SPI Slave
    UCB0CTLW0 &= ~UCSWRST;                              // Initialize USCI state machine
    UCB0IE |= UCRXIE;                                   // Enable USCI0 RX interrupt

}

// Function to initialise all functionality
void systemInit(void) {

    WDTCTL = WDTPW | WDTHOLD;                     // Stop watchdog
    PM5CTL0 &= ~LOCKLPM5;                         // Enable GPIO
    initDcoFrequency();                           // Clock
    initAdc();                                    // ADC
    initMicroTimer();                             // Timer B0 for microseconds
    initMilliTimer();                             // Timer B3 for milliseconds
    initLedPwmTimer();                            // Timer B1 for LED PWM
    initI2C();                                    // I2C setup
    initSPI();
    __enable_interrupt();                         // Enable global interrupts

}

//******************************************************************************
// Timer Calls *****************************************************************
//******************************************************************************

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

// Delay function in ms
void delay_ms(unsigned int ms) {

    unsigned int i = 0;

    for (i = 0; i < ms; i++) {
        __delay_cycles(1000); // For 1MHz SMCLK, 1000 cycles = 1ms
    }
}

//******************************************************************************
// Calculations  ***************************************************************
//******************************************************************************

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

//******************************************************************************
// SPI State Machine ***********************************************************
//******************************************************************************

// SPI operation modes for the slave device state machine.
typedef enum SPI_ModeEnum{

    IDLE_MODE,
    TX_REG_ADDRESS_MODE,
    RX_REG_ADDRESS_MODE,
    TX_DATA_MODE,
    RX_DATA_MODE,
    TIMEOUT_MODE

} SPI_Mode;

//******************************************************************************
// ISR's ***********************************************************************
//******************************************************************************

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

// Placeholder function for marking the start of a timed interval
void handleEvent() {
    waiting_for_interrupt = 1;
    waiting_t1 = micros();
}


int main(void){
    
    // Initialise system
    systemInit();

    P1DIR |= BIT0 | BIT1;
    P1OUT &= ~(BIT0 | BIT1);

    uint32_t last = 0;
    uint32_t last2 = 0;

    // Main program loop (to be implemented)        
    while (1) {

    
    }
    
}
