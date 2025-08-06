// **********************************************************
// *                                                        *
// *     Author: Chris Lawrence                             *
// *     Date:   31/07/25                                   *
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

//******************************************************************************
// Initialisation Functions ****************************************************
//******************************************************************************

// Function to perform pin setup routine for configuration
void initPins(){

    

}

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

// Function to initialise SPI communcation with OBC
void initSPI(void) {
    
    UCB0CTLW0 = UCSWRST;                           // Hold USCI in reset for configuration

    // Configure: 4-wire SPI slave, MSB first
    UCB0CTLW0 |= UCMSB | UCSYNC | UCMODE_2;        // 4-wire slave setup
    UCB0CTLW0 |= UCCKPL | UCCKPH;                  // TODO: adjust SPI mode depending on what OBC uses (clock polarity and phase)
    
    // Configure pins
    P1SEL0 |= BIT0 | BIT1 | BIT2 | BIT3;           // P1.0 = STE, P1.1=CLK, P1.2=SOMI, P1.3=SIMO
    P1SEL1 &= ~(BIT0 | BIT1 | BIT2 | BIT3);

    // Configure STE interrupt to detect transaction end
    P2DIR &= ~BIT0;                                // STE as input
    P2REN |= BIT0;                                 // Enable pull-up/down
    P2IES &= ~BIT0;                                // Rising edge detect (STE going high = transaction end)
    P2IFG &= ~BIT0;                                // Clear any pending
    
    UCB0IFG = 0;                                   // Clear interrupt flags 
    UCB0TXBUF = 0x00;                              // Dummy byte
    
    UCB0CTLW0 &= ~UCSWRST;                         // Release USCI for operation
    P2IE  |= BIT0;                                 // Enable interrupt for STE pin
    UCB0IE |= UCRXIE;                              // Enable RX interrupt

}

//******************************************************************************
// Pin Config Functions ********************************************************
//******************************************************************************

// States for pin 1.4
typedef enum {
    GPIO_14in,
    GPIO_14out,
    A4,
    UCA0STE
} pin_state14;

// States for pin 1.5
typedef enum {
    GPIO_15in,
    GPIO_15out,
    A5,
    UCA0CLK
} pin_state15;

// States for pin 1.6
typedef enum {
    GPIO_16in,
    GPIO_16out,
    A6,
    UCA0RX,
    MISO
} pin_state16;

// States for pin 1.7
typedef enum {
    GPIO_17in,
    GPIO_17out,
    A7,
    UCA0TX,
    MOSI
} pin_state17;

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

        case GPIO_15out:       
            P1DIR |= BIT5;      // Configured as output
            P1OUT |= BIT5;      // Set output mode to HIGH

        case A5:
            P1SEL0 |= BIT5;     // Select Bits for analog function
            P1SEL1 |= BIT5;

        case UCA0CLK:
            P1SEL0 |= BIT5;
            P1SEL1 &= ~BIT5;    
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

        case GPIO_16out:
            P1DIR |= BIT6;      // Configured as output
            P1OUT |= BIT6;      // Set output mode to HIGH

        case A6:
            P1SEL0 |= BIT6;     // Select Bits for analog function
            P1SEL1 |= BIT6;

        case UCA0RX:
            P1SEL0 |= BIT6;
            P1SEL1 &= ~BIT6;   

        case MISO:
            P1SEL0 |= BIT6;
            P1SEL1 &= ~BIT6;   

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
        
        case GPIO_17out:
            P1DIR |= BIT7;      // Configured as output
            P1OUT |= BIT7;      // Set output mode to HIGH

        case A7:
            P1SEL0 |= BIT7;     // Select Bits for analog function
            P1SEL1 |= BIT7;

        case UCA0TX:
            P1SEL0 |= BIT7;
            P1SEL1 &= ~BIT7;   

        case MOSI:
            P1SEL0 |= BIT7;
            P1SEL1 &= ~BIT7;  

    }

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

//******************************************************************************
// SPI State Machine ***********************************************************
//******************************************************************************

// SPI operation modes for the slave device state machine.
typedef enum {
    WAIT_FOR_START,
    READ_QUERY,
    READ_SUBQUERY,
    READ_LENGTH,
    READ_DATA
} SPI_PacketState;

// Variables for data transfer
volatile SPI_PacketState spi_state = WAIT_FOR_START;
volatile uint8_t spi_rx_buffer[MAX_BUFFER_SIZE];
volatile uint8_t spi_tx_buffer[MAX_BUFFER_SIZE];
volatile uint8_t rx_index = 0;
volatile uint8_t expected_length = 0;
volatile uint8_t tx_length = 0;
volatile uint8_t tx_index = 0;
volatile uint8_t packet_ready = 0;

// ===== Process Packet =====
void process_spi_packet(uint8_t *packet, uint8_t length) {
    uint8_t query_code = packet[1];
    uint8_t sub_code   = packet[2];
    uint8_t payload_len = packet[3];
    uint8_t *payload = &packet[4];

    // Build a simple echo response
    spi_tx_buffer[0] = 0xD8;
    spi_tx_buffer[1] = query_code | 0x80;  // Response code (echo + 0x80)
    spi_tx_buffer[2] = 1;                  // Response length
    spi_tx_buffer[3] = 0xAA;               // Dummy payload

    tx_length = 4;    // Header + payload
    tx_index = 0;
}

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

// SPI ISR
#pragma vector = USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void) {

    switch(__even_in_range(UCB0IV, USCI_SPI_UCTXIFG)) {

        case USCI_NONE: 
        break;

        // RX interrupt, new byte being recieved
        case USCI_SPI_UCRXIFG: {

            // Read byte in RX buffer
            uint8_t byte = UCB0RXBUF;

            // SPI state machine for assembling incoming packet
            switch (spi_state) {
                
                // Start of new packet: reset index, store start byte, move to next state
                case WAIT_FOR_START:
                    if (byte == 0xD8) {  
                        rx_index = 0;
                        spi_rx_buffer[rx_index++] = byte;
                        spi_state = READ_QUERY;
                    }
                    break;

                // Store query byte in buffer and advance index
                case READ_QUERY:
                    spi_rx_buffer[rx_index++] = byte;
                    spi_state = READ_SUBQUERY;
                    break;

                // Store sub-query byte in buffer and advance index
                case READ_SUBQUERY:
                    spi_rx_buffer[rx_index++] = byte;
                    spi_state = READ_LENGTH;
                    break;

                // Store length byte in buffer and advance index
                case READ_LENGTH:
                    spi_rx_buffer[rx_index++] = byte;
                    expected_length = byte;

                    // Check for a valid length
                    if (expected_length > MAX_BUFFER_SIZE - 4) {
                        spi_state = WAIT_FOR_START;
                    } else if (expected_length == 0) {
                        packet_ready = 1;
                        spi_state = WAIT_FOR_START;

                    // If length is valid proceed to reading
                    } else {
                        spi_state = READ_DATA;
                    }
                    break;

                // Store data byte in buffer and advance index
                case READ_DATA:
                    spi_rx_buffer[rx_index++] = byte;

                    // All expected bytes received, packet is now ready for processing
                    if (--expected_length == 0) {
                        packet_ready = 1;
                        spi_state = WAIT_FOR_START;
                    }
                    break;
            }

            // Transmit next response byte, or send dummy byte (0x00) if none left
            if (tx_index < tx_length) {
                UCB0TXBUF = spi_tx_buffer[tx_index++];
            } else {
                UCB0TXBUF = 0x00;
            }
            break;
        }
        default: break;
    }
}

// STE Pin ISR for resetting at end of transaction
#pragma vector=PORT1_VECTOR
__interrupt void PORT1_ISR(void) {
    if (P1IFG & BIT0) {     
        if (P1IN & BIT0) { // STE went high (end of transaction)
            spi_state = WAIT_FOR_START;
            rx_index = 0;
            expected_length = 0;
        }
        P1IFG &= ~BIT0; // Clear interrupt flag for P1.0
    }
}

// Millisecond ISR
#pragma vector = TIMER3_B0_VECTOR
__interrupt void TimerB3ISR(void) {

    // Increment ms counter
    interrupt_timer++;

}

int main(void){
    
    // Initialise system
    WDTCTL = WDTPW | WDTHOLD;                     // Stop watchdog
    configurePin1_4(GPIO_14in);
    configurePin1_5(A5);
    configurePin1_6(UCA0RX);
    configurePin1_7(UCA0TX);
    PM5CTL0 &= ~LOCKLPM5;                         // Enable GPIO
    P1IFG = 0;                                    // Clear to avoid erroneous port interrupts
    initDcoFrequency();                           // Clock
    initAdc();                                    // ADC
    initMicroTimer();                             // Timer B0 for microseconds
    initMilliTimer();                             // Timer B3 for milliseconds
    initI2C();                                    // I2C setup
    initSPI();
    __enable_interrupt();                         // Enable global interrupts

    // Main program loop (to be implemented)        
    while (1) {

        
    
    }
    
}
