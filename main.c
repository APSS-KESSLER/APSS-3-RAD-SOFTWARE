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

// Copies an array of bytes from source to destination
void CopyArray(uint8_t *source, uint8_t *dest, uint8_t count) 
{
    uint8_t copyIndex = 0;
    for (copyIndex = 0; copyIndex < count; copyIndex++)
    {
        dest[copyIndex] = source[copyIndex];
    }

}

// Sends a single byte via UCB0 SPI (waits until TX buffer is ready)
void SendUCB0Data(uint8_t val) {

    while (!(UCB0IFG & UCTXIFG));              // USCI_B0 TX buffer ready?
    UCB0TXBUF = val;

}


// Processes incoming SPI commands and prepares data for transmission or reception
// ** Note: commands from OBC are to be implemented once known **
void SPI_Slave_ProcessCMD(uint8_t cmd) {

    // Reset indexes and counters for a new transaction
    ReceiveIndex = 0;
    TransmitIndex = 0;
    RXByteCtr = 0;
    TXByteCtr = 0;

    switch (cmd) {

        // ==== MASTER REQUESTING DATA FROM SLAVE (SLAVE TRANSMITS) ====
        case (CMD_TYPE_0_SLAVE):                        
            SlaveMode = TX_DATA_MODE;
            TXByteCtr = TYPE_0_LENGTH;
            //Fill out the TransmitBuffer
            CopyArray(SlaveType0, TransmitBuffer, TYPE_0_LENGTH);
            //Send First Byte
            SendUCB0Data(TransmitBuffer[TransmitIndex++]);
            TXByteCtr--;
            break;

        case (CMD_TYPE_1_SLAVE):                      //Send slave device time (This device's time)
            SlaveMode = TX_DATA_MODE;
            TXByteCtr = TYPE_1_LENGTH;
            //Fill out the TransmitBuffer
            CopyArray(SlaveType1, TransmitBuffer, TYPE_1_LENGTH);
            //Send First Byte
            SendUCB0Data(TransmitBuffer[TransmitIndex++]);
            TXByteCtr--;
            break;

        case (CMD_TYPE_2_SLAVE):                  //Send slave device location (This device's location)
            SlaveMode = TX_DATA_MODE;
            TXByteCtr = TYPE_2_LENGTH;
            //Fill out the TransmitBuffer
            CopyArray(SlaveType2, TransmitBuffer, TYPE_2_LENGTH);
            //Send First Byte
            SendUCB0Data(TransmitBuffer[TransmitIndex++]);
            TXByteCtr--;
            break;

        // ==== MASTER SENDING DATA TO SLAVE (SLAVE RECEIVES) ====    
        case (CMD_TYPE_0_MASTER):
            SlaveMode = RX_DATA_MODE;
            RXByteCtr = TYPE_0_LENGTH;
            break;

        case (CMD_TYPE_1_MASTER):
            SlaveMode = RX_DATA_MODE;
            RXByteCtr = TYPE_1_LENGTH;
            break;

        case (CMD_TYPE_2_MASTER):
            SlaveMode = RX_DATA_MODE;
            RXByteCtr = TYPE_2_LENGTH;
            break;
        
        // Unknown command case
        default:

            __no_operation();
            break;

    }

}

// Finalises an SPI transaction based on the command type.
// - For master-to-slave commands: copies received data into the appropriate MasterType buffer.
// - For slave-to-master commands: no post-processing is needed.
void SPI_Slave_TransactionDone(uint8_t cmd) {
    switch (cmd) {

        case (CMD_TYPE_0_SLAVE):
            break;

        case (CMD_TYPE_1_SLAVE):
            break;

        case (CMD_TYPE_2_SLAVE):
            break;

        case (CMD_TYPE_0_MASTER):
            CopyArray(ReceiveBuffer, MasterType0, TYPE_0_LENGTH);
            break;

        case (CMD_TYPE_1_MASTER):
            CopyArray(ReceiveBuffer, MasterType1, TYPE_1_LENGTH);
            break;

        case (CMD_TYPE_2_MASTER):
            CopyArray(ReceiveBuffer, MasterType2, TYPE_2_LENGTH);
            break;

        default:
            __no_operation();
            break;

    }
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

// Millisecond ISR
#pragma vector = TIMER3_B0_VECTOR
__interrupt void TimerB3ISR(void) {

    // Increment ms counter
    interrupt_timer++;

}

int main(void){
    
    // Initialise system
    systemInit();

    // Main program loop (to be implemented)        
    while (1) {

    
    }
    
}
