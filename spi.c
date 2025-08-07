/******************************************************************************
*
* Author: Chris Lawrence
*
* Summary: Source file for SPI communication
*
******************************************************************************/

#include <stdint.h>
#include "spi.h"
#include <msp430fr2355.h>

// Variables for data transfer
volatile SPI_PacketState spi_state = WAIT_FOR_START;
volatile uint8_t spi_rx_buffer[MAX_BUFFER_SIZE];
volatile uint8_t spi_tx_buffer[MAX_BUFFER_SIZE];
volatile uint8_t rx_index = 0;
volatile uint8_t expected_length = 0;
volatile uint8_t tx_length = 0;
volatile uint8_t tx_index = 0;
volatile uint8_t packet_ready = 0;


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
