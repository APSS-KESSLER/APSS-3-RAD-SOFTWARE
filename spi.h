/******************************************************************************
*
* Author: Chris Lawrence
*
* Summary: Header file for SPI communication
*
******************************************************************************/

#ifndef SPI_H
#define SPI_H

#include <stdint.h>
#include "config.h"

// --- Global Variables ---
extern volatile uint8_t spi_rx_buffer[MAX_BUFFER_SIZE];
extern volatile uint8_t spi_tx_buffer[MAX_BUFFER_SIZE];
extern volatile uint8_t rx_index;
extern volatile uint8_t expected_length;
extern volatile uint8_t tx_length;
extern volatile uint8_t tx_index;
extern volatile uint8_t packet_ready;

// Buffers for data sent between master/slave
extern uint8_t MasterType2 [TYPE_2_LENGTH];
extern uint8_t MasterType1 [TYPE_1_LENGTH]; 
extern uint8_t MasterType0 [TYPE_0_LENGTH]; 
extern uint8_t SlaveType2 [TYPE_2_LENGTH]; 
extern uint8_t SlaveType1 [TYPE_1_LENGTH];
extern uint8_t SlaveType0 [TYPE_0_LENGTH]; 

// SPI operation modes for the slave device state machine.
typedef enum {
    WAIT_FOR_START,
    READ_QUERY,
    READ_SUBQUERY,
    READ_LENGTH,
    READ_DATA
} SPI_PacketState;

void initSPI();
void process_spi_packet(uint8_t *packet, uint8_t length);

#endif
