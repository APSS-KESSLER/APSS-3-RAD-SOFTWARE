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
#include <stdbool.h>
#include "datalog/datalog.h"

// Function prototypes
void setupSPI_OBC();
void spi_send_packet(uint8_t array[],uint16_t size);
uint8_t spiTransfer_OBC(uint8_t byte);
uint16_t crc16(uint8_t* pData, uint16_t length);
void build_packet(uint8_t packet[],uint16_t length, uint8_t query_code, uint8_t data[]);
void spiStateEvent(uint8_t byte);
static uint16_t crc16_update(uint16_t crc, uint8_t data);
static void state_reset();
inline bool rxq_pop(uint8_t *b);

// RX Buffer Variables
#define RX_SIZE (64)
static volatile uint8_t rxq[RX_SIZE];
static volatile uint8_t rx_head = 0; 
static volatile uint8_t rx_tail = 0;
static volatile uint8_t next = 0;
extern volatile bool spiEvent;
bool binData;
uint8_t binDataSize; 

// TX Buffer Variables
#define TXQ_SIZE 64
static volatile uint8_t txq[TXQ_SIZE];
static volatile uint8_t tx_head = 0;
static volatile uint8_t tx_tail = 0;

// SPI State Machine Variables
#define START_BYTE (0xD8)
#define MAX_SIZE (128)

typedef enum{
    WAITING_FOR_START = 0,
    READ_QUERY,
    READ_LENGTH_L,
    READ_LENGTH_H,
    READ_DATA,
    READ_CRC_H,
    READ_CRC_L,
    PREPARE_RESPONSE,
    SEND_RESPONSE
} SPI_PACKET_STATE;

static SPI_PACKET_STATE state = WAITING_FOR_START;
extern volatile bool spiEvent;
static uint16_t crc_calc = 0xFFFF;
static uint16_t crc_received = 0;
static uint16_t lenExpected = 0;
static uint16_t lenSeen = 0;
static uint8_t byteIndex = 0;
static uint8_t bufferRxData[MAX_SIZE];
static uint8_t queryCode = 0;
static volatile bool packet_built = false;

#endif
