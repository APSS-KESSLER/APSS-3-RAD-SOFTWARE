/******************************************************************************
*
* Author: Chris Lawrence
*
* Summary: Source file for SPI communication
*
******************************************************************************/

#include "spi.h"
#include <msp430fr2355.h>

void setupSPI_OBC(){
       
    P1SEL0 |= BIT7; // Select P1.7 -> MOSI (Data) GETTING DATA
    P1SEL1 &= ~BIT7;

    
    P1SEL0 |= BIT6; // Select P1.6 -> MISO (Data) SENDING DATA
    P1SEL1 &= ~BIT6;

    
    P1SEL0 |= BIT5; // Select P1.5 -> CLK (Clock)
    P1SEL1 &= ~BIT5;

    // STE Pin at 1.4 
    P1SEL0 |= BIT4;
    P1SEL1 &= ~BIT4;

    UCA0CTLW0 |= UCSWRST;                         // Hold USCI in reset state
    UCA0CTLW0 |= UCMSB | UCSYNC |UCCKPH; // MSB First, Synchronous, Clock phase high or something
    UCA0CTLW0 &= ~UCSWRST;                        // Initialise USCI module
    UCA0IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt

}

// I dont like how it makes a copy of memory.. maybe use a pointer instead ? then pointer + 1 and deref it?
//Note length is the length of the data!!!
//Ensure your packet is big enough for the data + 6 bytes for start, query, length and crc!
void build_packet(uint8_t packet[],uint16_t length, uint8_t query_code, uint8_t data[]){
    packet[0] = 0xD8; // Start byte
    packet[1] = query_code; // query code

    // variable (16 bit) Length
    uint8_t high_length = (length >> 8) & 0xFF;     // Extract high byte
    uint8_t low_length  = length & 0xFF;       // Extract low byte
    packet[2] = low_length;
    packet[3] = high_length; // Length 2 bytes
    uint16_t i;
    for(i=0;i<length;i++ ){
        packet[i+4] = data[i];
    }
    
}

uint16_t crc16(uint8_t* pData, uint16_t length) {
    length = length + 4; // for start query and length
    uint8_t i;
    uint16_t wCrc = 0xffff;
    while (length--) {
        wCrc ^= *(unsigned char *)pData++ << 8;
        for (i=0; i < 8; i++)
            wCrc = wCrc & 0x8000 ? (wCrc << 1) ^ 0x1021 : wCrc << 1;
    }
    return wCrc & 0xffff;
}

// ------------------
// RX Circular Buffer
// ------------------
// Push: adds a byte. Drops data on overflow.
static inline void rxq_push(uint8_t b)
{
    uint8_t next = (uint8_t)((rx_head + 1) % RX_SIZE);

    if (next == rx_tail) {
        // Overflow: drop the byte (or handle differently)
        return;
    }

    rxq[rx_head] = b;
    rx_head = next;
}

// Pop: returns next byte, or 0 if empty.
inline bool rxq_pop(uint8_t *b){
    if (rx_tail == rx_head){
        return false;
    }
    *b = rxq[rx_tail];
    rx_tail = (uint8_t)((rx_tail + 1) % RX_SIZE);
    return true;
}

/// ------------------
// TX Circular Buffer
// ------------------

static inline bool txq_pop(uint8_t *b){
    if (tx_tail == tx_head) {
        return false;
    }
    *b = txq[tx_tail];
    tx_tail = (uint8_t)((tx_tail + 1) % TXQ_SIZE);
    return true;
}
static inline bool txq_push(uint8_t b){
    uint8_t next = (uint8_t)((tx_head + 1) % TXQ_SIZE);
    if (next == tx_tail){
        return false;
    }
    txq[tx_head] = b; 
    tx_head = next; 
    return true;
}

static bool txq_empty(){ 
    return tx_tail == tx_head; 
}
// SPI RX interrupt
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
{

    //If the packet hasn't been built (still recieveing data)
    if(!packet_built){
    // Check RX flag
    if (UCA0IFG & UCRXIFG) {
        uint8_t in = UCA0RXBUF;
        
        // Push into RX queue
        rxq_push(in);

        // Tell main loop new data is available
        spiEvent = true;
    }
    } else {
    if (UCA0IFG & UCTXIFG) {
        uint8_t t; 
        if (txq_pop(&t)) {
            UCA0TXBUF = t;
        }else{
            state_reset();
        }
        
    }
}
}

void spiStateEvent(uint8_t byte){
    switch(state){
        case WAITING_FOR_START:
            // If start byte read, begin read process
            if (byte == START_BYTE){
                state = READ_QUERY;
                byteIndex = 0;

                crc_calc = 0xFFFF;
                crc_calc = crc16_update(crc_calc, byte);
            }

        break;
        
        case READ_QUERY:
            queryCode = byte;
            if(queryCode == 0x04){
                binData = true;
            }
            crc_calc = crc16_update(crc_calc, byte);
            state = READ_LENGTH_L;
                    

        break;
        
        case READ_LENGTH_L:
            crc_calc = crc16_update(crc_calc, byte);
            lenExpected = byte;
            state = READ_LENGTH_H;

        break;
        case READ_LENGTH_H:

            lenExpected |= ((uint16_t)byte) << 8;
            crc_calc = crc16_update(crc_calc, byte);
            state = (lenExpected == 0) ? READ_CRC_L : READ_DATA;

            
        break;
        
        case READ_DATA:
            bufferRxData[lenSeen++] = byte;
            binDataSize = byte;
            crc_calc = crc16_update(crc_calc, byte);
            if (lenSeen >= lenExpected){
                state = READ_CRC_L;
            }


        break;
        
        case READ_CRC_L:
            crc_received = byte;
            state = READ_CRC_H;

        break;
        
        case READ_CRC_H:
            crc_received |= ((uint16_t)byte) << 8;
            if (crc_calc == crc_received) {

                state = PREPARE_RESPONSE;
            } else {
                state_reset();
            }
        
        break;

        case PREPARE_RESPONSE: {

            uint8_t respData[32];
            uint16_t respLen = 0;

            switch (queryCode) {
                case 0x01: // Acknowledge Query
                    respData[0] = 0xAA; 
                    respLen = 1; 
                break;

                case 0x02: // Echo Query
                    respLen = (lenExpected <= sizeof(respData)) ? lenExpected : sizeof(respData);
                    uint16_t i;
                    for (i = 0; i < respLen; i++) {
                        respData[i] = bufferRxData[i];
                    }
                break;
                    
                case 0x03: // Read data and send
                    if (queueSize >= 1) {
                        respLen = queueSize * 4;
                        uint16_t j;
                        for (j = 0; j < respLen; j++) {
                            uint32_t val = dequeue();

                            respData[j*4 + 0] = val & 0xFF;         // LSB
                            respData[j*4 + 1] = (val >> 8) & 0xFF;
                            respData[j*4 + 2] = (val >> 16) & 0xFF;
                            respData[j*4 + 3] = (val >> 24) & 0xFF; // MSB
                        }
                    }
                break;

                case 0x04: // Read binned data and send
                {
                    uint16_t respIndex = 0;

                        while (queueSize > 0) {

                            uint16_t count = (queueSize >= binDataSize) ? binDataSize : queueSize;

                            // First timestamp
                            uint32_t startTS = dequeue();
                            uint32_t lastTS = startTS;

                            // Last timestamp
                            uint16_t z;
                            for (z = 1; z < count; z++) {
                                lastTS = dequeue();
                            }

                            // Start timestamp
                            respData[respIndex++] = (startTS >> 0) & 0xFF;  // LSB
                            respData[respIndex++] = (startTS >> 8) & 0xFF;
                            respData[respIndex++] = (startTS >> 16) & 0xFF;
                            respData[respIndex++] = (startTS >> 24) & 0xFF; // MSB

                            // End timestamp
                            respData[respIndex++] = (lastTS >> 0) & 0xFF;   // LSB
                            respData[respIndex++] = (lastTS >> 8) & 0xFF;
                            respData[respIndex++] = (lastTS >> 16) & 0xFF;
                            respData[respIndex++] = (lastTS >> 24) & 0xFF;  // MSB

                            // Event count
                            respData[respIndex++] = count & 0xFF;           // LSB
                            respData[respIndex++] = (count >> 8) & 0xFF;    // MSB
                        }

                        respLen = respIndex;
                    }
                break;

                case 0x05: // Read queue size (useful for determining whether to send bin or raw data)
                    respData[0] = queueSize;
                    respLen = 1;
                break;

                default:   // Unrecognised Query
                    respData[0] = 0xFF; 
                    respLen = 1; 
                break;
            }

            // Build Outgoing Packet
            static uint8_t packet[4 + 256];         // for now size is predefined, change later?
            build_packet(packet, respLen, queryCode, respData);

            // Calculate CRC
            uint16_t crc = crc16(packet, respLen);
            uint8_t crc_low  = (uint8_t)(crc & 0xFF);
            uint8_t crc_high = (uint8_t)(crc >> 8);

            // Push Packet onto TX Buffer
            uint16_t i;
            for (i = 0; i < (respLen + 4); i++){
                txq_push(packet[i]);
            }
            txq_push(crc_low);
            txq_push(crc_high);

            // Begin Transmission
            packet_built = true;
            // UCA0IE |= UCTXIE;
            
            if (UCA0IFG & UCTXIFG){
                uint8_t t; if (txq_pop(&t)) UCA0TXBUF = t;
            }
            state = SEND_RESPONSE;
        
        break;
        }

        case SEND_RESPONSE:

            if (txq_empty()){
                state_reset();
            }

        break;

    
        default: 
            //Should not be here at all
            P6OUT |= BIT6;
        break;
        }
}

// For RX
static uint16_t crc16_update(uint16_t crc, uint8_t data){
    crc ^= (uint16_t)data << 8;

    uint8_t i;
    for (i = 0; i < 8; i++){
        if (crc & 0x8000) {
            crc = (crc << 1) ^ 0x1021;
        }
        else{
            crc <<= 1;
        }
    }
    return crc;
}

static void state_reset(){
    state = WAITING_FOR_START;
    lenExpected = 0;
    lenSeen = 0;
    crc_calc = 0xFFFF;
    crc_received = 0;
    byteIndex = 0;
    packet_built = false;
    binData = false;
     if (UCA0IFG & UCTXIFG){
                UCA0TXBUF = 0xFF;
            }

}
