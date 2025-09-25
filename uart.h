/******************************************************************************
*
* Author: Chris Lawrence
*
* Summary: Header file for UART functions
*
******************************************************************************/

#ifndef UART_H
#define UART_H

#include <stdint.h>
#include "pinConfig.h"
#include "config.h"

// --- Function Prototypes ---
void initUart();
void serialPrintUInt(uint32_t num);
void serialPrintChar(char c);

#endif /* UART_H */
