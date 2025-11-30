/******************************************************************************
*
* Author: Chris Lawrence
*
* Summary: Header file for data logging operations
* 
******************************************************************************/

#ifndef DATALOG_H
#define DATALOG_H

#include <stdint.h>

// --- CONSTANTS ---
#define MAX_SIZE_QUEUE 100        // Max array size of queue

// --- VARIABLES ---
extern uint8_t queueSize;

// Function Prototypes
int isFull();
int isEmpty();
void enqueue(uint32_t data);
uint32_t dequeue();

#endif /* DATALOG_H */
