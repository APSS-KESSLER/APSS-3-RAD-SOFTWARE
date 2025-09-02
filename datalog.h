/******************************************************************************
*
* Author: Chris Lawrence
*
* Summary: Header file for data logging operations
* 
******************************************************************************/

#ifndef DATALOG_H
#define DATALOG_H

// --- CONSTANTS ---
#define MAX_SIZE 100        // Max array size of queue

// Function Prototypes
int isFull();
int isEmpty();
void enqueue(unsigned long data);
unsigned long dequeue();

#endif /* DATALOG_H */
