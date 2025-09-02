/******************************************************************************
*
* Author: Chris Lawrence
*
* Summary: Source file for data logging operations
*
******************************************************************************/

#include "datalog.h"

// Declare the queue array and front, rear variables
int queue[MAX_SIZE];
int front = -1, rear = -1;

// Function to check if the queue is full
int isFull(){
    return (rear + 1) % MAX_SIZE == front;
}

// Function to check if the queue is empty
int isEmpty(){
    return front == -1;
}
