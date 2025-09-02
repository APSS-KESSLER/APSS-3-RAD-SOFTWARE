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

// Function to enqueue (insert) an element
void enqueue(int data){
    // if queue full, do not enqueue
    if (isFull()) {
        return;
    }
    // If queue empty, set front to first position
    if (front == -1) {
        front = 0;
    }
    // Add data to queue and move rear pointer
    rear = (rear + 1) % MAX_SIZE;
    queue[rear] = data;
}