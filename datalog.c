/******************************************************************************
*
* Author: Chris Lawrence
*
* Summary: Source file for data logging operations
*
******************************************************************************/

#include "datalog.h"

// Declare the queue array and front, rear variables
uint32_t queue[MAX_SIZE];
int front = -1, rear = -1;
uint8_t size = 0;

// Function to check if the queue is full
int isFull(){
    return (rear + 1) % MAX_SIZE == front;
}

// Function to check if the queue is empty
int isEmpty(){
    return front == -1;
}

// Function to enqueue (insert) an element
void enqueue(uint32_t data){
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
    queueSize++;
    queue[rear] = data;
}

// Function to dequeue (remove) an element
uint32_t dequeue()
{
    // If the queue is empty, do not attempt to dequeue
    if (isEmpty()) {
        return 0;
    }
    // Get the data from the front of the queue
    uint32_t data = queue[front];

    // If the front and rear pointers are at the same
    // position, reset them
    if (front == rear) {
        front = rear = -1;
    }
    // Else move the front pointer to the next position
    else {
        front = (front + 1) % MAX_SIZE;
    }
    queueSize--;
    // Return the dequeued data
    return data;
}
