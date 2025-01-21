/*
 * fdcan_queue.c
 *
 *  Created on: Jan 19, 2025
 *      Author: kohlmanz
 */


#include "fdcan_queue.h"
#include <string.h>

void queueInit(Queue* queue)
{
	queue->head = -1;
	queue->tail = 0;
}

// If the head is equal to the --tail, then is empty since starts at -1
bool isEmpty(Queue* queue) { return (queue->head == queue->tail -1);}

// If the tail is at the queue size, then is full
bool isFull(Queue* queue) {return (queue->tail == MAX_QUEUE_SIZE);}

// Add an element to queue. Assuming rx_header is deep copied prior
bool enqueue(Queue* queue, CANMessage* msg)
{
	// If the queue is full, then return false
	if(isFull(queue))
	{
		return false;
	}

	// Add item to queue to end of queue
	memcpy(&(queue->messages[queue->tail].data), &(msg->data), sizeof(msg->data));

	// Update tail position
	queue->tail++;

	// Success
	return true;
}
// Remove an element from queue
bool dequeue(Queue* queue)
{
	if(isEmpty(queue))
	{
		return false;
	}

	// Update front
	queue->head++;
	return true;

}

// Get the element at from of queue. Assumes isEmpty() called before hand.
CANMessage peek(Queue* queue)
{
	// Return at head of queue -> +1 because we start at -1
	return queue->messages[queue->head + 1];
}

