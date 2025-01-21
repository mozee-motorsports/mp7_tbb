/*
 * fdcan_queue.h
 *
 *  Created on: Jan 19, 2025
 *      Author: kohlmanz
 */

#ifndef INC_FDCAN_QUEUE_H_
#define INC_FDCAN_QUEUE_H_

#include "fdcan.h"
#include <stdbool.h>

#define MAX_QUEUE_SIZE 10	// Max number of messages to queue
typedef struct
{
	CANMessage messages[MAX_QUEUE_SIZE];
	int tail;
	int head;
}Queue;

// FIFO queue prototypes for CAN Message Parsing
void queueInit(Queue* queue);
bool isEmpty(Queue* queue);
bool isFull(Queue* queue);
bool enqueue(Queue* queue, CANMessage* msg);	// Add an element to queue
bool dequeue(Queue* queue);						// Remove an element from queue
CANMessage peek(Queue* queue);					// Return head element




#endif /* INC_FDCAN_QUEUE_H_ */
