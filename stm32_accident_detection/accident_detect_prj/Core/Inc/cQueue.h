/*
 * cQueue.h
 *
 *  Created on: Mar 15, 2023
 *      Author: IoT02
 */

#ifndef INC_CQUEUE_H_
#define INC_CQUEUE_H_

#include "icm20948.h"

#define CQUEUE_SIZE 50

//typedef IMU element;

typedef struct{
  IMUDATA cQueue[CQUEUE_SIZE];
  uint8_t front, rear;
}QueueType;

QueueType* createCQueue();
int isCQueueEmpty(QueueType* cQ);
int isCQueueFull(QueueType* cQ);
void enCQueue(QueueType* cQ,IMUDATA item );
IMUDATA* deCQueue(QueueType* cQ);
IMUDATA* peekCQ(QueueType* cQ);
int sizeOfCQueue(QueueType* cQ);

#endif /* INC_CQUEUE_H_ */
