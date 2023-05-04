/*
 * cQueue.c
 *
 *  Created on: Mar 15, 2023
 *      Author: IoT02
 */

#include "cQueue.h"

QueueType* createCQueue(){
  QueueType* cQ;
  cQ = (QueueType*)malloc(sizeof(QueueType));
  cQ->front = 0;
  cQ->rear = 0;
  return cQ;
}
int isCQueueEmpty(QueueType* cQ)
{
  if(cQ -> front == cQ -> rear){
    return 1;
  }else{
    return 0;
  }
}
int isCQueueFull(QueueType* cQ)
{
  if(((cQ->rear + 1)%CQUEUE_SIZE) == cQ -> front){
    return 1;
  }else{
    return 0;
  }
}

void enCQueue(QueueType* cQ, IMUDATA item )
{
  if(!isCQueueFull(cQ)){
    cQ->rear = (cQ->rear + 1) % CQUEUE_SIZE;
    cQ->cQueue[cQ->rear] = item;
  }
}

IMUDATA* deCQueue(QueueType* cQ)
{
  if(!isCQueueEmpty(cQ)){
    cQ->front = (cQ->front + 1) % CQUEUE_SIZE;
    return &(cQ->cQueue[cQ->front]);
  }else{
    return NULL;
  }

}
IMUDATA* peekCQ(QueueType* cQ){
  if(!isCQueueEmpty(cQ)){
    return &(cQ->cQueue[(cQ->front+1)%CQUEUE_SIZE]);
  }else{
    return NULL;
  }
}
int sizeOfCQueue(QueueType* cQ)
{
  if(cQ->front <= cQ->rear){
    return cQ->rear - cQ->front;
  }else{
    return cQ->rear + CQUEUE_SIZE -cQ->front ;
  }
}
