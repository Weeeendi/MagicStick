#ifndef MYQUEUE_H
#define MYQUEUE_H

#include "stdio.h"
#include "stdlib.h"
#include <string.h>
#include "MPU9250.h"

typedef int Status;
typedef unsigned int QueueMems;
typedef INT16_XYZ QElemType;

typedef struct QNode //队列节点
{
QElemType data;
struct QNode *next;
}QNode,*QueuePtr;

typedef struct Link_queue//对列链表
{
QueuePtr front,rear; //头尾指针
QueueMems len;
}LinkQueue;

//入队操作
Status Enqueue(LinkQueue* Q,QElemType e);


//出队操作,使用后需要释放e的内存
Status DeQueue(LinkQueue* Q,QElemType* e);


#endif