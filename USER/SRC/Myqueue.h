#ifndef MYQUEUE_H
#define MYQUEUE_H

#include "stdio.h"
#include "stdlib.h"
#include <string.h>
#include "MPU9250.h"

typedef int Status;
typedef unsigned int QueueMems;
typedef INT16_XYZ QElemType;

typedef struct QNode //���нڵ�
{
QElemType data;
struct QNode *next;
}QNode,*QueuePtr;

typedef struct Link_queue//��������
{
QueuePtr front,rear; //ͷβָ��
QueueMems len;
}LinkQueue;

//��Ӳ���
Status Enqueue(LinkQueue* Q,QElemType e);


//���Ӳ���,ʹ�ú���Ҫ�ͷ�e���ڴ�
Status DeQueue(LinkQueue* Q,QElemType* e);


#endif