#include "Myqueue.h"



#define ERR_OK 0
#define ERR_NO 1

//入队操作
Status Enqueue(LinkQueue* Q,QElemType e)
{
QueuePtr newNode = (QueuePtr)malloc(sizeof(QNode)); //新建队列节点

newNode->data = (QElemType*)malloc(sizeof(INT16_XYZ));    //为节点数据域分配内存

//数据域循环赋值
newNode->data->x = e.x;
newNode->data->y = e.y;
newNode->data->z = e.z;	

newNode->next = NULL;

Q->rear->next = newNode; //队列的尾指针指向的当前节点的下一个位置，指向s
Q->rear = newNode;    //队列的尾指针向后移动至新节点
Q->len++;
return ERR_OK;

}

//出队操作,使用后需要释放e的内存
Status DeQueue(LinkQueue* Q,QElemType* e)
{
QueuePtr p;

p = Q->front->next; //要出队的是头结点的下一个节点
//*e = p->data;    //将要出队的节点数据赋值给e
unsigned char len = sizeof(QElemType);

e=(QElemType*)malloc(len);

(*e).x = p->data->x;
(*e).y = p->data->y;
(*e).z = p->data->z;	

Q->front->next = p->next;
Q->len--;
if(Q->rear == p) //尾指针指向p说明队列空了
{
Q->rear = Q->front;
Q->len = 0;
}

free(p->data);
free(p);
return ERR_OK;
}


//void main(void)
//{
//QueuePtr s = (QueuePtr)malloc(sizeof(QNode));

//s->data = 0;
//s->next = NULL;

//LinkQueue linkqueue;

//linkqueue.front = s;
//linkqueue.rear = s;

//Enqueue(&linkqueue,"hello1");
//Enqueue(&linkqueue,"hello2");
//Enqueue(&linkqueue,"hello3");
//Enqueue(&linkqueue,"hello4");
//Enqueue(&linkqueue,"hello5");

//QElemType value;
//QElemType value1;
//QElemType value2;
//QElemType value3;

//DeQueue(&linkqueue,&value);
//printf("value=%s",value);
////
//DeQueue(&linkqueue,&value1);
//printf("value1=%s",value);

//DeQueue(&linkqueue,&value2);
//printf("value2=%s",value);

//DeQueue(&linkqueue,&value3);
//printf("value3=%s",value);


//getchar();

//}


