#include <stdio.h>
#include <string.h>
#include "audio_queue.h"


/* 全局变量 */
//#define AUDIO_QUEUE_RX_LENGTH   131072
AudioQueue_DATA_TYPE audioQueueRxBuf[AUDIO_QUEUE_RX_LENGTH] = {0};





/*
 * 功能：初始化队列
 */
void queueInit(Queue * q, unsigned short size)
{
	q->front = 0;
	q->rear = 0;
	q->capacity = 0;
	q->size = size;
	//q->buf = buf;
	memset(q->buf,0,DSC_RX_BUF_LEN);
}

void audioQueueInit(audioQueue * q, Int size, AudioQueue_DATA_TYPE *buf)
{
	q->front = 0;
	q->rear = 0;
	q->capacity = 0;
	q->size = size;
	q->buf = buf;
}

/*
 * 功能：获取队列长度
 */
UShort queueLength(Queue *q)
{
	return q->capacity;
}

Int audioQueueLength(audioQueue *q)
{
//	printf("length return %d %d\n",q->capacity,q->size);
	return q->capacity;
}

/*
 * 功能：将data放入队尾
 */
state enQueue(Queue * q, QUEUE_DATA_TYPE data)
{
	//如果队列满
	if(q->front==(q->rear+1)&0x3fff)
    {
//		printf("Warming:The queue is full!\n");
		return ERROR;
    }

    q->buf[q->rear] = data;
    q->rear = (q->rear+1) & DSC_RX_BUF_LEN_1;
	q->capacity ++;
    return OK;
}


state enAudioQueue(audioQueue * q, AudioQueue_DATA_TYPE data)
{

	//如果队列满
    if(q->capacity >= q->size)
    {
//		printf("Warming:The AudioQueue is full %d!\n",q->size);
		return ERROR;
    }

    q->buf[q->rear] = data;
    q->rear = (q->rear+1) % q->size;
	q->capacity ++;
    return OK;
}

/*
 * 功能：将队列头的元素弹出队列
 */
QUEUE_DATA_TYPE deQueue(Queue * q)
{
	QUEUE_DATA_TYPE data = 0;

	//如果队列空
    if(q->capacity == 0)
    {
		printf("Warming:the queue is empty!\n");
        return ERROR;
    }

    data = q->buf[q->front];
    q->front = (q->front+1) & DSC_RX_BUF_LEN_1;
	q->capacity --;

    return data;
}

AudioQueue_DATA_TYPE deAudioQueue(audioQueue * q)
{
	AudioQueue_DATA_TYPE data = 0;
	//printf("deAudioQueue cap=%d!!!!!!!!!!!\n",q->capacity);
	//如果队列空
    if(q->capacity == 0)
    {
		printf("Warming:the AudioQueue is empty!\n");
        return ERROR;
    }

    data = q->buf[q->front];
    q->front = (q->front+1) % q->size;
	q->capacity --;

    return data;	
}


/*
 * 功能：获取队头所在位置
 */
UShort getFrontPosition(Queue *q)
{
	return q->front;
}

Int getAudioQFrontPosition(audioQueue *q)
{
	return q->front;
}


/*
 * 功能：访问队列中position位置的元素
 */
QUEUE_DATA_TYPE queuePoll(Queue *q,UShort position)
{
	return q->buf[position  & DSC_RX_BUF_LEN_1];
}

AudioQueue_DATA_TYPE audioQueuePoll(audioQueue *q,Int position)
{
	return q->buf[position  % q->size];
}


void queueFlush(Queue *q)
{
	q->front = 0;
	q->rear = 0;
	q->capacity = 0;
}

void audioQueueFlush(audioQueue *q)
{
	q->front = 0;
	q->rear = 0;
	q->capacity = 0;
}

