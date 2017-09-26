#ifndef _AUDIO_QUEUE_H
#define _AUDIO_QUEUE_H
#include <xdc/std.h>

#ifndef QUEUE_DATA_TYPE
#define QUEUE_DATA_TYPE     UChar
#endif
#define DSC_RX_BUF_LEN             0x4000//0x1000//0x3FFF
#define DSC_RX_BUF_LEN_1			0x3fff
#define AUDIO_QUEUE_RX_LENGTH   8192

#ifndef AudioQueue_DATA_TYPE
#define AudioQueue_DATA_TYPE     unsigned short
#endif

#define OK 1  
#define FAIL 0
#define ERROR 0
//#define TRUE 1  
//#define FALSE 0
typedef int state;

typedef struct Queue {
    UShort      front,rear;
	UShort      capacity;
	UShort      size;
	QUEUE_DATA_TYPE    buf[DSC_RX_BUF_LEN];
} Queue;

typedef struct audioQueue {
    Int      front,rear;
    Int      capacity;
    Int      size;
	AudioQueue_DATA_TYPE    *buf;
} audioQueue;


void queueInit(Queue * q, UShort size);
state enQueue(Queue * q, QUEUE_DATA_TYPE data);
QUEUE_DATA_TYPE deQueue(Queue * q);
UShort queueLength(Queue *q);
UShort getFrontPosition(Queue *q);
QUEUE_DATA_TYPE queuePoll(Queue *q,UShort position);
void queueFlush(Queue *q);
//void removeRearPosition(Queue *q);

void audioQueueInit(audioQueue * q, Int size, AudioQueue_DATA_TYPE *buf);
state enAudioQueue(audioQueue * q, AudioQueue_DATA_TYPE data);
AudioQueue_DATA_TYPE deAudioQueue(audioQueue * q);
Int audioQueueLength(audioQueue *q);
Int getAudioQFrontPosition(audioQueue *q);
AudioQueue_DATA_TYPE audioQueuePoll(audioQueue *q,Int position);
void audioQueueFlush(audioQueue *q);
//void removeRearPosition(audioQueue *q);

#endif

