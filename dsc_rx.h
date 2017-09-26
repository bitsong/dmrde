#ifndef _DSC_RX_H
#define _DSC_RX_H
#include <xdc/std.h>
#include "audio_queue.h"


/*********************************************** �ṹ�嶨�� ***************************************************/

typedef struct errorInf{
	UShort count;  //ͳ�ƴ������
	UChar  location;  //��־����λ��
}errorInf;



/*************************************************** �궨�� ***************************************************/
//DSC����֡�����Ϣ
#define DSC_SAMPLE_SCALE        7    //����ڲ����ʵĲ�����
#define BIT_THREAD              4
#define DSC_FRAME_LENGTH        100//62   //����֡��Ϣ��ġ��ۡ�����
#define SLOT_BIT_NUM            10   //һ��"��"��ı�����
#define DSC_FRAME_LENGTH_SCALE  2    //DSC_FRAME_LENGTH ����չ����

//DSC֡ͷ��Ϣ
#define DSC_FRAME_HEAD_SLOT     4   //DSC֡ͷ֡��Ϣ���ֵ�"��"����(�ⲿ�ֱ�ʾDSC����֡ͷ��ʼ��RX��DX��)
                                      //����б�Ҫ�ò���֡ͷ����Ҳ����"λ"������
#define DSC_FRAME_HEAD_DOTPAT   12   //DSC֡ͷ�ĵ��󲿷�"λ"����(�ò���Ϊ����֡ǰ��ĵ���)----------------------
//#define DSC_RX_BUF_LEN             DSC_FRAME_LENGTH*DSC_SAMPLE_SCALE*SLOT_BIT_NUM*DSC_FRAME_LENGTH_SCALE

#define DSC_RX_BUF_LEN_HALF        DSC_FRAME_LENGTH*DSC_SAMPLE_SCALE*SLOT_BIT_NUM
#define MATCH_FRAME_HEAD_SLOT_LEN    DSC_FRAME_HEAD_SLOT*DSC_SAMPLE_SCALE*SLOT_BIT_NUM
#define MATCH_FRAME_HEAD_DOTPAT_LEN  DSC_FRAME_HEAD_DOTPAT*DSC_SAMPLE_SCALE
#define MATCH_FRAME_HEAD_LEN       MATCH_FRAME_HEAD_SLOT_LEN//(MATCH_FRAME_HEAD_SLOT_LEN + MATCH_FRAME_HEAD_DOTPAT_LEN)
#define CARELESS_HEAD_LEN			DSC_FRAME_HEAD_SLOT*SLOT_BIT_NUM
#define CARELESS_HAD_MATCH_LEN		DSC_FRAME_HEAD_SLOT*SLOT_BIT_NUM-5		//40 had matched 35


//------------------------------------
#define MATCH_THREAD           (MATCH_FRAME_HEAD_LEN - 21)


//#define DSC_INFORMATION_LEN        22  //��ֵҲ����Ϊ21������ECC�����޸�----------------------------------------
#define DSC_SEND_DATA_LEN          46  //���͸�ARM�����֡����

/************************************************** ����ԭ�� **************************************************/
//����ԭ��
static UShort   generateFrameHead(UChar *frameHeadSrc1, UShort *frameHeadSrc2, QUEUE_DATA_TYPE *frameExtHead);


//static void   generateFrameHead(UChar *frameHeadSrc1, UShort *frameHeadSrc2, QUEUE_DATA_TYPE *frameHead);
static UShort oneMatch(Queue *q, QUEUE_DATA_TYPE * matchFrameHead, UShort startPosition, UShort len);
static UShort matchHead(Queue *q, QUEUE_DATA_TYPE * matchFrameHead);
static UShort fillFrame_Head(UShort *src, UShort *des, UShort startPostion);
static void fillFrame_Queue(Queue *q, UShort *des, UShort startPosition);
static UShort getNormalFrame(Queue *q, UShort *frameBuf);
static UShort correctError(UShort *infBuf, errorInf *rowError, errorInf *columError);
static UShort errorFixup(UShort *infBuf1, UShort *infBuf2);
void DSC_RX(Queue *q);


#endif
