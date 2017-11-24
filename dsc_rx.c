#include <xdc/runtime/System.h>
#include <ti/sysbios/hal/Timer.h>
#include <ti/sysbios/knl/clock.h>
#include <ti/sysbios/hal/Hwi.h>
#include "dsc_rx.h"
#include "audio_queue.h"
#include "syslink.h"
#include "main.h"


#include <xdc/std.h>
#include <xdc/runtime/Error.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include "syslink_init.h"



/************************************************** 变量定义 **************************************************/
//static UChar   frameHeadPatt[DSC_FRAME_HEAD_DOTPAT] = {0,1,0,1,0,1,0,1,0,1,0,1};
UChar dscInformationLen = 0;//dscInformationLen        22  //该值也可设为21，需根据ECC计算修改----------------------------------------

//static UShort  frameHeadPhase[DSC_FRAME_HEAD_SLOT] = {0x02F9, 0x03D9, 0x02F9, 0x01DA, 0x02F9, 0x02DA, \
		                                               0x02F9, 0x00DB, 0x02F9, 0x035A, 0x02F9, 0x015B};

//static UShort  frameHeadPhase[DSC_FRAME_HEAD_SLOT] = {0x02F9, 0x03D9, 0x02F9, 0x01DA, 0x02F9, 0x02DA, 0x02F9};
//static UShort  frameHeadPhase[DSC_FRAME_HEAD_SLOT] = {0x02AA,0x02AA,0x02AA};
//static UShort  frameHeadPhase[DSC_FRAME_HEAD_SLOT] = {0x02F9, 0x03D9, 0x02F9};
//static UShort  frameHeadPhase[DSC_FRAME_HEAD_SLOT] = {0x027D, 0x026F, 0x027D};


//static UShort  frameHeadPhase[DSC_FRAME_HEAD_SLOT] = {0x027D, 0x026F, 0x027D, 0x16E};

//匹配帧头buf
static QUEUE_DATA_TYPE   carelessHeadMat[CARELESS_HEAD_LEN] =
{
	1,0,1,1,1,1,1,0,0,1,
	1,1,1,1,0,1,1,0,0,1,
	1,0,1,1,1,1,1,0,0,1,
	0,1,1,1,0,1,1,0,1,0,
};

static QUEUE_DATA_TYPE   dscFrameHeadMat[MATCH_FRAME_HEAD_LEN] = {
//		 1,1,1,1,1,1,1,
//		 0,0,0,0,0,0,0,
//		 1,1,1,1,1,1,1,
//		 0,0,0,0,0,0,0,
//		 1,1,1,1,1,1,1,
//		 0,0,0,0,0,0,0,
//		 1,1,1,1,1,1,1,
//		 0,0,0,0,0,0,0,
//		 1,1,1,1,1,1,1,
//		 0,0,0,0,0,0,0,
//		 1,1,1,1,1,1,1,
//		 0,0,0,0,0,0,0,
		 1,1,1,1,1,1,1,
		 0,0,0,0,0,0,0,
		 1,1,1,1,1,1,1,
		 1,1,1,1,1,1,1,
		 1,1,1,1,1,1,1,
		 1,1,1,1,1,1,1,
		 1,1,1,1,1,1,1,
		 0,0,0,0,0,0,0,
		 0,0,0,0,0,0,0,
		 1,1,1,1,1,1,1,
		 1,1,1,1,1,1,1,
		 1,1,1,1,1,1,1,
		 1,1,1,1,1,1,1,
		 1,1,1,1,1,1,1,
		 0,0,0,0,0,0,0,
		 1,1,1,1,1,1,1,
		 1,1,1,1,1,1,1,
		 0,0,0,0,0,0,0,
		 0,0,0,0,0,0,0,
		 1,1,1,1,1,1,1,
		 1,1,1,1,1,1,1,
		 0,0,0,0,0,0,0,
		 1,1,1,1,1,1,1,
		 1,1,1,1,1,1,1,
		 1,1,1,1,1,1,1,
		 1,1,1,1,1,1,1,
		 1,1,1,1,1,1,1,
		 0,0,0,0,0,0,0,
		 0,0,0,0,0,0,0,
		 1,1,1,1,1,1,1,
		 0,0,0,0,0,0,0,
		 1,1,1,1,1,1,1,
		 1,1,1,1,1,1,1,
		 1,1,1,1,1,1,1,
		 0,0,0,0,0,0,0,
		 1,1,1,1,1,1,1,
		 1,1,1,1,1,1,1,
		 0,0,0,0,0,0,0,
		 1,1,1,1,1,1,1,
		 0,0,0,0,0,0,0};
static UShort frameBuf[DSC_FRAME_LENGTH] = {0}; //完整的DSC帧buf

//QUEUE_DATA_TYPE   dscRxBuf[DSC_RX_BUF_LEN] = {0};  //循环队列缓冲buf


static const UChar checkErrorTable[128] = {
	7, 3, 3, 5, 3, 5, 5, 1, 3, 5, 5, 1, 5, 1, 1, 6, 3, 5, 5, 1,
	5, 1, 1, 6, 5, 1, 1, 6, 1, 6, 6, 2, 3, 5, 5, 1, 5, 1, 1, 6,
	5, 1, 1, 6, 1, 6, 6, 2, 5, 1, 1, 6, 1, 6, 6, 2, 1, 6, 6, 2,
	6, 2, 2, 4, 3, 5, 5, 1, 5, 1, 1, 6, 5, 1, 1, 6, 1, 6, 6, 2,
	5, 1, 1, 6, 1, 6, 6, 2, 1, 6, 6, 2, 6, 2, 2, 4, 5, 1, 1, 6,
	1, 6, 6, 2, 1, 6, 6, 2, 6, 2, 2, 4, 1, 6, 6, 2, 6, 2, 2, 4,
	6, 2, 2, 4, 2, 4, 4, 0 };


static UShort *DSCInformation=NULL;			//后续重新分配大小
static UShort *DSCInformationCopy=NULL;
static UChar  DSCSendBuf[DSC_SEND_DATA_LEN] = {0};

/*
 * 功能：把DSC协议帧部分点阵和相位调整序列扩展为匹配帧
 * frameHeadSrc1：点阵部分帧头
 * frameHeadSrc2：相位调整部分帧头
 * frameExtHead：扩展帧头（用于匹配）
 */
//static void   generateFrameHead(UChar *frameHeadSrc1, UShort *frameHeadSrc2, QUEUE_DATA_TYPE *frameExtHead)
//static UShort   generateFrameHead(UChar *frameHeadSrc1, UShort *frameHeadSrc2, QUEUE_DATA_TYPE *frameExtHead)
//{
//	UShort i = 0;
//	UShort j = 0;
//	UShort k = 0;
//	UShort index = 0;
//	UChar temp1 = 0;
//	UShort temp2 = 0;
//
//	//扩展点阵部分帧头
//	for(i = 0;i < DSC_FRAME_HEAD_DOTPAT;++i){
//		temp1 = frameHeadSrc1[i];
//		for(j = 0; j < DSC_SAMPLE_SCALE; ++j){
//			frameExtHead[index++] = temp1;
//		}
//	}
//
//	//扩展相位调整序列部分枕头
//	for(i = 0; i < DSC_FRAME_HEAD_SLOT; ++i){
//		temp2 = frameHeadSrc2[i];
//		for(j = 0; j < SLOT_BIT_NUM; j++){
//			temp1 = (UChar)((temp2 >> j) & 0x0001);
//			for(k = 0; k < DSC_SAMPLE_SCALE; ++k){
//				frameExtHead[index++] = temp1;
//			}
//		}
//	}
//
//	//增加帧头设置的灵活性
////	//扩展点阵部分帧头
////	for(i = 0;i < DSC_FRAME_HEAD_DOTPAT;++i){
////		temp1 = frameHeadSrc1[i];
////
////		if(temp1 == 0){
////			for(j = 0; j < DSC_SAMPLE_SCALE; ++j){
////				frameExtHead[index++] = temp1;
////			}
////		}
////		else{
////			for(j = 0; j < DSC_SAMPLE_SCALE; ++j){
////				frameExtHead[index++] = temp1;
////			}
////		}
////	}
////
////	//扩展相位调整序列部分枕头
////	for(i = 0; i < DSC_FRAME_HEAD_SLOT; ++i){
////		temp2 = frameHeadSrc2[i];
////		for(j = 0; j < SLOT_BIT_NUM; j++){
////			temp1 = (UChar)((temp2 >> j) & 0x0001);
////
////			if(temp1 == 0){
////				for(k = 0; k < DSC_SAMPLE_SCALE; ++k){
////					frameExtHead[index++] = temp1;
////				}
////			}
////			else{
////				for(k = 0; k < DSC_SAMPLE_SCALE; ++k){
////					frameExtHead[index++] = temp1;
////				}
////			}
////
////		}
////	}
//
//	return index;
//}
/*
 * 功能:7:1抽样匹配
 */
static UShort carelessMatch(Queue *q, QUEUE_DATA_TYPE * matchFrameHead, UShort startPosition,UShort len)
{
	UShort i = 0;
	UShort matchResult_g = 0;
	QUEUE_DATA_TYPE temp = 0;
	matchResult_g = 0;
	for(i = 0; i < len; ++i){
        temp = q->buf[(i*7 + startPosition)   & DSC_RX_BUF_LEN_1];  //直接访问队列中的元素
		if(temp == matchFrameHead[i])
			++matchResult_g;
//		else {  //不相同减1
//			if(matchResult_g == 0)
//				matchResult_g = 0;
//			else
//				--matchResult_g;
		//}
	}

	return matchResult_g;
}


/*
 * 功能:一次匹配
 */
static UShort oneMatch(Queue *q, QUEUE_DATA_TYPE * matchFrameHead, UShort startPosition,UShort len)
{
	UShort i = 0;
	UShort matchResult = 0;
	QUEUE_DATA_TYPE temp = 0;

	for(i = 0; i < MATCH_FRAME_HEAD_LEN; ++i){
        temp = q->buf[(i + startPosition)   & DSC_RX_BUF_LEN_1];  //直接访问队列中的元素
		if(temp == matchFrameHead[i])
			++matchResult;
		else {  //不相同减1
			if(matchResult == 0)
				matchResult = 0;
			else
				--matchResult;

		}
	}

	return matchResult;
}

//===================================================================================================================
///*
// * 功能:匹配帧头
// */
//static UShort matchHead(Queue *q, QUEUE_DATA_TYPE * matchFrameHead)
//{
//	UShort len = queueLength(q);
//	UShort matchLen = 0;
//	UShort matchRate = 0;
//	UShort matchPosition = 0;
//
//	if(len >= MATCH_FRAME_HEAD_LEN*2){ //队列中获取的比特长度大于2倍的 MATCH_FRAME_HEAD_LEN
//		matchLen = len - MATCH_FRAME_HEAD_LEN; //-----------------------------------------------------------
//
//		while(matchLen > 0){
//			--matchLen;
//			matchPosition = getFrontPosition(q);
//			matchRate = oneMatch(q, matchFrameHead, matchPosition);
//
//			if(matchRate > MATCH_THREAD){
//				return OK;  //匹配成功
//			}
//
//			deQueue(q); //将匹配过的元素从队列中弹出
//		}
//	}
//
//	return FAIL;   //匹配失败
//}
/*
 * 功能:匹配帧头
 */
static UShort matchHead(Queue *q, QUEUE_DATA_TYPE * matchFrameHead)
{
	UShort len = queueLength(q);
	float  matchLen = 0;
	short carelessMatchRate=0,carefulMatchRate;
	short maxMatchRate = 0;
	short i=0,j=0,k=0;
	char temp_j = 0;
	UInt32 time =0;
	if(len >= DSC_RX_BUF_LEN_HALF)																//wait until enough length;
	{ //队列中获取的比特长度大于2倍的 MATCH_FRAME_HEAD_LEN
		time=Clock_getTicks();
		matchLen = (len - MATCH_FRAME_HEAD_LEN-7)/3;
		for(i=0;i<matchLen;i++)																//detect 0~(MATCH_FRAME_HEAD_LEN/7+1)
		{//q->buf
			carelessMatchRate = carelessMatch(q,carelessHeadMat,q->front+3,CARELESS_HEAD_LEN);	//start position is q->front+3(middle sample : 3,10,17...)
			if(carelessMatchRate >= CARELESS_HAD_MATCH_LEN)									//40 has matched 35
			{
				for(j=-3;j<=3;j++)															//find max value  2-1-6
				{
					carefulMatchRate = oneMatch(q,matchFrameHead,q->front+3+j,MATCH_FRAME_HEAD_LEN);
					if(carefulMatchRate>=MATCH_FRAME_HEAD_LEN-50)
					{
						if(carefulMatchRate>maxMatchRate)
						{
							maxMatchRate = carefulMatchRate;
							temp_j = j;
						}
						else
							break;
					}
				}
				if(maxMatchRate>0)
				{
					for(k=0;k<3+temp_j;k++)										//if matched success,delete queue size
						deQueue(q);
					return OK;														//match success
				}
				else
					for(j=0;j<3;j++)										//careless success,but careful unsuccess
						deQueue(q);
			}
			else
				for(j=0;j<3;j++)										//if matched unsuccessfully,delete queue size
					deQueue(q);
		}

		time=Clock_getTicks()-time;
		return FAIL;												//match unsuccessful
	}
	else
	{
		Task_sleep(20);												//no enough length,need delay 32ms,wait 40*0.8333;
		return FAIL;
	}
}

//===================================================================================================================
/*
 * 功能：填充用作帧头的相位调整序列部分
 */
//static UShort fillFrame_Head(UShort *src, UShort *des, UShort startPostion)
//{
//	UShort i = 0;
//	for(i = 0; i < DSC_FRAME_HEAD_SLOT; ++i){
//		des[startPostion + i] = src[i];
//	}
//	return i;
//}


/*
 *功能：填充来自队列中的帧信息
 */
static void fillFrame_Queue(Queue *q, UShort *des, UShort startPosition)
{
	UShort len = DSC_RX_BUF_LEN_HALF;// - MATCH_FRAME_HEAD_SLOT_LEN; //从Queue获取的比特位长度
	//UShort len = DSC_RX_BUF_LEN_HALF;
	UShort count1 = 0;
	UShort count2 = 0;
	UShort i = 0;
	UShort temp1 = 0;
	QUEUE_DATA_TYPE temp2 = 0;
	UShort bitCount = 0;
	UShort result = 0;
	UShort start = 0;
	while(len > 0){
		--len;
		++count1;
		temp2 = q->buf[(q->front+start++)&DSC_RX_BUF_LEN_1];//lao added
		//deQueue(q);
		bitCount += temp2;
	//	printf("%d",temp2);
		if(count1 == DSC_SAMPLE_SCALE){  //7bit
			count1 = 0;
		//	printf(" ");
			temp1 = (bitCount >= BIT_THREAD) ? 1 : 0;
			bitCount = 0; //清除bitCount 值

			result = result | (temp1 << count2);

			++count2;
			if(count2 == SLOT_BIT_NUM){ //10bit
			//	printf("\n");
				des[i + startPosition] = result;
				result = 0;
				i++;
				count2 = 0;
			}
		}
	}
}

/*
 *功能：获取DSC_FRAME_LENGTH字节的标准DSC帧
 */
int testcount;
static UShort getNormalFrame(Queue *q, UShort *frameBuf)
{
	UShort startPosition = 0;
	UShort len = queueLength(q);
//	UShort i = 0;


	//队列中有一个完整的帧数据
	//if(1){
	if(len > DSC_RX_BUF_LEN_HALF + MATCH_FRAME_HEAD_LEN){
		//在该处可关闭定时器
		// key=Hwi_disable();
		// Timer_stop(timer);

//		i = 0;

		//将匹配帧头从队列中弹出
//		for(i = 0; i < MATCH_FRAME_HEAD_LEN; ++i)
//			deQueue(q);


		//填充用作帧头的相位调整序列部分
//		startPosition = fillFrame_Head(frameHeadPhase, frameBuf, startPosition);

		//填充来自队列中的帧信息
		fillFrame_Queue(q, frameBuf, startPosition);
//		if(++count >= 5)
//		{
//			printf("count = %d\n",count);
//			count = 0;
//		}

		return 1; //填充成功
	}

	return 0; //没填充完
}

/*
 * 功能：将接收到的完整帧两条重复的信息分开
 * src：需要被分开的原始信息
 * des1：原始帧分出的第一条信息
 * des2：原始帧分出的第二条信息
 */
static void seperateInf(UShort *src, UShort *des1, UShort *des2)
{
	//UShort start1 = 12;   //第一条信息抽取起始地址
	UShort start1 = 14;     //该值也可设成14，要根据ECC计算方法来确认
	//UShort start2 = 17;   //第二条信息抽取起始地址
	UShort start2 = 19;     //该值也可设成19，要根据ECC计算方法来确认
	UShort index = start1;
	UShort i = 0;
	UShort count = 0;

	//提取第一条信息(A--I,22个信息码)
	for(count = 0; count < dscInformationLen; ++count){//hihh
		des1[i++] = src[index];
		index += 2;
	}

	//提取第二条信息(A--I,21个信息码)
	i = 0;
	index = start2;
	for(count = 0; count < dscInformationLen; ++count){
		des2[i++] = src[index];
		index += 2;
	}
}

UChar  getInformationLen(const UShort *frameBuf)
{
	char i;
	char start  = 14;
	for(i=start;i<DSC_FRAME_LENGTH;i++)
	{
		if((frameBuf[i]&0x7f)==117 || (frameBuf[i]&0x7f)==122 || (frameBuf[i]&0x7f)==127)
		{
			//*len = i+1;
			//*ecc = infBuf[i+1]&0x7f;
			if((frameBuf[12]&0x7f)==112 || (frameBuf[16]&0x7f)==112 || (frameBuf[26]&0x7f)==112)	//判断是否为遇险，若遇险需要加扩展序列
				return (i-start)/2+2+9;
			else
				return (i-start)/2+2;
		}
	}
	return 0;
}

/*
 * 功能：去除队列内数据
 * 参数：size:去除数据长度
 */
void abandonQueue(Queue *q,UShort size)
{
	UShort i;
	for(i=0;i<size;i++)
		deQueue(q);
}

/*
 * 功能：行检测
 */
void checkRow(const UShort *infBuf, errorInf * row)
{
	UShort i = 0;
	UShort index = 0;
	UChar  zeroCountBit = 0;

	for(i = 0; i < dscInformationLen; ++i ){
		index = infBuf[i] & 0x007F;
		zeroCountBit = (UChar)(infBuf[i] >> 7);  //获取监督码

		if(zeroCountBit != checkErrorTable[index]){
			row->location= i;
			row->count += 1;
		}
	}
}

/*
 *功能：模二相加
 */
static UShort mode2Adder(UShort num1, UShort num2, UShort addBitsNum)
{
	UShort addResult = 0;
	UShort i = 0;
	UShort mask = 0;
	UShort temp1 = 0;
	UShort temp2 = 0;

	for(i = 0; i < addBitsNum; ++i){
		mask = 1 << i;
		temp1 = num1 & mask;
		temp2 = num2 & mask;

		addResult += ((temp1 + temp2) & mask);
		mask = 0;
	}

	return addResult;
}

static void checkEcc(const UShort *informationBuf,char len,errorInf * colum)
{
	UShort i = 0;
	UShort temp1 = 0;
	UShort temp2 = 0;
	UShort mode2AddResult = 0;

	for(i = 0; i < len; ++i){
		mode2AddResult = mode2Adder(mode2AddResult, informationBuf[i], 7); //该处的7表示仅仅验证7位信息部分
	}

	//统计模二相加结果与ECC不同位的个数
	for(i = 0; i < 7; ++i){
		temp1 = ((informationBuf[len] >> i) & 0x0001);
		temp2 = ((mode2AddResult >> i) & 0x0001);

		if(temp1 != temp2){
			colum->location = i;
			colum->count += 1;
		}
	}
}

/*
 * 功能：列检测
 */
static void checkColum(const UShort *infBuf, errorInf * colum)
{
	//UShort i = 0;
//	UShort len = 0;//dscInformationLen - 1;
	//如果遇险，先判断前ecc，后判断扩展序列ecc
	if((frameBuf[12]&0x7f)==112 || (frameBuf[16]&0x7f)==112 || (frameBuf[26]&0x7f)==112)	//判断是否为遇险，若遇险需要加扩展序列
	{
		//除去最后的ecc和扩展序列
		checkEcc(infBuf,dscInformationLen-10,colum);
		//printf("!!!!!!!!!!!!!%p %p %d %x\n",infBuf,&infBuf[dscInformationLen-7],dscInformationLen-7,infBuf[dscInformationLen-7]);
		checkEcc(infBuf,dscInformationLen-1,colum);
	}
	else
		checkEcc(infBuf,dscInformationLen-1,colum);
}

/*
 *功能：检错
 */
static void checkOneInf(const UShort *infBuf, errorInf *rowError, errorInf *columError)
{
	//行检测
	checkRow(infBuf, rowError);

	//列检测
	checkColum(infBuf, columError);
}

/*
 * 功能：纠错
 */
static UShort correctError(UShort *infBuf, errorInf *rowError, errorInf *columError)
{
	UShort bitTemp = 0;
	UShort index = infBuf[dscInformationLen - 1] & 0x007F; //取ECC得信息码


	//检测infBuf中的ECC信息码和监督码是否匹配
	//checkEcc(infBuf,dscInformationLen - 1,columError);
	//printf("test : ecc error %x %x\n",infBuf[dscInformationLen - 1] >> 7,checkErrorTable[index]);
	if((infBuf[dscInformationLen - 1] >> 7) != checkErrorTable[index])
		return 0;

	//获取infBuf行错误、列错误信息
	checkOneInf(infBuf, rowError, columError);

	if(rowError->count > 1 || columError->count > 1)  //行或列有两个错误，无法修复
		return 0;
	else if(rowError->count == 0 || columError->count == 0) //行或列都没错误，不用修复
		return 1;
	else if(rowError->count == 1 && columError->count == 1){ //行和列都错误1 位
		bitTemp = (1 << columError->location);
		mode2Adder(infBuf[rowError->location], bitTemp, 7);  //将错误位取反

		return 1;  //修复成功
	} else
		return 0;  //无法修复
}


/*
 * 功能：检错、纠错
 * 返回值：0--不能修复；1--infBuf1 修复成功；2--infBuf2 修复成功
 */
#if 0
static UShort errorFixup(UShort *infBuf1, UShort *infBuf2)
{
	UShort status = 0;
	errorInf rowError;
	errorInf columError;

	rowError.count = 0;
	rowError.location = 0;
	columError.count = 0;
	columError.location = 0;


	//修复infBuf1
	status = correctError(infBuf1, &rowError, &columError);

	//infBuf1 没修复成功，则修复infBuf2
	if(status == 0){
		status = correctError(infBuf2, &rowError, &columError);

		if(status == 1)
			status = 2; //infBuf2 修复成功
	}

	return status;
}
#else
static UShort errorFixup(UShort *infBuf1, UShort *infBuf2)
{
	char zeroCountBit = 0;
	char i = 0;
	char index = 0;
	for(i = 0; i < dscInformationLen; ++i )
	{
		index = infBuf1[i] & 0x007F;
		zeroCountBit = (UChar)(infBuf1[i] >> 7);  //获取监督码
		if(zeroCountBit != checkErrorTable[index])
		{
			index = infBuf2[i] & 0x007F;
			zeroCountBit = (UChar)(infBuf2[i] >> 7);  //获取监督码
			if(zeroCountBit == checkErrorTable[index])
			{
				infBuf1[i] = infBuf2[i];
			}
			else
				return 0;
		}
	}

	return 1;
}
#endif

/*
 * 功能：打包
 * 发送数据格式：##&&TABBBCCCDDEEEFFFGHHHHHHHHHHHIIIIIIIIIIJ$$$
 */
void package(const UShort *in, UChar *out)
{
	UShort  i = 0;
	UShort  j = 0;
//	UShort  k = 0;

	//格式帧头
	//out[i++] = '#';
	//out[i++] = '#';
	//out[i++] = '&';
	//out[i++] = '&';
	//out[i++] = 255;
	//out[i++] = 244;

	//帧类型
	//out[i++] = '0';

	//帧数据
	for(j = 0; j < dscInformationLen; ++j){
		out[i++] = (UChar)(in[j] & 0x007F);
	}

	//printf("package len %hu\n",dscInformationLen);
//	j=39-dscInformationLen;
//	if(j>0) for(k = 0; k < k; ++k){
//		out[i++] = 0;
//	}

	//帧尾格式
	//out[i++] = '$';
	//out[i++] = '$';
	//out[i++] = '$';
	//out[41] = 254;
}

//=======================================================================================================
void printBuf(UShort *buf, int len){
	int i = 0;
	
	
	printf("The complete dsc frame is:\n");
	for(i = 0; i < len; ++i){
		printf("%x，",buf[i]);
	}
	printf("\n");	
}

//=======================================================================================================
void printInf(unsigned char *buf){
	int i = 0;
	
	
//	printf("The send information is:\n");
	for(i = 0; i < 46; ++i){
//		printf("%x,",buf[i]);
	}
//	printf("\n");
}

extern Timer_Handle timer;
extern Void hwiFxn(UArg arg);

//extern dsc_message_t *msg_dsc_se;

dsc_message_t *msg_dsc_se;




//UShort buf[DSC_FRAME_LENGTH] = {0};
/*
 *功能：DSC接收主任务
 */

int sendcount1,sendcount2,fixcount;

void DSC_RX(Queue *q)
{
	UShort matchFrameHeadFlag = FAIL;  //帧头匹配成功标志
	UShort getFrameComFlag = 0;        //帧信息提取完成标志
	UShort status = 0;                 //检错、纠错标志
	int status1 = 0;
//	printf("-->DSCTask:\n");
	//产生匹配帧头
	DSCInformation = (UShort*)malloc(sizeof(UShort));
	DSCInformationCopy = (UShort*)malloc(sizeof(UShort));
	while(1){
		/****　匹配帧头  ****/
		if(matchFrameHeadFlag == FAIL)
		{
			matchFrameHeadFlag = matchHead(q, dscFrameHeadMat);
		}
		else
		{
			/****  提取DSC完整帧信息  ****/
			getFrameComFlag = getNormalFrame(q, frameBuf);
//
			if(getFrameComFlag == 1){
				testcount++;
				//printf("get complete frame(62 short) ok!\n");
				getFrameComFlag = 0;
				matchFrameHeadFlag = FAIL;
				//获取有效信息长度，并根据该长度修改内存
				dscInformationLen = getInformationLen(frameBuf);
				abandonQueue(q,(dscInformationLen+9) * 2 * DSC_SAMPLE_SCALE * SLOT_BIT_NUM);//delete the information data
//				if(dscInformationLen==0)
//					printf("information had received error\n");
				DSCInformation = (UShort*)realloc(DSCInformation,dscInformationLen*sizeof(UShort));
				DSCInformationCopy = (UShort*)realloc(DSCInformationCopy,dscInformationLen*sizeof(UShort));
				/****  分开信息  ****/
				seperateInf(frameBuf, DSCInformation, DSCInformationCopy);
				//printBuf(frameBuf, 62);
				
				
				/****  检错、纠错  ****/
				status = errorFixup(DSCInformation, DSCInformationCopy);

			    /****  打包、发送  ****/
				//打包DSCInformation
				if(status == 1){
					status = 0;
					package(DSCInformation, DSCSendBuf);
					
//					printf("Start send data!\n");
					//发送数据格式：##&&TABBBCCCDDEEEFFFGHHHHHHHHHHHIIIIIIIIIIJ$$$

					msg_dsc_se = (dsc_message_t *)message_alloc(msgbuf[1], sizeof(dsc_message_t));
					if(!msg_dsc_se){
						log_error("msg_dsc_se malloc fail");
					}

					memcpy(msg_dsc_se->data.dsc_msg, DSCSendBuf, 46);
					msg_dsc_se->data.mid=67;
					msg_dsc_se->data.dsc_len = dscInformationLen;
					status1=messageq_send(&msgq[1],(messageq_msg_t)msg_dsc_se,0,0,0);
					if(status1>=0)
						sendcount1++;
					else
						log_error("send1 error");

					printInf(DSCSendBuf);
				}
				else if(status == 2){ //打包DSCInformationCopy
					status = 0;
					package(DSCInformationCopy, DSCSendBuf);
					//printf("Start send data!\n");
					//发送数据格式：##&&TABBBCCCDDEEEFFFGHHHHHHHHHHHIIIIIIIIIIJ$$$
					msg_dsc_se = (dsc_message_t *)message_alloc(msgbuf[1], sizeof(dsc_message_t));
					if(!msg_dsc_se){
						log_error("msg_dsc_se malloc fail");
					}

					memcpy(msg_dsc_se->data.dsc_msg, DSCSendBuf, 46);
					msg_dsc_se->data.mid=67;
					msg_dsc_se->data.dsc_len = dscInformationLen;
					status1=messageq_send(&msgq[1],(messageq_msg_t)msg_dsc_se,0,0,0);
					if(status1>=0)
						sendcount2++;
					else
						log_error("send2 error");
					printInf(DSCSendBuf);
				}
				else {  //信息出错并且无法修复
					fixcount++;
					status = 0;
				}
			}
			else
			{
				Task_sleep(10);//sleep
			}
		}		
	}
	//free(DSCInformation);
	//free(DSCInformationCopy);
}
