/*
 * audio_loop.c
 *
 *  Created on: 2016-12-20
 *      Author: ws
 */
#include <ti/sysbios/hal/Timer.h>
#include <ti/sysbios/BIOS.h>
#include <math.h>

#include "syslink_init.h"
#include "main.h"
#include "audio_queue.h"
#include "dsc_rx.h"
#include "amc7823.h"

CSL_GpioRegsOvly     gpioRegs = (CSL_GpioRegsOvly)(CSL_GPIO_0_REGS);

//unsigned short buf_temp[9600];

short intersam[RPE_DATA_SIZE/2*5];

UInt	tx_flag, rx_flag, tx_submit, rx_submit;
Semaphore_Handle sem1,sem2;
unsigned char *buf_transmit;
struct rpe *prpe = &rpe[0];
rpe_attr_t attr=RPE_ATTR_INITIALIZER;

unsigned char buf_adc[REC_BUFSIZE];
float eeprom_data[150];
short buf_de[240];
Int RXSS_THRESHOLD =0;
float RSSI_db=0;
float channel_freq=0;
float transmit_power;

Queue q;


/* private functions */
Void smain(UArg arg0, UArg arg1);
Void task_enque(UArg arg0, UArg arg1);
Void task_receive(UArg arg0, UArg arg1);
extern Void task_mcbsp(UArg arg0, UArg arg1);
extern void Rx_process(unsigned short* ad_data,	short* de_data);
extern unsigned int lmx_init[];
extern float FSK_FAST_SPI_calc(void);
extern void LMX2571_FM_CAL(unsigned short ch, double fm, unsigned char vco);
void data_process(short *buf_in, unsigned char *buf_out, unsigned int size);
void eeprom_cache();
Void data_send(uint8_t *buf_16);

extern void audioSignalRX();
void sys_configure(void);
void dsp_logic();
Void hwiFxn(UArg arg);
Void DSCRxTask(UArg a0, UArg a1);

/*
 *  ======== main =========
 */
Int main(Int argc, Char* argv[])
{
    Error_Block     eb;
    Task_Params     taskParams;

    log_init();
    tx_flag=0;
    rx_flag=1;
    Error_init(&eb);

    log_info("-->main:");
    /* create main thread (interrupts not enabled in main on BIOS) */
    Task_Params_init(&taskParams);
    taskParams.instance->name = "smain";
    taskParams.arg0 = (UArg)argc;
    taskParams.arg1 = (UArg)argv;
    taskParams.stackSize = 0x1000;
    taskParams.priority=3;
    Task_create(smain, &taskParams, &eb);
    if(Error_check(&eb)) {
        System_abort("main: failed to create application startup thread");
    }

    /* start scheduler, this never returns */
    BIOS_start();

    /* should never get here */
    log_info("<-- main:\n");
    return (0);
}


/*
 *  ======== smain ========
 */
Void smain(UArg arg0, UArg arg1)
{
    Error_Block		eb;
    Task_Params     taskParams;
    Int 			status=0;

    log_info("-->smain:");
    if((status=syslink_prepare())<0){
    	log_error("syslink prepare log_error status=%d",status);
       	return;
    }

    queueInit(&q, DSC_RX_BUF_LEN);

    sem1=Semaphore_create(0,NULL,&eb);
    sem2=Semaphore_create(0,NULL,&eb);

    sys_configure();

    status=NO_ERR;

    Spi_dev_init();
    status=amc7823_init();
    if(status<0)
    	log_error("amc7823 initial error:%d",status);
    else
    	log_info("amc7823 initial done.");

    eeprom_cache();

    dac_write(0, eeprom_data[24]+eeprom_data[25]);
    dac_write(3, 0);
    transmit_power=eeprom_data[39];

    //task create
    Error_init(&eb);
    Task_Params_init(&taskParams);
    taskParams.instance->name = "task_receive";
    taskParams.arg0 = (UArg)arg0;
    taskParams.arg1 = (UArg)arg1;
    taskParams.stackSize = 0x1000;
    taskParams.priority=4;
    Task_create(task_receive, &taskParams, &eb);
    if(Error_check(&eb)) {
        System_abort("main: failed to create application 0 thread");
    }

    Error_init(&eb);
    Task_Params_init(&taskParams);
    taskParams.instance->name = "task_mcbsp";
    taskParams.arg0 = (UArg)arg0;
    taskParams.arg1 = (UArg)arg1;
    taskParams.stackSize = 0x1000;
    Task_create(task_mcbsp, &taskParams, &eb);
    if(Error_check(&eb)) {
    	System_abort("main: failed to create application 1 thread");
    }

    Error_init(&eb);
    Task_Params_init(&taskParams);
    taskParams.instance->name = "task_enque";
    taskParams.arg0 = (UArg)arg0;
    taskParams.arg1 = (UArg)arg1;
    taskParams.stackSize = 0x4000;
    Task_create(task_enque, &taskParams, &eb);
    if(Error_check(&eb)) {
    	System_abort("main: failed to create application 2 thread");
    }

//    Error_init(&eb);
//    Task_Params_init(&taskParams);
//    taskParams.instance->name = "audioSignalRX";
//    taskParams.arg0 = (UArg)arg0;
//    taskParams.arg1 = (UArg)arg1;
//    taskParams.stackSize = 0x4000;
//    Task_create(audioSignalRX, &taskParams, &eb);
//    if(Error_check(&eb)) {
//        System_abort("main: failed to create application 0 thread");
//    }
#if 1
//
    Task_Params_init(&taskParams);
//	taskParams.priority = 1;
    taskParams.instance->name = "DSCRxTask";
    taskParams.stackSize = 0x5000;
    Task_create(DSCRxTask, &taskParams, NULL);
    if(Error_check(&eb)) {
           System_abort("main: failed to create application DSCRxTask thread");
    }

    Timer_Handle 		timer;
    Timer_Params 		timerParams;

    Error_init(&eb);
    Timer_Params_init(&timerParams);
    /*
     * The Timer interrupt will be handled by 'hwiFxn' which
     * will run as a Hwi thread.
     */
    //f=8.4khz t=119us
    timerParams.period =119;
    timer = Timer_create(1, hwiFxn, &timerParams, &eb);
    if (timer == NULL) {
    	System_abort("Timer create failed");
    }

#endif

    for(;;){
    	Task_sleep(20);
//    	Task_yield();

    	RSSI_db=-24.288+10*log10(RSSI)-eeprom_data[84];//120      -125_10
//    	vol_A=0.061*sqrt(2*RSSI);
    	dsp_logic();
    }

    //cleanup
//        sync_send(SYNC_CODE_MSGOUT);
//        sync_send(SYNC_CODE_NTFOUT);
//        sync_send(SYNC_CODE_RPEOUT);
//        status=syslink_cleanup();
//        if(status<0){
//        	log_error("syslink cleanup failed!");
//        }
}


/* Here on dsc_timer interrupt */
Void hwiFxn(UArg arg)
{
	 /* get the GIPO6_5 value		                                           */
	 if(CSL_FEXT(gpioRegs->BANK[1].IN_DATA,GPIO_IN_DATA_IN1)==1){
		 enQueue(&q, 1);
	 }
	 else{
		 enQueue(&q, 0);
	 }
}

/*
 *  ======== DSCRxTask ========
 */
Void DSCRxTask(UArg a0, UArg a1)
{
	DSC_RX(&q);
}

/*
 *  ======== task_receive ========
 */
Void task_receive(UArg arg0, UArg arg1)
{
    Error_Block         eb;
	uint8_t 		*buf = NULL;
    uint32_t		size;
    Int status=0;
    buf_transmit=(unsigned char*)malloc(RPE_DATA_SIZE/2*15);

    log_info("-->task_receive:");
    Error_init(&eb);
    while(1){
    		Semaphore_pend(sem2, BIOS_WAIT_FOREVER);

    		size = RPE_DATA_SIZE;
    		status = rpe_acquire_reader(prpe,(rpe_buf_t*)&buf,&size);
    		if(status == ERPE_B_PENDINGATTRIBUTE){
    			status = rpe_get_attribute(prpe,&attr,RPE_ATTR_FIXED);
    			if(!status && attr.type == RPE_DATAFLOW_END){

//    				Task_sleep(3);
    				memset(buf_transmit,0x80,RPE_DATA_SIZE/2*15);
    				tx_flag=0;
        			dac_write(3, 0);

        			CSL_FINS(gpioRegs->BANK[3].OUT_DATA,GPIO_OUT_DATA_OUT6,0); //TX_SW
        			CSL_FINS(gpioRegs->BANK[3].OUT_DATA,GPIO_OUT_DATA_OUT7,1);//RX_SW
        			CSL_FINS(gpioRegs->BANK[3].OUT_DATA,GPIO_OUT_DATA_OUT13,0); //R:F1

//        			rx_flag=1;
    				rx_submit=1;
//    				log_info("data flow end!");
    				continue;
    			}
    		}else if(status < 0){
    			if(status == ERPE_B_BUFEMPTY){
//    			    log_info("rpe data empty! status = %d",status);
    			}
    			else{
    				log_warn("rpe acquire data failed,status = %d",status);
    			}
    			Task_sleep(5);
    			continue;
    		}


    		data_process((short*)buf, buf_transmit, size);


    		status = rpe_release_reader_safe(prpe,buf,size);
    		if(status < 0){
    			log_warn("rpe release writer end failed!");
    			continue;
    		}
    	}
}

/*
 *功能：信号发送时低通滤波
 *参数：inBuf:输入为240个数据的指针; outBuf:输出为低通滤波后数据指针; len:输入数据的长度（默认240）
 */
short tempBuf[304] = {0};	//240+64

void sendLowFreqFilter(short *inBuf,short *outBuf,short len)
{
	register short i = 0;

//	float coffe0= 0.274359222240358, coffe1=0.241181466150155, coffe2=0.156134567192084,coffe3=0.055060450043507,
//		coffe4=-0.023275459456141,coffe5=-0.055906398885724,coffe6=-0.044476160570719, coffe7=-0.010265971386949,
//		coffe8=0.020269571908203,  coffe9=0.030227314205989,coffe10=0.019022089736789, coffe11=-0.001270205366588,
//		coffe12=-0.015998538619177, coffe13=-0.017329264216821, coffe14=-0.007494844067617, coffe15=0.004821048965657,
//		coffe16=0.011324222287550,coffe17= 0.009237547274474,coffe18= 0.001836190006871 ,coffe19= -0.004983913940164,
//		coffe20= -0.007058493738798,coffe21=-0.004220501207019,coffe22=0.000531177568480 ,coffe23= 0.003720884004466,
//		coffe24=0.003753370771847 ,coffe25=0.001465530261336 ,coffe26= -0.001054426387385,coffe27=-0.002162717961628 ,
//		coffe28= -0.001604314160974,coffe29= -0.000270161279237 ,coffe30=0.000741650125149 ,coffe31=0.000920156408832 ,coffe32=0.000484098004814 ;
	float coffe0= 0.241573335759647, coffe1=0.218992079362489, coffe2=0.158786037164879,coffe3=0.080575526092448,
		coffe4=0.008380246927960,coffe5=-0.038458988032623,coffe6=-0.051951738731848, coffe7=-0.037107487691542,
		coffe8=-0.008242081638380,  coffe9=0.018058541361647,coffe10=0.030020320206990, coffe11=0.024772571504055,
		coffe12=0.008015396412343, coffe13=-0.010053106283831, coffe14=-0.020248128104113, coffe15=-0.018639306968899 ,
		coffe16=-0.007705466021025 ,coffe17= 0.005756438342760,coffe18= 0.014563107850161 ,coffe19= 0.014782237828384,
		coffe20= 0.007319457637066,coffe21=-0.003101782015224,coffe22=-0.010771320330543 ,coffe23= -0.012016062724458,
		coffe24=-0.006866221876609 ,coffe25=0.001344551953574 ,coffe26= 0.008034112390769,coffe27=0.009865772059648 ,
		coffe28= 0.006356036017521,coffe29= -0.000149026564685 ,coffe30=-0.005963374242723 ,coffe31=-0.008108015223888 ,coffe32=-0.005800307730482 ;
	//组合数据：前64位放上一帧数据末尾64个，后240放最新数据，形成流处理方式

	for(i=0;i<240;i++)
		tempBuf[i+64] = inBuf[i];


	for(i=0;i<len;i++)
		outBuf[i] = tempBuf[i+32]*coffe0
			+(tempBuf[i+33]+tempBuf[i+31])*coffe1  + (tempBuf[i+34]+tempBuf[i+30])*coffe2  + (tempBuf[i+35]+tempBuf[i+29])*coffe3  + (tempBuf[i+36]+tempBuf[i+28])*coffe4
			+(tempBuf[i+37]+tempBuf[i+27])*coffe5  + (tempBuf[i+38]+tempBuf[i+26])*coffe6  + (tempBuf[i+39]+tempBuf[i+25])*coffe7  + (tempBuf[i+40]+tempBuf[i+24])*coffe8
			+(tempBuf[i+41]+tempBuf[i+23])*coffe9  + (tempBuf[i+42]+tempBuf[i+22])*coffe10 + (tempBuf[i+43]+tempBuf[i+21])*coffe11 + (tempBuf[i+44]+tempBuf[i+20])*coffe12
			+(tempBuf[i+45]+tempBuf[i+19])*coffe13 + (tempBuf[i+46]+tempBuf[i+18])*coffe14 + (tempBuf[i+47]+tempBuf[i+17])*coffe15 + (tempBuf[i+48]+tempBuf[i+16])*coffe16
			+(tempBuf[i+49]+tempBuf[i+15])*coffe17 + (tempBuf[i+50]+tempBuf[i+14])*coffe18 + (tempBuf[i+51]+tempBuf[i+13])*coffe19 + (tempBuf[i+52]+tempBuf[i+12])*coffe20
			+(tempBuf[i+53]+tempBuf[i+11])*coffe21 + (tempBuf[i+54]+tempBuf[i+10])*coffe22 + (tempBuf[i+55]+tempBuf[i+9])*coffe23  + (tempBuf[i+56]+tempBuf[i+8])*coffe24
			+(tempBuf[i+57]+tempBuf[i+7])*coffe25  + (tempBuf[i+58]+tempBuf[i+6])*coffe26  + (tempBuf[i+59]+tempBuf[i+5])*coffe27  + (tempBuf[i+60]+tempBuf[i+4])*coffe28
			+(tempBuf[i+61]+tempBuf[i+3])*coffe29  + (tempBuf[i+62]+tempBuf[i+2])*coffe30  + (tempBuf[i+63]+tempBuf[i+1])*coffe31  + (tempBuf[i+64]+tempBuf[i])*coffe32;


	for(i=0;i<64;i++)
		tempBuf[i] = tempBuf[240+i];
}

/*
 *功能：24k采样转换120k
 *参数：inBuf:输入为240个数据的指针; outBuf:输出为1200个数据的指针; len:输入数据的长度（默认240）
 */
void sendPreEmphasis(short *inBuf,short *outBuf,short len)
{
	register short i = 0;
	static short firstData = 0;
	outBuf[0] = inBuf[0] - (0.91*firstData);
	for(i=1;i<len;i++)
		outBuf[i] = inBuf[i] - (0.91*inBuf[i-1]);
	firstData = inBuf[len-1];
}

/*
 *功能：24k采样转换120k
 *参数：inBuf:输入为240个数据的指针; outBuf:输出为1200个数据的指针; len:输入数据的长度（默认240）
 */
void from24To120(short *inBuf,short *outBuf,short len)
{
	short i;
	static short tempBuf[247] = {0};
	float	coffe0=0.154830207920112,	coffe1=0.147009839249942,	coffe2=0.125337104718526,	coffe3=0.094608021712450,
			coffe4=0.061139668733940,	coffe5=0.030930241474728,	coffe6=0.008127364075467,	coffe7=-0.005711418466492,
			coffe8=-0.011436697663118,  coffe9=-0.011435901979223,	coffe10=-0.008517719961896, coffe11=-0.004993842265637,
			coffe12=-0.002219513516004, coffe13=-0.000610415386896, coffe14=0.000034684251061,	coffe15=0.000140032519269;

	for(i=0;i<7;i++)
		tempBuf[i] = tempBuf[240+i];
	for(i=0;i<240;i++)
		tempBuf[i+7] = inBuf[i];

		for(i=0;i<len;i++)
		{
			outBuf[5*i]   = coffe0*tempBuf[i+3] + (coffe5*tempBuf[i+2]) + (coffe5*tempBuf[i+4]) + (coffe10*tempBuf[i+1]) + (coffe10*tempBuf[i+5]) + (coffe15*tempBuf[i]) + (coffe15*tempBuf[i+6]);
			outBuf[5*i+1] = coffe1*tempBuf[i+3] + (coffe6*tempBuf[i+2]) + (coffe4*tempBuf[i+4]) + (coffe11*tempBuf[i+1]) + (coffe9*tempBuf[i+5]) + (coffe14*tempBuf[i+6]);
			outBuf[5*i+2] = coffe2*tempBuf[i+3] + (coffe7*tempBuf[i+2]) + (coffe3*tempBuf[i+4]) + (coffe12*tempBuf[i+1]) + (coffe8*tempBuf[i+5]) + (coffe13*tempBuf[i+6]);
			outBuf[5*i+3] = coffe3*tempBuf[i+3] + (coffe8*tempBuf[i+2]) + (coffe2*tempBuf[i+4]) + (coffe13*tempBuf[i+1]) + (coffe7*tempBuf[i+5]) + (coffe12*tempBuf[i+6]);
			outBuf[5*i+4] = coffe4*tempBuf[i+3] + (coffe9*tempBuf[i+2]) + (coffe1*tempBuf[i+4]) + (coffe14*tempBuf[i+1]) + (coffe6*tempBuf[i+5]) + (coffe11*tempBuf[i+6]);
		}
}
/*
 *功能：数据发送去直流
 *参数：inBuf:输入为240个数据的指针，并带回240个去直流后数据; len:输入数据的长度（默认240）
 */
void delDc(short *inbuf,short len)
{
	short i;
	int sun=0;
	short average;
	for(i=0;i<len;i++)
		sun = sun + inbuf[i];
	average = sun/len;
	for(i=0;i<len;i++){
		inbuf[i] -= average;
	}

}

void scopeLimit(short *inBuf,short len)
{
	short i = 0;
	for(i=0;i<len;i++)
	{
		if(inBuf[i]>=18000)
			inBuf[i] = 18000;
		else if(inBuf[i]<=-18000)
			inBuf[i] = -18000;
	}
}



/*
 *功能：数据发送端数据处理。（64阶低通滤波、预加重、24k->120k处理）
 *参数：inBuf:输入为240个数据的指针; outBuf:输出为1200个数据的指针; len:输入数据的长度（默认240）
 */
void dataFilterAndTrans(short *inBuf,short *outBuf,short len)
{
	delDc(inBuf,len);
	sendPreEmphasis(inBuf,outBuf,len);		//input:outBuf,output:inBuf(inBuf as a temp buffer)
	scopeLimit(outBuf,len);
	sendLowFreqFilter(outBuf,inBuf,len);
	from24To120(inBuf,outBuf,len);
}



#if 1
void data_process(short *buf_in, unsigned char *buf_out, unsigned int size)
{
	uint32_t tempdata=0;
	uint32_t tempCount;
	reg_16 reg16;
	float k;
	static float factor;
//	static short index;
//	short intersam[RPE_DATA_SIZE/2*5];

	//5  24to120
	k=1.525333/5/3;
	factor=FSK_FAST_SPI_calc();

//    for (tempCount = 0; tempCount < RPE_DATA_SIZE/2; tempCount++){
//    	if(buf_in[tempCount]>16400)
//    		buf_in[tempCount]=5000;
//    	else if(buf_in[tempCount]<-4000)
//    		buf_in[tempCount]=-4000;
//
//    	buf_temp[index++]=buf_in[tempCount];
//    	if(index>=1199)
//    		index=0;

//    }

    dataFilterAndTrans(buf_in, intersam,RPE_DATA_SIZE/2);


//    for (tempCount = 0; tempCount < RPE_DATA_SIZE/2*5; tempCount++){
//    	if(intersam[tempCount]>102)
//    		intersam[tempCount]=102;
//    	else if(intersam[tempCount]<-102)
//    		intersam[tempCount]=-102;
//
//    	buf_temp[index++]=intersam[tempCount];
//    	if(index>=9599)
//    		index=0;
//
//    }

    for(tempCount=0,tempdata=0;tempCount<RPE_DATA_SIZE/2*5;tempCount++)
    {
    	//Interpolation
    	reg16.all=(unsigned short)(factor*(intersam[tempCount]/k));

    	((uint8_t *)buf_out)[tempdata++] 	 	 = reg16.dataBit.data0;
    	((uint8_t *)buf_out)[tempdata++] 	 	 = reg16.dataBit.data1;
    	((uint8_t *)buf_out)[tempdata++] 	     = (uint8_t)33;
    }
}

#else
void data_process(short *buf_in, unsigned char *buf_out, unsigned int size)
{
	uint32_t tempdata=0;
	uint32_t tempCount;
	reg_16 reg16;
	float k;
	static float factor;
	short xn,xn1;

#if 0

	k=20000.000000/3000.000000;
	factor=FSK_FAST_SPI_calc();

    for (tempCount = 0; tempCount < RPE_DATA_SIZE/2; tempCount++){
    	if(ring_ar[tempCount]>20000)
    		ring_ar[tempCount]=20000;
    	else if(ring_ar[tempCount]<-20000)
    		ring_ar[tempCount]=-20000;

    	//Interpolation
    	reg16.all=(unsigned short)(factor*(ring_ar[tempCount]/k));

    	((uint8_t *)buf_out)[tempdata] 	 	 = reg16.dataBit.data0;
    	((uint8_t *)buf_out)[tempdata+3] 	 = reg16.dataBit.data0;
    	((uint8_t *)buf_out)[tempdata+6]	 = reg16.dataBit.data0;
    	((uint8_t *)buf_out)[tempdata+9] 	 = reg16.dataBit.data0;
    	((uint8_t *)buf_out)[12+tempdata++]  = reg16.dataBit.data0;

    	((uint8_t *)buf_out)[tempdata] 	 	 = reg16.dataBit.data1;
    	((uint8_t *)buf_out)[tempdata+3] 	 = reg16.dataBit.data1;
    	((uint8_t *)buf_out)[tempdata+6]	 = reg16.dataBit.data1;
    	((uint8_t *)buf_out)[tempdata+9] 	 = reg16.dataBit.data1;
    	((uint8_t *)buf_out)[12+tempdata++]  = reg16.dataBit.data1;

    	((uint8_t *)buf_out)[tempdata] 	     = (uint8_t)33;
    	((uint8_t *)buf_out)[tempdata+3] 	 = (uint8_t)33;
    	((uint8_t *)buf_out)[tempdata+6]	 = (uint8_t)33;
    	((uint8_t *)buf_out)[tempdata+9] 	 = (uint8_t)33;
    	((uint8_t *)buf_out)[12+tempdata++]  = (uint8_t)33;

    	tempdata+=12;
    }
#else
	static reg_16 reg16plus;
	reg_16 reg[4];

	k=20000.000000/3000.000000;
	factor=FSK_FAST_SPI_calc();

    for (tempCount = 0; tempCount < RPE_DATA_SIZE/2; tempCount++){
    	if(ring_ar[tempCount]>20000)
    		ring_ar[tempCount]=20000;
    	else if(ring_ar[tempCount]<-20000)
    		ring_ar[tempCount]=-20000;

    	//Interpolation
    	xn=factor*(ring_ar[tempCount]/k);
    	if(tempCount+1<RPE_DATA_SIZE/2)
    		xn1=factor*(ring_ar[tempCount+1]/k);
    	else
    		xn1=xn;
    	reg16.all=(unsigned short)xn;
    	reg[0].all=(unsigned short)(0.8*xn+0.2*xn1);
    	reg[1].all=(unsigned short)(0.6*xn+0.4*xn1);
    	reg[2].all=(unsigned short)(0.4*xn+0.6*xn1);
    	reg[3].all=(unsigned short)(0.2*xn+0.8*xn1);

    	((uint8_t *)buf_out)[tempdata] 	 	 = reg16.dataBit.data0;
    	((uint8_t *)buf_out)[tempdata+3] 	 = reg[0].dataBit.data0;
    	((uint8_t *)buf_out)[tempdata+6]	 = reg[1].dataBit.data0;
    	((uint8_t *)buf_out)[tempdata+9] 	 = reg[2].dataBit.data0;
    	((uint8_t *)buf_out)[12+tempdata++]  = reg[3].dataBit.data0;

    	((uint8_t *)buf_out)[tempdata] 	 	 = reg16.dataBit.data1;
    	((uint8_t *)buf_out)[tempdata+3] 	 = reg[0].dataBit.data1;
    	((uint8_t *)buf_out)[tempdata+6]	 = reg[1].dataBit.data1;
    	((uint8_t *)buf_out)[tempdata+9] 	 = reg[2].dataBit.data1;
    	((uint8_t *)buf_out)[12+tempdata++]  = reg[3].dataBit.data1;

    	((uint8_t *)buf_out)[tempdata] 	     = (uint8_t)33;
    	((uint8_t *)buf_out)[tempdata+3] 	 = (uint8_t)33;
    	((uint8_t *)buf_out)[tempdata+6]	 = (uint8_t)33;
    	((uint8_t *)buf_out)[tempdata+9] 	 = (uint8_t)33;
    	((uint8_t *)buf_out)[12+tempdata++]  = (uint8_t)33;

    	tempdata+=12;
    }
#endif
}
#endif

/*
 *  ======== data_send ========
 */
Void data_send(uint8_t *buf_16)
{
//    struct rpe *prpe = NULL;
//    prpe=&rpe[0];
	uint8_t 		*buf = NULL;
    uint32_t		size;
    Int status=0;
    static uint32_t count1,count2;
    static unsigned char silence;
    //data send
    size=RPE_DATA_SIZE;

    while(rx_flag){
		status=rpe_acquire_writer(prpe,(rpe_buf_t*)&buf,&size);
		if((status&&status!=ERPE_B_BUFWRAP)||size!=RPE_DATA_SIZE||status<0){
	        rpe_release_writer(prpe,0);
	        size=RPE_DATA_SIZE;
	        continue;
		}

		//silence
	    if(RSSI_db<RXSS_THRESHOLD){
	    	if(++count1==3){
	    		silence=0;
	    		count1--;
	    		count2=0;
	    	}
	    }
	    else if(RSSI_db>RXSS_THRESHOLD+3){
	    	if(++count2==3){
	    		silence=1;
		    	count2--;
		    	count1=0;
	    	}
	    }

	    if(0==silence)
	    	memset(buf,0,size);
	    else
	    	memcpy(buf,(unsigned char*)buf_16,size);	//buf_16: data to be sent

        status=rpe_release_writer_safe(prpe,buf,size);
        if(status<0){
        	log_warn("rpe release writer end failed!");
        	continue;
        }else
        	break;
    }

    return;
}


//24bit-->16bit
void data_extract(unsigned char *input, unsigned char *output)
{
	unsigned int i;

	for(i=0;i<REC_BUFSIZE/3;i++){
		*output++=*input++;
		*output++=*input++;
		input++;
	}
	return;
}

/*
 *  ======== task_enque ========
 */
Void task_enque(UArg arg0, UArg arg1)
{
	extern audioQueue audioQ;
	extern AudioQueue_DATA_TYPE audioQueueRxBuf[];
//	int i=0;
	unsigned short buf_16[REC_BUFSIZE/3] = {0};

//	static short index;

//	audioQueueInit(&audioQ, AUDIO_QUEUE_RX_LENGTH, audioQueueRxBuf);

	do{
		Semaphore_pend(sem1,BIOS_WAIT_FOREVER);
		data_extract(buf_adc,(unsigned char*)buf_16);
		Rx_process(buf_16,buf_de);
		data_send((uint8_t*)buf_de);

//		//data enqueue
//		for(i = 0; i < REC_BUFSIZE/3; i++)
//		{
//			enAudioQueue(&audioQ, buf_16[i]);
//
////			buf_temp[index++]=buf_16[i];
////			if(index>=9600)
////					index=0;
//		}

	}while(1);
}

void sys_configure(void)
{
    CSL_SyscfgRegsOvly syscfgRegs = (CSL_SyscfgRegsOvly)CSL_SYSCFG_0_REGS;

    //Select PLL0_SYSCLK2
//    syscfgRegs->CFGCHIP3 &= ~CSL_SYSCFG_CFGCHIP3_ASYNC3_CLKSRC_MASK;
//    syscfgRegs->CFGCHIP3 |= ((CSL_SYSCFG_CFGCHIP3_ASYNC3_CLKSRC_PLL0)
//        						  <<(CSL_SYSCFG_CFGCHIP3_ASYNC3_CLKSRC_SHIFT));
    //mcbsp1
    syscfgRegs->PINMUX1 &= ~(CSL_SYSCFG_PINMUX1_PINMUX1_7_4_MASK |
                             CSL_SYSCFG_PINMUX1_PINMUX1_11_8_MASK |
                             CSL_SYSCFG_PINMUX1_PINMUX1_15_12_MASK |
                             CSL_SYSCFG_PINMUX1_PINMUX1_19_16_MASK |
                             CSL_SYSCFG_PINMUX1_PINMUX1_23_20_MASK |
                             CSL_SYSCFG_PINMUX1_PINMUX1_27_24_MASK |
                             CSL_SYSCFG_PINMUX1_PINMUX1_31_28_MASK);
    syscfgRegs->PINMUX1   = 0x22222220;
    //spi1
    syscfgRegs->PINMUX5 &= ~(CSL_SYSCFG_PINMUX5_PINMUX5_3_0_MASK |
    						 CSL_SYSCFG_PINMUX5_PINMUX5_7_4_MASK |
                             CSL_SYSCFG_PINMUX5_PINMUX5_11_8_MASK |
                             CSL_SYSCFG_PINMUX5_PINMUX5_19_16_MASK |
                             CSL_SYSCFG_PINMUX5_PINMUX5_23_20_MASK );
    syscfgRegs->PINMUX5   |= 0x00110111;

    syscfgRegs->PINMUX6 &= ~(CSL_SYSCFG_PINMUX6_PINMUX6_23_20_MASK|
    						CSL_SYSCFG_PINMUX6_PINMUX6_27_24_MASK);
    syscfgRegs->PINMUX6  |= 0x08800000;

    syscfgRegs->PINMUX7  &= ~(CSL_SYSCFG_PINMUX7_PINMUX7_11_8_MASK|
    						CSL_SYSCFG_PINMUX7_PINMUX7_15_12_MASK);
    syscfgRegs->PINMUX7  |= 0X00008800;

//    syscfgRegs->PINMUX10 &= ~(CSL_SYSCFG_PINMUX10_PINMUX10_23_20_MASK|
//    						 CSL_SYSCFG_PINMUX10_PINMUX10_31_28_MASK);
//    syscfgRegs->PINMUX10 |= 0x80800000;
    syscfgRegs->PINMUX13 &= ~(CSL_SYSCFG_PINMUX13_PINMUX13_11_8_MASK|
    						CSL_SYSCFG_PINMUX13_PINMUX13_15_12_MASK);
    syscfgRegs->PINMUX13 |= 0x00008800;
    syscfgRegs->PINMUX14 &= ~(CSL_SYSCFG_PINMUX14_PINMUX14_3_0_MASK|
    						CSL_SYSCFG_PINMUX14_PINMUX14_7_4_MASK);
    syscfgRegs->PINMUX14 |= 0x00000088;

    syscfgRegs->PINMUX19 &=~(CSL_SYSCFG_PINMUX19_PINMUX19_27_24_MASK);
    syscfgRegs->PINMUX19 |= 0x08000000;

	 /* Configure GPIO2_1 (GPIO2_1_PIN) as an input                            */
	 CSL_FINS(gpioRegs->BANK[1].DIR,GPIO_DIR_DIR1,1);

    //GP2[2]	CE
    CSL_FINS(gpioRegs->BANK[GP2].DIR,GPIO_DIR_DIR2,0);
//    CSL_FINS(gpioRegs->BANK[1].OUT_DATA,GPIO_OUT_DATA_OUT2,0);
    CSL_FINS(gpioRegs->BANK[GP2].OUT_DATA,GPIO_OUT_DATA_OUT2,1);
    //GP4[0]	MUX
//    CSL_FINS(gpioRegs->BANK[2].DIR,GPIO_DIR_DIR0,1);
//    //GP4[2]	T/R
//    CSL_FINS(gpioRegs->BANK[2].DIR,GPIO_DIR_DIR2,0);
//    CSL_FINS(gpioRegs->BANK[2].OUT_DATA,GPIO_OUT_DATA_OUT2,1);
//    CSL_FINS(gpioRegs->BANK[2].OUT_DATA,GPIO_OUT_DATA_OUT2,0);

    //GP3[12] LNA_ATT_EN
    CSL_FINS(gpioRegs->BANK[1].DIR,GPIO_DIR_DIR28,0);
    CSL_FINS(gpioRegs->BANK[1].OUT_DATA,GPIO_OUT_DATA_OUT28,0);
    //GP3[13] IF_AGC_CTL
    CSL_FINS(gpioRegs->BANK[1].DIR,GPIO_DIR_DIR29,0);
    CSL_FINS(gpioRegs->BANK[1].OUT_DATA,GPIO_OUT_DATA_OUT29,0);

    //GP6[0]
    CSL_FINS(gpioRegs->BANK[3].DIR,GPIO_DIR_DIR0,0);
    CSL_FINS(gpioRegs->BANK[3].OUT_DATA,GPIO_OUT_DATA_OUT0,1);

    //GP6[13]
    CSL_FINS(gpioRegs->BANK[3].DIR,GPIO_DIR_DIR13,0);
    CSL_FINS(gpioRegs->BANK[3].OUT_DATA,GPIO_OUT_DATA_OUT13,0);
//    CSL_FINS(gpioRegs->BANK[3].OUT_DATA,GPIO_OUT_DATA_OUT13,1); //T:F2
    //GP6[6] GP6[7]	TX_SW RX_SW
    CSL_FINS(gpioRegs->BANK[3].DIR,GPIO_DIR_DIR6,0);//TX_SW
    CSL_FINS(gpioRegs->BANK[3].OUT_DATA,GPIO_OUT_DATA_OUT6,0);
    CSL_FINS(gpioRegs->BANK[3].DIR,GPIO_DIR_DIR7,0);//RX_SW
    CSL_FINS(gpioRegs->BANK[3].OUT_DATA,GPIO_OUT_DATA_OUT7,1);

}

void ch_chPara(){
	reg_24 reg_24data;
	uint32_t tempCount=0;


	for (tempCount = 0; tempCount < 36; tempCount++){
	    reg_24data.all=lmx_init[45+tempCount/3];
	    buf_transmit[tempCount++] = reg_24data.dataBit.data0;
	    buf_transmit[tempCount++] = reg_24data.dataBit.data1;
	    buf_transmit[tempCount]   = reg_24data.dataBit.data2;
	}
}

void dsp_logic()
{
	int status=NO_ERR;
	message_t msg_temp;
	int ad_ch;
	float ad_value;
	char str_temp[32];
	char* pstr=NULL;
	char* poffs=NULL;
    message_t *msg =NULL, *msg_send=NULL;
//    static float transmit_power=L_TXPWR;



    if(1==rx_submit){
    	msg_send=(message_t *)message_alloc(msgbuf[0],sizeof(message_t));
    	if(!msg_send){
//    	    log_warn("msgq out of memmory");
    	    goto out;
    	}
		msg_send->type=DATA_END;
		msg_send->data.d[0]='\0';
		status=messageq_send_safe(&msgq[0],msg_send,0,0,0);
		if(status<0){
			log_error("message send error");
			message_free(msg_send);
		}
    	rx_flag=1;
    	rx_submit=0;
    }
    //
    status=messageq_receive(&msgq[0],&msg,0);
    if (status>=0){
    	msg_temp.type=msg->type;
    	memcpy(msg_temp.data.d,msg->data.d,100);
//    	log_info("message type	  is %d",msg->type);
//    	log_info("message content is %s",msg->data.d);
    	message_free(msg);
    }
    if(status>=0){
    		switch (msg_temp.type){
    		case LMX2571_TX:
    			CSL_FINS(gpioRegs->BANK[3].OUT_DATA,GPIO_OUT_DATA_OUT7,0);//RX_SW
    			CSL_FINS(gpioRegs->BANK[3].OUT_DATA,GPIO_OUT_DATA_OUT6,1); //TX_SW
    			CSL_FINS(gpioRegs->BANK[3].OUT_DATA,GPIO_OUT_DATA_OUT13,1); //T:F2
    		    //TX_DAC
    		    dac_write(3, transmit_power); //1w
    			rx_flag=0;
    			tx_flag=1;
    			break;
    		case LMX2571_RX:
    			rpe_flush(prpe,RPE_ENDPOINT_READER,TRUE,&attr);//返回 flush的数据量

    			dac_write(3, 0);
    			CSL_FINS(gpioRegs->BANK[3].OUT_DATA,GPIO_OUT_DATA_OUT6,0); //TX_SW
    			CSL_FINS(gpioRegs->BANK[3].OUT_DATA,GPIO_OUT_DATA_OUT7,1);//RX_SW
    			CSL_FINS(gpioRegs->BANK[3].OUT_DATA,GPIO_OUT_DATA_OUT13,0); //R:F1

				tx_flag=0;
				tx_submit=0;
				memset(buf_transmit,0x80,RPE_DATA_SIZE/2*15);
    			rx_flag=1;
    			break;
    		case RSSI_RD:
    	    	msg_send=(message_t *)message_alloc(msgbuf[0],sizeof(message_t));
    	    	if(!msg_send){
    	    	    log_warn("msgq out of memmory");
    	    	}
    			msg_send->type=RSSI_RD;
    			sprintf(msg_send->data.d,"%f",RSSI_db);
    			status=messageq_send_safe(&msgq[0],msg_send,0,0,0);
    			if(status<0){
    				log_error("message send error");
    				message_free(msg_send);
    			}
    			break;
    		case IF_AGC_ON:
    			CSL_FINS(gpioRegs->BANK[1].OUT_DATA,GPIO_OUT_DATA_OUT29,1);
    			break;
    		case IF_AGC_OFF:
    		    CSL_FINS(gpioRegs->BANK[1].OUT_DATA,GPIO_OUT_DATA_OUT29,0);
    		    break;
    		case LNA_ATT_ON:
    			CSL_FINS(gpioRegs->BANK[1].OUT_DATA,GPIO_OUT_DATA_OUT28,1);
    			break;
    		case LNA_ATT_OFF:
    		    CSL_FINS(gpioRegs->BANK[1].OUT_DATA,GPIO_OUT_DATA_OUT28,0);
    		    break;
    		case TX_ON:
    			CSL_FINS(gpioRegs->BANK[3].OUT_DATA,GPIO_OUT_DATA_OUT6,1);
    			break;
    		case TX_OFF:
    			CSL_FINS(gpioRegs->BANK[3].OUT_DATA,GPIO_OUT_DATA_OUT6,0);
    			break;
    		case RX_ON:
    			CSL_FINS(gpioRegs->BANK[3].OUT_DATA,GPIO_OUT_DATA_OUT7,1);
    			break;
    		case RX_OFF:
    			CSL_FINS(gpioRegs->BANK[3].OUT_DATA,GPIO_OUT_DATA_OUT7,0);
    			break;
    		case LMX2571_LD:
    	    	msg_send=(message_t *)message_alloc(msgbuf[0],sizeof(message_t));
    	    	if(!msg_send){
    	    	    log_warn("msgq out of memmory");
    	    	}
    			msg_send->type=LMX2571_LD;
    			sprintf(msg_send->data.d,"%d",CSL_FEXT(gpioRegs->BANK[3].IN_DATA,GPIO_IN_DATA_IN12));
    			log_info("lmx2571_ld is %s",msg_send->data.d);
    			status=messageq_send_safe(&msgq[0],msg_send,0,0,0);
    			if(status<0){
    				log_error("message send error");
    				message_free(msg_send);
    			}
    			break;
    		case TX_CH:
//    			log_info("tx_ch:%f",atof(msg_temp.data.d));
    			channel_freq=atof(msg_temp.data.d);
    			LMX2571_FM_CAL(1,atof(msg_temp.data.d), 1);
    			lmx_init[53]=0xBC3;
    			lmx_init[54]=lmx_init[55]=0x9C3;
    			lmx_init[56]=0x9C3;
    			ch_chPara();
    			Task_sleep(10);
    			memset(buf_transmit,0x80,RPE_DATA_SIZE/2*15);
    			break;
    		case RX_CH:
//    			log_info("rx_ch:%f",49.95+atof(msg_temp.data.d));
    			LMX2571_FM_CAL(0,49.95+atof(msg_temp.data.d), 0);
    			lmx_init[53]=0xB83;
    			lmx_init[54]=lmx_init[55]=0x983;
    			lmx_init[56]=0x983;
    			ch_chPara();
    			Task_sleep(10);
    			memset(buf_transmit,0x80,RPE_DATA_SIZE/2*15);
    			break;
    		case AMC7823_AD:
    			ad_ch=atoi(msg_temp.data.d);
    			if(ad_ch>8||ad_ch<0)
    				log_error("error ad_ch parameter %d",ad_ch);
    			ad_value=adc_read(ad_ch);
    	    	msg_send=(message_t *)message_alloc(msgbuf[0],sizeof(message_t));
    	    	if(!msg_send){
    	    	    log_warn("msgq out of memmory");
    	    	}
    			msg_send->type=AMC7823_AD;
    			sprintf(msg_send->data.d,"%f",ad_value);
    			status=messageq_send_safe(&msgq[0],msg_send,0,0,0);
    			if(status<0){
    			    log_error("message send error");
    			    message_free(msg_send);
    			}
    			break;
    		case AMC7823_DA:
    			poffs=strstr(msg_temp.data.d,":");
    			strncpy(str_temp,msg_temp.data.d,poffs-msg_temp.data.d);
    			str_temp[poffs-msg_temp.data.d]='\0';
    			ad_ch=atoi(str_temp);
    			if(ad_ch>8||ad_ch<0)
    				log_error("error ad_ch parameter %d",ad_ch);
    			pstr=poffs+1;
//    			poffs=strstr(pstr,":");
//    			strncpy(str_temp,pstr,poffs-pstr);
    			strcpy(str_temp,pstr);
//    			str_temp[poffs-pstr]='\0';
    			ad_value=atof(str_temp);
    			dac_write(ad_ch,ad_value);
    			break;
    		case P_TEMP2:
    		case PA_TEMP:
    	    	msg_send=(message_t *)message_alloc(msgbuf[0],sizeof(message_t));
    	    	if(!msg_send){
    	    	    log_warn("msgq out of memmory");
    	    	}
//    			ad_value=temperature_read();
    	    	ad_value=adc_read(1);
    	    	ad_value=(ad_value*1000-500)*0.1;
    		    msg_send->type=msg_temp.type;
    		    sprintf(msg_send->data.d,"%f",ad_value);
    		    status=messageq_send_safe(&msgq[0],msg_send,0,0,0);
    		    if(status<0){
    		    	log_error("message send error");
    		    	message_free(msg_send);
    		    }
    		    break;
    		case TX_CHF:
//    			log_info("tx_ch:%f",atof(msg_temp.data.d));
    			channel_freq=atof(msg_temp.data.d);
    			LMX2571_FM_CAL(1,atof(msg_temp.data.d), 1);
    			lmx_init[53]=0xBC3;
    			lmx_init[54]=lmx_init[55]=0x9C3;
    			lmx_init[56]=0x9C3;
    			ch_chPara();
    			Task_sleep(10);
    			memset(buf_transmit,0x80,RPE_DATA_SIZE/2*15);
    			break;
    		case RX_CHF:
//    			log_info("rx_ch:%f",49.95+atof(msg_temp.data.d));
    			LMX2571_FM_CAL(0,49.95+atof(msg_temp.data.d), 0);
    			lmx_init[53]=0xB83;
    			lmx_init[54]=lmx_init[55]=0x983;
    			lmx_init[56]=0x983;
    			ch_chPara();
    			Task_sleep(10);
    			memset(buf_transmit,0x80,RPE_DATA_SIZE/2*15);
    			break;
    		case H_TX:
    			transmit_power=eeprom_data[28];
    			break;
    		case L_TX:
    			transmit_power=eeprom_data[39];
    			break;
    		case RSSTH:
    			RXSS_THRESHOLD=2*atoi(msg_temp.data.d)-115;
    			if(-115==RXSS_THRESHOLD)
    				RXSS_THRESHOLD=-150;
    			break;

    		case PA_CURRENT:
    		case P_CURRENT2:
    			ad_value=adc_read(6)/0.3;

    	    	msg_send=(message_t *)message_alloc(msgbuf[0],sizeof(message_t));
    	    	if(!msg_send){
    	    	    log_warn("msgq out of memmory");
    	    	}
    			msg_send->type=msg_temp.type;
    			sprintf(msg_send->data.d,"%f",ad_value);
    			status=messageq_send(&msgq[0],(messageq_msg_t)msg_send,0,0,0);
    			if(status<0){
    				log_error("message send error");
    				message_free(msg_send);
    			}
    			break;
    		case VSWR:
    		case VSWR2:
    			ad_value=adc_read(0);
    			if(ad_value<=eeprom_data[105])
    				ad_ch=15;
    			else if(eeprom_data[105]<ad_value&&ad_value<=eeprom_data[108])
    				ad_ch=3-0.1*ad_value;
    			else if(eeprom_data[108]<ad_value&&ad_value<=eeprom_data[111])
    				ad_ch=3.25-0.125*ad_value;
    			else if(eeprom_data[111]<ad_value&&ad_value<=eeprom_data[114])
    				ad_ch=3.5-0.167*ad_value;
    			else if(ad_value>=eeprom_data[114])
    				ad_ch=3;
    			//send
    	    	msg_send=(message_t *)message_alloc(msgbuf[0],sizeof(message_t));
    	    	if(!msg_send){
    	    	    log_warn("msgq out of memmory");
    	    	}
    			msg_send->type=msg_temp.type;
    			sprintf(msg_send->data.d,"%d",ad_ch);
    			status=messageq_send(&msgq[0],(messageq_msg_t)msg_send,0,0,0);
    			if(status<0){
    				log_error("message send error");
    				message_free(msg_send);
    			}
    			break;
    		case RSSI2:
    		case RXSSI:
    	    	msg_send=(message_t *)message_alloc(msgbuf[0],sizeof(message_t));
    	    	if(!msg_send){
    	    	    log_warn("msgq out of memmory");
    	    	}
    			msg_send->type=msg_temp.type;
    			sprintf(msg_send->data.d,"%f",RSSI_db);
    			status=messageq_send(&msgq[0],(messageq_msg_t)msg_send,0,0,0);
    			if(status<0){
    				log_error("message send error");
    				message_free(msg_send);
    			}
    			break;
    		case TXPI:
    	    	msg_send=(message_t *)message_alloc(msgbuf[0],sizeof(message_t));
    	    	if(!msg_send){
    	    	    log_warn("msgq out of memmory");
    	    	}
    			msg_send->type=TXPI;
    			sprintf(msg_send->data.d,"%f",transmit_power*10);
    			status=messageq_send(&msgq[0],(messageq_msg_t)msg_send,0,0,0);
    			if(status<0){
    			    log_error("message send error");
    			    message_free(msg_send);
    			}
    			break;
    		case V138:
    	    	msg_send=(message_t *)message_alloc(msgbuf[0],sizeof(message_t));
    	    	if(!msg_send){
    	    	    log_warn("msgq out of memmory");
    	    	}
    			msg_send->type=V138;
    			ad_value=adc_read(7);
    			ad_value=ad_value*8;
    			sprintf(msg_send->data.d,"%f",ad_value);
    			status=messageq_send(&msgq[0],(messageq_msg_t)msg_send,0,0,0);
    			if(status<0){
    			    log_error("message send error");
    			    message_free(msg_send);
    			}
    			break;
    		case V6:
    	    	msg_send=(message_t *)message_alloc(msgbuf[0],sizeof(message_t));
    	    	if(!msg_send){
    	    	    log_warn("msgq out of memmory");
    	    	}
    			msg_send->type=V6;
    			ad_value=6;
    			sprintf(msg_send->data.d,"%f",ad_value);
    			status=messageq_send(&msgq[0],(messageq_msg_t)msg_send,0,0,0);
    			if(status<0){
    			    log_error("message send error");
    			    message_free(msg_send);
    			}
    			break;
    		case ADJVCO:
    	    	msg_send=(message_t *)message_alloc(msgbuf[0],sizeof(message_t));
    	    	if(!msg_send){
    	    	    log_warn("msgq out of memmory");
    	    	}
    			msg_send->type=ADJVCO;
    			sprintf(msg_send->data.d,"%f",eeprom_data[25]);
    			status=messageq_send(&msgq[0],(messageq_msg_t)msg_send,0,0,0);
    			if(status<0){
    			    log_error("message send error");
    			    message_free(msg_send);
    			}
    			break;
    		case VPWR25:
    	    	msg_send=(message_t *)message_alloc(msgbuf[0],sizeof(message_t));
    	    	if(!msg_send){
    	    	    log_warn("msgq out of memmory");
    	    	}
    			msg_send->type=VPWR25;
    			sprintf(msg_send->data.d,"%f",eeprom_data[28]);
    			status=messageq_send(&msgq[0],(messageq_msg_t)msg_send,0,0,0);
    			if(status<0){
    			    log_error("message send error");
    			    message_free(msg_send);
    			}
    			break;
    		case VPWR14:
    	    	msg_send=(message_t *)message_alloc(msgbuf[0],sizeof(message_t));
    	    	if(!msg_send){
    	    	    log_warn("msgq out of memmory");
    	    	}
    			msg_send->type=VPWR14;
    			sprintf(msg_send->data.d,"%f",eeprom_data[34]);
    			status=messageq_send(&msgq[0],(messageq_msg_t)msg_send,0,0,0);
    			if(status<0){
    			    log_error("message send error");
    			    message_free(msg_send);
    			}
    			break;
    		case VPWR1:
    	    	msg_send=(message_t *)message_alloc(msgbuf[0],sizeof(message_t));
    	    	if(!msg_send){
    	    	    log_warn("msgq out of memmory");
    	    	}
    			msg_send->type=VPWR1;
    			sprintf(msg_send->data.d,"%f",eeprom_data[39]);
    			status=messageq_send(&msgq[0],(messageq_msg_t)msg_send,0,0,0);
    			if(status<0){
    			    log_error("message send error");
    			    message_free(msg_send);
    			}
    			break;
    		case ADJRSSI:
    	    	msg_send=(message_t *)message_alloc(msgbuf[0],sizeof(message_t));
    	    	if(!msg_send){
    	    	    log_warn("msgq out of memmory");
    	    	}
    			msg_send->type=ADJRSSI;
    			sprintf(msg_send->data.d,"%f",eeprom_data[84]);
    			status=messageq_send(&msgq[0],(messageq_msg_t)msg_send,0,0,0);
    			if(status<0){
    			    log_error("message send error");
    			    message_free(msg_send);
    			}
    			break;
    		case ADJ_VCO:
    			eeprom_data[25]=atof(msg_temp.data.d);
    			dac_write(0,eeprom_data[24]+eeprom_data[25]);
    			break;
    		case VOL_25W:
    			if(transmit_power==eeprom_data[28])
    				transmit_power=atof(msg_temp.data.d);
    			eeprom_data[28]=atof(msg_temp.data.d);
    			break;
    		case VOL_14W:
    			if(transmit_power==eeprom_data[34])
    				transmit_power=atof(msg_temp.data.d);
    			eeprom_data[34]=atof(msg_temp.data.d);
    			break;
    		case VOL_1W:
    			if(transmit_power==eeprom_data[39])
    				transmit_power=atof(msg_temp.data.d);
    			eeprom_data[39]=atof(msg_temp.data.d);
    			break;
    		case ADJ_RSSI:
//    			delta_rssi=atoi(msg_temp.data.d);
    			eeprom_data[84]=atoi(msg_temp.data.d);
    			break;
    		default:
    			log_error("unknown message  type is %d", msg_temp.type);
    			break;
    		}
    }
out:
	return;
}


void eeprom_cache()
{
	  static short count0;
	  int status=NO_ERR;
	  message_t *msg =NULL;

	  while(count0<101){
	    status=messageq_receive(&msgq[0],&msg,0);
	    if (status>=0&&msg->type==EEPROM){
			memcpy(eeprom_data+count0,msg->data.d,100);
			count0+=25;
	    	message_free(msg);
	    }
	  }
}
