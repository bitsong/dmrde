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

extern void* bufRxPingPong[2];
extern void* bufTxPingPong[2];
extern int TxpingPongIndex,RxpingPongIndex;

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

QUEUE_DATA_TYPE    dsc_buf[DSC_RX_BUF_LEN];

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

    queueInit(&q, DSC_RX_BUF_LEN, dsc_buf);

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
    taskParams.instance->name = "task_enque";
    taskParams.arg0 = (UArg)arg0;
    taskParams.arg1 = (UArg)arg1;
    taskParams.stackSize = 0x4000;
    taskParams.priority=15;
    Task_create(task_enque, &taskParams, &eb);
    if(Error_check(&eb)) {
    	System_abort("main: failed to create application 2 thread");
    }

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
	taskParams.priority = 1;
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

    	//fitting : y = - 0.16365*x^{2} - 36.798*x - 2182.9
    	if(RSSI_db<-115.4)
//    		RSSI_db=-0.16365*RSSI_db*RSSI_db-36.798*RSSI_db-2182.9;
    		//y = 0.011574*x^{3} + 3.9377*x^{2} + 447.59*x + 16885
//    		RSSI_db=0.011574*powi(RSSI_db,3)+ 3.9377*powi(RSSI_db,2)+ 447.59*RSSI_db + 16885;
    		//y = 0.071842*x^{3} + 25.165*x^{2} + 2939.6*x + 1.144e+05
    		RSSI_db=0.071842*powi(RSSI_db,3)+ 25.165*powi(RSSI_db,2)+ 2939.6*RSSI_db + 114397;
//    	if(RSSI_db<-117.5)
//    		RSSI_db +=(RSSI_db+115);
//    	else if(RSSI_db<-115)
//    		RSSI_db +=0.5*(RSSI_db+115);
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
	static int flag;

	 /* get the GIPO6_5 value		                                           */
	 flag=CSL_FEXT(gpioRegs->BANK[1].IN_DATA,GPIO_IN_DATA_IN1);

//	 {
		 enQueue(&q, flag);
//	 }
//	 else{
//		 enQueue(&q, 0);
//	 }
//	enQueue(&q, CSL_FEXT(gpioRegs->BANK[1].IN_DATA,GPIO_IN_DATA_IN1));
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
//    static int txpingpongflag;

    buf_transmit=(unsigned char*)malloc(RPE_DATA_SIZE/2*15);

    log_info("-->task_receive:");
    Error_init(&eb);
    while(1){
    		Semaphore_pend(sem2, BIOS_WAIT_FOREVER);

//    		txpingpongflag=(TxpingPongIndex)?0:1;
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
//    		data_process((short*)buf, (unsigned char*)bufTxPingPong[txpingpongflag], size);

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
void LP_Filter(short *inBuf,short *outBuf)
{
	register short i = 0;
	static float temp[272] = {0};
	float coffe0=0.299496059427060,	coffe1=0.256986632618021, 	coffe2=0.150967710628479,	coffe3= 0.032995877067145,
	coffe4=-0.045687558189818,		coffe5=-0.062214363053361,	coffe6=-0.030565720303875 , 	coffe7=0.012959340065669 ,
	coffe8=0.035504661411791,  	coffe9=  0.026815438823179,	coffe10=0.000482815488849, 	coffe11=-0.020603625992256,
	coffe12=-0.022150036648907, 		coffe13=-0.006883093026563,	coffe14=0.010689881755938,	coffe15=0.017054011686632,
	coffe16= 0.009462869502823;

	for(i = 0; i < 32; ++i)
		temp[i] = temp[240 + i];
	for(i=0; i<240;i++)
		temp[i+32] = inBuf[i];
	//
	for(i = 0; i < 240; ++i){
		outBuf[i] = temp[i+16]*coffe0
		+(temp[i+17]+temp[i+15])*coffe1 + (temp[i+18]+temp[i+14])*coffe2  + (temp[i+19]+temp[i+13])*coffe3  + (temp[i+20]+temp[i+12])*coffe4
		+(temp[i+21]+temp[i+11])*coffe5 + (temp[i+22]+temp[i+10])*coffe6  + (temp[i+23]+temp[i+9])*coffe7   + (temp[i+24]+temp[i+8])*coffe8
		+(temp[i+25]+temp[i+7])*coffe9  + (temp[i+26]+temp[i+6])*coffe10  + (temp[i+27]+temp[i+5])*coffe11  + (temp[i+28]+temp[i+4])*coffe12
		+(temp[i+29]+temp[i+3])*coffe13 + (temp[i+30]+temp[i+2])*coffe14  + (temp[i+31]+temp[i+1])*coffe15  + (temp[i+32]+temp[i])*coffe16;
	}
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
		if(inBuf[i]>18000)
			inBuf[i] = 18000;
		else if(inBuf[i]<-18000)
			inBuf[i] = -18000;
	}
}

void hpFilter(short *inBuf,short *outBuf)
{
	short i = 0;
	static float x[242] = {0};
	static float y[242] = {0};

	x[0] = x[240];
	x[1] = x[241];

	y[0] = y[240];
	y[1] = y[241];

	for(i=0;i<240;i++)
		x[i+2] = inBuf[i];

	for(i=0;i<240;i++)
		y[i+2] = 0.9816583*(x[i+2]-2*x[i+1]+x[i]) + 1.9629801*y[i+1] - 0.9636530*y[i];

	for(i=0;i<240;i++)
		outBuf[i] = y[i+2];
}


/*
 *功能：数据发送端数据处理。（64阶低通滤波、预加重、24k->120k处理）
 *参数：inBuf:输入为240个数据的指针; outBuf:输出为1200个数据的指针; len:输入数据的长度（默认240）
 */
void dataFilterAndTrans(short *inBuf,short *outBuf,short len)
{
//	delDc(inBuf,len);
	hpFilter(inBuf, inBuf);
	sendPreEmphasis(inBuf,outBuf,len);		//input:outBuf,output:inBuf(inBuf as a temp buffer)
	LP_Filter(outBuf,outBuf);
//	scopeLimit(inBuf,len);
//	sendLowFreqFilter(inBuf,outBuf,len);
	from24To120(outBuf,outBuf,len);
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
//	static short index
	unsigned short buf_16[REC_BUFSIZE/3] = {0};

	static short pingpongflag;

//	audioQueueInit(&audioQ, AUDIO_QUEUE_RX_LENGTH, audioQueueRxBuf);

	do{
		Semaphore_pend(sem1,BIOS_WAIT_FOREVER);

		pingpongflag=RxpingPongIndex?0:1;
		data_extract((unsigned char*)bufRxPingPong[pingpongflag],(unsigned char*)buf_16);
		Rx_process(buf_16,buf_de);
		data_send((uint8_t*)buf_de);

//		//data enqueue
//		for(i = 0; i < REC_BUFSIZE/15; i++)
//		{
////			enAudioQueue(&audioQ, buf_16[i]);
//
//			buf_temp[index++]=buf_de[i];
//			if(index>=9599)
//					index=0;
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
    			rpe_flush(prpe,RPE_ENDPOINT_WRITER,TRUE,&attr);//返回 flush的数据量
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
    			RXSS_THRESHOLD=2*atoi(msg_temp.data.d)-125;
    			if(-125==RXSS_THRESHOLD)
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
