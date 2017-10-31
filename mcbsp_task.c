
/* ========================================================================== */
/*                            INCLUDE FILES                                   */
/* ========================================================================== */

#include <ti/sysbios/BIOS.h>
#include <xdc/std.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <string.h>
#include <ti/sysbios/knl/Queue.h>
#include <xdc/runtime/System.h>

#include <xdc/cfg/global.h>

/* Include EDMA3 Driver */
#include <ti/sdo/edma3/drv/edma3_drv.h>

#include <ti/sysbios/hal/Cache.h>

/* MCBSP Driver Include File. */
#include <ti/drv/mcbsp/mcbsp_drv.h>
#include <ti/drv/mcbsp/mcbsp_osal.h>

#include "log.h"
#include "main.h"

/* ========================================================================== */
/*                        EXTERNAL FUNCTIONS                                  */
/* ========================================================================== */

extern EDMA3_DRV_Handle edma3init(unsigned int edma3Id, EDMA3_DRV_Result *);

extern void McbspDevice_init(void);

extern int32_t Osal_dataBufferInitMemory(uint32_t dataBufferSize);

extern void McbspXmtInterrupt_init(void *mcbspTxChan);
extern void McbspRcvInterrupt_init(void *mcbspRxChan);
//extern const Int REC_BUFSIZE;
//extern const Int RPE_DATA_SIZE;
extern unsigned int lmx_init[];
extern UInt	tx_flag, rx_flag, tx_submit, rx_submit;
extern unsigned char buf_adc[];
extern unsigned char *buf_transmit;
extern Semaphore_Handle sem1;
extern Semaphore_Handle sem2;
extern void LMX2571_FM_CAL(unsigned short ch, double fm, unsigned char vco);
extern unsigned int FSK_FAST_SPI_calc(int Freq_dev);
extern void FSK_FAST_SPI(double FMout);

/* ========================================================================== */
/*                           MACRO DEFINTIONS                                 */
/* ========================================================================== */

#define NUM_BUFS                    1          /* Max of buffers used in each direction */
#define NUM_OF_ENABLED_CHANNELS      1          /* Number of slots to be used     */
#define NUM_OF_MAX_CHANNELS	       1			/* Maximum number of time slots available based on clock settings   */
/* Defines the core number responsible for system initialization. */
#define CORE_SYS_INIT         0
/* Number of MCBSP Frame structures used for submit channel */
#define NUM_OF_MCBSP_FRAMES 1

 Int FRAMESIZE=RPE_DATA_SIZE/2*15;          /* e.g 10 ms 24 KHz samples         */
 Int BUFSIZE=NUM_OF_ENABLED_CHANNELS *RPE_DATA_SIZE/2*15;



/*============================================================================*/
/*                            GLOBAL VARIABLES                                */
/*============================================================================*/

/* Shared Memory Variable to ensure synchronizing MCBSP initialization
 * with all the other cores. */
/* Created an array to pad the cache line with MCBSP_CACHE_LENGTH size */
#pragma DATA_ALIGN   (isMCBSPInitialized, MCBSP_MAX_CACHE_ALIGN)
#pragma DATA_SECTION (isMCBSPInitialized, ".mcbspSharedMem");
volatile Uint32 isMCBSPInitialized[(MCBSP_CACHE_LENGTH / sizeof(Uint32))] = { 0 };

/* Handle to the EDMA driver instance */
#pragma DATA_ALIGN   (hEdma, MCBSP_MAX_CACHE_ALIGN)
EDMA3_DRV_Handle hEdma[(MCBSP_CACHE_LENGTH / sizeof(EDMA3_DRV_Handle))] = { NULL };

/* Handle to MCBSP driver instance */
typedef void* Mcbsp_DevHandle;
Mcbsp_DevHandle  hMcbspDev;

/* Handle to MCBSP driver channel instance */
typedef void* Mcbsp_ChanHandle;
Mcbsp_ChanHandle  hMcbspTxChan;
Mcbsp_ChanHandle  hMcbspRxChan;

/* Core Number Identifier */
UInt32 coreNum = 0xFFFF;

/* Array to hold the pointer to the allocated buffers     */
void* bufRx[NUM_BUFS];
void* bufTx[NUM_BUFS];

#ifdef MCBSP_LOOP_PING_PONG
//#define INIT_SUBMIT_Q_CNT 2
/* Ping pong buffers used to submit to Mcbsp lld, which will be used in a loop */
void* bufRxPingPong[2];
void* bufTxPingPong[2];

int TxpingPongIndex,RxpingPongIndex;
#endif
/* Global Error call back function prototype */
void mcbsp_GblErrCallback(uint32_t chanHandle,uint32_t spcr_read,uint32_t Arg3);

/* Variables to indicate status of EDMA TX and RX requests */
volatile uint32_t edmaTxDone = 0;
volatile uint32_t edmaRxDone = 0;
/* Global variables to track number of buffers submitted, number of iterations, error counts etc */
int rxSubmitCount=0, txSubmitCount=0;
uint32_t num_iterations=0;
uint32_t num_rx_Call_backs=0, num_tx_Call_backs=0, dummy_call_backs=0;
uint32_t rxunderflowcnt=0, txunderflowcnt=0;
uint32_t errBuffCount=0;
/* Debug variables */
volatile int debugVar=1;  /* This can be used to maintain a while loop, If set to 0 will exit loop */


/**
 * \brief    Mcbsp Sample rate generator default parameters.
 *
 */
Mcbsp_srgConfig mcbspSrgCfg =
{
    FALSE,                     /* No gsync to be used as input is not CLKS    */
    Mcbsp_ClkSPol_RISING_EDGE, /* Dont care as input clock is not clks        */
    Mcbsp_SrgClk_CLKS,       /* McBSP internal clock to be used             */
    288000000,                 /* Mcbsp internal clock frequency(PLL-SYSCLK6) */
    3                          /* frame sync pulse width (val+1) is used      */
};

/**
 * \brief    Mcbsp device creation default parameters.
 *
 */
const Mcbsp_Params Mcbsp_PARAMS =
{
    Mcbsp_DevMode_McBSP,       /* Use the device as MCBSP                     */
    Mcbsp_OpMode_DMAINTERRUPT, /* Use DMA mode of operation                   */
    TRUE,                      /* cache coherency taken care of by driver     */
    Mcbsp_EmuMode_FREE,        /* Emulation mode free is to be enabled        */
    Mcbsp_Loopback_DISABLE,     /* Loop back mode enabled                      */
    &mcbspSrgCfg,              /* sample rate generator configuration         */
    NULL,                      /* TX pending buffer queue from application    */
    NULL,                      /* TX floating buffer queue in DMA             */
    NULL,                      /* RX pending buffer queue from application    */
    NULL                       /* RX floating buffer queue in DMA             */
};


#pragma DATA_ALIGN(loopTxJob, MCBSP_MAX_CACHE_ALIGN)
static Int32 loopTxJob[16] = {
    /* Filling with Mu law silence pattern : Can be any user defined pattern */
    0x88888888, 0x88888888, 0x88888888, 0x88888888, 0x88888888, 0x88888888, 0x88888888, 0x88888888,
    0x88888888, 0x88888888, 0x88888888, 0x88888888, 0x88888888, 0x88888888, 0x88888888, 0x88888888
};
#pragma DATA_ALIGN(loopRxJob, MCBSP_MAX_CACHE_ALIGN)
static Int32 loopRxJob[16] = {
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0
};

/**< settings to configure the TX or RX hardware sections                 */
Mcbsp_DataConfig mcbspChanConfigTx =
{
    Mcbsp_Phase_SINGLE,
    Mcbsp_WordLength_24,
    Mcbsp_WordLength_8,    /* Dont care for single phase*/
    NUM_OF_MAX_CHANNELS,
    NUM_OF_MAX_CHANNELS,      // Only used with dual phase
    Mcbsp_FrmSync_IGNORE,
    Mcbsp_DataDelay_0_BIT,
    Mcbsp_Compand_OFF_MSB_FIRST,
    Mcbsp_BitReversal_DISABLE,
    Mcbsp_IntMode_ON_READY,
    Mcbsp_RxJust_RZF,  /* Dont care for TX         */
    Mcbsp_DxEna_OFF
};

/**< settings to configure the TX or RX hardware sections                 */
Mcbsp_DataConfig mcbspChanConfigRx =
{
    Mcbsp_Phase_SINGLE,
    Mcbsp_WordLength_24,
    Mcbsp_WordLength_8,    /* Dont care for single phase*/
    NUM_OF_MAX_CHANNELS,
    NUM_OF_MAX_CHANNELS,      // Only used with dual phase
    Mcbsp_FrmSync_IGNORE,
    Mcbsp_DataDelay_0_BIT,
    Mcbsp_Compand_OFF_MSB_FIRST,
    Mcbsp_BitReversal_DISABLE,
    Mcbsp_IntMode_ON_READY,
    Mcbsp_RxJust_RZF,  /* Dont care for TX         */
    Mcbsp_DxEna_OFF
};
/**< clock setup for the TX section                     */
Mcbsp_ClkSetup mcbspClkConfigTx =
{
    Mcbsp_FsClkMode_INTERNAL,
    107000,                   /* 8KHz                   */
    Mcbsp_TxRxClkMode_INTERNAL,
    Mcbsp_FsPol_ACTIVE_LOW,
    Mcbsp_ClkPol_FALLING_EDGE
};

/**< clock setup for the RX section                     */
Mcbsp_ClkSetup mcbspClkConfigRx =
{
    Mcbsp_FsClkMode_INTERNAL,
    107000,                   /* 8KHz                   */
    Mcbsp_TxRxClkMode_INTERNAL,
    Mcbsp_FsPol_ACTIVE_LOW,
    Mcbsp_ClkPol_RISING_EDGE
};

/**< Multi channel setup                                                      */
Mcbsp_McrSetup mcbspMultiChanCtrl =
{
    Mcbsp_McmMode_ALL_CHAN_ENABLED_UNMASKED,
    Mcbsp_PartitionMode_CHAN_0_15,
    Mcbsp_PartitionMode_CHAN_16_31,
    Mcbsp_PartitionMode_2
};


Mcbsp_ChanParams mcbspChanparamTx =
{
    Mcbsp_WordLength_24,  /* wordlength configured    */
    &loopTxJob[0],          /* loop job buffer internal */
    8,                    /* user loopjob length      */
    mcbsp_GblErrCallback, /* global error callback    */
    NULL,                 /* edma Handle              */
    1,                    /* EDMA event queue         */
    8,                    /* hwi number               */
    Mcbsp_BufferFormat_1SLOT,
    TRUE,                 /* FIFO mode enabled        */
    &mcbspChanConfigTx,   /* channel configuration    */
    &mcbspClkConfigTx,    /* clock configuration      */
    &mcbspMultiChanCtrl,  /* multi channel control    */
    0x00,  /* Enabled timeslots: 0, 4, 5, 8, 10, 12, 13, 14 */
    0x01
};

Mcbsp_ChanParams mcbspChanparamRx =
{
    Mcbsp_WordLength_24,  /* wordlength configured    */
    &loopRxJob[0],          /* loop job buffer internal */
    8,                    /* user loopjob length      */
    mcbsp_GblErrCallback, /* global error callback    */
    NULL,                 /* edma Handle              */
    1,                    /* EDMA event queue         */
    8,                    /* hwi number               */
    Mcbsp_BufferFormat_1SLOT,
    TRUE,                 /* FIFO mode enabled        */
    &mcbspChanConfigRx,   /* channel configuration    */
    &mcbspClkConfigRx,    /* clock configuration      */
    &mcbspMultiChanCtrl,  /* multi channel control    */
    0x00,  /* Enabled timeslots: 0, 4, 5, 8, 10, 12, 13, 14 */
    0x01
};

/* ========================================================================== */
/*                           FUNCTION DEFINITIONS                             */
/* ========================================================================== */

/*
 *   This is the application's callback function. The driver will
 *   call this function whenever an EDMA I/O operation is over.
 *
 */
void mcbspAppCallback(void* arg, Mcbsp_IOBuf *ioBuf)
{
    int32_t mode;
    int32_t *pmode = (int32_t *)arg;
//    static unsigned int flag=0;
    mode = *pmode;
    if (mode == MCBSP_MODE_OUTPUT)
    {
//    	if(ioBuf) {
//            num_tx_Call_backs++;
//            edmaTxDone = 1;
//        }else
//            txunderflowcnt++;
    	memcpy((unsigned char*)bufTxPingPong[TxpingPongIndex],buf_transmit,BUFSIZE);
    	TxpingPongIndex=(TxpingPongIndex)?0:1;
    	if(1==tx_flag)
    		Semaphore_post(sem2);
    }
    else if (mode == MCBSP_MODE_INPUT)
    {
//        if(ioBuf) {
//            num_rx_Call_backs++;
//            edmaRxDone = 1;
//        }else
//            rxunderflowcnt++;
//        memcpy(buf_adc,(unsigned char*)bufRxPingPong[RxpingPongIndex],REC_BUFSIZE);
        RxpingPongIndex=(RxpingPongIndex)?0:1;
        if(1==rx_flag)
        	Semaphore_post(sem1);
    }else
        dummy_call_backs++;
    return;
}

/*
 *   This is the application's Global error callback function 
 */
void mcbsp_GblErrCallback(uint32_t chanHandle,uint32_t spcr_read,uint32_t Arg3)
{
    System_printf ("Debug(Core %d): ERROR callback called SPCR: %x", coreNum, spcr_read);
}

/*
 * \brief   This function demostrates the use of Mcbsp using digital loopback
 *          communication setup.
 *
 * \param   None
 *
 * \return  None
 */
void _task_mcbsp(void)
{
    /**< Mcbsp device params                                                  */
    Mcbsp_Params  mcbspParams;

    /**< Queue to hold the pending packets received from the application      */
    Queue_Struct  txQueuePendingList, rxQueuePendingList;
    /**< Queue to manage floating packets in DMA                              */
    Queue_Struct  txQueueFloatingList, rxQueueFloatingList;

    uint32_t count   = 0, tempCount = 0;
    int32_t  status  = 0;
    int32_t  txChanMode = MCBSP_MODE_OUTPUT;
    int32_t  rxChanMode = MCBSP_MODE_INPUT;
//    uint32_t mcbspTxDone = 0, mcbspRxDone = 0;
    Mcbsp_IOBuf txFrame[NUM_OF_MCBSP_FRAMES], rxFrame[NUM_OF_MCBSP_FRAMES];
    int txFrameIndex=0, rxFrameIndex=0;
//    int init_count=0;
//    uint32_t tempdata=0;
#ifdef MCBSP_LOOP_PING_PONG
    TxpingPongIndex=RxpingPongIndex=0;
#endif
    /* Initialize the OSAL */
    if (Osal_dataBufferInitMemory(BUFSIZE) < 0)
    {
        System_printf ("Debug(Core %d): Error: Unable to initialize the OSAL. \n", coreNum);
        return;
    }

    /* update EDMA3 handle to channel parameters */
    mcbspChanparamTx.edmaHandle = hEdma[0];
    mcbspChanparamRx.edmaHandle = hEdma[0];

    /* create the pending and floating queue for the TX channel           */
    Queue_construct(&txQueuePendingList, NULL);
    Queue_construct(&txQueueFloatingList, NULL);

    /* create the pending and floating queue for the RX channel           */
    Queue_construct(&rxQueuePendingList, NULL);
    Queue_construct(&rxQueueFloatingList, NULL);


    mcbspParams                 = Mcbsp_PARAMS;
    mcbspParams.txQPendingList  = &txQueuePendingList;
    mcbspParams.txQFloatingList = &txQueueFloatingList;
    mcbspParams.rxQPendingList  = &rxQueuePendingList;
    mcbspParams.rxQFloatingList = &rxQueueFloatingList;


    /* Bind the driver instance with device instance */
    status = mcbspBindDev(&hMcbspDev, 1, &mcbspParams);

    if (status != MCBSP_STATUS_COMPLETED)
    {
        System_printf ("Debug(Core %d): MCBSP LLD Bind Device Failed\n", coreNum);
        return;
    }

    /* If the user loopjob buffer is in local L2 memory: Convert into Global memory address space */
    if(mcbspChanparamRx.userLoopJobBuffer)
        mcbspChanparamRx.userLoopJobBuffer = (void *)Mcbsp_osalLocal2Global(mcbspChanparamRx.userLoopJobBuffer);

    /* Create a RX channel for receiving */
    status = mcbspCreateChan(&hMcbspRxChan, hMcbspDev, MCBSP_MODE_INPUT, &mcbspChanparamRx, mcbspAppCallback, &rxChanMode);
    if (MCBSP_STATUS_COMPLETED != status)
    {
        System_printf ("Debug(Core %d): Error: Create Channel (RX) failed\n", coreNum);
        return;
    }

    /* If the user loopjob buffer is in local L2 memory: Convert into Global memory address space */
    if(mcbspChanparamTx.userLoopJobBuffer)
        mcbspChanparamTx.userLoopJobBuffer = Mcbsp_osalLocal2Global(mcbspChanparamTx.userLoopJobBuffer);

    /* Create a TX channel for the transmission */
    status = mcbspCreateChan(&hMcbspTxChan, hMcbspDev, MCBSP_MODE_OUTPUT, &mcbspChanparamTx, mcbspAppCallback, &txChanMode);
    if (MCBSP_STATUS_COMPLETED != status)
    {
        System_printf ("Debug(Core %d): Error: Create Channel (TX) failed\n", coreNum);
        return;
    }

    /* create the buffers required for the TX and RX operations */

#ifdef MCBSP_LOOP_PING_PONG
    /* create the ping pong buffers required for the TX and RX operations */
    for (count = 0; count < (NUM_BUFS+1); count++)
    {
        bufRxPingPong[count] = (uint8_t *)Osal_mcbspDataBufferMalloc(REC_BUFSIZE);
        bufTxPingPong[count] = (uint8_t *)Osal_mcbspDataBufferMalloc(BUFSIZE);

        if (bufTxPingPong[count] == NULL)
        {
            System_printf ("Debug(Core %d): Error: Tx Ping pong Buffer (%d) Memory Allocation Failed\n", coreNum, count);
            return;
        }
        if (bufRxPingPong[count] == NULL)
        {
            System_printf ("Debug(Core %d): Error: Rx Ping pong Buffer (%d) Memory Allocation Failed\n", coreNum, count);
            return;
        }
    }
#else
    for (count = 0; count < (NUM_BUFS); count++)
    {
        bufTx[count] = (uint8_t *)Osal_mcbspDataBufferMalloc(BUFSIZE);
        bufRx[count] = (uint8_t *)Osal_mcbspDataBufferMalloc(REC_BUFSIZE);

        if (bufTx[count] == NULL)
        {
            System_printf ("Debug(Core %d): Error: Tx Buffer (%d) Memory Allocation Failed\n", coreNum, count);
            return;
        }
        if (bufRx[count] == NULL)
        {
            System_printf ("Debug(Core %d): Error: Rx Buffer (%d) Memory Allocation Failed\n", coreNum, count);
            return;
        }
    }
#endif


    txFrameIndex=0;
    rxFrameIndex=0;
//    init_count=0;
    reg_24 reg_24data;
    /* Fill the buffers with known data and transmit the same and
       check if the same pattern is received */
    for (count = 0; count < (NUM_BUFS); count++)
    {
        memset((uint8_t *)bufTxPingPong[count], 0, BUFSIZE);
        LMX2571_FM_CAL( 0,  206.75, 0);//415.125 	156.525;
        for (tempCount = 0; tempCount < 162; tempCount++){
        		reg_24data.all=lmx_init[tempCount/3];
               	((uint8_t *)bufTxPingPong[count])[tempCount++] = reg_24data.dataBit.data0;
               	((uint8_t *)bufTxPingPong[count])[tempCount++] = reg_24data.dataBit.data1;
               	((uint8_t *)bufTxPingPong[count])[tempCount]   = reg_24data.dataBit.data2;
        }
        for (tempCount = 162; tempCount < BUFSIZE; tempCount++){
        	((uint8_t *)bufTxPingPong[count])[tempCount++] =0x80;
        	((uint8_t *)bufTxPingPong[count])[tempCount++] =0x80;
        	((uint8_t *)bufTxPingPong[count])[tempCount]   = 0x80;
        }
    }
    memset(buf_transmit,0x80, BUFSIZE);
    memcpy((unsigned char*)bufTxPingPong[1],buf_transmit,BUFSIZE);

    /* Start main loop to iterate through frames */
//    while(debugVar)
//    {
        /* submit frames to the driver */
        for (count = 0; count < NUM_BUFS+1; count++)
        {
            /* RX frame processing */
            rxFrame[rxFrameIndex].cmd = Mcbsp_IOBuf_Cmd_READ;
#ifdef	MCBSP_LOOP_PING_PONG
            memset((uint8_t *)bufRxPingPong[count], 0, REC_BUFSIZE);
            rxFrame[rxFrameIndex].addr = (void*)bufRxPingPong[count];
#else
            memset((uint8_t *)bufRx[count], 0, REC_BUFSIZE);
            rxFrame[rxFrameIndex].addr = (void*)bufRx[count];
#endif
            rxFrame[rxFrameIndex].size = REC_BUFSIZE;
            rxFrame[rxFrameIndex].arg = (uint32_t) hMcbspRxChan;
            rxFrame[rxFrameIndex].status = MCBSP_STATUS_COMPLETED;
            rxFrame[rxFrameIndex].misc = 1;   /* reserved - used in callback to indicate asynch packet */

            status = mcbspSubmitChan(hMcbspRxChan, (void *)&rxFrame[rxFrameIndex]);
            if (status != MCBSP_STATUS_PENDING)
            {
                System_printf ("Debug(Core %d): Error: RX buffer #%d submission FAILED\n", coreNum, count);
            }
            rxFrameIndex++;
            rxFrameIndex = (rxFrameIndex >= (NUM_OF_MCBSP_FRAMES)) ? 0 : rxFrameIndex;
            rxSubmitCount++;


            txFrame[txFrameIndex].cmd = Mcbsp_IOBuf_Cmd_WRITE;
#ifdef	MCBSP_LOOP_PING_PONG
            txFrame[txFrameIndex].addr = (void*)bufTxPingPong[count];
#else
            txFrame[txFrameIndex].addr = (void*)bufTx[count];
#endif
            txFrame[txFrameIndex].size = BUFSIZE;
            txFrame[txFrameIndex].arg = (uint32_t)hMcbspTxChan;
            txFrame[txFrameIndex].status = MCBSP_STATUS_COMPLETED;
            txFrame[txFrameIndex].misc = 1;   /* reserved - used in callback to indicate asynch packet */

            status = mcbspSubmitChan(hMcbspTxChan, (void *)&txFrame[txFrameIndex]);
            if (status != MCBSP_STATUS_PENDING)
            {
                System_printf ("Debug(Core %d): Error: TX buffer  #%d submission FAILED\n", coreNum, count);
            }
            txFrameIndex++;
            txFrameIndex = (txFrameIndex >= (NUM_OF_MCBSP_FRAMES)) ? 0 : txFrameIndex;
            txSubmitCount++;


            }
//     }// end of submit
    	while (1){
    		Task_sleep(50);
    	}
//    return;
}

/*
 * \brief  Void main(Void)
 *
 *         Main function of the sample application. This function calls the
 *         function to configure the mcbsp instance.
 *
 * \param  None
 *
 * \return None
 */


/*psc enable */
void psc_init(void)
{

  CSL_PscRegsOvly psc1Regs = (CSL_PscRegsOvly)CSL_PSC_1_REGS;

  // deassert UART local PSC reset and set NEXT state to ENABLE
  psc1Regs->MDCTL[CSL_PSC_MCBSP1] = CSL_FMKT( PSC_MDCTL_NEXT, ENABLE )
                               | CSL_FMKT( PSC_MDCTL_LRST, DEASSERT );
  // move UART PSC to Next state
  psc1Regs->PTCMD = CSL_FMKT(  PSC_PTCMD_GO0, SET );

  // wait for transition
  while ( CSL_FEXT( psc1Regs->MDSTAT[CSL_PSC_MCBSP1], PSC_MDSTAT_STATE )
          != CSL_PSC_MDSTAT_STATE_ENABLE );

}


/*
 *  ======== task_mcbsp ========
 */


Void task_mcbsp(UArg arg0, UArg arg1)
{
	log_info("-->task_mcbsp:");
    EDMA3_DRV_Result edmaResult = 0;

    /* Get the core number. */
    coreNum = 0; //CSL_chipReadReg (CSL_CHIP_DNUM);
    psc_init();

    /* Initialize the system only if the core was configured to do so. */
    if (coreNum == CORE_SYS_INIT)
    {
        /* MCBSP Driver Initialization: This should always be called before
         * invoking the MCBSP Driver. */
        mcbspInit();

        /* Device Specific MCBSP Initializations */
        McbspDevice_init();
        
        /* MCBSP Driver is operational at this time. */
        System_printf ("Debug(Core %d): MCBSP Driver Initialization Done\n", coreNum);

        /* Write to the SHARED memory location at this point in time. The other cores cannot execute
         * till the MCBSP Driver is up and running. */
        isMCBSPInitialized[1] = 1;

        /* The MCBSP IP block has been initialized. We need to writeback the cache here because it will
         * ensure that the rest of the cores which are waiting for MCBSP to be initialized would now be
         * woken up. */
        //CACHE_wbL1d ((void *) &isMCBSPInitialized[0], MCBSP_MAX_CACHE_ALIGN, CACHE_FENCE_WAIT);
        Cache_wb ((void *) &isMCBSPInitialized[1], MCBSP_CACHE_LENGTH,0x7fff, 1);
    }
    else
    {
        /* All other cores need to wait for the MCBSP to be initialized before they proceed with the test. */ 
        System_printf ("Debug(Core %d): Waiting for MCBSP to be initialized.\n", coreNum);

        /* All other cores loop around forever till the MCBSP is up and running. 
         * We need to invalidate the cache so that we always read this from the memory. */
        while (isMCBSPInitialized[1] == 0)
            //CACHE_invL1d ((void *) &isMCBSPInitialized[0], MCBSP_MAX_CACHE_ALIGN, CACHE_FENCE_WAIT);
        	Cache_inv ((void *) &isMCBSPInitialized[1], MCBSP_CACHE_LENGTH, 0x7fff, 1);

        System_printf ("Debug(Core %d): MCBSP can now be used.\n", coreNum);
    }

    /* Initialize EDMA3 library */
    hEdma[0] = edma3init(0, &edmaResult);

    if (edmaResult != EDMA3_DRV_SOK)
    {
        /* Report EDMA Error */
        System_printf("Debug(Core %d): EDMA Driver Initialization FAILED\n", coreNum);
    }
    else
    {
        System_printf("Debug(Core %d): EDMA Driver Initialization Done\n", coreNum);
    }

    _task_mcbsp();
}



/* ========================================================================== */
/*                                END OF FILE                                 */
/* ========================================================================== */
