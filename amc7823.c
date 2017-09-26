/*
================================================================================
                            INLCUDE FILES
================================================================================
*/

#include <stdio.h>
#include <ti/csl/soc_C6748.h>
#include <ti/csl/cslr_spi.h>
#include <ti/csl/cslr_psc_C6748.h>

#include <ti/csl/cslr_syscfg0_OMAPL138.h>
#include <ti/csl/cslr_gpio.h>

#include"amc7823.h"

#define SPI_NUM_OF_TXBITS    16

CSL_SpiRegsOvly  spiRegs=(CSL_SpiRegsOvly)CSL_SPI_1_REGS;
//CSL_GpioRegsOvly     gpioRegs = (CSL_GpioRegsOvly)(CSL_GPIO_0_REGS);
unsigned short test_data=0;
/*
================================================================================
                         LOCAL FUNCTION PROTOTYPES
================================================================================
*/

//void Spi_dev_init();
void amc_write(unsigned short page, unsigned short addr,unsigned short data);
unsigned short amc_read(unsigned short page, unsigned short addr);

//int amc7823_init(void);
//static void Spi_test();
//void configureSPI(void);

/*------------------------------------------------------------------------------
* static void Spi_dev_init(void)
------------------------------------------------------------------------------*/
/*
*
*  @Param1 : None
*  @RetVal : None
*  @see    : refer main()
*
*  @Description :This function initialises the SPI interface in the loopback
*                mode with 8 bit transmission and all timer settings as default.
*/
void Spi_dev_init(void)
{
	//PSC
    CSL_PscRegsOvly psc1Regs = (CSL_PscRegsOvly)CSL_PSC_1_REGS;
    /* deassert SPI1 local PSC reset and set NEXT state to ENABLE             */
    psc1Regs->MDCTL[CSL_PSC_SPI1] = CSL_FMKT( PSC_MDCTL_NEXT, ENABLE )
                                 | CSL_FMKT( PSC_MDCTL_LRST, DEASSERT );
    /* Move SPI1 PSC to Next state                                            */
    psc1Regs->PTCMD = CSL_FMKT(  PSC_PTCMD_GO0, SET );
    /* Wait for transition                                                    */
    while ( CSL_FEXT( psc1Regs->MDSTAT[CSL_PSC_SPI1], PSC_MDSTAT_STATE )
            != CSL_PSC_MDSTAT_STATE_ENABLE );
    /* First reset the SPI chip                                               */
    spiRegs->SPIGCR0 = CSL_FMK(SPI_SPIGCR0_RESET,
                                            CSL_SPI_SPIGCR0_RESET_IN_RESET);
    /* now bring the chip out of reset state                                  */
    spiRegs->SPIGCR0 = CSL_FMK(SPI_SPIGCR0_RESET,
                                            CSL_SPI_SPIGCR0_RESET_OUT_OF_RESET);
    /* enable the CLKMOD and MASTER bits in the SPI global control reg        */
    spiRegs->SPIGCR1 |=  CSL_FMK( SPI_SPIGCR1_MASTER,0x01)
                                  | CSL_FMK(SPI_SPIGCR1_CLKMOD,0x01);

    //PINMUTIPLEX
     /* enable the pins so that they are used for the SPI interface(Multiplex) */
    spiRegs->SPIPC0  = CSL_FMK(SPI_SPIPC0_CLKFUN ,0x01)
                                  | CSL_FMK(SPI_SPIPC0_SOMIFUN ,0x01)
                                  | CSL_FMK(SPI_SPIPC0_SCS0FUN0,0x01)
                                  | CSL_FMK(SPI_SPIPC0_SCS0FUN1,0x01)
                                  | CSL_FMK(SPI_SPIPC0_SIMOFUN ,0x01);
    //
//    spiRegs->SPIPC1  = CSL_FMK(SPI_SPIPC1_SCS0DIR0 ,0x01)
//					| CSL_FMK(SPI_SPIPC1_SCS0DIR1 ,0x01);

    //
//    spiRegs->SPIDELAY = CSL_FMK(SPI_SPIDELAY_C2TDELAY,138u)
//    						|CSL_FMK(SPI_SPIDELAY_T2CDELAY,138u);

    /* configure the data format in SPIFMT                                    */
    spiRegs->SPIFMT[0] = CSL_FMK(SPI_SPIFMT_CHARLEN,SPI_NUM_OF_TXBITS)
                                  | CSL_FMK(SPI_SPIFMT_PRESCALE,0x31)
                                  | CSL_FMK(SPI_SPIFMT_PHASE,0x00)
                                  | CSL_FMK(SPI_SPIFMT_POLARITY,0x00)
//                                  | CSL_FMK(SPI_SPIFMT_DISCSTIMERS,0x01)
                                  | CSL_FMK(SPI_SPIFMT_SHIFTDIR,0x00);

    /* set the preconfigure data format as 0 which is already set above       */
    spiRegs->SPIDAT1 = CSL_FMKT(SPI_SPIDAT1_DFSEL,FORMAT0);

    /* dont use any interrupts hence disable them                             */
    spiRegs->SPIINT0 = CSL_FMKT(SPI_SPIINT0_RXINTENA,DISABLE);

    spiRegs->SPIGCR1 |=  CSL_FMK(SPI_SPIGCR1_ENABLE,0x01)
                                  | CSL_FMK(SPI_SPIGCR1_LOOPBACK,0x00);

}
/*------------------------------------------------------------------------------
* void main(void)
------------------------------------------------------------------------------*/
/*
*
*  @Param1 : None
*  @RetVal : None
*  @see    : None
*
*  @Description : This function is the main function for the SPI loop back test
*
*/
//void main(void)
//{
//	int status;
//	status=NO_ERR;
//
//    configureSPI();
//
//    Spi_dev_init();
//
//    status=amc7823_init();
//    if(status<0)
//    	printf("\namc7823 initial error:%d\n",status);
//    else
//    	printf("\namc7823 initial done.\n");
//
//    Spi_test();
//
//}

/*------------------------------------------------------------------------------
* static void Spi_test(void)
------------------------------------------------------------------------------*/
/*
*
*  @Param1 : None
*  @RetVal : None
*  @see    : refer main()
*
*  @Description : This function tests the SPI loop back interface by sending a
*                 string and comparing with the data recieved.
*
*/

//static void Spi_test(void)
//{
//	int i;
//	float vol;
//
////    while (1)
//    {
//    	for(i=0;i<0x20;i++){
//    		test_data=amc_read(0x00,i);
//    		printf("SPI  read data %x ......\n",test_data);
//    	}
//
//
//    	for(i=0;i<0x20;i++){
//    	    		test_data=amc_read(0x01,i);
//    	    		printf("SPI  read data %x ......\n",test_data);
//    	    	}
//
//    	for(i=0;i<8;i++)
//    	{
//    		vol=adc_read(i);
//    		printf("adc data is %f\n",vol);
//    		dac_write(i,1.25);
//    	}
//    	vol=temperature_read();
//		printf("temperature is %f\n",vol);
//    }
//}


void configureSPI(void)
{
    CSL_SyscfgRegsOvly syscfgRegs = (CSL_SyscfgRegsOvly)CSL_SYSCFG_0_REGS;

    syscfgRegs->PINMUX5 &= ~(CSL_SYSCFG_PINMUX5_PINMUX5_3_0_MASK |
    						 CSL_SYSCFG_PINMUX5_PINMUX5_7_4_MASK |
                             CSL_SYSCFG_PINMUX5_PINMUX5_11_8_MASK |
                             CSL_SYSCFG_PINMUX5_PINMUX5_19_16_MASK |
                             CSL_SYSCFG_PINMUX5_PINMUX5_23_20_MASK );
    syscfgRegs->PINMUX5   |= 0x00110111;
}

/*
================================================================================
                            END OF FILE
================================================================================
*/

void spi_write(unsigned short data)
{
	/* write the data to the transmit buffer                              */
    CSL_FINS(spiRegs->SPIDAT0,SPI_SPIDAT0_TXDATA,data);
    return;
}

unsigned short spi_read()
{
	unsigned short data;
	/* check if data is recieved                                          */
	while ((CSL_FEXT(spiRegs->SPIBUF,SPI_SPIBUF_RXEMPTY)) == 1);
	           data = (CSL_FEXT(spiRegs->SPIBUF,SPI_SPIBUF_RXDATA));
	return data;
}

unsigned short amc_read(unsigned short page, unsigned short addr)
{
	unsigned short command;
	command=addr|addr<<6|page<<12|0x8000;

	spiRegs->SPIDAT1=CSL_FMK(SPI_SPIDAT1_DFSEL,0)
			|CSL_FMK(SPI_SPIDAT1_CSHOLD,1)
			|CSL_FMK(SPI_SPIDAT1_CSNR,2)			//IF ADF4002 THEN CSNR :1
			|CSL_FMK(SPI_SPIDAT1_TXDATA,command);
	spi_read();
	spiRegs->SPIDAT1=CSL_FMK(SPI_SPIDAT1_DFSEL,0)
				|CSL_FMK(SPI_SPIDAT1_CSHOLD,0)
				|CSL_FMK(SPI_SPIDAT1_CSNR,2)
				|CSL_FMK(SPI_SPIDAT1_TXDATA,0);

	return (spi_read());
}

void amc_write(unsigned short page, unsigned short addr,unsigned short data)
{
	unsigned short command;
	command=addr|addr<<6|page<<12;

	spiRegs->SPIDAT1=CSL_FMK(SPI_SPIDAT1_DFSEL,0)
			|CSL_FMK(SPI_SPIDAT1_CSHOLD,1)
			|CSL_FMK(SPI_SPIDAT1_CSNR,2)
			|CSL_FMK(SPI_SPIDAT1_TXDATA,command);
	spi_read();
	spiRegs->SPIDAT1=CSL_FMK(SPI_SPIDAT1_DFSEL,0)
				|CSL_FMK(SPI_SPIDAT1_CSHOLD,0)
				|CSL_FMK(SPI_SPIDAT1_CSNR,2)
				|CSL_FMK(SPI_SPIDAT1_TXDATA,data);

	spi_read();
}

float adc_read(unsigned int ch)
{
	return AMC1_VOLTAGE*(0x0fff&amc_read(0,ch))/AMC1_RESOLUTION;
}
float dac_read(unsigned int ch)
{
	return AMC1_VOLTAGE*(0x0fff&amc_read(1,ch))/AMC1_RESOLUTION;
}
void dac_write(unsigned int ch, float voltage)
{
	amc_write(1,ch,voltage*AMC1_RESOLUTION/AMC1_VOLTAGE);
}

float temperature_read()
{
	return AMC1_TEM_RES*(0xfff&amc_read(00,8))-273;
}

int amc7823_init(void)
{
	unsigned short temp;

	//reset 
	amc_write(0x01,0x0c,0xbb30);
//
	//power down
	amc_write(0x01,0x0d,AMC1_PWRD);
	temp=amc_read(0x01,0x0d);
	if(temp!=AMC1_PWRD)
		return ERR_REGS1;

	//amc status/config
	amc_write(0x01,0x0a,AMC1_STATCON);
	temp=amc_read(0x01,0x0a);
	if(temp!=AMC1_STATCON)
		return ERR_REGS2;

	//adc config
	amc_write(0x01,0x0b,AMC1_ADCCON);
	temp=amc_read(0x01,0x0b);
	if(temp!=AMC1_ADCCON)
		return ERR_REGS3;

	//dac config
	amc_write(0x01,0x09,AMC1_DACCON);
	temp=amc_read(0x01,0x09);
	if(temp!=AMC1_DACCON)
		return ERR_REGS4;

	return NO_ERR;
}
