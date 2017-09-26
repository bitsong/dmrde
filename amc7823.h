#ifndef	__AMC7823_H__
#define __AMC7823_H__


#if 1
/* AMC 1 parameter data */
#define    AMC1_TYPE          AMC7823
#define	   AMC1_VOLTAGE		  (2.5f)
#define    AMC1_RESOLUTION	  (4096)
#define    AMC1_SPIPORT       (0)
#define    AMC1_SPIREGS       (0x7040)
#define    AMC1_BAUDRATE      (3)
#define    AMC1_ADCCON        (0x8080)
#define    AMC1_LOADDAC       (0x0000)
#define    AMC1_DACCON        (0x00FF)
#define    AMC1_GPIO          (0xFFFF)
#define    AMC1_PWRD          (0xFFA0)
#define    AMC1_STATCON       (0x0000)
#define    AMC1_TRL0          (0x0000)
#define    AMC1_TRL1          (0x0000)
#define    AMC1_TRL2          (0x0000)
#define    AMC1_TRL3          (0x0000)
#define    AMC1_TRH0          (0x0FFF)
#define    AMC1_TRH1          (0x0FFF)
#define    AMC1_TRH2          (0x0FFF)
#define    AMC1_TRH3          (0x0FFF)
#define    AMC1_RESET         (0x0000)
#define    AMC1_ALR           (0x0000)
#define    AMC1_TEM_RES		  (1.6f)
#else
/* AMC 1 parameter data */
#define    AMC1_TYPE          AMC7823
#define	   AMC1_VOLTAGE		  (5)
#define    AMC1_RESOLUTION	  (4096)
#define    AMC1_SPIPORT       (0)
#define    AMC1_SPIREGS       (0x7040)
#define    AMC1_BAUDRATE      (3)
#define    AMC1_ADCCON        (0x8080)
#define    AMC1_LOADDAC       (0x0000)
#define    AMC1_DACCON        (0x00FF)
#define    AMC1_GPIO          (0xFFFF)
#define    AMC1_PWRD          (0xFFA0)
#define    AMC1_STATCON       (0x0040)
#define    AMC1_TRL0          (0x0000)
#define    AMC1_TRL1          (0x0000)
#define    AMC1_TRL2          (0x0000)
#define    AMC1_TRL3          (0x0000)
#define    AMC1_TRH0          (0x0FFF)
#define    AMC1_TRH1          (0x0FFF)
#define    AMC1_TRH2          (0x0FFF)
#define    AMC1_TRH3          (0x0FFF)
#define    AMC1_RESET         (0x0000)
#define    AMC1_ALR           (0x0000)
#define    AMC1_TEM_RES		  (3.2f)
#endif

/*ERROR STATUS*/
#define NO_ERR	(0)

#define ERR_REGS1	(-1)
#define ERR_REGS2	(-2)
#define ERR_REGS3	(-3)
#define ERR_REGS4	(-4)

/* EXPORTED API  */
void configureSPI(void);
void Spi_dev_init();
int amc7823_init(void);

float adc_read(unsigned int ch);
void dac_write(unsigned int ch, float voltage);
float temperature_read();
float dac_read(unsigned int ch);

#endif//__amc7823_h_
