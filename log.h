#ifndef _LOG_H_
#define _LOG_H_

/* 0: printf; 1: UART */
#define OUTPUT_TO_UART1 0

/* Switch Log Output */
#if OUTPUT_TO_UART1
    /* For UART1 Initial */
	extern unsigned int UARTPuts(char *pTxBuffer, int numBytesToWrite);
	extern unsigned int UARTGets(char *pRxBuffer, int numBytesToRead);
	extern unsigned int UARTwrite(const char *pcBuf, unsigned int len);
	extern void UARTprintf(const char *pcString, ...);
	extern void UARTPutHexNum(unsigned int hexValue);
	extern void UARTPutc(unsigned char byteTx);
	extern unsigned int UARTGetHexNum(void);
	extern unsigned char UARTGetc(void);
	extern void UARTPutNum(int value);
	extern void UARTStdioInit(void);
	extern int UARTGetNum(void);
    #define OUTPUT UARTprintf
#else
    #include <stdio.h>
    #include <stdbool.h>

    #define OUTPUT printf
#endif

/* Log init */
#if OUTPUT_TO_UART1
    #define log_init() do {                                     \
        UARTStdioInit();                                        \
    } while (0)
#else
    #define log_init() do { } while (0)
#endif

/* Log Output */
#define log_info(format, ...)      \
    do {                           \
        OUTPUT("[%s|%s@%s,%d] " format "\n", "INFO ", __func__, __FILE__, __LINE__, ##__VA_ARGS__ ); \
    } while (0)

#define log_error(format, ...)     \
    do {                           \
        OUTPUT("[%s|%s@%s,%d] " format "\n", "ERROR", __func__, __FILE__, __LINE__, ##__VA_ARGS__ ); \
    } while (0)

#define log_debug(format, ...)     \
    do {                           \
        OUTPUT("[%s|%s@%s,%d] " format "\n", "DEBUG", __func__, __FILE__, __LINE__, ##__VA_ARGS__ ); \
    } while (0)

#define log_warn(format, ...)      \
    do {                           \
        OUTPUT("[%s|%s@%s,%d] " format "\n", "WARN ", __func__, __FILE__, __LINE__, ##__VA_ARGS__ ); \
    } while (0)

#define log_trace(format, ...)     \
    do {                           \
        OUTPUT("[%s|%s@%s,%d] " format "\n", "TRACE", __func__, __FILE__, __LINE__, ##__VA_ARGS__ ); \
    } while (0)

#define log_fatal(format, ...)     \
    do {                           \
        OUTPUT("[%s|%s@%s,%d] " format "\n", "FATAL", __func__, __FILE__, __LINE__, ##__VA_ARGS__ ); \
        exit(1);                   \
    } while (0)

/* Assert */
#define assert(EXP)                \
    do{                            \
        if (!(EXP)) {              \
            OUTPUT("[%s@%s,%d] ASSERT: " #EXP "\n", __func__, __FILE__, __LINE__ );                  \
            exit(1);               \
        }                          \
    } while(0)

#endif
