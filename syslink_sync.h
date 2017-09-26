#ifndef __SYNC_H
#define __SYNC_H

#include "syslink_prepare_common.h"

#ifdef __cplusplus
extern "C"{
#endif

int sync_prepare(void);
int sync_cleanup(void);
int sync_send(unsigned int code);
int sync_wait(unsigned int *code,unsigned int timeout,int times);
int sync_wait_for(unsigned int code);
int sync_waitfor_prepare(void);
int sync_waitfor_cleanup(void);

#ifdef __cplusplus
}
#endif

#endif
