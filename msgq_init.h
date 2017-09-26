#ifndef __MSGQ_INIT_H
#define __MSGQ_INIT_H

#include "syslink_prepare_common.h"

#ifdef __cplusplus
extern "C"{
#endif

extern message_buffer_t *msgbuf[MBUF_MAX];
extern messageq_t 		msgq[MSG_MAX];

int msgq_init(messageq_t *msgq,const char *name,int havebox);
int msgq_destroy(messageq_t *msgq);

int message_buffer_prepare(void);
int message_buffer_cleanup(void);

int msgq_prepare(void);
int msgq_cleanup(void);

#ifdef __cplusplus
}
#endif

#endif
