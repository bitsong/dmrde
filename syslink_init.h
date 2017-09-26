#ifndef __SYSLINK_INIT_H
#define __SYSLINK_INIT_H
#include "syslink_prepare_common.h"
#include "syslink_sync.h"
#include "msgq_init.h"
#include "rpe_init.h"
#include "ntf_init.h"

#ifdef __cplusplus
extern "C"{
#endif

extern syslink_t slk;

int slk_prepare(void);
int slk_cleanup(void);

int syslink_prepare(void);
int syslink_cleanup(void);

#ifdef __cplusplus
}
#endif



#endif
