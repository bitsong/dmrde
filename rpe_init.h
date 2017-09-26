#ifndef __RPE_INIT_H
#define __RPE_INIT_H

#include "syslink_prepare_common.h"

#ifdef __cplusplus
extern "C"{
#endif

extern struct rpe rpe[RPE_MAX];

int rpe_prepare(void);
int rpe_cleanup(void);



#ifdef __cplusplus
}
#endif

#endif
