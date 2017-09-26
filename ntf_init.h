#ifndef __NTF_INIT_H
#define __NTF_INIT_H

#include "syslink_prepare_common.h"

#ifdef __cplusplus
extern "C"{
#endif

extern notifier_t notify[NTF_MAX];

int notifier_prepare(void);
int notifier_cleanup(void);


#ifdef __cplusplus
}
#endif



#endif
