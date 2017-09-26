#include <stdio.h>
#include <string.h>
#include "ntf_init.h"

typedef struct ntf_init_param{
	unsigned int id;
	unsigned int option;
}ntf_cpar_t;

notifier_t notify[NTF_MAX];

int notifier_prepare(void)
{
	return 1;
}

int notifier_cleanup(void)
{
	return 0;
}
