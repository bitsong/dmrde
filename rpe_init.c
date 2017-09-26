#include <string.h>
#include <stdio.h>
#include "rpe_init.h"
#include "syslink_prepare_common.h"

typedef struct rpe_init_params{
	//char 	 *name;
	unsigned int id;
	unsigned int option;
	unsigned int dsize;
	unsigned int asize;
}rpe_cpar_t;

struct rpe rpe[RPE_MAX];

#ifdef SYSLINUX

rpe_cpar_t param[RPE_MAX] = {
	[0] = {
		.id			= 0,
		.option 	= O_RW,
		.dsize 		= 20 * KB,
		.asize		= 1  * KB
	},
/*
	[1] = {
		.id			= 1
		.option		= ...
		.dsize		= ...
		.asize		= ...
	},
	...
*/
};

#else

rpe_cpar_t param[RPE_MAX] = {
	[0] = {
		.id			= 0,
		.option 	= O_RW,
		.dsize 		= 24000,
		.asize		= 1  * KB
	}
	/*
	,
	[1] = {
		.id			= 1,
		.option		= O_RW,
		.dsize		= 10*KB,
		.asize		= 1*KB,
	}*/
};
#endif


static int rpes_init(struct rpe *rpe,const unsigned int id)
{
	char name[64];
	sprintf(name,"%s_%d",local_rpe_basenm,id);
	return rpe_construct(rpe,name,NULL);
}

/*
	id���Ҫ�򿪵�rpe��id
*/
static int rpes_open(struct rpe *rpe,rpe_cpar_t *par,const int id)
{
	if(!rpe)
		return -1;
	
	if(par->option & O_IGN)
		return -2;
	
	
	int 			rc = 0;
	int 			timeout;
	char 			name[64];
	rpe_params_t 	rpepar;
	
	
	//�����Ҫ��д�˿ڡ�
	if(par->option & O_WR){
	
		rpepar.ctrlsr	= 3;
		rpepar.datasr	= 3;
		rpepar.attrsr	= 3;
		rpepar.dsize	= par->dsize;
		rpepar.asize	= par->asize;
		rpepar.rpid		= syslink_getprocid(remote_proc_name);
		
		if(!par->dsize ||!par->asize)
			log_warn("ring buffer or attr size is 0!");
		
		rc = rpe_open(rpe,RPE_ENDPOINT_WRITER,&rpepar);
		if(rc < 0)
			goto out;
	}
	
	//�����Ҫ�򿪶��˿�.
	if(par->option & O_RD){
		
		timeout = TRYTIMES;
		sprintf(name,"%s_%d",remote_rpe_basenm,id);
		while(timeout-- && (rc = rpe_open(rpe,RPE_ENDPOINT_READER,name)) < 0){
//			log_info("try the %dth time to open remote rpe failed",TRYTIMES - timeout);
			msleep(TYRINTERVAL);
		}
		
		if(rc < 0)
			rpe_close(rpe,RPE_ENDPOINT_WRITER);
	}

out:	
	return rc;
}

static int rpes_destroy(struct rpe *rpe)
{
	return rpe_destruct(rpe);
}

/*
	����ֵ���ɹ���ʼ���ĸ���
	Ȼ��֮Ϊ0��ʶ��ʼ��ʧ�ܡ�
*/
int rpe_prepare(void)
{
	int i;
	int rc;
	int failcnt = 0;
	
	for(i = 0; i < RPE_MAX; i++){
		if((rc = rpes_init(&rpe[i],param[i].id)) < 0){
			log_warn("rpe%d init failed:rc = %d",i,rc);
			failcnt++;
			param[i].option |= O_IGN;
			continue;
		}
//		log_info("rpe%d init done!",i);
		
		if((rc = rpes_open(&rpe[i],&param[i],i)) < 0){
			log_warn("rpe%d connect failed:rc = %d",i,rc);
			rpes_destroy(&rpe[i]);
			failcnt++;
			param[i].option |= O_IGN;
		}
//		log_info("rpe%d open done!",i);
	}
	
	return (RPE_MAX - failcnt);
}

int rpe_cleanup(void)
{
	int i;
	int rc;
	
	for(i = 0; i < RPE_MAX; i++){
		
		if(param[i].option & O_IGN){
			param[i].option &= ~O_IGN;
			continue;
		}
	
		if((rc = rpes_destroy(&rpe[i])) < 0){
			log_warn("rpe destroy failed,rc = %d,id = %u",i,param[i].id);
			return rc;
		}
		
	}

    return 0;
}
