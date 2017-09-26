#include <stdio.h>
#include <string.h>
#include "msgq_init.h"

//message buffer 
/*-----------------------------------------------------------------------------------------------------------------------------*/
//����msgbuf��ʹ��heapid����idֵ�����idȡֵ��Χ��0 - 7
typedef struct msgq_buffer_param{
	unsigned int id;						//attach and heap id(0 - 7)		
	unsigned int option;
	unsigned int size;
}msgq_bufcpar_t;

message_buffer_t *msgbuf[MBUF_MAX];

#ifdef SYSLINUX

static msgq_bufcpar_t   bufpar[MBUF_MAX] = {
	[0] = {
		.id 	= 0,
		.option = O_CBUF,
		.size 	= 10 * KB
	},
/*	
	[1] = {
		.id 	= 1,
		.option = O_CBUF,
		.size	= 10 * KB
	}
*/
};

#else	

static msgq_bufcpar_t   bufpar[MBUF_MAX] = {
	{
		0,		//id
		0,		//don't create message buffer
		0,		//buffer size
		
	},

	{
		1,		//id
		0,		//don't create message buffer
		0,		//buffer size
	}

};
#endif

int msgbuf_prepare(void)
{
	int i;
	int rc;
	message_buffer_params_t mpar;
	char name[MAX_NAME_LEN];
	int failcnt = 0;
	int timeout = TRYTIMES;
	
	for(i = 0; i < MBUF_MAX; i++){
		
		if(bufpar[i].option & O_IGN)
			continue;
		
		message_buffer_params_init(&mpar);
		mpar.heapid 	= bufpar[i].id;
		mpar.heaptype 	= HEAP_TYPE_MEMMP;
		
		if(bufpar[i].option & O_CBUF){
			
			sprintf(name,"%s_%d",local_buf_basenm,bufpar[i].id);
			mpar.memmp_name = name;
			mpar.memmp_rid 	= 1;
			mpar.memmp_size = bufpar[i].size;
			rc = message_buffer_create(&msgbuf[i],&mpar);
			
		}else{
			
			sprintf(name,"%s_%d",remote_buf_basenm,bufpar[i].id);
			mpar.memmp_name = name;
			while(timeout-- && (rc = message_buffer_open(&msgbuf[i],&mpar)) < 0){
				log_info("the %dth time to opening msgbuf:%d failed",TRYTIMES - timeout,bufpar[i].id);
				msleep(TYRINTERVAL);
			}
			
		}
		
		if(rc < 0){
			failcnt++;
			bufpar[i].option |= O_IGN;
			log_warn("message buffer create or open failed,rc = %d,id = %u",rc,bufpar[i].id);
		}
	}
	
	return MBUF_MAX - failcnt;
}

int msgbuf_cleanup(void)
{
	int i;
	int rc;
	
	for(i = 0; i < MBUF_MAX; i++){
		if(bufpar[i].option & O_IGN){
			bufpar[i].option &= ~O_IGN;
			continue;
		}
		
		if(bufpar[i].option & O_CBUF)
			rc = message_buffer_destroy(msgbuf[i]);
		else
			rc = message_buffer_close(msgbuf[i]);
		
		if(rc < 0){
			log_warn("message buffer destroy or close failed,rc = %d,id = %d",rc,bufpar[i].id);
			continue;
		}
		msgbuf[i] = NULL;
	}
	
	return 0;
}
/*-----------------------------------------------------------------------------------------------------------------------------*/
//message queue
typedef struct msgq_init_param{
	unsigned int id;
	unsigned int option;
}msgq_cpar_t;


messageq_t msgq[MSG_MAX];


#ifdef SYSLINUX

static msgq_cpar_t msgqpar[MSG_MAX] = {
	[0] = {
		.id 		= 0,
		.option 	= O_RS,
	},
	[1] = {
		.id 		= 1,
		.option		= O_RS,
	}
/*
	[1] = {
		.id 		= 1
		.option 	= ...
	},
*/
};

#else
static msgq_cpar_t msgqpar[MSG_MAX] = {
	[0] = {
		0,
		O_RS,
	},
	[1]=  {
		1,
		O_RS,
	}
/*
	[1] = {
		.id 		= 1
		.option 	= ...
	},
*/
};
#endif

int msgq_init(messageq_t *msgq,const char *name,int havebox)
{
	int rc;
	
	rc = messageq_construct(msgq,name,NULL);
	if(rc < 0)
		goto out;
	
	if(havebox){
		rc = messageq_msgbox_create(msgq);
		if(rc < 0){
			messageq_destruct(msgq,0,NULL);
		}
	}
	
out:	
	return rc;
}

int msgq_destroy(messageq_t *msgq)
{
	return messageq_destruct(msgq,0,NULL);
}

int msgq_prepare(void)
{
	int i;
	int rc;
	int failcnt = 0;
    unsigned int timeout = 0;
	char name[MAX_NAME_LEN];
	
	rc = msgbuf_prepare();
	if(rc < 0)
		return rc;
	
	for(i = 0; i < MSG_MAX; i++){
		//�Ƿ�Ҫ���Ը�msgq
		if(msgqpar[i].option & O_IGN)
			continue;
		
		//���첢����msgbuf
		sprintf(name,"%s_%d",local_msg_basenm,msgqpar[i].id);
		rc = msgq_init(&msgq[i],name,msgqpar[i].option & O_RCV);
		if(rc < 0){
			log_warn("messageq init failed,rc = %d,id = %d",rc,msgqpar[i].id);
			msgqpar[i].option |= O_IGN;
			failcnt++;
			continue;
		}
		
		//�����Ҫ���������ӷ��Ͷ��������
		if((msgqpar[i].option & (O_SND | O_IGN)) == O_SND){
			timeout = TRYTIMES;
			sprintf(name,"%s_%d",remote_msg_basenm,msgqpar[i].id);
			while(timeout-- && (rc = messageq_msgbox_attach(&msgq[i],name)) < 0){
//				log_info("try the %dth time to attach msgq:%d failed!",TRYTIMES - timeout,msgqpar[i].id);
				msleep(TYRINTERVAL);
			}
			if(rc < 0){
				log_warn("messageq attach box failed,rc = %d,id = %d",rc,msgqpar[i].id);
				rc = msgq_destroy(&msgq[i]);
				if(!rc)
					msgqpar[i].option |= O_IGN;
				
				failcnt++;
			
			}
		}
	}
	
	return MSG_MAX - failcnt;
}

int msgq_cleanup(void)
{
	int i;
	int rc;
	
	for(i = 0; i < MSG_MAX; i++){
		if(msgqpar[i].option & O_IGN)
			continue;
		
		rc = msgq_destroy(&msgq[i]);
		if(rc < 0){
			log_warn("messageq destroy failed,rc = %d,id = %d",rc,msgqpar[i].id);
			goto out;
		}
	}
	
	rc = msgbuf_cleanup();
	
out:
	return rc;
}

