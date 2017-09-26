#include <stdio.h>
#include <string.h>
#include "syslink_sync.h"
#include "msgq_init.h"

#define MAX_MSG_LEN	0

struct msgstr{
	char message[MAX_MSG_LEN];
};

declare_message_type(msg_t,struct msgstr);

static message_buffer_t *msync_buf = NULL;
static messageq_t msync;

static int sync_buf_init(void)
{
    int rc = 0;
	char name[MAX_NAME_LEN];
	message_buffer_params_t par;
	message_buffer_params_init(&par);
	

	par.heapid  = MESSAGEQ_MAX_HEAPID;
	par.heaptype = HEAP_TYPE_MEMMP;

#ifdef SYSLINUX
	par.memmp_rid = 1;
	par.memmp_size = 1 * KB;
	sprintf(name,"%s_sync_buf",local_buf_basenm);
	par.memmp_name = name;
	rc = message_buffer_create(&msync_buf,&par);
#else
	sprintf(name,"%s_sync_buf",remote_buf_basenm);
	par.memmp_name = name;
	unsigned int timeout=TRYTIMES;
//	log_info("before open:%s",par.memmp_name);
	while(timeout-- && (rc = message_buffer_open(&msync_buf,&par)) < 0){
//		log_info("%s",par.memmp_name);
//		log_info("try the first time open sync buffer failed,rc = %d",rc);
		msleep(TYRINTERVAL);
	}
#endif

	if(rc < 0){
		log_error("sync buffer init failed,rc = %d",rc);
	}
	
	return rc;
}

int sync_buf_destroy(void)
{
	int rc;
	
#ifdef SYSLINUX
	rc = message_buffer_destroy(msync_buf);
#else
	rc = message_buffer_close(msync_buf);
#endif

	msync_buf = NULL;
	return rc;
	
}

int sync_prepare(void)
{
	int rc;
    unsigned int timeout;
	
	char name[MAX_NAME_LEN];
	sprintf(name,"%s_syncronyzer",local_msg_basenm);
	
	if((rc = sync_buf_init()) < 0){
		log_error("sync_buf init failed,rc = %d",rc);
		return rc;
	}

//	log_info("syne_buf success");

	rc = msgq_init(&msync,name,1);
	if(rc < 0){
		log_error("syncronyzer init failed,rc = %d",rc);
		return rc;
	}
//	log_info("msgq_init success");

	timeout = TRYTIMES;
	sprintf(name,"%s_syncronyzer",remote_msg_basenm);
	while(timeout-- && (rc = messageq_msgbox_attach(&msync,name)) < 0){
		log_info("try the %dth time to attach msgq:syncronyzer",TRYTIMES - timeout);
		msleep(TYRINTERVAL);
	}

	if(rc < 0){
		log_error("attach remote syncronyzer failed,rc = %d",rc);
		msgq_destroy(&msync);
	}
	
	return rc;
}

int sync_cleanup(void)
{
	int rc;
	
	if((rc = sync_send(SYNC_CODE_OUT0)) < 0)
		return rc;
	
	if((rc = sync_wait_for(SYNC_CODE_OUT0)) < 0)
		return rc;
	
	if((rc = sync_send(SYNC_CODE_ACK0)) < 0)
		return rc;
	
	if((rc = sync_wait_for(SYNC_CODE_ACK0)) < 0)
		return rc;
	
	msgq_destroy(&msync);
	sync_buf_destroy();
	
	
	return rc;
}

int sync_send(unsigned int code)
{
	msg_t *msg;
	int rc = 0;

	msg = (msg_t*)message_alloc(msync_buf,sizeof(msg_t));
	if(msg < 0)
		return -1;
	
	msg->type = code;
	
	rc = messageq_send_safe(&msync,msg,0,0,0);
	if(rc < 0){
		log_warn("message(%u) send failed,rc = %d",code,rc);
	}
	
	return rc;
}

int sync_wait(unsigned int *code,unsigned int timeout,int times)
{
	msg_t *msg = NULL;
	int rc = 0;
	
	
	while(times-- && (rc = messageq_recv(&msync,(messageq_msg_t*)&msg,timeout)) < 0);
	if(rc < 0 || msg == NULL){
		log_warn("wait for msg failed,rc = %d",rc);
		return -1;
	}
	
	*code = msg->type;
	
	message_free(msg);
	
	return 0;
}

int sync_wait_for(unsigned int code)
{
	unsigned msg = 0;
	int rc = 0;
	
	while(code != msg){
		rc = sync_wait(&msg,MESSAGEQ_WAITFOREVER,1);
		if(rc < 0)
			break;
		
		if(msg != code)
			log_info("sync got msg:%u",msg);
	}
	
	return rc;
}

int sync_waitfor_prepare(void)
{
	static int 		flag = 0;
	
	unsigned int 	code = 0;
	unsigned char 	msgq_flag = 0;
	unsigned char 	rpe_flag = 0;
	unsigned char	ntf_flag = 0;
    int rc = 0;
	
	if(flag == 1){
		log_warn("recalled!");
		return 0;
	}
	
	while(1){
		rc = sync_wait(&code,SYNC_WAIT_TIMEOUT,SYNC_WAIT_TRYTIMES);
		if(rc < 0)
			break;
		
		if(code == SYNC_CODE_MSGOK){
			msgq_flag = 1;
//			log_info("msg ok received");
		}else if(code == SYNC_CODE_RPEOK){
			rpe_flag = 1;
//			log_info("rpe ok received");
		}else if(code == SYNC_CODE_NTFOK){
			ntf_flag = 1;
//			log_info("ntf ok received");
		}else{
			log_warn("unrecognized code(%u) recieved",code);
			continue;
		}
		
		if(msgq_flag && rpe_flag && ntf_flag){
//			log_info("syslink prepare done!");
			flag=1;
			break;
		}
	}
	
	return rc;
}

int sync_waitfor_cleanup(void)
{

	static int 		flag = 0;
	
	unsigned int 	code = 0;
	unsigned char 	msgq_flag = 0;
	unsigned char 	rpe_flag = 0;
	unsigned char	ntf_flag = 0;
    int rc = 0;
	
	if(flag == 1){
		log_info("recalled!");
		return 0;
	}

	while(1){
		rc = sync_wait(&code,SYNC_WAIT_TIMEOUT,SYNC_WAIT_TRYTIMES);
		if(rc < 0)
			break;

		if(code == SYNC_CODE_MSGOUT){
			msgq_flag = 1;
		}else if(code == SYNC_CODE_RPEOUT){
			rpe_flag = 1;
		}else if(code == SYNC_CODE_NTFOUT){
			ntf_flag = 1;
		}else{
			log_warn("unrecognized code(%u) recieved",code);
			continue;
		}
		
		if(msgq_flag && rpe_flag && ntf_flag){
			log_info("syslink cleanup done!");
			break;
		}
	}
	
	return rc;
}
