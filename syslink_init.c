#include <stdio.h>
#include <string.h>
#include "syslink_init.h"


syslink_t slk;

int slk_prepare(void)
{
	int rc;
	
	rc = syslink_construct(&slk,local_slk_basenm,slk_role);
	if(rc < 0){
		log_error("syslink construct failed,rc = %d",rc);
		goto out;
	}
	
	rc = syslink_start(&slk,remote_proc_name);
	if(rc < 0){
		log_error("syslink start failed,rc = %d",rc);
		syslink_destruct(&slk);
		goto out;
	}
	
	rc = syslink_attach(&slk,remote_proc_name);
	if(rc < 0){
		log_error("syslink detach failed,rc = %d",rc);
		syslink_stop(&slk,remote_proc_name);
		syslink_destruct(&slk);
	}
	
out:
	return rc;
}

int slk_cleanup(void)
{
	int rc;
	
	rc = syslink_detach(&slk,remote_proc_name);
	if(rc < 0){
		log_error("syslink detach failed,rc = %d",rc);
		goto out;
	}
	
	rc = syslink_stop(&slk,remote_proc_name);
	if(rc < 0){
		log_error("syslink stop failed,rc = %d",rc);
		goto out;
	}
	
	rc = syslink_destruct(&slk);
	if(rc < 0){
		log_error("syslink destruct failed,rc = %d",rc);
		goto out;
	}

out:
	return rc;
}

int syslink_prepare(void)
{
	int rc;
	if((rc = slk_prepare()) < 0)
		goto out;
	
	if((rc = sync_prepare()) < 0){
		goto sync_failed;
	}
	
	if((rc = notifier_prepare()) <= 0){
		goto ntf_failed;
	}
//	log_info("ntf prepare done");
	rc=sync_send(SYNC_CODE_NTFOK);
//	log_info("sync send ntf ok rc=%d",rc);
	
	if((rc = msgq_prepare()) <= 0){
		log_error("msgq init failed!");
		goto msgq_failed;
	}
//	log_info("msgq_prepare done");
	rc=sync_send(SYNC_CODE_MSGOK);
//	log_info("sync send msg ok rc=%d",rc);
	
	if((rc = rpe_prepare()) <= 0){
		log_error("rpe init failed!");
		goto rpe_failed;
	}
//	log_info("rpe prepare done");
	rc=sync_send(SYNC_CODE_RPEOK);
//	log_info("sync send rpe ok rc=%d",rc);

	
	rc = sync_waitfor_prepare();
	
	goto out;
	
rpe_failed:
	msgq_cleanup();
msgq_failed:
	notifier_cleanup();
ntf_failed:
	sync_cleanup();
sync_failed:
	slk_cleanup();
out:
		return rc;
}

int syslink_cleanup(void)
{
    int rc;
	rc = sync_waitfor_cleanup();
	if(rc < 0)
		return rc;
	
	rc = rpe_cleanup();
	if(rc < 0){
		log_warn("rpe clean up failed!");
	}
	rc = msgq_cleanup();
	if(rc < 0){
		log_warn("rpe clean up failed!");
	}
	rc = notifier_cleanup();
	if(rc < 0){
		log_warn("rpe clean up failed!");
	}
	rc = sync_cleanup();
	if(rc < 0){
		log_warn("sync clean up failed!");
	}
	rc = slk_cleanup();
	if(rc < 0){
		log_warn("slk clean up failed!");
	}
	
	return 0;
}
