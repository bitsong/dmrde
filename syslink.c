#include "syslink.h"


#define SLK_TRUE  1
#define SLK_FALSE 0

static int have_syslink = SLK_FALSE;

int syslink_construct(syslink_t *syslink,const char *name,proc_role_t role)
{
	
	if(have_syslink)
		return ESYSLINK_EXISTAREADY;
	
	if(role != PROC_ROLE_HOST && role != PROC_ROLE_SLAVE)
		return ESYSLINK_ROLE;
	
	int rc;
	rc = object_init(&syslink->obj,name,role,NULL);
	if(rc < 0)
		return ESYSLINK_OBJNAME;
	
	bilist_construct(&syslink->list_procs);
	bilist_construct(&syslink->list_objects);
	
	//pthread_mutex_init(&syslink->lock_procs);
	//pthread_mutex_init(&syslink->lock_objects);

	
	have_syslink = SLK_TRUE;
	
	return 0;
}

int syslink_destruct(syslink_t *syslink)
{
	int rc;
	
	if(have_syslink == SLK_FALSE)
		return ESYSLINK_NOSYSLINK;
	
	if(object_busy(&syslink->obj))
		return ESYSLINK_BUSY;
	
	SysLink_destroy();
	
	if((rc = object_destruct(&syslink->obj)) < 0)
		return rc;
	
	//pthread_mutex_destroy(&syslink->lock_procs);
	//pthread_mutex_destroy(&syslink->lock_objects);
	
	bilist_destruct(&syslink->list_procs);
	bilist_destruct(&syslink->list_objects);
	
	have_syslink = SLK_FALSE;
	
	return 0;
}

int syslink_new(syslink_t **pslnk,const char *name,proc_role_t role)
{
	int rc;
	*pslnk = NULL;
	syslink_t *syslink;
	
	syslink = malloc(sizeof(syslink_t));
	if(!syslink)
		return ESYSLINK_MEMORY;
	
	rc = syslink_construct(syslink,name,role);
	if(rc < 0){
		free(syslink);
		return rc;
	}
	
	*pslnk = syslink;
	
	return 0;
}

int syslink_destroy(syslink_t *syslink)
{
	int rc;
	if((rc = syslink_destruct(syslink)) < 0)
		return rc;
	
	free(syslink);
	
	return 0;
}

int syslink_start(syslink_t *syslink,char *proc_name)
{
	int rc = 0;
	//Log_print1(Diags_INFO,"ipc for proc %s stoped!",(xdc_IArg)(syslink->obj.name));
	//Log_print1(Diags_INFO,"ipc for proc %d stoped!",rc);
	
	//Log_info0((syslink->obj.name));
	/*
	uint16_t rpid;
	
	if(object_type(&syslink->obj) == OBJECT_TYPE_SLNK_HOST){
		
		rpid = MultiProc_getId(proc_name);
	
		rc = Ipc_control(rpid, Ipc_CONTROLCMD_LOADCALLBACK, NULL);
		if(rc < 0){
			Log_error0("ipc load callback failed!");
			goto out;
		}
	
		rc = Ipc_control(rpid, Ipc_CONTROLCMD_STARTCALLBACK, NULL);
		if(rc < 0)
			Log_error0("ipc start callback failed!");
	*/	
		//if(rc == 0)
		//将procid插入链表
	//}else{
		
	do{
			rc = Ipc_start();
	}while(rc == Ipc_E_NOTREADY);
	

	if(rc < 0)
	 	Log_error0("ipc start error!");
	if(rc == 0){		
		SysLink_setup();
		syslink_status_set(syslink,SYSLINK_STAT_START);
	}		
	//}
	
	
		
//out:

	return rc;
}

int syslink_stop(syslink_t *syslink,char *proc_name)
{
	int rc = 0;
	
	//uint16_t rpid;
	
	syslink_object_clear(syslink);
	
	//syslink_detach(syslink,proc_name);//可以？
	

		
		Ipc_stop();
		
		//Log_print1(Diags_INFO,"ipc for proc %s stoped!",(xdc_IArg)(syslink->obj.name));
		printf("ipc for proc %s stoped!\n",object_name(&syslink->obj));
		syslink_status_clear(syslink,SYSLINK_STAT_START);

	return rc;
}

int syslink_attach(syslink_t *syslink,char *proc_name)
{
	int rc;
	
	uint16_t rpid;
	
	rpid = MultiProc_getId(proc_name);
	
	do{
		rc = Ipc_attach(rpid);
		if(rc == Ipc_E_NOTREADY)
			Task_sleep(100);			//SYSBIOS测使用
	}while(rc == Ipc_E_NOTREADY);
	
	//if(!rc)
	//将procid插入链表
	
	return rc;
}

static inline int _syslink_detach(syslink_t *syslink,uint16_t procid)
{
	int rc;
	do{
		rc = Ipc_detach(procid);
		if(rc == Ipc_E_NOTREADY)
			 Task_sleep(100);//SYSBIOS测使用
	}while(rc == Ipc_E_NOTREADY);
	
	return rc;
}

int syslink_detach(syslink_t *syslink,char *proc_name)
{
	int rc;
	
	uint16_t rpid;
	
	rpid = MultiProc_getId(proc_name);
		
	rc =  _syslink_detach(syslink,rpid);
	
	//if(!rc)
	//将procid从链表移除
	
	return rc;
}


/*---------------------------------------------------------------------------------------*/

int syslink_detach_all(syslink_t *syslink)
{
	Log_info0("detach all procs ...");
	return 0;
}

syslink_object_t* syslink_object_create(syslink_t *syslink,syslink_objtype_t type,syslink_object_params_t *params)
{
	Log_info0("creating syslink object ...");
/*
	bilist_node_t 	*node;
	object_t 		*obj;
	
	syslink_objhead_t *objh;
	int rc = 0;
	
	if(!params || !syslink)
		return NULL;
	
	switch(type){
		case OBJECT_TYPE_NOTIFY:
			
			objh = (syslink_objhead_t*)malloc(sizeof(syslink_notifier_t));
			if(!objh){
				log_error("out of memory(notifier)!\n");
				goto out;
			}
			//检查参数
			if(!params->notify_nm){
				log_error("invalid parameter(notifier)!\n");
				goto error;
			}
			//初始化
			rc = notifier_construct((syslink_notifier_t*)(&objh->obj),params->notify_nm,params->notify_cap);
			if(rc < 0){
				log_error("notifier create failed!");
				goto error;
			}
			break;
			
		case OBJECT_TYPE_SYNC:
			
			log_error("object type not support!");
			goto out;
			
		default:
			log_error("object type error!");
			goto out;
	}
	
	bilist_add(&syslink->list,&objh->node);

	return &objh->obj;
	
error:
	if(slnk_obj)
		free(slnk_obj);

*/
//out:	
	return NULL;
}
#if 0
static inline int _syslink_object_destruct(syslink_object_t *sobj)
{
	syslink_objtype_t type;
	
	type = sobj->type;
	
	int rc = 0;
	switch(type){
		case OBJECT_TYPE_NOTIFIER:
			rc = notifier_destruct((notifier_t*)sobj);
			break;
		//case OBJECT_TYPE_XXXX:
		//	rc = EOBJTYPE;
		//	break;
		default:
			rc = EOBJTYPE;
			break;
	}
	
	return rc;
}

static inline int _syslink_object_remove(syslink_objhead_t *objh)
{
	int rc = _syslink_object_destruct(&objh->obj);
	if(rc < 0)
		return rc;
	
	bilist_remove(&objh->node);
	
	free(objh);
	return 0;
}
#endif
int syslink_object_destroy(syslink_object_t *sobj)
{	
	Log_info1("remove object %s ...",(xdc_IArg)object_name(sobj));
/*
	syslink_objhead_t *objh;
	
	objh = container_of(sobj,syslink_objhead_t,obj);
	
	return _syslink_object_remove(objh);
*/
	return 0;
}

int syslink_object_clear(syslink_t *syslink)
{
	//Log_info0("clear syslink objects ...");
/*
	int rc;
	
	bilist_node_t *pnode,*pnext;
	syslink_objhead_t *objh;
	
	bilist_for_each_safe(&syslink->list_objects,pnode,pnext){
		objh = container_of(pnode,syslink_objhead_t,node);
		if((rc = syslink_object_destroy(objh)) < 0){
			log_info("object:%s remove failed,code:%d",objh->obj.name,rc);
			return rc;
		}
	}
*/
	
	return 0;
}

