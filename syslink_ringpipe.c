
#include "syslink_ringpipe.h"


/*-----------------------------------------------------------------------------------------------*/
int _rpe_iobuf_create(struct rpe *rpe, rpe_params_t *params)
{
	RingIOShm_Params par;
	RingIOShm_Params_init(&par);

	par.commonParams.name=object_name(&rpe->obj);
	if(params->ctrlsr!=RPE_INVALID_CTRLSR)
		par.ctrlRegionId=params->ctrlsr;

	if(params->datasr!=RPE_INVALID_DATASR){
		par.dataRegionId=params->datasr;
		par.dataSharedAddrSize=params->dsize;
	}

	if(params->attrsr!=RPE_INVALID_ATTRSR){
		par.attrRegionId=params->attrsr;
		par.attrSharedAddrSize=params->asize;
	}

	par.attrRegionId=params->rpid;
	par.gateHandle=NULL;

	rpe->handle=RingIO_create(&par);
	if(!rpe->handle)
		return ERPE_IOBUFHANDLE;
	return 0;
}

int _rpe_iobuf_delete(struct rpe *rpe)
{
	int rc;

	if(rpe->handle){
		log_info("deleting rpe buffer handle...\n");
		rc = RingIO_delete(&rpe->handle);
		if(rc < 0)
			return rc;
	}

	return 0;
}
static inline int _rpe_open_rwhandle(RingIO_Handle *rwhandle,rpe_endpoint_t endpoint, char *iobuf_name)
{
	RingIO_openParams 		 param;

	param.openMode  = endpoint;
	param.flags = 0;

	return  RingIO_open(iobuf_name,&param,NULL,rwhandle);
}

static inline int _rpe_close_rwhandle(RingIO_Handle *rwhandle)
{
	if(*rwhandle == NULL)
		return 0;

	return RingIO_close(rwhandle);
}

/*-----------------------------------------------------------------------------------------------*/
//创建与销毁。
int rpe_construct(struct rpe *rpe,const char *name,object_t *parent)
{
	rpe->handle = NULL;
	int rc = object_init(&rpe->obj,name,OBJECT_TYPE_RINGPIE,parent);
	if(rc < 0)
		return rc;
	
	rpe->ghandle = NULL;
	rpe->rhandle = NULL;
	rpe->whandle = NULL;
	rpe->status = RPESTAT_INITIALIZER;
	rpe_mark_set(rpe->status,RPESTAT_MARK_VALID);
	
	return 0;
}

int rpe_destruct(struct rpe *rpe)
{
	int rc = 0;
	
	if(!rpe_mark_test(rpe->status,RPESTAT_MARK_VALID)){
		log_error("invalid ring pipe!\n");
		return ERPE_INVALID;
	}
	rpe_attr_t attr = RPE_ATTR_INITIALIZER;

    if(rpe_mark_test(rpe->status,RPESTAT_MARK_READER))
	    rpe_flush(rpe,RPE_ENDPOINT_READER,TRUE,&attr);
	if(rpe_mark_test(rpe->status,RPESTAT_MARK_WRITER))
	    rpe_flush(rpe,RPE_ENDPOINT_WRITER,TRUE,&attr);
	
	if(rpe_mark_test(rpe->status,RPESTAT_MARK_RNOTIFY)){
		log_info("unregister reader notify...\n");
		if((rc = rpe_unregister_notifier(rpe,RPE_ENDPOINT_READER)) < 0)
			return rc;
	}
	
	if(rpe_mark_test(rpe->status,RPESTAT_MARK_WNOTIFY)){
		log_info("unregister reader notify...\n");
		if((rc = rpe_unregister_notifier(rpe,RPE_ENDPOINT_WRITER)) < 0)
			return rc;
	}
	
	if(rpe_mark_test(rpe->status,RPESTAT_MARK_OPEN)){
		log_info("close ring pipe...\n");
		if((rc = rpe_close_all(rpe)) < 0)
			return rc;
	}
	rpe->status = RPESTAT_INITIALIZER;
	object_destruct(&rpe->obj);
	
	return 0;
}

int rpe_create(struct rpe **prpe,const char *name,object_t *parent)
{
	struct rpe *rpe = malloc(sizeof(struct rpe));
	if(!rpe){
		log_info("out of memeory\n");
		return ERPE_MEMEROY;
	}
	
	int rc = rpe_construct(rpe,name,parent);
	if(rc < 0){
		free(rpe);
        return rc;
	}

    *prpe = rpe;

    return 0;
}

int rpe_destroy(struct rpe *rpe)
{
	int rc = rpe_destruct(rpe);
	if(rc < 0)
		return rc;
	
	free(rpe);
	return 0;
}

//打开关闭。
static inline int rpe_open_reader(struct rpe *rpe, char *iobuf_name)
{
	if(rpe->rhandle){
		log_error("rpe reader endpoint handle re-opened!\n");
		return ERPE_REOPEN;
	}
	
	int rc;

	rc = _rpe_open_rwhandle(&rpe->rhandle,RPE_ENDPOINT_READER,iobuf_name);
	if(rc < 0)
		log_warn("rpe rhandle open failed!\n");

	rpe_mark_set(rpe->status,RPESTAT_MARK_READER|RPESTAT_MARK_OPEN);
	
	return rc;
}

static inline int rpe_open_writer(struct rpe *rpe,rpe_params_t *params)
{
	if(rpe->whandle){
		log_error("rpe writer endpoint handle re-opened!\n");
		return ERPE_REOPEN;
	}

	if(rpe->handle){
		log_error("rpe writer endpoint iobuf re-create!\n");
		return ERPE_BUFEXIST;
	}

	int rc;

	rc = _rpe_iobuf_create(rpe,params);
	if(rc < 0)
		return rc;

	rc = _rpe_open_rwhandle(&rpe->whandle,RPE_ENDPOINT_WRITER,object_name(&rpe->obj));
	if(rc < 0){
		log_error("open whandle failed!\n");
		_rpe_iobuf_delete(rpe);
		return rc;
	}
	
	rpe_mark_set(rpe->status,RPESTAT_MARK_WRITER|RPESTAT_MARK_OPEN);
	
	return 0;
}

int rpe_open(struct rpe *rpe,rpe_endpoint_t endpoint,void *params)
{
	if(!rpe_mark_test(rpe->status,RPESTAT_MARK_VALID)){
		log_error("invalid ring pipe!\n");
		return ERPE_INVALID;
	}
	if(!params){
		log_error("invalid parameter!\n");
		return ERPE_INVALIDPARAM;
	}
	if(RPE_ENDPOINT_READER == endpoint)
		return rpe_open_reader(rpe,(char*)params);
	else
		return rpe_open_writer(rpe,(rpe_params_t*)params);
}

int rpe_close(struct rpe *rpe,rpe_endpoint_t endpoint)
{
	if(!rpe_mark_test(rpe->status,RPESTAT_MARK_VALID)){
		log_error("invalid rpe!\n");
		return ERPE_INVALID;
	}
	
	int rc = 0;
	uint32_t mark = 0;

	if(RPE_ENDPOINT_WRITER == endpoint){
		mark = RPESTAT_MARK_WNOTIFY;
		rc = _rpe_close_rwhandle(&rpe->whandle);
		if(rc < 0)
				log_error("close whandle failed!\n");
		if(rpe->handle){
			rc = _rpe_iobuf_delete(rpe);
			if(rc < 0){
				log_error("rpe delete buffer failed!\n");
				return rc;
			}
		}
	}else{
		mark = RPESTAT_MARK_RNOTIFY;
		rc = _rpe_close_rwhandle(&rpe->rhandle);
		if(rc < 0)
				log_error("close rhandle failed!\n");
	}

	rpe_mark_clear(rpe->status,mark);
	if(!rpe_mark_test(rpe->status,RPESTAT_MARK_WNOTIFY | RPESTAT_MARK_RNOTIFY))
		rpe_mark_clear(rpe->status,RPESTAT_MARK_OPEN);

	return rc;

}

int rpe_close_all(struct rpe *rpe)
{
	int rc = 0;
	
	rc = rpe_close(rpe,RPE_ENDPOINT_READER);
	if(rc < 0){
		log_error("rpe close reader failed!\n");
		return ERPE_FAIL;
	}
	
	rc = rpe_close(rpe,RPE_ENDPOINT_WRITER);
	if(rc < 0){
		log_error("rpe close writer failed!\n");
		return ERPE_FAIL;
	}
	
	return 0;
}

//注册/注销notify、修改notify类型及watermark大小。
int rpe_register_notifier(struct rpe *rpe,rpe_endpoint_t endpoint,rpe_notifier_t *ntf)
{
    int rc;
	RingIO_Handle rwhandle = NULL;
	unsigned int mark=0;

	if(!rpe_mark_test(rpe->status,RPESTAT_MARK_VALID)){
		log_error("invalid ring pipe!\n");
		return ERPE_INVALID;
	}
	
	if(!ntf || !rpe_check_notifier(ntf)){
		log_error("invalid notifier!\n");
		return ERPE_INVALIDPARAM;
	}

	if(RPE_ENDPOINT_READER == endpoint){
		if(rpe_mark_test(rpe->status,RPESTAT_MARK_RNOTIFY)){
			log_error("reader notifier exist\n");
			return ERPE_NOTIFYEXIST;
		}
		rwhandle = rpe->rhandle;
		mark 	 = RPESTAT_MARK_RNOTIFY;
	}else{
		if(rpe_mark_test(rpe->status,RPESTAT_MARK_WNOTIFY)){
			log_error("writer notifier exist\n");
			return ERPE_NOTIFYEXIST;
		}
		rwhandle = rpe->whandle;
		mark 	 = RPESTAT_MARK_WNOTIFY;
	}

	if(!rwhandle){
		log_error("reader/writer endpoint not open!\n");
		return ERPE_OPEN;
	}
	
	if((rc = RingIO_registerNotifier(rwhandle,ntf->type,ntf->watermark,ntf->func,rpe)) < 0){
		log_error("ring pipe register notifier failed!\n");
		return rc;
	}

	rpe_mark_set(rpe->status,mark);

	
	return 0;
}

int rpe_unregister_notifier(struct rpe *rpe,rpe_endpoint_t endpoint)
{
    int rc;
	RingIO_Handle rwhandle = NULL;
	unsigned int mark=0;

	if(!rpe_mark_test(rpe->status,RPESTAT_MARK_VALID)){
		log_error("invalid ring pipe!\n");
		return ERPE_INVALID;
	}
	
	if(RPE_ENDPOINT_READER == endpoint){
		if(!rpe_mark_test(rpe->status,RPESTAT_MARK_RNOTIFY) || !rpe->rhandle){
			log_error("reader notifier not registered!\n");
			return 0;
		}
		rwhandle = rpe->rhandle;
		mark 	 = RPESTAT_MARK_RNOTIFY;
	}else{
		if(!rpe_mark_test(rpe->status,RPESTAT_MARK_WNOTIFY) || !rpe->whandle){
			log_error("writer notifier not registered!\n");
			return 0;
		}
		rwhandle = rpe->whandle;
		mark 	 = RPESTAT_MARK_WNOTIFY;
	}
	
	if((rc = RingIO_unregisterNotifier(rwhandle)) < 0){
		log_error("rpe unregister notifier failed!\n");
		return rc;
	}
	
	rpe_mark_clear(rpe->status,mark);

	return 0;
}


int rpe_set_notifytype(struct rpe *rpe,rpe_endpoint_t endpoint,rpe_notify_type_t type)
{
	RingIO_Handle rwhandle = NULL;

	if(!rpe_mark_test(rpe->status,RPESTAT_MARK_VALID)){
		log_error("invalid rpe!\n");
		return ERPE_INVALID;
	}
	
	if(RPE_ENDPOINT_READER == endpoint){
		if(!rpe->rhandle){
			log_error("rpe reader endpoint not opened!\n");
			return ERPE_OPEN;
		}
		rwhandle = rpe->rhandle;
	}else{
		if(!rpe->whandle){
			log_error("rpe writer endpoint not opened!\n");
			return ERPE_OPEN;
		}
		rwhandle = rpe->whandle;
	}

	return RingIO_setNotifyType(rwhandle,type);
}

int rpe_set_watermark(struct rpe *rpe,rpe_endpoint_t endpoint,uint32_t watermark)
{
	RingIO_Handle rwhandle = NULL;

	if(!rpe_mark_test(rpe->status,RPESTAT_MARK_VALID)){
		log_error("invalid ring pipe!\n");
		return ERPE_INVALID;
	}

	if(RPE_ENDPOINT_READER == endpoint){
		if(!rpe->rhandle){
			log_error("rpe reader endpoint not opened!\n");
			return ERPE_OPEN;
		}
		rwhandle = rpe->rhandle;
	}else{
		if(!rpe->whandle){
			log_error("rpe writer endpoint not opened!\n");
			return ERPE_OPEN;
		}
		rwhandle = rpe->whandle;
	}

	return RingIO_setWaterMark(rwhandle,watermark);
}

int rpe_get_watermark(struct rpe *rpe,rpe_endpoint_t endpoint)
{
	RingIO_Handle rwhandle = NULL;

	if(!rpe_mark_test(rpe->status,RPESTAT_MARK_VALID)){
		log_error("invalid ring pipe!\n");
		return ERPE_INVALID;
	}
	
	if(RPE_ENDPOINT_READER == endpoint){
		if(!rpe->rhandle){
			log_error("rpe reader endpoint not opened!\n");
			return ERPE_OPEN;
		}
		rwhandle = rpe->rhandle;
	}else{
		if(!rpe->whandle){
			log_error("rpe writer endpoint not opened!\n");
			return ERPE_OPEN;
		}
		rwhandle = rpe->whandle;
	}

	return RingIO_getWaterMark(rwhandle);
}
//发送通知
int rpe_send_notify(struct rpe *rpe,rpe_endpoint_t endpoint,uint16_t msg)
{
	RingIO_Handle rwhandle = NULL;

	if(!rpe_mark_test(rpe->status,RPESTAT_MARK_VALID)){
		log_error("invalid ring pipe!\n");
		return ERPE_INVALID;
	}
	
	if(RPE_ENDPOINT_READER == endpoint){
		if(!rpe->rhandle){
			log_error("rpe reader endpoint not opened!\n");
			return ERPE_OPEN;
		}
		rwhandle = rpe->rhandle;
	}else{
		if(!rpe->whandle){
			log_error("rpe writer endpoint not opened!\n");
			return ERPE_OPEN;
		}
		rwhandle = rpe->whandle;
	}

	return RingIO_sendNotify(rwhandle,msg);
}

//设置与获取属性 -- 设置标志、识别符、传递特殊消息等。
int rpe_set_attribute(struct rpe *rpe,rpe_attr_t *attr,Bool sendnotify)
{
	RingIO_Handle rwhandle = NULL;

	if(!rpe_mark_test(rpe->status,RPESTAT_MARK_VALID)){
		log_error("invalid ring pipe!\n");
		return ERPE_INVALID;
	}
/*
	if(RPE_ENDPOINT_READER == endpoint){
		if(!rpe->rhandle){
			log_error("rpe reader endpoint not opened!\n");
			return ERPE_OPEN;
		}
		rwhandle = rpe->rhandle;
	}else{
		if(!rpe->whandle){
			log_error("rpe writer endpoint not opened!\n");
			return ERPE_OPEN;
		}
		rwhandle = rpe->whandle;
	}
*/
	rwhandle = rpe->whandle;
	if(attr->pdata){
		return RingIO_setvAttribute(rwhandle,attr->type,attr->param,attr->pdata,attr->size,sendnotify);
	}else{
		return RingIO_setAttribute(rwhandle,attr->type,attr->param,sendnotify);
	}
}

int rpe_get_attribute(struct rpe *rpe,rpe_attr_t *attr,Bool flag)
{
	RingIO_Handle rwhandle = NULL;

	if(!rpe_mark_test(rpe->status,RPESTAT_MARK_VALID)){
		log_error("invalid ring pipe!\n");
		return ERPE_INVALID;
	}
/*
	if(RPE_ENDPOINT_READER == endpoint){
		if(!rpe->rhandle){
			log_error("rpe reader endpoint not opened!\n");
			return ERPE_OPEN;
		}
		rwhandle = rpe->rhandle;
	}else{
		if(!rpe->whandle){
			log_error("rpe writer endpoint not opened!\n");
			return ERPE_OPEN;
		}
		rwhandle = rpe->whandle;
	}
*/
	rwhandle = rpe->rhandle;
	if(flag == RPE_ATTR_VARIABLE){
		return RingIO_getvAttribute(rwhandle,&attr->type,&attr->param,attr->pdata,&attr->size);
	}else{
		return RingIO_getAttribute(rwhandle,&attr->type,&attr->param);
	}
}


//冲刷缓存数据。
int rpe_flush(struct rpe *rpe,rpe_endpoint_t endpoint,Bool hardflush,rpe_attr_t *attr)//返回 flush的数据量
{
	RingIO_Handle rwhandle = NULL;

	if(!rpe_mark_test(rpe->status,RPESTAT_MARK_VALID)){
		log_error("invalid ring pipe!\n");
		return ERPE_INVALID;
	}
	
	if(RPE_ENDPOINT_READER == endpoint){
		if(!rpe->rhandle){
			log_error("rpe reader endpoint not opened!\n");
			return ERPE_OPEN;
		}
		rwhandle = rpe->rhandle;
	}else{
		if(!rpe->whandle){
			log_error("rpe writer endpoint not opened!\n");
			return ERPE_OPEN;
		}
		rwhandle = rpe->whandle;
	}

	uint16_t *type  = NULL;
	uint32_t *param = NULL;
	uint32_t *bytes = NULL;

	if(attr){
		type  = &attr->type;
		param = &attr->param;
		bytes = &attr->bytes;
	}

	return RingIO_flush(rwhandle,hardflush,type,param,bytes);
}

//获取有效及空闲大小(包括数据及属性)
int rpe_get_validsize(struct rpe *rpe,rpe_endpoint_t endpoint,rpe_space_t type)
{
	RingIO_Handle rwhandle = NULL;

	if(!rpe_mark_test(rpe->status,RPESTAT_MARK_VALID)){
		log_error("invalid ring pipe!");
		return ERPE_INVALID;
	}

	if(RPE_ENDPOINT_READER == endpoint){
		if(!rpe->rhandle){
			log_error("rpe reader endpoint not opened!");
			return ERPE_OPEN;
		}
		rwhandle = rpe->rhandle;
	}else{
		if(!rpe->whandle){
			log_error("rpe writer endpoint not opened!");
			return ERPE_OPEN;
		}
		rwhandle = rpe->whandle;
	}

	switch(type){
		case RPE_SPACE_DATA:

			return RingIO_getValidSize(rwhandle);

		case RPE_SPACE_ATTR:

			return RingIO_getValidAttrSize(rwhandle);

		default:

			return ERPE_SPACE_TYPE;
	}
}

int rpe_get_emptysize(struct rpe *rpe,rpe_endpoint_t endpoint,rpe_space_t type)
{
	RingIO_Handle rwhandle = NULL;

	if(!rpe_mark_test(rpe->status,RPESTAT_MARK_VALID)){
		log_error("invalid ring pipe!\n");
		return ERPE_INVALID;
	}
	
	if(RPE_ENDPOINT_READER == endpoint){
		if(!rpe->rhandle){
			log_error("rpe reader endpoint not opened!\n");
			return ERPE_OPEN;
		}
		rwhandle = rpe->rhandle;
	}else{
		if(!rpe->whandle){
			log_error("rpe writer endpoint not opened!\n");
			return ERPE_OPEN;
		}
		rwhandle = rpe->whandle;
	}

	switch(type){
		case RPE_SPACE_DATA:
			
			return RingIO_getEmptySize(rwhandle);
			
		case RPE_SPACE_ATTR:
			
			return RingIO_getEmptyAttrSize(rwhandle);
			
		default:
		
			return ERPE_SPACE_TYPE;
	}
}

int rpe_print_status(rpe_t *rpe)
{
	if(!rpe){
		log_error("invalid rpe!\n");
		return -1;
	}

	log_info("print rpe info:\n");
	log_info("RPE VALID?%s",rpe_mark_test(rpe->status,RPESTAT_MARK_VALID) ? "YES!" : "NO");
	log_info("HAVE OPENED RPE?%s,handle %p ",rpe_mark_test(rpe->status,RPESTAT_MARK_OPEN) ? "YES!" : "NO!",rpe->handle);
	log_info("HAVE READER?%s ",rpe_mark_test(rpe->status,RPESTAT_MARK_READER) ? "YES!" : "NO!");
	log_info("HAVE WRITER?%s ",rpe_mark_test(rpe->status,RPESTAT_MARK_WRITER) ? "YES!" : "NO!");
	log_info("HAVE RNOTIFY?%s ",rpe_mark_test(rpe->status,RPESTAT_MARK_RNOTIFY) ? "YES!" : "NO!");
	log_info("HAVE WNOTIFY?%s ",rpe_mark_test(rpe->status,RPESTAT_MARK_WNOTIFY) ? "YES!" : "NO!\n");
	//log_info("HAVE BUFFER?%s ", rpe_mark_test(rpe->status,PRESTAT_MARK_BUFFER)  ? "YES!" : "NO!\n");
	return 0;
} 

