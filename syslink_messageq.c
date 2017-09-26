#include "syslink_messageq.h"

void message_buffer_params_init(message_buffer_params_t *params)
{
	memset(params,0,sizeof(message_buffer_params_t));
	params->heaptype 	= HEAP_TYPE_MEMMP;
	params->bufmp_rid 	= 1;//SR0鏈夌壒娈婄敤閫斾笉鍏佽浣跨敤
}

static void _message_buffer_init(message_buffer_t *mbuf)
{
	mbuf->handle	= NULL;
	mbuf->heapid 	= HEAPID_INVALID;
	mbuf->heaptype 	= HEAP_TYPE_MEMMP;
}

static void _message_buffer_deinit(message_buffer_t *mbuf)
{
	mbuf->handle 	= NULL;
	mbuf->heapid 	= HEAPID_INVALID;
	mbuf->heaptype 	= HEAP_TYPE_LAST;
}

static int _message_buffer_heap_create(message_buffer_t *mbuf,message_buffer_params_t *params)
{
	HeapBufMP_Params hbpar;
	HeapMemMP_Params hmpar;
	
	if(params->heaptype == HEAP_TYPE_BUFMP){
		
		HeapBufMP_Params_init(&hbpar);
		
		hbpar.name 		    = params->bufmp_name;
		hbpar.regionId    	= params->bufmp_rid;
		hbpar.blockSize   	= params->bufmp_bsize;
		hbpar.numBlocks    	= params->bufmp_nblock;
		hbpar.align 	  	= params->bufmp_align;
		hbpar.exact 		= params->bufmp_exact;
		
		mbuf->handle 		= (Ptr)HeapBufMP_create(&hbpar);
	}else{
		HeapMemMP_Params_init(&hmpar);

		//strcpy(hmpar.name,params->memmp_name);
		hmpar.name			= params->memmp_name;
		hmpar.regionId 		= params->memmp_rid;
		hmpar.sharedBufSize = params->memmp_size;
		
		mbuf->handle  		= (Ptr)HeapMemMP_create(&hmpar);
	}
	
	if(!mbuf->handle)
		return EMESSAGEQ_HEAPCREATE;
	
	return 0;
}

static void _message_buffer_heap_destroy(message_buffer_t *mbuf)
{
	if(mbuf->heaptype == HEAP_TYPE_BUFMP)
		HeapBufMP_delete((HeapBufMP_Handle*)(&mbuf->handle));
	else
		HeapMemMP_delete((HeapMemMP_Handle*)(&mbuf->handle));
}

static int _message_buffer_heap_open(message_buffer_t *mbuf,message_buffer_params_t *params)
{
	int rc = 0;

	if(params->heaptype == HEAP_TYPE_BUFMP){
		
//		printf("heap name:%s\n",params->bufmp_name);
		rc = HeapBufMP_open(params->bufmp_name,(HeapBufMP_Handle*)&mbuf->handle);
	}else{
//		printf("heap name:%s\n",params->memmp_name);
		rc = HeapMemMP_open(params->memmp_name,(HeapMemMP_Handle*)&mbuf->handle);
	}
	
	if(rc < 0)
		return rc;
	
	mbuf->heaptype = params->heaptype;
	mbuf->heapid   = params->heapid;
	
	return 0;
}

static void _message_buffer_heap_close(message_buffer_t *mbuf)
{
	if(mbuf->heaptype == HEAP_TYPE_BUFMP){
		
		HeapBufMP_close((HeapBufMP_Handle*)&mbuf->handle);
	}else{
		HeapMemMP_close((HeapMemMP_Handle*)&mbuf->handle);
	}
}

static int _message_buffer_register(message_buffer_t *mbuf)
{
	return MessageQ_registerHeap(mbuf->handle,mbuf->heapid);
}

static int _message_buffer_unregister(message_buffer_t *mbuf)
{
	return MessageQ_unregisterHeap(mbuf->heapid);
}

int message_buffer_construct(message_buffer_t *mbuf,message_buffer_params_t *params)
{
	
	if(message_buffer_valid(mbuf)){
		printf("message buffer reconstruct!\n");
		return EMESSAGEQ_RECONSTRUCT;
	}
	
	if(_message_buffer_heap_create(mbuf,params) < 0){
		_message_buffer_deinit(mbuf);
		return EMESSAGEQ_HEAPCREATE;
	}
	
	mbuf->heapid	= params->heapid;
	mbuf->heaptype  = params->heaptype;
	
	return 0;
}

void message_buffer_destruct(message_buffer_t *mbuf)
{
	if(!message_buffer_valid(mbuf))
		return;
	
	_message_buffer_heap_destroy(mbuf);
	
	return;
}

int message_buffer_create(message_buffer_t **pmbuf,message_buffer_params_t *params)
{
	int rc;
	
	if(params->bufmp_rid == 0)
		return EMESSAGEQ_SREGION;
	
	if(params->heaptype >= HEAP_TYPE_LAST){
		printf("invalid message buffer type.\n");
		return EMESSAGEQ_BUFTYPE;
	}
	
	if(!check_heapid(params->heapid)){
		printf("invalid heapid!\n");
		return EMESSAGEQ_INVALIDHEAPID;
	}
	
	message_buffer_t *mbuf = malloc(sizeof(message_buffer_t));
	if(!mbuf){
		printf("out of memory!\n");
		return EMESSAGEQ_MEMORY;
	}
	
	_message_buffer_init(mbuf);
	
	if((rc = message_buffer_construct(mbuf,params)) < 0){
		printf("message pool construct failed!\n");
		_message_buffer_deinit(mbuf);
		return rc;
	}

	if((rc = _message_buffer_register(mbuf)) < 0){
		printf("heap already exist with heapid\n");
		message_buffer_destruct(mbuf);
		return EMESSAGEQ_HIDEXIST;
	}
	
	*pmbuf = mbuf;
	return 0;
}

int message_buffer_destroy(message_buffer_t *mbuf)
{
	if(!mbuf)
		return EMESSAGEQ_INVALIDPAR;
	
	_message_buffer_unregister(mbuf);
	
	message_buffer_destruct(mbuf);
	
	_message_buffer_deinit(mbuf);

	free(mbuf);
	
	return 0;
}

int message_buffer_open(message_buffer_t **pmbuf,message_buffer_params_t *params)
{
	int rc = 0;
	
	if(params->heaptype >= HEAP_TYPE_LAST){
		printf("invalid message buffer type.\n");
		return EMESSAGEQ_BUFTYPE;
	}
	
	if(!check_heapid(params->heapid)){
		printf("invalid heapid!\n");
		return EMESSAGEQ_INVALIDHEAPID;
	}
	
	message_buffer_t *mbuf = malloc(sizeof(message_buffer_t));
	if(!mbuf){
		return EMESSAGEQ_MEMORY;
	}
	
	_message_buffer_init(mbuf);
	
	if((rc = _message_buffer_heap_open(mbuf,params)) < 0){
//		printf("message buffer(heap) open failed!\n");
		return rc;
	}
	
	if((rc = _message_buffer_register(mbuf)) < 0){
		printf("heap already exist with heapid\n");
		_message_buffer_heap_close(mbuf);
		return EMESSAGEQ_HIDEXIST;
	}

	*pmbuf = mbuf;
	
	return 0;
}

int message_buffer_close(message_buffer_t *mbuf)
{
	_message_buffer_unregister(mbuf);
	
	_message_buffer_heap_close(mbuf);
	
	free(mbuf);
	
	return 0;
}

/*********************************************************************************************************/
int messageq_construct(messageq_t *msgq,const char *name,object_t *parent)
{
	int rc;
	
	if(status_mark_bits_test(msgq->status,MSGQSTAT_MARK_VALID)){
		return EMESSAGEQ_REINIT;
	}
	
	if((rc = object_init(&msgq->obj,name,OBJECT_TYPE_MSGQ,parent)) < 0)
		return rc;
	
	msgq->handle = NULL;
	msgq->func	 = NULL;
	msgq->boxid.qid	 = MessageQ_INVALIDMESSAGEQ;
	msgq->status = MSGQSTAT_INITIALIZER;
	msgq->error		 = 0;
	msgq->interval   = MESSAGEQ_DEFAULT_INTERVAL;
	status_mark_bits_set(msgq->status,MSGQSTAT_MARK_VALID);
	status_evtlvl_set(msgq->status,(int)MESSAGEQ_EVENT_DEFAULT);
	
	return 0;
}

/*
	boxid鏆傛椂娌＄敤锛岀瓑瀹炵幇閾捐〃绠＄悊鎵撳紑鐨刡oxid鍚庯紝boxid鍙互鎸囧畾浠绘剰鎵撳紑鐨刴sgbox
 */
int messageq_destruct(messageq_t *msgq,Bool sync,messageq_boxid_t *boxid)
{

	if(!status_mark_bits_test(msgq->status,MSGQSTAT_MARK_VALID))
		return EMESSAGEQ_INVALID;
	
	//妫�煡鏄惁杩樻湁msgbox娌℃湁鍏抽棴锛�
	if(status_rboxs_test(msgq->status)){
		printf("some msgbox opened but not closed!\n");
		return EMESSAGEQ_RBOXNOTCLOSED;
	}
	
	if(sync){
		if(msgq->boxid.qid == MessageQ_INVALIDMESSAGEQ){
			printf("cannot sync,no msgbox attached\n");
			return EMESSAGEQ_INVALIDBOXID;
		}
		//鍙戦�鍐呴儴鍛戒护銆�
		//绛夊緟瀵规柟鍥炲簲銆�
	}
	//妫�煡鏄惁鏈夋湭娉ㄩ攢鐨刵otifier
	if(status_notifys_get(msgq->status) != 0){
		printf("some notifiers art not unregistered!\n");
		return EMESSAGEQ_NOTIFYNOTCLOSED;
	}

	//detach杩滅msgbox
	printf("dettach message box...\n");
	messageq_msgbox_detach(msgq);
	
	printf("delete local message box...\n");
	messageq_msgbox_delete(msgq);

	status_mark_bits_clear(msgq->status,MSGQSTAT_MARK_VALID);
	status_notifys_clear(msgq->status);
	
	
	msgq->handle = NULL;
	msgq->func   = NULL;
	msgq->boxid.qid	 = MessageQ_INVALIDMESSAGEQ;
	msgq->status = MSGQSTAT_INITIALIZER;
	msgq->error		 = 0;
	msgq->interval	 = 0;
	return object_destruct(&msgq->obj);
}

int messageq_create(messageq_t **pmsgq,const char *name)
{
	int rc;
	messageq_t *msgq;
	
	msgq = malloc(sizeof(messageq_t));
	if(!msgq)
		return EMESSAGEQ_MEMORY;
	
	if((rc = messageq_construct(msgq,name,NULL)) < 0){
		free(msgq);
		return rc;
	}

	*pmsgq = msgq;
	
	return 0;
}

int messageq_destroy(messageq_t *msgq,Bool sync,messageq_boxid_t *boxid)
{
	if(!msgq)
		return 0;

    int rc;
	
	if((rc = messageq_destruct(msgq,sync,boxid)) < 0)
		return rc;
	
	free(msgq);
	
	return 0;
}

int messageq_msgbox_create(messageq_t *msgq)//void?
{

	MessageQ_Params param;
	
	if(!status_mark_bits_test(msgq->status,MSGQSTAT_MARK_VALID))
		return EMESSAGEQ_INVALID;

	if(msgq->handle){
		printf("message box already exist!\n");
		return EMESSAGEQ_MSGBOXEXIST;
	}
	
	MessageQ_Params_init(&param);
	msgq->handle = MessageQ_create(msgq->obj.name,&param);
	if(msgq->handle==NULL){
		printf("creat messageq failed!\n");
		return EMESSAGEQ_FAIL;
	}
	status_mark_bits_set(msgq->status,MSGQSTAT_MARK_BOX);
	
	return 0;
}

int messageq_msgbox_delete(messageq_t *msgq)
{
	
	if(!status_mark_bits_test(msgq->status,MSGQSTAT_MARK_VALID))
		return EMESSAGEQ_INVALID;
	
	if(!msgq->handle)
		return 0;
	
	if(MessageQ_delete(&msgq->handle) < 0)
		return EMESSAGEQ_FAIL;
	
	msgq->handle = NULL;
	status_mark_bits_clear(msgq->status,MSGQSTAT_MARK_BOX);
	
	return 0;
}

int messageq_msgbox_open(messageq_t *msgq,messageq_boxid_t *boxid,char *name)
{
	uint32_t qid;

	if(!status_mark_bits_test(msgq->status,MSGQSTAT_MARK_VALID))
		return EMESSAGEQ_INVALID;

	int rc;
	
	if((rc = MessageQ_open(name,&qid)) != 0)
		return rc;
	
	boxid->qid = qid;
	status_rboxs_increase(msgq->status);
	
	return 0;
}

int messageq_msgbox_close(messageq_t *msgq,messageq_boxid_t *boxid)
{
	if(!status_mark_bits_test(msgq->status,MSGQSTAT_MARK_VALID))
		return EMESSAGEQ_INVALID;
	
	if(MessageQ_close(&boxid->qid) < 0)
		return EMESSAGEQ_FAIL;
	
	boxid->qid	 = MessageQ_INVALIDMESSAGEQ;
	if(status_rboxs_get(msgq->status) == 0){
		printf("remote msgbox counter error!\n");
		return EMESSAGEQ_OUTOFRBOX;
	}
	status_rboxs_decrease(msgq->status);
	
	return 0;
}

int messageq_msgbox_attach(messageq_t *msgq,char *name)
{
	
	int rc;
	
	if(msgq->boxid.qid != MessageQ_INVALIDMESSAGEQ){
		printf("another msgbox have been attached!\n");
		return EMESSAGEQ_REATTACHED;
	}
	
	if((rc = messageq_msgbox_open(msgq,&msgq->boxid,name)) < 0)
		return rc;
	
	//鍥犱负attach鐨勪笉璁＄畻鍦ㄦ墦寮�殑msgbox涔嬪唴锛屾墍浠ヨ繖閲屽簲-1
	status_rboxs_decrease(msgq->status);
	
	return 0;
}

int messageq_msgbox_detach(messageq_t *msgq)
{
	if(msgq->boxid.qid == MessageQ_INVALIDMESSAGEQ)
		return 0;
	
	int rc;
	//鍥犱负attach鐨勪笉璁＄畻鍦ㄦ墦寮�殑msgbox涔嬪唴锛屾墍浠ュ簲鍏�1锛岀瓑璋冪敤messageq_msgbox_close鏃跺噺鍘�
	status_rboxs_increase(msgq->status);
	
	if((rc = messageq_msgbox_close(msgq,&msgq->boxid)) < 0){
		//濡傛灉鍏抽棴澶辫触鍒欏啀鎭㈠鍘熷�
		status_rboxs_decrease(msgq->status);
		return rc;
	}
	
	msgq->boxid.qid = MessageQ_INVALIDMESSAGEQ;
	
	return 0;
}

static inline void messageq_set_sender(messageq_t *msgq,messageq_msg_t msg)
{	
	if(!msgq->handle)
		return;
	
	MessageQ_setReplyQueue(msgq->handle,msg);
}

int messageq_sendto(messageq_t *msgq,messageq_boxid_t *boxid,messageq_msg_t msg,uint32_t flag,uint16_t level,const uint32_t retry_times)
{
	Bool wait_clear = TRUE;
	int rc=0;
	UInt32 	times=retry_times;

	if(!status_mark_bits_test(msgq->status,MSGQSTAT_MARK_VALID))
		return EMESSAGEQ_INVALID;
	
	if(boxid->qid == MessageQ_INVALIDMESSAGEQ){
		printf("invalid boxid!\n");
		return EMESSAGEQ_INVALIDBOXID;
	}

	messageq_set_sender(msgq,msg);
	if(MessageQ_put(boxid->qid,msg) < 0)
		return EMESSAGEQ_FAIL;
	
	
	if(flag & MESSAGEQ_FLAG_NOCLR)
		wait_clear = FALSE;

	if(flag & MESSAGEQ_FLAG_NOTIFY){
		
		if(!check_event_level(level)){
			printf("invalid event level!\n");
			level = MESSAGEQ_EVENT_DEFAULT;
		}
		do{
			rc = Notify_sendEvent(box2procid(boxid),
						 MESSAGEQ_DEFAULT_LINE,
						 level,
						 MESSAGEQ_NOTIFY_SEND,wait_clear);
			if(rc<0 ){				
				if(times--==0)
					break;
				Task_sleep(msgq->interval);
			}				
		}while(rc<0 );		
		if(rc < 0)
			printf("send notify failed:rc=%d\n",rc);
	}
						 
	return rc;
}

int messageq_send(messageq_t *msgq,messageq_msg_t msg,uint32_t flag,uint16_t level,const uint32_t retry_times)
{
	return messageq_sendto(msgq,&msgq->boxid,msg,flag,level,retry_times);
}

int messageq_recv(messageq_t *msgq,messageq_msg_t *msg,unsigned int timeout)
{
	int rc;
	
	if(!status_mark_bits_test(msgq->status,MSGQSTAT_MARK_VALID)){
		printf("invalid messageq!\n");
		return EMESSAGEQ_INVALID;
	}

    if(!msgq->handle){
        printf("can not recv messages,no msgbox created!");
        return EMESSAGEQ_INVALIDMSGBOX;
    }

    if(status_mark_bits_test(msgq->status,MSGQSTAT_MARK_INTCONTEXT) &&
                timeout > MESSAGEQ_MAXTIMEOUT_INTCONTEXT)
        timeout = MESSAGEQ_MAXTIMEOUT_INTCONTEXT;

	if((rc = MessageQ_get(msgq->handle,msg,timeout)) < 0){
		return rc;
	}
	
	return SUCCESS;
}

/*妫�煡閿欒杩斿洖閿欒鐮佸苟娓呴櫎閿欒鐮�*/
int messageq_check_errors(messageq_t *msgq)
{
	int error;
	if(status_mark_bits_test(msgq->status,MSGQSTAT_MARK_ERROR)){
		status_mark_bits_clear(msgq->status,MSGQSTAT_MARK_ERROR);
		error=msgq->error;
		msgq->error=0;
		return error;
	}	
	return 0;
}

int messageq_clear_errors(messageq_t *msgq)
{
	status_mark_bits_clear(msgq->status,MSGQSTAT_MARK_ERROR);
	msgq->error=0;
	return 0;
}

static void message_notifier_callback(uint16_t pid,uint16_t lid,uint32_t eid,UArg arg,uint32_t payload)
{
	int rc = 0;
	messageq_t *msgq = (messageq_t*)arg;
	//messageq_event_t mevt = {{pid,lid,eid}, payload};
	messageq_event_t mevt;

  	if(!msgq){
        printf("recv event:pid = %hu,lig = %hu,eid = %u,but msgq is NULL!",
                    pid,lid,eid);
        return;
    }

	mevt.evid.pid=pid;
	mevt.evid.lid=lid;
	mevt.evid.eid=eid;
	mevt.evcode=payload;

    status_mark_bits_set(msgq->status,MSGQSTAT_MARK_INTCONTEXT);

	if(msgq->func){
		rc = msgq->func(msgq,&mevt);	
		if(rc == MESSAGEQ_RETURN_HANDLED || rc == MESSAGEQ_RETURN_SUCCESS){

		}else if(rc < 0){
			status_mark_bits_set(msgq->status,MSGQSTAT_MARK_ERROR);
			msgq->error = rc;
		}		
	}
	printf("entering callback\n");
    status_mark_bits_clear(msgq->status,MSGQSTAT_MARK_INTCONTEXT);
}

/*濡傛灉瑕佸湪澶氬鐞嗗櫒闂达紝鏀寔鍚屼竴瀵硅薄娉ㄥ唽閽堝涓嶅悓proc鐨勪簨浠讹紝骞朵笖涓嶅悓浜嬩欢鍙互浣跨敤涓嶅悓浼樺厛绾э紝鍒欓渶瑕佺敤閾捐〃绠＄悊锛屽皢procid锛宔vent_level锛宖unc鎵撳寘閾炬帴璧锋潵锛�
  褰撲簨浠跺彂鐢熸椂锛岄亶鍘嗛摼琛紝渚濇鎵ц鎵�彂鐢熺殑proc-level(eventid)浜嬩欢锛屽悓涓�roc-level浜嬩欢锛屽厛娉ㄥ唽鍒欏厛鎵ц銆備絾鏄敞鍐屼簨浠朵笉瀹滆繃澶氥�
  鐩墠澶氬鐞嗗櫒闂村彲浠ュ悓鏃舵敞鍐屼笉鍚宲roc鐨勪簨浠讹紝浣嗘槸鎵�湁鐨勪簨浠堕兘浣跨敤鍚屼竴浼樺厛绾э紝浼樺厛绾у彲浠ュ姩鎬佷慨鏀癸紝浣嗘槸蹇呴』鍦ㄦ湭娉ㄥ唽浜嬩欢鐨勬儏鍐典笅锛涙墍鏈変簨浠跺叡浜悓涓�釜鍥炶皟鍑芥暟銆�
 */
int messageq_notify_level_set(messageq_t *msgq,uint16_t level) 
{
	printf("input level is %d\n",level);
	if(!status_mark_bits_test(msgq->status,MSGQSTAT_MARK_VALID)){
		printf("invalid messageq!\n");
		return EMESSAGEQ_INVALID;
	}
	
	/*濡傛灉娉ㄥ唽*/
	if(status_notifys_get(msgq->status) != 0){
		printf("can not change event level when events registered.\n");
		return EMESSAGEQ_EVTLEVEL;
	}
	
	if(!check_event_level(level)){
		printf("invalid event level\n");
		return EMESSAGEQ_EVTLEVEL;
	}
	
	status_evtlvl_set(msgq->status,level);
	printf("the evtlvlis %d\n",status_evtlvl_get(msgq->status));
	
	return 0;
}
 
int messageq_notifier_interval_set(messageq_t *msgq,uint32_t interval)
{
	if(!status_mark_bits_test(msgq->status,MSGQSTAT_MARK_VALID)){
		printf("invalid messageq!\n");
		return EMESSAGEQ_INVALID;
	}
	
	msgq->interval = interval;
	return 0;
}

int messageq_notifier_register(messageq_t *msgq,const uint16_t rpid,messageq_func_t func)
{
	int rc;

	if(!status_mark_bits_test(msgq->status,MSGQSTAT_MARK_VALID)){
		printf("invalid messageq!\n");
		return EMESSAGEQ_INVALID;
	}
	
	if(rpid == MultiProc_self())
		return EMESSAGEQ_PROCID;
	
	/*涓嶅厑璁竑unc銆乵sgq->func閮戒负绌猴紝涔熶笉鍏佽func銆乵sgq->func閮藉瓨鍦紝浣嗘槸func != msgq->func*/
	if(!func && !msgq->func){
		printf("messageq have't registered but func is null!\n");
		return EMESSAGEQ_CALLBACK;
	}else if(func && msgq->func && (msgq->func != func)){
		printf("some events have been registered,can't change callback!\n");
		return EMESSAGEQ_CALLBACK;
	}
	printf("rpid is %hu,line id is %d,event id is %d\n",rpid,MESSAGEQ_DEFAULT_LINE,status_evtlvl_get(msgq->status));
	rc = Notify_registerEvent(rpid,
							  MESSAGEQ_DEFAULT_LINE,
							  status_evtlvl_get(msgq->status),
							  message_notifier_callback,
							  (UArg)msgq);
	if(rc < 0)
		return EMESSAGEQ_FAIL;
	
	msgq->func = func;
	
	status_notifys_increase(msgq->status);
	
	return SUCCESS;
}

int messageq_notifier_unregister(messageq_t *msgq,const uint16_t rpid)
{
	int rc;

	if(!status_mark_bits_test(msgq->status,MSGQSTAT_MARK_VALID)){
		printf("invalid messageq!\n");
		return EMESSAGEQ_INVALID;
	}
	
	if(status_notifys_get(msgq->status) == 0){
		printf("notifier unregistered more than registered!\n");
	}
	
	if(!msgq->func)
		return 0;
	
	rc = Notify_unregisterEvent(rpid,
						   MESSAGEQ_DEFAULT_LINE,
						   status_evtlvl_get(msgq->status),
						   message_notifier_callback,
						   (UArg)msgq);
	
	if(rc < 0)
		return EMESSAGEQ_PROCID;
	
	status_notifys_decrease(msgq->status);
	if(status_notifys_get(msgq->status) == 0)
		msgq->func = NULL;
	
	return SUCCESS;
}
void  print_buf(message_buffer_t *message_buffer)
{
	printf("message_buffer heap id is %hu",message_buffer->heapid);
	if(message_buffer->heaptype==0)
		printf("buffer heap type is bufmp\n");
	else if(message_buffer->heaptype==1)
		printf("buffer heap type is memmp\n");
	else
		printf("buffer heap type is last type\n");
	printf("buffer handle is %p\n",message_buffer->handle);
}
int messageq_print_status(messageq_t *msgq)
{
	if(!msgq){
		printf("invalid messageq!");
		return -1;
	}
	
	if(!status_mark_bits_test(msgq->status,MSGQSTAT_MARK_VALID)){
		printf("invalid messageq?");
	}
	
	printf("status of messageq %s as below:\n",object_name(&msgq->obj));

	printf("Have mailbox?:%s,handle %p ",
		status_mark_bits_test(msgq->status,MSGQSTAT_MARK_BOX) ? "YES!" : "NO!",msgq->handle);

	printf("Mailbox opened:%d ",status_rboxs_get(msgq->status));
	printf("Notifiers:%d,callback:%p ",status_notifys_get(msgq->status),msgq->func);
	printf("Event level:%d ",status_evtlvl_get(msgq->status));
	printf("Mailbox attached:%s,qid %u \n",(msgq->boxid.qid == MessageQ_INVALIDMESSAGEQ) ? "none" : "yes",msgq->boxid.qid);	
	return 0;
}
