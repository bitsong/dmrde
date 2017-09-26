
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/BIOS.h>

#include "syslink_notifier.h"

/****************************event queue****************************/
typedef enum event_queue_stat{
	EVQUEUE_STAT_NORMAL,
	EVQUEUE_STAT_NOMEM,
	EVQUEUE_STAT_OVERFLOW,
	EVQUEUE_STAT_LAST
}evque_stat_t;

typedef struct event_queue{
	bilist_node_t	 node;
	notifier_event_t *event;
}event_queue_t;

struct event_queue_head{
	bilist_head_t 	list;
	uint16_t 		capacity;	//
	Semaphore_Handle	avail;		//dsp 侧
	evque_stat_t	status;
};



static inline void event_queue_init(event_queue_head_t *evq,uint16_t capacity)
{
	bilist_construct(&evq->list);
	
	evq->capacity = capacity;
//	sem_init(&evq->avail,0,0);

	Error_Block eb;
	Semaphore_Params        semParams;//In Task
	Semaphore_Params_init(&semParams) ;
	semParams.mode =  Semaphore_Mode_COUNTING_PRIORITY;

	evq->avail= Semaphore_create(0, &semParams, &eb);

	evq->status	  = EVQUEUE_STAT_NORMAL;
}

static inline void event_queue_deinit(event_queue_head_t *evq)
{
	evq->capacity 	= 0;
	evq->status		= EVQUEUE_STAT_LAST;
//	sem_destroy(&evq->avail);
	Semaphore_delete(&evq->avail);

	bilist_destruct(&evq->list);
}
/*
static inline Bool event_queue_empty(event_queue_head_t *evq)
{
	int semval;
//	sem_getvalue(&evq->avail,&semval);
	semval=Semaphore_getCount(evq->avail);
	
	return (semval == 0);
}
*/
static inline Bool event_queue_full(event_queue_head_t *evq)//通过list_empty?
{
	int semval;
//	sem_getvalue(&evq->avail,&semval);
	semval=Semaphore_getCount(evq->avail);
	
	return (evq->capacity != 0 && semval >= evq->capacity);
	
	//return bilist_empty(&evq->list);
}
/*
static inline evque_stat_t event_queue_status(event_queue_head_t *evq)
{
	return evq->status;
}

static inline void event_queue_resume(event_queue_head_t *evq)
{
	evq->status = EVQUEUE_STAT_NORMAL;
}
*/
static inline int event_queue_push(event_queue_head_t *evq,notifier_event_t *event)
{
	//如果event_queue处于非正常状态则什么都不做。
	if(evq->status != EVQUEUE_STAT_NORMAL)
		return ENOTIFIER_EVQUESTAT;
	
	//检查event_queue是否溢出，如果溢出则记录此状态
	if(event_queue_full(evq)){
		evq->status = EVQUEUE_STAT_OVERFLOW;
		return ENOTIFIER_EVQOVERFLOW;
	}
	
	//如果内存不足则记录此状态。
	event_queue_t *evnew = (event_queue_t*)malloc(sizeof(event_queue_t));
	if(!evnew){
		evq->status = EVQUEUE_STAT_NOMEM;
		return ENOTIFIER_MEMORY;
	}
	
	//初始化event_queue_t节点并添加进队列链表，并且事件计数 +1
	evnew->event = event;
	bilist_node_init(&evnew->node);
	bilist_add_tail(&evq->list,&evnew->node);
	Semaphore_post(evq->avail);
	
	return 0;
}

static notifier_event_t* _event_queue_pop(event_queue_head_t *evq)
{
	bilist_node_t 	*qnode;
	event_queue_t 	*q;
	notifier_event_t *event;
	
	//取出队列头节点，并转换成event_queue_t类型
	qnode = evq->list.next;
	q = container_of(qnode,event_queue_t,node);
	//删除队列头节点
	bilist_del(qnode);
	
	//缓存event，释放event_queue_t指针
	event = q->event;
	free(q);
	
	return event;
}
/*
取出对列头事件，并移除该事件节点。
返回值：队列不为空则返回事件，否则返回NULL。
*/
static notifier_event_t*
	event_queue_pop(event_queue_head_t *evq)
{
//	sem_wait(&evq->avail);
	Semaphore_pend(evq->avail,BIOS_WAIT_FOREVER);
	return _event_queue_pop(evq);
}

static notifier_event_t* event_queue_try_pop(event_queue_head_t *evq)
{
//	if(sem_trywait(&evq->avail) < 0)
	if(Semaphore_pend(evq->avail,BIOS_NO_WAIT) == FALSE)
		return NULL;
	
	return _event_queue_pop(evq);
}

static notifier_event_t* event_queue_pop_timeout(event_queue_head_t *evq,UInt time)
{
	//errno = 0;
//	if(sem_timedwait(&evq->avail,time) < 0){
	if(Semaphore_pend(evq->avail,time) == FALSE){
		Log_error0("sem_timedwait");
		return NULL;
	}
	
	return _event_queue_pop(evq);
}

/*
精确查找对应的事件，找到后从队列中移除事件,time 为空则阻塞等待，time不为空则阻塞time后返回NULL。
匹配条件：event所有域均相等。
返回值：找到返回1，失败返回0
*/

static Bool event_equal(const notifier_event_t *that,const notifier_event_t *this)
{
	return ((this->ev_proc    ==  that->ev_proc)&&
			(this->ev_line    ==  that->ev_line)&&
			(this->ev_event   ==  that->ev_event)&&
			(this->ev_message ==  that->ev_message));
}
static Bool _event_queue_search_event_out(
					event_queue_head_t *evq,
					const notifier_event_t *event)
{
	bilist_node_t *qnode,*qnext;
	event_queue_t *q;
	
	//遍历所有事件，如果找到第一个匹配的则返回1
	bilist_for_each_safe(&evq->list,qnode,qnext){
		
		//转换为event_queue_t类型，并比较该事件与event是否相等
		q = container_of(qnode,event_queue_t,node);
		if(!event_equal(q->event,event))
			continue;
		
		//如果该事件与event匹配，则移除该事件链表节点，并释放event_queue_t
		bilist_del(qnode);
		free(q);
		return 1;
	}
	
	//没有找到匹配的事件则返回0.
	return 0;
}

static Bool event_queue_search_event_out(
					event_queue_head_t *evq,
					const notifier_event_t *event,
					UInt time)
{
	int rc;

	//首先确认队列有事件，如果没有则等待。
	if(time && (Semaphore_pend(evq->avail,time) == FALSE)){
		Log_error0("sem_timedwait");
		return 0;
	}else{
		Semaphore_pend(evq->avail, BIOS_WAIT_FOREVER);
	}
	
	rc = _event_queue_search_event_out(evq,event);
	
	//没有找到匹配的事件则返回0.
	if(!rc)
		Semaphore_post(evq->avail);
	
	return rc;
}

static int event_queue_search_event_tryout(
					event_queue_head_t *evq,
					const notifier_event_t *event)
{
	int rc;
	
	if(Semaphore_pend(evq->avail,BIOS_NO_WAIT) == FALSE)
		return ENOTIFIER_FAIL;
	
	rc = _event_queue_search_event_out(evq,event);
	
	if(!rc)
		Semaphore_post(evq->avail);
	
	return rc;
}

/*
根据evid查找对应事件，可以选择性的忽略evid部分域的匹配
返回值：找到返回匹配的event，失败返回 NULL。
匹配策略：比较evid所有域，如果该域设置为忽略则不进行比较。
如果有任何一个需要比较的域不想等，则不匹配，反之，如果所有需要
比较的域均相等，则匹配。
*/
static inline Bool event_queue_match(const event_queue_t *q,const notifier_evid_t *evid)
{
	return !((evid->procid  != NOTIFIER_IGNORE_PROCID  && q->event->ev_proc  != evid->procid) || 
			 (evid->lineid  != NOTIFIER_IGNORE_LINEID  && q->event->ev_line  != evid->lineid) || 
			 (evid->eventid != NOTIFIER_IGNORE_EVENTID && q->event->ev_event != evid->eventid));
}

static inline notifier_event_t* 
	_event_queue_search_evid_out(
				event_queue_head_t *evq,
				const notifier_evid_t *evid)
{
	bilist_node_t *qnode,*qnext;
	event_queue_t *q;
	notifier_event_t *event;

	//遍历所有事件，如果找到第一个匹配的事件则返回该事件。
	bilist_for_each_safe(&evq->list,qnode,qnext){
		
		//转换为event_queue_t类型，并比较该事件的evid与evid是否相等
		q = container_of(qnode,event_queue_t,node);
		if(!event_queue_match(q,evid))
			continue;
		
		bilist_del(qnode);
		event = q->event;
		free(q);
		return event;
	}
	
	return NULL;
}

static notifier_event_t* 
	event_queue_search_evid_out(
				event_queue_head_t *evq,
				const notifier_evid_t *evid,
				UInt time)
{
	notifier_event_t *event = NULL;
	
	if(time && (Semaphore_pend(evq->avail,time) == FALSE)){
		Log_error0("sem_timedwait");
		return NULL;
	}else{
		Semaphore_pend(evq->avail, BIOS_WAIT_FOREVER);
	}
	
	event = _event_queue_search_evid_out(evq,evid);
	if(!event)
		Semaphore_post(evq->avail);
	
	return event;
}

static notifier_event_t* 
	event_queue_search_evid_tryout(
				event_queue_head_t *evq,
				const notifier_evid_t *evid)
{
	notifier_event_t *event = NULL;
	
	if(Semaphore_pend(evq->avail, BIOS_NO_WAIT)==FALSE)
		return NULL;
	
	event = _event_queue_search_evid_out(evq,evid);
	if(!event)
		Semaphore_post(evq->avail);
	
	return event;
}

static inline void event_queue_clear(event_queue_head_t *evq)  //是否需要注销event，并且须在notifier层完成？
{
//	while(sem_trywait(&evq->avail) == 0){
	while(Semaphore_pend(evq->avail, BIOS_NO_WAIT)==TRUE){
		_event_queue_pop(evq);
	}
}

event_queue_head_t*
	event_queue_create(uint16_t capacity)
{
	event_queue_head_t *queue;
	
	queue = malloc(sizeof(event_queue_head_t));
	if(!queue){
		Log_error0("out of memory for queue!");
		return NULL;
	}
	
	event_queue_init(queue,capacity);
	
	return queue;
}

static inline void event_queue_destroy(event_queue_head_t *evq)
{
	event_queue_clear(evq);
	event_queue_deinit(evq);
}
/******************************************************************/


/****************************notifier****************************/

int notifier_construct(notifier_t *notify,const char *name,object_type_t type,object_t *parent)
{
	int rc;
	
	if((rc = object_init(&notify->obj,name,OBJECT_TYPE_NOTIFIER,parent)) < 0)
		return rc;
	
	//notify->evid 	= NULL;
	notify->queue 	= NULL;
	notify->func	= NULL;
	notify->status  = NTFSTAT_INITIALIZER;
	notifier_mark_set(notify->status,NTFSTAT_MARK_VALID);
	
	return SUCCESS;
}

int notifier_destruct(notifier_t *notify)
{
	if(!notifier_mark_test(notify->status,NTFSTAT_MARK_VALID)){
		Log_warning0("invalid notifier");
		return ENOTIFIER_INVALID;
	}
	
	if(notifier_mark_test(notify->status,NTFSTAT_MARK_NTFREG)){
		Log_error0("some event not unregistered!");
		return ENOTIFIER_NOTUNREGISTERED;
	}
	
	if(notifier_mark_test(notify->status,NTFSTAT_MARK_QUEUE) && (notify->queue!=NULL)){
		event_queue_destroy(notify->queue);
		free(notify->queue);
		notify->queue = NULL;
		Log_info0("event queue released!");
	}
	
	notify->status = NTFSTAT_INITIALIZER;
	
	object_deinit(&notify->obj);
	
	return SUCCESS;
}

int	notifier_create(notifier_t **pnotify,const char *name,object_type_t type,object_t *parent) //返回值？
{
    int rc = 0;

	notifier_t *notify = malloc(sizeof(notifier_t));
	if(!notify)
		return ENOTIFIER_MEMORY;

	if((rc = notifier_construct(notify,name,OBJECT_TYPE_NOTIFIER,parent)) < 0){
		Log_error0("notifier construct failed!");
		free(notify);
		return rc;
	}
	
	*pnotify = notify;
	
	return SUCCESS;
}
					
int notifier_destroy(notifier_t *notify)
{
	int rc;
	
	rc = notifier_destruct(notify);
	if(rc == ENOTIFIER_NOTUNREGISTERED)
		return rc;
	
	free(notify);
	
	return SUCCESS;
}

int notifier_set_parent(notifier_t *notify,object_t *parent)
{
	return object_change_parent(&notify->obj,parent);
}

static void notifier_callback(uint16_t procid,uint16_t lineid,uint32_t eventid,UArg arg,uint32_t payload)
{
	notifier_event_t *event;
	notifier_t *notify = (notifier_t*)arg;
	int rc = 0;

	//创建一个event，创建失败时候，如果创建了event_queue，则push入队，否则直接退出。
	event = event_malloc();
	if(!event){
		Log_error0("event malloc out of memory");
		if(notify->queue)
			notify->queue->status = EVQUEUE_STAT_NOMEM;
		return;
	}
	
	event->ev_proc  	= procid;
	event->ev_line  	= lineid;
	event->ev_event 	= eventid;
	event->ev_message   = payload;
	
	if(notify->func)
		rc = notify->func(notify,event);
	
	if( rc == EVENT_HANDLED)
		return;
	
	if(rc < 0)
		Log_info4("event:%u,%u,%u,payload:%u process failed.",procid,lineid,eventid,payload);
		
	
	if(notify->queue && (rc = event_queue_push(notify->queue,event)) < 0)//??????????????????????????????????????
		Log_warning0("event_queue_push failed!");
}

/*single模式是该notifier只使用单一事件源。general则可以使用任何可用事件但是在open时候没有打开。*/
/*single模式需要填充params的evid，general模式则只需要填充evt_func，只发模式，不需要parmas参数*/
int notifier_open(notifier_t *notify,uint32_t mode,notifier_params_t *params)
{
	event_queue_head_t *queue;

	if(!notifier_mark_test(notify->status,NTFSTAT_MARK_VALID)){
		Log_error0("invalid notifier!");
		return ENOTIFIER_INVALID;
	}
	
	if(notifier_mark_test(notify->status,NTFSTAT_MARK_OPEN)){
		Log_error0("notifier have been open!");
		return ENOTIFIER_REOPEN;
	}
	
	if(mode & NOTIFIER_MODE_CACHE){
		if(params->capacity == EVENTQ_IGNORE_CAPACITY)
			params->capacity = EVENTQ_DEFAULT_CAPACITY;
			
		queue = malloc(sizeof(event_queue_head_t));
		if(!queue){
			Log_error0("out of memory for queue!");
			return ENOTIFIER_MEMORY;
		}
		
		event_queue_init(queue,params->capacity);
		notify->queue = queue;
		notifier_mark_set(notify->status,NTFSTAT_MARK_QUEUE);	
	}
	
	if(mode & NOTIFIER_MODE_NOSEND)
		notifier_mark_set(notify->status,NTFSTAT_MARK_NOSEND);
	
	if(mode * NOTIFIER_MODE_NORECV)
		notifier_mark_set(notify->status,NTFSTAT_MARK_NORECV);

	notifier_mark_set(notify->status,NTFSTAT_MARK_OPEN);
	
	return SUCCESS;
	
}

int notifier_close(notifier_t *notify)
{
	
	if(!notifier_mark_test(notify->status,NTFSTAT_MARK_OPEN))
		return 0;
	
	if(	notifier_mark_test(notify->status,NTFSTAT_MARK_QUEUE) && 
		notifier_mark_test(notify->status,NTFSTAT_MARK_NTFREG) ){
		Log_error0("please unregister all events before colse the notifier!");
		return -ENOTIFIER_NOTUNREGISTER;
	}
	
	if(notify->queue){
		event_queue_destroy(notify->queue);
		free(notify->queue);
		notify->queue = NULL;
		notifier_mark_clear(notify->status,NTFSTAT_MARK_QUEUE);
	}

	notifier_mark_clear(notify->status,NTFSTAT_MARK_OPEN);
	
	return SUCCESS;
}

/*允许func为NULL，但是必须已经有注册成功的事件，才可以
  已经成功注册过事件后，func必须与之前的相同，即所有的
  事件必须共享同一个func
*/
int notifier_register_event(notifier_t *notify,const notifier_evid_t *evid,notifier_func_t func)
{
	int rc;
	if(!notifier_mark_test(notify->status,NTFSTAT_MARK_OPEN)){
		Log_error0("notifier not open!");
		return ENOTIFIER_OPEN;
	}
	
	if(notifier_mark_test(notify->status,NTFSTAT_MARK_NORECV)){
		Log_error0("mode error!");
		return ENOTIFIER_MODE;
	}
	
	if(!func && !notify->func){
		Log_error0("notifier have't registered but func is null!");
		return ENOTIFIER_CALLBACK;
	}else if(func && notify->func && (notify->func != func)){
		Log_error0("notifier events have been registered,can't change callback!");
		return ENOTIFIER_CALLBACK;
	}
	
	rc = Notify_registerEventSingle(evid->procid,evid->lineid,evid->eventid,notifier_callback,(UArg)notify);
	if(rc < 0)
		return rc;
	
	notifier_ntfcnt_increase(notify->status);
	notifier_mark_set(notify->status,NTFSTAT_MARK_NTFREG);
	
	return rc;
}

int notifier_unregister_event(notifier_t *notify,const notifier_evid_t *evid)
{

    int rc;

	if(!notifier_mark_test(notify->status,NTFSTAT_MARK_OPEN)){
		Log_error0("notifier not open!");
		return ENOTIFIER_OPEN;
	}
	
	if(notifier_mark_test(notify->status,NTFSTAT_MARK_NORECV)){
		Log_error0("mode error!");
		return ENOTIFIER_MODE;
	}
	
	if(!notifier_mark_test(notify->status,NTFSTAT_MARK_NTFREG))
		Log_warning0("no event registered!");
	
	rc = Notify_unregisterEventSingle(evid->procid,evid->lineid,evid->eventid);
	if(rc < 0)
		return rc;
	
	if(notifier_ntfcnt_test(notify->status)){
		notifier_ntfcnt_decrease(notify->status);
		if(notifier_ntfcnt_get(notify->status) == 0)
			notifier_mark_clear(notify->status,NTFSTAT_MARK_NTFREG);
	}
	
	return 0;
}

int notifier_event_get(notifier_t *notify,notifier_event_t **event)
{
	//int rc;
	notifier_event_t *evt;
	
	if(notify->queue == NULL){
		Log_error0("notifier event queue not created!");
		return ENOTIFIER_MODE;
	}
	
	if(!notifier_mark_test(notify->status,NTFSTAT_MARK_NTFREG)){
		Log_error0("notifier event not registerd!");
		return ENOTIFIER_NOTREGISTERED;
	}
	
	evt = event_queue_pop(notify->queue);
	if(!evt){
		*event = NULL;
		return ENOTIFIER_FAIL;
	}
	
	*event = evt;
	
	return 0;
}

int notifier_event_get_timeout(notifier_t *notify,UInt time,notifier_event_t **event)
{
	notifier_event_t *evt;
	
	if(notify->queue == NULL){
		Log_error0("notifier event queue not created!");
		return ENOTIFIER_MODE;
	}
	
	if(!notifier_mark_test(notify->status,NTFSTAT_MARK_NTFREG)){
		Log_error0("notifier event not registerd!");
		return ENOTIFIER_NOTREGISTERED;
	}
	
	evt = event_queue_pop_timeout(notify->queue,time);
	if(!evt){
		*event = NULL;
		return ENOTIFIER_FAIL;
	}
	
	*event = evt;
	
	return 0;
}

int notifier_event_tryget(notifier_t *notify,notifier_event_t **event)
{
	notifier_event_t *evt;
	
	if(notify->queue == NULL){
		Log_error0("notifier event queue not created!");
		return ENOTIFIER_MODE;
	}
	
	if(!notifier_mark_test(notify->status,NTFSTAT_MARK_NTFREG)){
		Log_error0("notifier event not registerd!");
		return ENOTIFIER_NOTREGISTERED;
	}
	
	evt = event_queue_try_pop(notify->queue);
	if(!evt){
		*event = NULL;
		return ENOTIFIER_FAIL;
	}
	
	*event = evt;
	
	return 0;
}

int notifier_event_match_get(notifier_t *notify,const notifier_evid_t *evid,notifier_event_t **event)
{
	if(notify->queue == NULL){
		Log_error0("notifier event queue not created!");
		return ENOTIFIER_MODE;
	}
	
	if(!notifier_mark_test(notify->status,NTFSTAT_MARK_NTFREG)){
		Log_error0("notifier event not registerd!");
		return ENOTIFIER_NOTREGISTERED;
	}
	
	notifier_event_t *evt;
	
	//while(!(evt = event_queue_search_evid_out(notify->queue,evid,NULL)));
	
	evt = event_queue_search_evid_out(notify->queue,evid,NULL);
	if(!evt){
		*event = NULL;
		return ENOTIFIER_FAIL;
	}
	
	*event = evt;
	
	return 0;
}

int notifier_event_match_tryget(notifier_t *notify,const notifier_evid_t *evid,notifier_event_t **event)
{
	if(notify->queue == NULL){
		Log_error0("notifier event queue not created!");
		return ENOTIFIER_MODE;
	}
	
	if(!notifier_mark_test(notify->status,NTFSTAT_MARK_NTFREG)){
		Log_error0("notifier event not registerd!");
		return ENOTIFIER_NOTREGISTERED;
	}
	
	notifier_event_t *evt;
	
	evt = event_queue_search_evid_tryout(notify->queue,evid);
	if(!evt){
		*event = NULL;
		return ENOTIFIER_FAIL;
	}
	
	*event = evt;
	
	return 0;
}

int notifier_event_match_timeout_tryget(
						notifier_t *notify,
						const notifier_evid_t *evid,
						UInt time,
						notifier_event_t **event)
{
	if(notify->queue == NULL){
		Log_error0("notifier event queue not created!");
		return ENOTIFIER_MODE;
	}
	
	if(!notifier_mark_test(notify->status,NTFSTAT_MARK_NTFREG)){
		Log_error0("notifier event not registerd!");
		return ENOTIFIER_NOTREGISTERED;
	}
	
	notifier_event_t *evt;
	
	evt = event_queue_search_evid_out(notify->queue,evid,time);
	if(!evt){
		*event = NULL;
		return ENOTIFIER_FAIL;
	}
	
	*event = evt;
	
	return 0;
}

int notifier_event_wait(notifier_t *notify,const notifier_event_t *event)
{
	if(notify->queue == NULL){
		Log_error0("notifier event queue not created!");
		return ENOTIFIER_MODE;
	}
	
	if(!notifier_mark_test(notify->status,NTFSTAT_MARK_NTFREG)){
		Log_error0("notifier event not registerd!");
		return ENOTIFIER_NOTREGISTERED;
	}

	//while(!(event_queue_search_event_out(notify->queue,event,NULL)));
	if(!event_queue_search_event_out(notify->queue,event,NULL))
		return ENOTIFIER_FAIL;
	
	return 0;
}

int notifier_event_trywait(notifier_t *notify,const notifier_event_t *event)
{
	if(notify->queue == NULL){
		Log_error0("notifier event queue not created!");
		return ENOTIFIER_MODE;
	}
	
	if(!notifier_mark_test(notify->status,NTFSTAT_MARK_NTFREG)){
		Log_error0("notifier event not registerd!");
		return ENOTIFIER_NOTREGISTERED;
	}
	
	if(!event_queue_search_event_tryout(notify->queue,event))
		return ENOTIFIER_FAIL;
	
	return 0;
}

int notifier_event_timeout_trywait(notifier_t *notify,const notifier_event_t *event,UInt time)
{
	if(notify->queue == NULL){
		Log_error0("notifier event queue not created!");
		return ENOTIFIER_MODE;
	}
	
	if(!notifier_mark_test(notify->status,NTFSTAT_MARK_NTFREG)){
		Log_error0("notifier event not registerd!");
		return ENOTIFIER_NOTREGISTERED;
	}
	
	if(!event_queue_search_event_out(notify->queue,event,time))
		return ENOTIFIER_FAIL;
	
	return 0;
}

int notifier_send_event(notifier_t *notify,notifier_evid_t *evid,uint32_t payload)
{
	if(!notifier_mark_test(notify->status,NTFSTAT_MARK_VALID)){
		Log_error0("invalid notifier!");
		return ENOTIFIER_INVALID;
	}
	
	if(notifier_mark_test(notify->status,NTFSTAT_MARK_NOSEND)){
		Log_error0("recv only mode!");
		return ENOTIFIER_MODE;
	}
	
	return Notify_sendEvent(evid->procid,evid->lineid,evid->eventid,payload,TRUE);
}


int notifier_send_event_nowait(notifier_t *notify,notifier_evid_t *evid,uint32_t payload)
{
	if(!notifier_mark_test(notify->status,NTFSTAT_MARK_VALID)){
		Log_error0("invalid notifier!");
		return ENOTIFIER_INVALID;
	}
	
	if(notifier_mark_test(notify->status,NTFSTAT_MARK_NOSEND)){
		Log_error0("recv only mode!");
		return ENOTIFIER_MODE;
	}
	
	return Notify_sendEvent(evid->procid,evid->lineid,evid->eventid,payload,FALSE);
}
