#ifndef _SYSLINK_NOTIFIER_H
#define _SYSLINK_NOTIFIER_H

#include "syslink_common.h"
#include <ti/ipc/Notify.h>

#include "object.h"
#include "bilist.h"
#include "bit_region.h"


#ifdef _cplusplus
extern "C"{
#endif

#define EVENTQ_DEFAULT_CAPACITY					128u
#define EVENTQ_IGNORE_CAPACITY					0xFFFF

/*procid 默认值、最大值、忽略标志值*/
#define NOTIFIER_DEFAULT_PROCID					1u

#define NOTIFIER_MAX_PROCID						2u

#define NOTIFIER_IGNORE_PROCID					0xffff

/*lineid 默认值、最大值、忽略标志值*/
#define NOTIFIER_DEFAULT_LINEID					1u

#define NOTIFIER_MAX_LINEID						4u

#define NOTIFIER_IGNORE_LINEID					0xffff

/*eventid 默认值、最大值、忽略标志值*/
#define NOTIFIER_DEFAULT_EVENTID				5u

#define NOTIFIER_MAX_EVENTID					32u

#define NOTIFIER_IGNORE_EVENTID					0xffffu

/*默认evid值*/
#define NOTIFIER_DEFAULT_EVID					{ NOTIFIER_DEFAULT_PROCID,NOTIFIER_DEFAULT_LINEID,NOTIFIER_MAX_EVENTID }

#define NOTIFIER_DEFAULT_CAPACITY				EVENTQ_DEFAULT_CAPACITY

#define NOTIFIER_IGNORE_CAPACITY				EVENTQ_IGNORE_CAPACITY

/*notifier open mode*/
#define NOTIFIER_MODE_CACHE						(1 << 0)

#define NOTIFIER_MODE_NOSEND					(1 << 1)

#define NOTIFIER_MODE_NORECV					(1 << 2)

/*对event的处理结果*/
#define EVENT_FAILED   -1	//处理失败，需要插入缓存。

#define EVENT_HANDLED	1	//事件已经完全处理，不会插入到缓存队列里。

#define EVENT_SUCCESS	0	//处理成功，插入缓存。

/*---------------------------------------------------------------------------------------------------------------------------------------------*/
//notifier status 域的定义及操作
/*notifier 的状态标记位域*/
#define NTFSTAT_INITIALIZER						0

#define NTFSTAT_MARK_START						0
#define NTFSTAT_MARK_BITS						8
#define NTFSTAT_MARK_REGION						REGION(NTFSTAT_MARK_START,NTFSTAT_MARK_BITS)

#define NTFSTAT_MARK_VALID						REGION_FLAG_BIT(NTFSTAT_MARK_REGION,NTFSTAT_MARK_START,0)
#define NTFSTAT_MARK_OPEN						REGION_FLAG_BIT(NTFSTAT_MARK_REGION,NTFSTAT_MARK_START,1)
#define NTFSTAT_MARK_QUEUE						REGION_FLAG_BIT(NTFSTAT_MARK_REGION,NTFSTAT_MARK_START,2)
#define NTFSTAT_MARK_NTFREG						REGION_FLAG_BIT(NTFSTAT_MARK_REGION,NTFSTAT_MARK_START,3)
#define NTFSTAT_MARK_NOSEND						REGION_FLAG_BIT(NTFSTAT_MARK_REGION,NTFSTAT_MARK_START,4)
#define NTFSTAT_MARK_NORECV						REGION_FLAG_BIT(NTFSTAT_MARK_REGION,NTFSTAT_MARK_START,4)

#define notifier_mark_set(stat,bits)			bits_set(stat,bits,NTFSTAT_MARK_REGION)
#define notifier_mark_clear(stat,bits)			bits_clear(stat,bits,NTFSTAT_MARK_REGION)
#define notifier_mark_test(stat,bits)			bits_test(stat,bits,NTFSTAT_MARK_REGION)
#define notifier_mark_setval(stat,val)			region_set(stat,val,NTFSTAT_MARK_REGION,NTFSTAT_MARK_START)
#define notifier_mark_clear_all(stat)			region_clear(stat,NTFSTAT_MARK_REGION)

/*notifier的notify计数位域，用于记录已注册的notify个数*/
#define NTFSTAT_NTFCNT_START					(NTFSTAT_MARK_START + NTFSTAT_MARK_BITS)
#define NTFSTAT_NTFCNT_BITS						8
#define NTFSTAT_NTFCNT_REGION					REGION(NTFSTAT_NTFCNT_START,NTFSTAT_NTFCNT_BITS)

#define notifier_ntfcnt_get(stat)				region_get(stat,NTFSTAT_NTFCNT_REGION,NTFSTAT_NTFCNT_START)
#define notifier_ntfcnt_set(stat,val)			region_set(stat,val,NTFSTAT_NTFCNT_REGION,NTFSTAT_NTFCNT_START)
#define notifier_ntfcnt_compare(stat,val)		region_compare(stat,val,NTFSTAT_NTFCNT_REGION,NTFSTAT_NTFCNT_START)
#define notifier_ntfcnt_clear(stat)				region_clear(stat,NTFSTAT_NTFCNT_REGION)
#define notifier_ntfcnt_increase(stat)			region_add(stat,1,NTFSTAT_NTFCNT_REGION,NTFSTAT_NTFCNT_START)
#define notifier_ntfcnt_decrease(stat)			region_sub(stat,1,NTFSTAT_NTFCNT_REGION,NTFSTAT_NTFCNT_START)
#define notifier_ntfcnt_test(stat)				notifier_ntfcnt_compare(stat,0)


/*---------------------------------------------------------------------------------------------------------------------------------------------*/
typedef struct notifier_event_id notifier_evid_t;
typedef struct notifier_event notifier_event_t;
typedef struct notifier notifier_t;
typedef struct event_queue_head event_queue_head_t;
typedef int (*notifier_func_t)(void *,notifier_event_t *);
typedef union notifier_params notifier_params_t;

/***********notifier_event_t、notifer_evid_t*************/
struct notifier_event_id{
	uint16_t			procid;
	uint16_t 			lineid;
	uint32_t 			eventid;
};



struct notifier_event{
	notifier_evid_t 	ev_evid;
	uint32_t 			ev_message;
	
	#define ev_proc		ev_evid.procid
	#define ev_line		ev_evid.lineid
	#define ev_event	ev_evid.eventid
};

#define event_malloc() 		(notifier_event_t*)malloc(sizeof(notifier_event_t))
#define event_free(event) 	free(event)
/******************************************************/


/********************notifier**************************/
struct event_queue_head;

struct notifier{
	object_t 			 obj;		//object 头
	uint32_t	         status;
	notifier_func_t 	 func;		//事件处理回调函数
	event_queue_head_t	*queue;		//事件队列
};

//arg目前只支持notifer_evid_t
union notifier_params{
	
	uint32_t capacity;
	
};

/******************************************************/



/************************apis**************************/
/*初始化、创建、销毁*/
int notifier_construct(notifier_t *notify,const char *name,object_type_t type,object_t *parent);
int notifier_destruct(notifier_t *notify);
int	notifier_create(notifier_t **pnotify,const char *name,object_type_t type,object_t *parent);
int notifier_destroy(notifier_t *notify);

/*设置继承关系*/
int notifier_set_parent(notifier_t *notify,object_t *parent);

/*打开、关闭*/
int notifier_open(notifier_t *notify,uint32_t mode,notifier_params_t *params);
int notifier_close(notifier_t *notify);

/*注册、注销事件*/
int notifier_register_event(notifier_t *notify,const notifier_evid_t *evid,notifier_func_t func);
int notifier_unregister_event(notifier_t *notify,const notifier_evid_t *evid);

/*发送事件消息*/
int notifier_send_event(notifier_t *notify,notifier_evid_t *evid,uint32_t payload);
int notifier_send_event_nowait(notifier_t *notify,notifier_evid_t *evid,uint32_t playload);

/*获取队列头的第一个事件，队列空时，会等待/超时等待/直接返回状态*/
int notifier_event_get(notifier_t *notify,notifier_event_t **event);
int notifier_event_get_timeout(notifier_t *notify,UInt time,notifier_event_t **event);
int notifier_event_tryget(notifier_t *notify,notifier_event_t **event);

/*根据获取队列中第一个域evid相匹配的事件，如果队列为空，会等待/超时等待/直接返回*/
//其中timeout的意思是在队列为空时，等待多久后返回，而如果队列不为空则会立即返回，即便没有与evid相匹配的事件。
//并不是指阻塞等待与evid相匹配的事件多久后返回。
int notifier_event_match_get(notifier_t *notify,const notifier_evid_t *evid,notifier_event_t **event);
int notifier_event_match_tryget(notifier_t *notify,const notifier_evid_t *evid,notifier_event_t **event);
int notifier_event_match_timeout_tryget(notifier_t *notify,const notifier_evid_t *evid,UInt time,notifier_event_t **event);

/*等待具体的事件，只到指定事件返回为止。*/
//查询队列中是否有对应事件，并立即返回查询结果
//查询队列中是否有对应事件，并返回查询结果，如果队列为空则等待time后返回结果。
int  notifier_event_wait(notifier_t *notify,const notifier_event_t *event);
int notifier_event_trywait(notifier_t *notify,const notifier_event_t *event);
int notifier_event_timeout_trywait(notifier_t *notify,const notifier_event_t *event,UInt time);


/******************************************************/

#ifdef _cplusplus
}
#endif

#endif
