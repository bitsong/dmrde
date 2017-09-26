#ifndef _SYSLINK_MESSAGEQ_H
#define _SYSLINK_MESSAGEQ_H

#include "syslink_common.h"
#include <ti/ipc/MessageQ.h>
#include <ti/ipc/Notify.h>
#include <ti/ipc/SharedRegion.h>
#include <ti/ipc/HeapBufMP.h>
#include <ti/ipc/HeapMemMP.h>

#include "object.h"
#include "bilist.h"
#include "bit_region.h"


typedef HeapBufMP_Params heapbuf_param;
typedef HeapMemMP_Params heapmem_param;


#ifdef _cplusplus
extern "C"{
#endif
/*---------------------------------------------------------------------------------------------------------------------------------------------*/
#define MESSAGEQ_WAITFOREVER 		MessageQ_FOREVER

/*鍐呴儴浣跨敤鐨別vent id*/
#define MESSAGEQ_EVENT_LVL0			6u

#define MESSAGEQ_EVENT_LVL1			9u

#define MESSAGEQ_EVENT_LVL2			12u

#define MESSAGEQ_EVENT_DEFAULT		MESSAGEQ_EVENT_LVL1

#define MESSAGEQ_EVENT_INVALID		0xff

#define check_event_level(lvl)		((lvl) == MESSAGEQ_EVENT_LVL0 || (lvl) == MESSAGEQ_EVENT_LVL1 || (lvl) == MESSAGEQ_EVENT_LVL2)

/*鍐呴儴浣跨敤鐨刲ine id*/
#define MESSAGEQ_RESERVED_LINE		0u

#define MESSAGEQ_DEFAULT_LINE		MESSAGEQ_RESERVED_LINE


/*鍐呴儴鍐呴儴浣跨敤鐨刵otify 鍛戒护*/
#define MESSAGEQ_NOTIFY_SEND		0x10000001u

//...
#define MESSAGEQ_FLAG_NOTIFY        (1 << 0)

#define MESSAGEQ_FLAG_NOCLR         (1 << 1)

//#define MESSAGEQ_NOTIFY_XXXX		0x10000002u
//...

//榛樿鍏变韩鍐呭瓨鍖哄煙
#define MESSAGEQ_DEFAULT_SR			1

//鍥炶皟鍑芥暟杩斿洖鍊硷細
#define MESSAGEQ_RETURN_HANDLED		1

#define MESSAGEQ_RETURN_SUCCESS		0

#define MESSAGEQ_RETURN_ERROR		-1

//#define MESSAGEQ_INVALID_MSGBOX 	()MessageQ_INVALIDMESSAGEQ//0 - FFFFFFFE

#define name_of(obj) 				object_name((object_t*)(obj))

#define box2procid(boxid)			(uint16_t)((boxid)->qid >> 16)//b2pid
/*浠ュ悗鏀惧湪鍚屼竴鐨勪竴涓枃浠朵腑*/

#define MESSAGEQ_MAXTIMEOUT_INTCONTEXT  5//ms
/*messageq鐘舵�鎿嶄綔鍙婄浉鍏冲煙瀹氫箟*/
#define MSGQSTAT_INITIALIZER				0

#define MSGQSTAT_MARK_MASK					0xff
#define MSGQSTAT_MARK_VALID					1 << 0
#define MSGQSTAT_MARK_BOX					1 << 1						//鏈湴msgbox鐨勭姸鎬�
#define MSGQSTAT_MARK_ERROR					1 << 2
#define MSGQSTAT_MARK_INTCONTEXT            1 << 3
//#define MSGQSTAT_ERROR_FATAL				1

#define MSGQSTAT_RBOXS_SHIFT				8							//remote box 琛ㄧず鎵撳紑杩滅鐨刴sgbox
#define MSGQSTAT_RBOXS_MASK					(0xff << MSGQSTAT_RBOXS_SHIFT)	

#define MSGQSTAT_EVTLVL_SHIFT				16
#define MSGQSTAT_EVTLVL_MASK				(0xff << MSGQSTAT_EVTLVL_SHIFT)	

#define MSGQSTAT_NOTIFYS_SHIFT				24
#define MSGQSTAT_NOTIFYS_MASK				(0xff << MSGQSTAT_NOTIFYS_SHIFT)


#define status_mark_bits_set(stat,bits)		bits_set(stat,bits,MSGQSTAT_MARK_MASK)
#define status_mark_bits_clear(stat,bits) 	bits_clear(stat,bits,MSGQSTAT_MARK_MASK)
#define status_mark_bits_test(stat,bits)	bits_test(stat,bits,MSGQSTAT_MARK_MASK)
#define status_mark_region_clear(stat)		region_clear(stat,MSGQSTAT_MARK_MASK)


#define status_rboxs_add(stat,x) 			region_add(stat,x,MSGQSTAT_RBOXS_MASK,MSGQSTAT_RBOXS_SHIFT)
#define status_rboxs_sub(stat,x)			region_sub(stat,x,MSGQSTAT_RBOXS_MASK,MSGQSTAT_RBOXS_SHIFT)
#define status_rboxs_set(stat,x)			region_set(stat,x,MSGQSTAT_RBOXS_MASK,MSGQSTAT_RBOXS_SHIFT)
#define status_rboxs_compare(stat,x)			region_compare(stat,x,MSGQSTAT_RBOXS_MASK,MSGQSTAT_RBOXS_SHIFT)
#define status_rboxs_get(stat)				region_get(stat,MSGQSTAT_RBOXS_MASK,MSGQSTAT_RBOXS_SHIFT)
#define status_rboxs_clear(stat)			region_clear(stat,MSGQSTAT_RBOXS_MASK)
#define status_rboxs_decrease(stat)			status_rboxs_sub(stat,1)
#define status_rboxs_increase(stat)			status_rboxs_add(stat,1)
#define status_rboxs_test(stat)				status_rboxs_compare(stat,0)


#define status_notifys_add(stat,x) 			region_add(stat,x,MSGQSTAT_NOTIFYS_MASK,MSGQSTAT_NOTIFYS_SHIFT)
#define status_notifys_sub(stat,x)			region_sub(stat,x,MSGQSTAT_NOTIFYS_MASK,MSGQSTAT_NOTIFYS_SHIFT)
#define status_notifys_set(stat,x)			region_set(stat,x,MSGQSTAT_NOTIFYS_MASK,MSGQSTAT_NOTIFYS_SHIFT)
#define status_notifys_compare(stat,x)			region_compare(stat,x,MSGQSTAT_NOTIFYS_MASK,MSGQSTAT_NOTIFYS_SHIFT)
#define status_notifys_get(stat)			region_get(stat,MSGQSTAT_NOTIFYS_MASK,MSGQSTAT_NOTIFYS_SHIFT)
#define status_notifys_clear(stat)			region_clear(stat,MSGQSTAT_NOTIFYS_MASK)
#define status_notifys_decrease(stat)			status_notifys_sub(stat,1)
#define status_notifys_increase(stat)			status_notifys_add(stat,1)
#define status_notifys_test(stat)	 		status_notifys_compare(stat,0)

#define status_evtlvl_set(stat,x)			region_set(stat,x,MSGQSTAT_EVTLVL_MASK,MSGQSTAT_EVTLVL_SHIFT)
#define status_evtlvl_get(stat)				region_get(stat,MSGQSTAT_EVTLVL_MASK,MSGQSTAT_EVTLVL_SHIFT)
#define status_evtlvl_clear(stat)			region_clear(stat,MSGQSTAT_EVTLVL_MASK)

/*---------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------begin----------------------------------------------------------------------*/
/*
 *********************************************************************************
 *								  message buffer								 *
 *	heapbufmp:																	 *
 *				char 		*name;	  - object name							 	 *
 *				uint16_t 	 rid;	  -	region id							 	 *
 *				uint32_t 	 bsize;	  -	block size						 	 	 *
 *				uint32_t 	 nblock;  -	num of blocks						 	 *
 *				uint32_t 	 align;	  -	must be power of 2??				 	 *
 *				bool	 	 exact;	  -	alloc when request size equal bsize? 	 *
 *	heapmemmp:																	 *
 *				char 		*name;												 *
 *				uint16_t	 rid;												 *
 *				uint32_t 	 size;												 *
 *********************************************************************************
 */

typedef enum heap_type{
	HEAP_TYPE_BUFMP = 0,
	HEAP_TYPE_MEMMP,
	HEAP_TYPE_LAST
}heap_type_t;

typedef struct message_buffer_params{
	uint16_t 		heapid;
	enum 	heap_type 	heaptype;
	union {
		struct {
			char 		*name;
			uint16_t 	 rid;
			uint32_t 	 bsize;
			uint32_t 	 nblock;
			uint32_t 	 align;
			Bool	 	 exact;
		}bfp; 
		struct {
			char 		*name;
			uint16_t 	 rid;
			uint32_t 	 size;
		}mep;
		
	}heap;
	
	#define bufmp_name		heap.bfp.name
	#define bufmp_rid		heap.bfp.rid
	#define bufmp_bsize		heap.bfp.bsize
	#define bufmp_nblock		heap.bfp.nblock
	#define bufmp_align		heap.bfp.align
	#define bufmp_exact		heap.bfp.exact
	
	#define memmp_name		heap.mep.name
	#define memmp_rid		heap.mep.rid
	#define memmp_size		heap.mep.size

}message_buffer_params_t;


typedef struct message_buffer{
	Ptr				handle;
	uint16_t 		heapid;
	heap_type_t 	heaptype;
}message_buffer_t;
#define MESSAGE_BUFFER_INITIALIZER {NULL,HEAPID_INVALID,HEAP_TYPE_LAST}

#define MESSAGEQ_MAX_HEAPID			7u

#define HEAPID_INVALID          	(MESSAGEQ_MAX_HEAPID + 1)

#define check_heapid(heapid)		(heapid <= MESSAGEQ_MAX_HEAPID)


typedef MessageQ_Msg messageq_msg_t;

#define declare_message_type(mtype,dtype) 		\
	typedef struct {					\
		MessageQ_MsgHeader head;			\
		uint32_t type;						\
		dtype data;							\
	} mtype

void message_buffer_params_init(message_buffer_params_t *params);

int  message_buffer_construct(message_buffer_t *mbuf,message_buffer_params_t *params);

void message_buffer_destruct(message_buffer_t *mbuf);

int  message_buffer_create(message_buffer_t **pmbuf,message_buffer_params_t *params);
				
int  message_buffer_destroy(message_buffer_t *mbuf);

int  message_buffer_open(message_buffer_t **pmbuf,message_buffer_params_t *params);

int  message_buffer_close(message_buffer_t *mbuf);

#define message_alloc_private(mbuf,size) 	MessageQ_alloc((mbuf)->heapid,(size))

#define message_alloc(mbuf,size)		message_alloc_private(mbuf,size)

#define message_free(msg)					do{MessageQ_free((MessageQ_Msg)(msg));(msg)=NULL;}while(0)
	
#define message_buffer_valid(mbuf) 			((mbuf)->handle && (mbuf)->heapid != HEAPID_INVALID && (mbuf)->heaptype != HEAP_TYPE_LAST)

/*--------------------------------------------------messageq---------------------------------------------------------*/

/*
 *********************************************************************************
 *								  message queue								 	 *
 *	messageq_evid_t:															 *
 *				uint16_t pid;				-	procid							 *
 *				uint16_t lid;				-	lineid							 *
 *				uint16_t eid;				-	eventid							 *
 *																				 *
 *	messageq_event_t:							 								 *
 *				messageq_evid_t   evid;		-	message event id		 	 	 *
 *				uint32_t 		  evcode;	- 	event code						 *
 *																				 *
 *	messageq_t:																	 *
 *				object_t 		  obj;		-	object head						 *
 *				messageq_handle handle;		-	message box handle				 *
 *				messageq_func_t   func;		-	message notify callback			 *
 *				messageq_boxid_t  boxid;	-	message box id attached			 *
 *********************************************************************************
 */

typedef struct messageq_evid{
	uint16_t pid;
	uint16_t lid;
	uint32_t eid;
}messageq_evid_t;

typedef struct messageq_event{
	messageq_evid_t 	evid;
	uint32_t 			evcode;
	
}messageq_event_t;

typedef struct messageq_boxid{
	uint32_t qid;
}messageq_boxid_t;

typedef struct messageq messageq_t;
typedef int   (*messageq_func_t)(messageq_t *,messageq_event_t *);

struct messageq{
	object_t 				obj;
	int32_t					status;
	int						error;
	unsigned int			interval;
	MessageQ_Handle 		handle;	
	messageq_func_t 		func;		  //鎺ュ彈鍒颁俊鎭悗鐨勫鐞嗗洖璋冨嚱鏁�
	messageq_boxid_t		boxid;
};

#define MESSAGEQ_INITIALIZER {OBJECT_INITIALIZER,0,0,0,NULL,NULL,{0}}

#define MESSAGEQ_DEFAULT_INTERVAL			10

static inline int  messageq_count(messageq_t *msgq)
{
	return MessageQ_count(msgq->handle);
}

static inline void messageq_unblock(messageq_t *msgq)
{
	MessageQ_unblock(msgq->handle);
}

int messageq_construct(messageq_t *msgq,const char *name,object_t *parent);						

int messageq_destruct(messageq_t *msgq,Bool sync,messageq_boxid_t *boxid);

int messageq_create(messageq_t **pmsgq,const char *name);

int messageq_destroy(messageq_t *msgq,Bool sync,messageq_boxid_t *boxid);

int messageq_msgbox_create(messageq_t *msgq);

int messageq_msgbox_delete(messageq_t *msgq);

int messageq_msgbox_open(messageq_t *msgq,messageq_boxid_t *boxid,char *name);

int messageq_msgbox_close(messageq_t *msgq,messageq_boxid_t *boxid);

int messageq_msgbox_attach(messageq_t *msgq,char *name);

int messageq_msgbox_detach(messageq_t *msgq);

int messageq_notify_level_set(messageq_t *msgq,uint16_t level);
int messageq_notifier_interval_set(messageq_t *msgq,uint32_t interval);

int messageq_notifier_register(messageq_t *msgq,const uint16_t rpid,messageq_func_t func);

int messageq_notifier_unregister(messageq_t *msgq,const uint16_t rpid);

int messageq_send(messageq_t *msgq,messageq_msg_t msg,uint32_t flag,uint16_t level,const uint32_t retry_times);

int messageq_sendto(messageq_t *msgq,messageq_boxid_t *boxid,messageq_msg_t msg,uint32_t flag,uint16_t level,const uint32_t retry_times);

int messageq_recv(messageq_t *msgq,messageq_msg_t *msg,unsigned int timeout);

#define messageq_sendto_safe(msgq,boxid,msg,flag,level,retry_times)                                     \
                ({                                                                                      \
                    int rc = messageq_sendto(msgq,boxid,(messageq_msg_t)msg,flag,level,retry_times);    \
                    if(rc==0)																			\
		 				msg = NULL;                                                                     \
                    rc;                                                                                 \
                })

#define messageq_send_safe(msgq,msg,flag,level,retry_times)                                             \
                ({                                                                                      \
                    int rc = messageq_send(msgq,(messageq_msg_t)msg,flag,level,retry_times);            \
                    if(rc==0)																			\
						msg=NULL;																		\
					rc;																					\
				})

#define messageq_receive(msgq,pmsg,timeout)     messageq_recv(msgq,(messageq_msg_t*)pmsg,timeout)
int messageq_check_errors(messageq_t *msgq);
int messageq_clear_errors(messageq_t *msgq);
int messageq_print_status(messageq_t *msgq);
void  print_buf(message_buffer_t *message_buffer);
/****************************************************************************************************/

#ifdef _cplusplus
}
#endif

#endif
