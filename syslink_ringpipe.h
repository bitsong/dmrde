#ifndef _SYSLINK_RINGPIPE_H
#define _SYSLINK_RINGPIPE_H

#include "syslink_common.h"
#include <ti/syslink/RingIO.h>
#include <ti/syslink/RingIOShm.h>
#include <ti/ipc/GateMP.h>

#include "object.h"
#include "bilist.h"
#include "bit_region.h"


#ifdef _cplusplus
extern "C"{
#endif

/*默认共享内存区域*/
#define RINGIO_DEFAULT_SR				1

/*打开模式*/
#define RPE_MODE_READER					RingIO_MODE_READER
#define RPE_MODE_WRITER					RingIO_MODE_WRITER

/*RPE 构造/创建参数的无效值定义*/
#define RPE_INVALID_CTRLSR				0xffff
#define RPE_INVALID_DATASR				0xffff
#define RPE_INVALID_ATTRSR				0xffff
#define RPE_INVALID_RPID				0xffff
#define RPE_INVALID_DSIZE				0
#define RPE_INVALID_ASIZE				0

/*设置属性相关默认值*/
#define RPE_ATTR_DEFAULT_TYPE			0
#define RPE_ATTR_DEFAULT_PARAM			0
#define RPE_ATTR_DEFAULT_PDATA			NULL
#define RPE_ATTR_DEFAULT_SIZE			0
/*获取属性时，表示获取属性类型（简单、variable）*/
#define RPE_ATTR_FIXED				0
#define RPE_ATTR_VARIABLE			1

#define name_of(obj) 					object_name((object_t*)(obj))

/*-----------------------------------------------------------------------------------------------------------*/
//ring pipe status 标记域

#define RPESTAT_INITIALIZER				0

#define RPESTAT_MARK_START				0
#define RPESTAT_MARK_BITS				8
#define RPESTAT_MARK_REGION				REGION(RPESTAT_MARK_START,RPESTAT_MARK_BITS)

#define RPESTAT_MARK_VALID				REGION_FLAG_BIT(RPESTAT_MARK_REGION,RPESTAT_MARK_START,0)
#define RPESTAT_MARK_OPEN				REGION_FLAG_BIT(RPESTAT_MARK_REGION,RPESTAT_MARK_START,1)
#define RPESTAT_MARK_READER				REGION_FLAG_BIT(RPESTAT_MARK_REGION,RPESTAT_MARK_START,2)
#define RPESTAT_MARK_WRITER				REGION_FLAG_BIT(RPESTAT_MARK_REGION,RPESTAT_MARK_START,3)
#define RPESTAT_MARK_RNOTIFY			REGION_FLAG_BIT(RPESTAT_MARK_REGION,RPESTAT_MARK_START,4)
#define RPESTAT_MARK_WNOTIFY			REGION_FLAG_BIT(RPESTAT_MARK_REGION,RPESTAT_MARK_START,5)

#define rpe_mark_set(stat,bits)			bits_set(stat,bits,RPESTAT_MARK_REGION)
#define rpe_mark_clear(stat,bits)		bits_clear(stat,bits,RPESTAT_MARK_REGION)
#define rpe_mark_test(stat,bits)		bits_test(stat,bits,RPESTAT_MARK_REGION)
#define rpe_mark_setval(stat,val)		region_set(stat,val,RPESTAT_MARK_REGION,NTFSTAT_MARK_START)
#define rpe_mark_clear_all(stat)		region_clear(stat,RPESTAT_MARK_REGION)


//ring_pipe
struct rpe{
	object_t obj;
	uint32_t status;
	RingIO_Handle handle;
	RingIO_Handle rhandle;
	RingIO_Handle whandle;
	GateMP_Handle ghandle;
};
#define RPE_INITIALIZER					{OBJECT_INVALID_INITIALIZER,RPESTAT_INITIALIZER,NULL,NULL,NULL,NULL}

struct rpe_params{
//	char 		*name;
	uint16_t	ctrlsr;			//ringio控制信息所在共享区域
	uint16_t	datasr;			//数据所在共享区域
	uint16_t	attrsr;			//属性所在共享区域
	uint32_t	dsize;			//数据缓存大小
	uint32_t	asize;			//attr缓存大小
	uint16_t	rpid;			//remote procid
};

struct rpe_attr{
	uint16_t 		type;
	uint32_t 		param;
	uint32_t		size;
	RingIO_BufPtr 	pdata;			//暂时不管
	#define bytes	size
};
#define RPE_ATTR_INITIALIZER			{RPE_ATTR_DEFAULT_TYPE,RPE_ATTR_DEFAULT_PARAM,\
										 RPE_ATTR_DEFAULT_SIZE,RPE_ATTR_DEFAULT_PDATA}

struct rpe_notifier{
	RingIO_NotifyType 	type;
	uint32_t 			watermark;
	RingIO_NotifyFxn 	func;
};

#define RPE_NOTIFY_NONE 			RingIO_NOTIFICATION_NONE
#define RPE_NOTIFY_ONCE 			RingIO_NOTIFICATION_ONCE
#define RPE_NOTIFY_ALWAYS 			RingIO_NOTIFICATION_ALWAYS
#define RPE_NOTIFY_HARD_ONCE		RingIO_NOTIFICATION_HDWRFIFO_ONCE
#define RPE_NOTIFY_HARD_ALWAYS 		RingIO_NOTIFICATION_HDWRFIFO_ALWAYS

#define INVALID_WATERMARK			0xffffffff

#define RPE_NOTIFIER_INITIALIZER 	{RPE_NOTIFY_NONE,INVALID_WATERMARK,NULL}

#define rpe_check_notifier(ntf)			(((ntf)->type != RPE_NOTIFY_NONE) && ((ntf)->watermark != INVALID_WATERMARK) && ((ntf)->func != NULL))

#define RPE_ENDPOINT_READER	RPE_MODE_READER
#define RPE_ENDPOINT_WRITER	RPE_MODE_WRITER

enum rpe_space_type{
	RPE_SPACE_DATA,
	RPE_SPACE_ATTR,
	RPE_SPACE_INVAL
};

typedef RingIO_OpenMode					rpe_endpoint_t;

typedef RingIO_NotifyType				rpe_notify_type_t;

typedef RingIO_NotifyFxn				rpe_notify_func_t;

typedef RingIO_BufPtr					rpe_buf_t;

typedef struct rpe						rpe_t;

typedef struct rpe_params 				rpe_params_t;

typedef struct rpe_attr 				rpe_attr_t;

typedef enum   rpe_space_type 			rpe_space_t;

typedef struct rpe_notifycb				rpe_notify_cb_t;

typedef struct rpe_notifier				rpe_notifier_t;

void rpe_params_init(rpe_params_t *params);

//创建与销毁。
int  rpe_construct(struct rpe *rpe,const char *name,object_t *parent);
int  rpe_destruct(struct rpe *rpe);

int  rpe_create(struct rpe **prpe,const char *name,object_t *parent);
int  rpe_destroy(struct rpe *rpe);

//打开关闭。
int rpe_open(struct rpe *rpe,rpe_endpoint_t endpoint,void *params);
int rpe_close(struct rpe *rpe,rpe_endpoint_t endpoint);
int  rpe_close_all(struct rpe *rpe);

//注册/注销notify、修改notify类型及watermark大小。
int  rpe_register_notifier(struct rpe *rpe,rpe_endpoint_t endpoint,rpe_notifier_t *ntf);
int  rpe_unregister_notifier(struct rpe *rpe,rpe_endpoint_t endpoint);
int  rpe_set_notifytype(struct rpe *rpe,rpe_endpoint_t endpoint,rpe_notify_type_t type);

/*设置阈值*/
int  rpe_set_watermark(struct rpe *rpe,rpe_endpoint_t endpoint,uint32_t watermark);
int  rpe_get_watermark(struct rpe *rpe,rpe_endpoint_t endpoint);
 
//冲刷缓存数据。
int  rpe_flush(struct rpe *rpe,rpe_endpoint_t endpoint,Bool hardflush,rpe_attr_t *attr);//返回 flush的数据量

//发送通知
int  rpe_send_notify(struct rpe *rpe,rpe_endpoint_t endpoint,uint16_t msg);

//获取有效及空闲大小(包括数据及属性)
int  rpe_get_validsize(struct rpe *rpe,rpe_endpoint_t endpoint,rpe_space_t type);
int  rpe_get_emptysize(struct rpe *rpe,rpe_endpoint_t endpoint,rpe_space_t type);

//设置与获取属性 -- 设置标志、识别符、传递特殊消息等。
int  rpe_set_attribute(struct rpe *rpe,rpe_attr_t *attr,Bool sendnotify);
int  rpe_get_attribute(struct rpe *rpe,rpe_attr_t *attr,Bool flag);

int rpe_print_status(rpe_t *rpe);

//获取、释放/提交 数据

#define rpe_acquire_reader(rpe,ppdata,psize) RingIO_acquire((rpe)->rhandle,ppdata,psize)
#define rpe_acquire_writer(rpe,ppdata,psize) RingIO_acquire((rpe)->whandle,ppdata,psize)

#define rpe_release_reader(rpe,size) RingIO_release((rpe)->rhandle,size)
#define rpe_release_writer(rpe,size) RingIO_release((rpe)->whandle,size)

#define rpe_release_reader_safe(rpe,pdata,size)	({int ret = rpe_release_reader(rpe,size);pdata = NULL;ret;})
#define rpe_release_writer_safe(rpe,pdata,size) ({int ret = rpe_release_writer(rpe,size);pdata = NULL;ret;})


static inline int rpe_check_endpoint(const struct rpe *rpe,rpe_endpoint_t endpoint)
{
	if(RPE_ENDPOINT_READER == endpoint)
		return !!rpe->rhandle;
	else
		return !!rpe->whandle;
}

#ifdef _cplusplus
}
#endif

#endif
