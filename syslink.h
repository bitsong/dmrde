#ifndef _SYSLINK_H_
#define _SYSLINK_H_

#include "syslink_common.h"
#include <ti/syslink/IpcHost.h>
#include <ti/syslink/SysLink.h>
#include <ti/ipc/Ipc.h>


#include "bilist.h"
#include "object.h"
#include "syslink_ecode.h"
#include "syslink_notifier.h"
#include "syslink_messageq.h"
#include "syslink_ringpipe.h"

//#include "syslink_init.h"

#if defined(_cplusplus)
extern "C"{
#endif

#define PROC_ROLE_HOST				OBJECT_TYPE_SLNK_HOST
#define PROC_ROLE_SLAVE				OBJECT_TYPE_SLNK_SLAVE

#define SHIFT_BASE					0
#define SHIFT_IPC_START				SHIFT_BASE
#define SYSLINK_STAT_START			(1 << SHIFT_IPC_START)

#define syslink_status_set(syslink,st)		((syslink)->status |= (st))
#define syslink_status_clear(syslink,st)	((syslink)->status &= ~(st))
#define syslink_status_check(syslink,st)	(((syslink)->status && (st)) == (st))
/*---------------------------------------------------------------------------------------------------*/

typedef object_t 	  syslink_object_t;

typedef object_type_t syslink_objtype_t;

typedef object_type_t proc_role_t;

typedef struct proc proc_t;

typedef struct proc_node proc_node_t;

typedef struct syslink_module syslink_t;

typedef struct syslink_object_params syslink_object_params_t;

//鏆傛椂涓嶇敤
typedef struct syslink_objhead syslink_objhead_t;

typedef struct syslink_object_notifier syslink_notifier_t;

typedef struct syslink_object_messageq syslink_messageq_t;

typedef struct syslink_object_ringpipe syslink_rpe_t;

/*---------------------------------------------------------------------------------------------------*/
struct proc{
	uint16_t rpid;	//remote procid
	uint16_t rcnt;	//reference counter
};

struct proc_node{
	bilist_node_t 	node;
	struct proc 	proc;
};

struct syslink_module{
	object_t		 obj;
	uint32_t		 status;
//鍚庤竟鐪嬫儏鍐靛疄鐜�
//	pthread_mutex_t	 lock_objects
//	pthread_mutex_t  lock_procs
	bilist_head_t 	 list_objects;
	bilist_head_t	 list_procs;
};

struct syslink_object_params{
	
	union{
		struct{
			char 	*nm;
			uint16_t cap;
		}ntf;
		//......
	}un;
	#define ntfy_nm 	un.ntf.nm
	#define ntfy_cap 	un.ntf.cap
	//...
	
};

/*鏆傛椂涓嶇敤*/
struct syslink_objhead{
	bilist_node_t		node;
	syslink_object_t	obj;
};

struct syslink_object_notifier{
	bilist_node_t		node;
	notifier_t			notify;
};

struct syslink_object_messageq{
	bilist_node_t		node;
	messageq_t 			msgq;
};

struct syslink_object_ringpipe{
	bilist_node_t 		node;
	struct rpe  		atnl;//audio_ternel_t
};


int syslink_construct(syslink_t *syslink,const char *name,proc_role_t role);

int syslink_destruct(syslink_t *syslink);
 
int syslink_new(syslink_t **pslnk,const char *name,proc_role_t role);

int syslink_destroy(syslink_t *syslink);

int syslink_start(syslink_t *syslink,char *name);

int syslink_stop(syslink_t *syslink,char *proc_name);

int syslink_attach(syslink_t *syslink,char *proc_name);

int syslink_detach(syslink_t *syslink,char *proc_name);

int syslink_getprocid(char* name);

/*鏆傛椂涓嶇敤*/
syslink_object_t* 
	syslink_object_create(syslink_t *syslink,syslink_objtype_t type,syslink_object_params_t *params);

int syslink_object_destroy(syslink_object_t *sobj);

int syslink_object_clear(syslink_t *syslink);

int syslink_detach_all(syslink_t *syslink);

#define syslink_getprocid(proc_name) 	MultiProc_getId(proc_name)

#define syslink_getprocnm(proc_id)	 	MultiProc_getName(proc_id)

#define syslink_getnprocs			 	MultiProc_getNumProcessors()

#if defined(_cplusplus)
}
#endif

#endif
