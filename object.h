#ifndef _OBJECT_H
#define _OBJECT_H
#include "syslink_common.h"

//#include "debug.h"
#include "syslink_ecode.h"

#ifdef cplusplus
extern "C"{
#endif

typedef enum object_type{
	
	OBJECT_TYPE_FIRST=1,			    //锟斤拷锟斤拷锟斤拷小值
	OBJECT_TYPE_SLNK_HOST,			//syslink 锟斤拷锟借备锟剿讹拷锟斤拷锟斤拷锟斤拷
	OBJECT_TYPE_SLNK_SLAVE,			//syslink 锟斤拷锟借备锟剿讹拷锟斤拷锟斤拷锟斤拷
	OBJECT_TYPE_RINGPIE,			//ring pipe 锟斤拷锟斤拷锟斤拷锟斤拷
	OBJECT_TYPE_MSGQ,				//锟斤拷息锟斤拷锟叫讹拷锟斤拷锟斤拷锟斤拷
	OBJECT_TYPE_NOTIFIER,			//notify 锟斤拷锟斤拷锟斤拷锟斤拷
	OBJECT_TYPE_LAST				//锟斤拷锟斤拷锟街�
	
}object_type_t;

typedef void (*object_destruct_t)(void *);

typedef struct object_meta{
	
	object_type_t 		 	type;
	object_destruct_t	 	destruct;				 
	char 	 			   *name;
	struct object_meta	   *parent;
	int16_t 				cnt;
	//pthread_mutex_lock	lock;//使锟斤拷应锟矫诧拷锟叫达拷锟斤拷锟�
	
}object_t;

#define OBJECT_INITIALIZER {OBJECT_TYPE_LAST,NULL,NULL,NULL,0}

#define OBJECT_CNT_INVALID	-1//0
#define OBJECT_CNT_INIT		 0//1

#define object_objtype_valid(type) ((type) >= OBJECT_TYPE_FIRST && (type) <=  OBJECT_TYPE_LAST)

#define object_valid(obj) 		((obj)->cnt >= OBJECT_CNT_INIT)
#define object_busy(obj)  		((obj)->cnt >  OBJECT_CNT_INIT)
#define object_could_free(obj)	((obj)->cnt == OBJECT_CNT_INIT)
#define object_need_free(obj)   ((obj)->cnt == OBJECT_CNT_INVALID)
#define object_name(obj)		((obj)->name)
#define object_type(obj)		((obj)->type)

/*object:4 bit,private:28 bit*/

int object_init(object_t *obj,
				const char *name,
				object_type_t type,
				object_t *parent);

int object_destruct(object_t *obj);

int object_change_parent(object_t* obj,object_t *parent);

int object_set_destruct(object_t *obj,object_destruct_t destruct);

/*
inline object_t* __object_get(object_t *obj)
{
	obj->cnt++;
}

inline object_t* _object_get(object_t *obj)
{
	_object_lock(obj);
	__object_get(obj);
	_object_unlock(obj);
	return obj;
}
*/


/*
inline void _object_lock(object_t *obj)
{
	pthread_mutex_lock(&obj->lock);
}

inline int _object_trylock()
{
	pthread_mutex_trylock(&obj->lock);
}

inline void _object_unlock(obj)
{
	pthread_mutex_unlock(&obj->lock);
}

*/

/*

#define object_lock(obj)	_object_lock(obj)
#define object_unlock(obj)	_object_unlock(obj)
#define object_trylock(obj) _object_trylock(obj)

*/

static inline object_t* _object_get(object_t *obj)
{
	obj->cnt++;
	return obj;
}

static inline void _object_put(object_t *obj)
{
	obj->cnt--;
}

static inline uint16_t object_getvalue(object_t *obj)
{
	return obj->cnt;
}

static inline int object_get(object_t *obj)
{
	if(0==(object_valid(obj)))
		return -EOBJINVALID;
	
	_object_get(obj);
	
	return 0;
}

static inline Bool object_tryput(object_t *obj)
{
	
	if(!object_busy(obj))
		return 0;
	
	_object_put(obj);
	
	return TRUE;
}


static inline int object_put(object_t *obj)
{
	if(!object_busy(obj))
		return EOBJINVALID;
	
	_object_put(obj);
	
	return 0;
}

static const object_t* object_get_parent(object_t *obj)
{
	return obj->parent;
}
void object_deinit(object_t *obj);

#ifdef cplusplus
}
#endif

#endif
