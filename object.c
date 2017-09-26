#include "object.h"


int object_init(object_t *obj,const char *name,object_type_t type,object_t *parent)
{
	unsigned int len=0;

	if(!object_objtype_valid(type))
		return EOBJTYPE;
	
	if(!name)
		return EOBJNAME;
	
	obj->type 		= type;
	obj->parent 	= NULL;
	obj->cnt  		= OBJECT_CNT_INIT;
	obj->destruct	= NULL;
	len=strlen(name)+1;
	obj->name 		=  malloc(len);
	strcpy(obj->name,name);
	
	if(parent)
		obj->parent = _object_get(parent);
	
	//pthread_mutex_init(&obj->lock);
	
	return 0;
}

void object_deinit(object_t *obj)
{
	if(obj->name){
		free(obj->name);
		obj->name = NULL;
	}
	
	obj->cnt 		= OBJECT_CNT_INVALID;
	obj->type 		= OBJECT_TYPE_LAST;
	if(obj->parent){
		object_put(obj->parent);
		obj->parent = NULL;
	}
	
	//pthread_mutex_destroy(&obj->lock);
	obj->destruct 	= NULL;
}

int object_destruct(object_t *obj)
{
	if(object_busy(obj))
		return EOBJISBUSY;
	
	if(0==object_valid(obj))
		return EOBJINVALID;
	
	if(obj->destruct)
		obj->destruct(obj);
	
	object_deinit(obj);
	
	return SUCCESS;
}

int object_change_parent(object_t* obj,object_t *parent)
{
	if(!parent)
		return EPARMINVALID;
	
	if(0==object_valid(obj))
		return EOBJINVALID;
	
	if(0==object_valid(parent))
		return EPARENTINVALID;
	
	
	if(obj->parent)
		object_put(parent);
	
	obj->parent = _object_get(parent);
	return 0;
}

int object_set_destruct(object_t *obj,object_destruct_t destruct)
{
	if(0==object_valid(obj))
		return EOBJINVALID;
	
	obj->destruct = destruct;
	
	return 0;
}

