/*
本链表因为所有操作都只有指针域，而具体的结构要潜入节点结构，因此，这里的删除不包含对节点内存空间的释放。
相应的操作放在具体更高层的应用中。
*/
#ifndef _BILIST_H
#define _BILIST_H

#include "syslink_common.h"

#if defined(_cplusplus)
extern "C"{
#endif

typedef struct biloop_list{
	struct biloop_list *prev,*next;
}bilist_node_t;

#define _bilist_head_init 			_bilist_node_init

//节点及列表头构造
#define bilist_head_t 				bilist_node_t

#define bilist_node_construct 		bilist_node_init
#define bilist_node_destruct  		bilist_node_init

//判断节点是否为孤立节点
#define check_node_unlinked(node) 	((node)->prev == (node) && (node)->next == (node))

//列表遍历
#define container_of(addr,type,name) 			(type*)((char*)(addr) - (size_t)&(((type*)0)->name))
#define bilist_for_each(head,pos)				for((pos) = (head)->next ; (pos) != (head) ; (pos) = (pos)->next)											
#define bilist_for_each_safe(head,pos,_next) 	for((pos) = (head)->next , (_next) = (pos)->next ; (pos) != (head) ; (pos) = (_next) , (_next) = (pos)->next)

static inline void _bilist_node_init(struct biloop_list *node)
{
	node->prev = node;
	node->next = node;
}

static inline void _bilist_node_insert(
					bilist_node_t *prev,
					bilist_node_t *newn,
					bilist_node_t *next)
{
	newn->next = next;
	newn->prev = prev;
	prev->next = newn;
	next->prev = newn;
}

static inline void _bilist_remove(bilist_node_t *prev,bilist_node_t *next)
{
	next->prev = prev;
	prev->next = next;
}

/***************************节点增删***************************/
static inline void bilist_add(bilist_head_t *list,bilist_node_t *newn)
{
	bilist_head_t *head = list;
	_bilist_node_insert(head,newn,head->next);
}

static inline void bilist_del(bilist_node_t *node)
{
	bilist_node_t *prev = node->prev;
	bilist_node_t *next = node->next;
	
	_bilist_remove(prev,next);//从链表中移除
	_bilist_node_init(node);  //将该节点前后指针指向自己
}

static inline void bilist_add_tail(bilist_head_t *list,bilist_node_t *newn)
{
	bilist_head_t *head = list;
	_bilist_node_insert(head->prev,newn,head);
}

static inline void bilist_insert(bilist_head_t *pos,bilist_node_t *node)
{

	_bilist_node_insert(pos->prev,node,pos);
}

static inline Bool bilist_empty(bilist_head_t *list)
{
	bilist_head_t *head = list;
	return (head->prev == head && head-> next == head);
}

/**************************链表、节点构造***************************/
static inline void bilist_node_init(bilist_node_t *newn)
{
	_bilist_node_init(newn);
}

/*构造链表*/
static inline void bilist_construct(bilist_head_t *list)
{
	_bilist_head_init(list);
}
/*析构链表*/
static inline void bilist_destruct(bilist_head_t *list)
{
	_bilist_head_init(list);
}

/*动态创建链表*/
static inline bilist_head_t* bilist_create(void)
{
	bilist_head_t *head = (bilist_head_t*)malloc(sizeof(bilist_head_t));
	if(!head)
		return NULL;
	
	_bilist_head_init(head);
	
	return head;
}

static inline void bilist_destroy(bilist_head_t *list)
{
	bilist_head_t *head = list;
	while(!bilist_empty(head)){
		bilist_del(head->next);
	}
	
	free(list);
}	

#if defined(_cplusplus)
}
#endif

#endif

