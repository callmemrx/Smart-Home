#ifndef __CMD_CACHE_H__
#define __CMD_CACHE_H__

#include <stdio.h>
#include <stdlib.h>
#include "cmd_cache.h"
#include <string.h>

//typedef void* uart_cache_data;

typedef struct _uart_node_
{
	char* data;
	struct _uart_node_ *next;
}cmd_cache_node, *cmd_cache_list;

extern cmd_cache_list CreateEmptyCacheList ();
extern int EmptyCacheList (cmd_cache_list);
extern cmd_cache_list GetCacheNode (cmd_cache_list, cmd_cache_list *);
extern int InsertCacheNode (cmd_cache_list *, char*);

#endif
