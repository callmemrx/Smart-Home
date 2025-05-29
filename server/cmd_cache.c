/* 从消息队列获取的命令存放的链表相关操作 */
#include "cmd_cache.h"

cmd_cache_list CreateEmptyCacheList ()
{
	cmd_cache_list h;
	h = (cmd_cache_list) malloc (sizeof (cmd_cache_node));
	h->next = NULL;
	return h;
}

int EmptyCacheList(cmd_cache_list h) 
{
    if (h == NULL) {
        return -1; // 表示链表头指针无效
    }
    return (h->next == NULL); // 如果链表为空返回1，否则返回0
}

cmd_cache_list GetCacheNode(cmd_cache_list h, cmd_cache_list *t)
{
    if (EmptyCacheList(h)){
        return NULL;
    }

    cmd_cache_list p = h->next;  // 待移除的节点
    h->next = p->next;            // 头节点指向下一个节点

    // 若移除后链表为空，更新尾指针为头节点
    if (p->next == NULL){
        *t = h;
    }

    return p; // 返回被移除的节点
}

int InsertCacheNode(cmd_cache_list *t, char* x)
{
    cmd_cache_list q = malloc(sizeof(cmd_cache_node));
    if (!q){
        perror("malloc node failed");
        return -1;
    }

    q->data = strdup(x); // 深拷贝数据
    if (!q->data){
        perror("strdup failed");
        free(q);
        return -1;
    }

    q->next = NULL;
    (*t)->next = q;  // 原尾节点的next指向新节点
    *t = q;          // 更新尾指针为新节点

    return 0;
}

