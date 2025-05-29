/* 从 tcp 采集到的数据存放的链表 */
#include "data_list.h"
#include "data_global.h"

datalist CreateEmptyDataList()
{
	datalist h;
	h = (datalist)malloc (sizeof (datanode));
	h->next = NULL;
	return h;
}

int EmptyDataList (datalist h)
{
	if(h == NULL){
        return -1; // 表示链表头指针无效
    }
    return (h->next == NULL); // 如果链表为空返回1，否则返回0
}

datalist GetDataNode(datalist h, datalist *t)
{
    if(EmptyDataList(h)){
        return NULL;
    }

    datalist p = h->next;  // 待移除的节点
    h->next = p->next;            // 头节点指向下一个节点

    // 若移除后链表为空，更新尾指针为头节点
    if(p->next == NULL){
        *t = h;
    }

    return p; // 返回被移除的节点

}

int InsertDataNode(datalist *t, SensorData* x)
{
	datalist q = malloc(sizeof(datanode));
	if(!q){
		perror("malloc node failed");
		return -1;
	}
	
	q->data = x; // 深拷贝数据
	if(!q->data){
		perror("strdup failed");
		free(q);
		return -1;
	}
	
	q->next = NULL;
	(*t)->next = q;  // 原尾节点的next指向新节点
	*t = q;		  // 更新尾指针为新节点
	
	return 0;

}

