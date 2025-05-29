#ifndef __DATA_LIST_H____
#define __DATA_LIST_H____
#include "data_global.h"

typedef struct _node_
{
	SensorData *data; 
	struct _node_ *next;
}datanode, *datalist;

extern datalist CreateEmptyDataList ();
extern int EmptyDataList (datalist h);
extern datalist GetDataNode (datalist h, datalist *t);
extern int InsertDataNode (datalist *t, SensorData* x);
#endif
