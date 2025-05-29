#include "data_list.h"
#include "data_global.h"
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/time.h>

#define BUFFER_SIZE 1024

extern int client_fd;
extern datalist dataHead, dataTail;

extern pthread_mutex_t mutex_datalist;
extern pthread_cond_t cond_refresh;

void *pthread_tcp_rcv (void *arg)
{
	dataHead = CreateEmptyDataList();
	dataTail = dataHead;
	char buffer[BUFFER_SIZE];

	printf("%s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);
	while(1){
		memset(buffer, 0, BUFFER_SIZE);
		int len = recv(client_fd, buffer, BUFFER_SIZE - 1, 0);
		printf("recv data len = %d\n", len);
		if (len > 0) {
			SensorData *data = malloc(sizeof(SensorData)); //分配内存，否则段错误
			memcpy(&data->temperature, buffer, 4);
	        memcpy(&data->humidity, buffer+4, 4);
			printf ("recv : %0.2f %0.2f\n", data->temperature, data->humidity);
			pthread_mutex_lock (&mutex_datalist);
			//接收到的额数据加入到链表中
			if ((InsertDataNode (&dataTail, data)) == -1)
			{
				pthread_mutex_unlock (&mutex_datalist);
				printf ("NONMEM\n");
			}
			pthread_mutex_unlock (&mutex_datalist);
			pthread_cond_signal (&cond_refresh);
		}else if(len == 0){
			printf("Connection closed bt server\n");
			break;
		}else{
			perror("recv error");
			break;
		}
	}
	return NULL;
}
