#include <arpa/inet.h>
#include "sem.h"
#include "data_global.h"
#include "data_list.h"

#define N 1024

extern pthread_mutex_t mutex_refresh;
extern pthread_mutex_t mutex_global;
extern pthread_mutex_t mutex_datalist;

extern pthread_cond_t cond_refresh;

extern int shmid; 
extern int semid;
extern datalist dataHead, dataTail;

void *pthread_refresh (void *arg)
{
	key_t key_info;
	int shmid;
	datalist node;
	SensorData* shm_buf;
	SensorData* data;

	if ((key_info = ftok ("/home/book/app", 'i')) < 0)
	{
		perror ("ftok info");
		exit (-1);
	}

	if ((semid = semget (key_info, 1, IPC_CREAT | IPC_EXCL |0666)) < 0)
	{
		if (errno == EEXIST)
		{
			semid = semget (key_info, 1, 0666);
		}
		else
		{
			perror ("semget");
			exit (-1);
		}
	}
	else
	{
		init_sem (semid, 0, 1);
	}

	if ((shmid = shmget (key_info, N, IPC_CREAT | IPC_EXCL | 0666)) < 0)
	{
		if (errno == EEXIST)
		{
			shmid = shmget (key_info, N, 0666);
			shm_buf = (SensorData *)shmat (shmid, NULL, 0);
		}
		else
		{
			perror ("shmget");
			exit (-1);
		}
	}
	else
	{
		if ((shm_buf = (SensorData *)shmat (shmid, NULL, 0)) == (void *)-1)
		{
			perror ("shmat");
			exit (-1);
		}
	}

	printf ("pthread_refresh is ok\n");

	while (1)
	{
		pthread_mutex_lock (&mutex_refresh);
		pthread_cond_wait (&cond_refresh, &mutex_refresh);
		// 处理从链表获取的数据
		while (1)
		{
			pthread_mutex_lock (&mutex_datalist);
			if ((node = GetDataNode (dataHead, &dataTail)) == NULL)
			{
				pthread_mutex_unlock (&mutex_datalist);
				break;
			}
			data = node->data;
			pthread_mutex_unlock (&mutex_datalist);
		}
		// 更新共享内存
		sem_p (semid, 0);
		pthread_mutex_lock (&mutex_global);
		memcpy(shm_buf, data, sizeof(SensorData));
		printf ("recv : %0.2f %0.2f\n", shm_buf->temperature, shm_buf->humidity);
		pthread_mutex_unlock (&mutex_global);
		sem_v (semid, 0);
		pthread_mutex_unlock (&mutex_refresh);

		free(node);
		free(data);
	}
	return 0;
}

