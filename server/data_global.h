#ifndef __DATA_GLOBAL__H__
#define __DATA_GLOBAL__H__

#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <pthread.h>
#include <termios.h>
#include <syscall.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/msg.h>
#include <sys/sem.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <linux/fs.h>
#include <linux/ioctl.h>
#include "cmd_cache.h"

#define MAIN 1
#define QUEUE_MSG_LEN 32

extern void *pthread_tcp_rcv (void *);			//数据接收线程
extern void *pthread_tcp_send (void *);			//命令发送线程
extern void *pthread_main (void *);	            //接收CGI，QT,app线程请求
extern void *pthread_refresh (void *);	        //共享内存数据刷新线程

struct msg
{
	long type;
	char text[QUEUE_MSG_LEN];
};

typedef struct {
    float temperature;
    float humidity;
} SensorData;

#endif
