#include "data_global.h"
#include "cmd_cache.h"
#include <arpa/inet.h>
#include <sys/socket.h>
#include "data_list.h"

//条件变量
pthread_cond_t cond_tcp_send;
pthread_cond_t cond_refresh;

//互斥锁
pthread_mutex_t mutex_cmdlist;
pthread_mutex_t mutex_refresh;
pthread_mutex_t mutex_global;  
pthread_mutex_t mutex_datalist;

unsigned char dev_uart_mask;

cmd_cache_list cmd_cache_head, cmd_cache_tail;
datalist dataHead, dataTail;

int msgid; //消息队列
int shmid; //共享内存
int semid; //信号量

int server_fd, client_fd;
struct sockaddr_in server_addr, client_addr;
