#include <stdio.h>
#include <pthread.h>
#include <errno.h>
#include <signal.h>
#include <unistd.h>
#include <sys/shm.h>
#include <sys/sem.h>
#include <sys/ipc.h>
#include "data_global.h"
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include "data_list.h"

#define SERVER_IP "192.168.5.11"
#define PORT 8888
#define BUFFER_SIZE 1024

extern pthread_cond_t cond_tcp_send;
extern pthread_cond_t cond_refresh;

extern pthread_mutex_t mutex_cmdlist;
extern pthread_mutex_t mutex_refresh;
extern pthread_mutex_t mutex_global;
extern pthread_mutex_t mutex_datalist;

extern int msgid;
extern int shmid;
extern int semid;

extern int server_fd, client_fd;
extern struct sockaddr_in server_addr, client_addr;

pthread_t 	id_tcp_rcv,
			id_tcp_send,
			id_main_thread,
			id_refresh;

void ReleaseResource (int signo)
{

	close(client_fd);
	close(server_fd);
	printf("Client Disconnected\n");

	pthread_cancel (id_refresh);
	pthread_cancel (id_main_thread);
	pthread_cancel (id_tcp_send);
	pthread_cancel (id_tcp_rcv);
	
	pthread_mutex_destroy (&mutex_datalist);
	pthread_mutex_destroy (&mutex_global);
	pthread_mutex_destroy (&mutex_refresh);
	pthread_mutex_destroy (&mutex_cmdlist);

	pthread_cond_destroy (&cond_refresh);
	pthread_cond_destroy (&cond_tcp_send);

	msgctl (msgid, IPC_RMID, NULL);
	shmctl (shmid, IPC_RMID, NULL);
	semctl (semid, 1, IPC_RMID, NULL);
	
	printf ("All quit\n");
	exit(0);
}
int main(int argc, char **argv)
{
	pthread_mutex_init (&mutex_cmdlist, NULL);
	pthread_mutex_init (&mutex_refresh, NULL);
	pthread_mutex_init (&mutex_global, NULL);
	pthread_mutex_init (&mutex_datalist, NULL);

	pthread_cond_init (&cond_tcp_send, NULL);
	pthread_cond_init (&cond_refresh, NULL);

    socklen_t client_len = sizeof(client_addr);

    // 创建TCP socket
    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    // 设置socket选项，允许在同一个端口上重新绑定套接字
    int opt = 1;
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) {
        perror("setsockopt failed");
        exit(EXIT_FAILURE);
    }

    // 绑定socket
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(PORT);
    server_addr.sin_addr.s_addr = INADDR_ANY;
    memset(server_addr.sin_zero, 0, sizeof(server_addr.sin_zero));

    if (bind(server_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }

    // 监听连接
    if (listen(server_fd, 128) < 0) {
        perror("listen failed");
        exit(EXIT_FAILURE);
    }
    printf("Server running on port %d...\n", PORT);

	while (1) {
	    client_fd = accept(server_fd, (struct sockaddr *)&client_addr, &client_len);
	    if (client_fd < 0) {
	        perror("accept failed");
	        continue;
	    }
		printf("Client connected: %s\n", inet_ntoa(client_addr.sin_addr));

		pthread_create (&id_main_thread, 0, pthread_main, NULL);
		pthread_create (&id_tcp_send, 0, pthread_tcp_send, NULL);
		pthread_create (&id_tcp_rcv, 0, pthread_tcp_rcv, NULL);
		pthread_create (&id_refresh, 0, pthread_refresh, NULL);

		pthread_join (id_tcp_rcv, NULL);
		printf ("g1\n");
		pthread_join (id_tcp_send, NULL);
		printf ("g2\n");
		pthread_join (id_main_thread, NULL);
		printf ("g3\n");
		pthread_join (id_refresh, NULL);
		printf ("g4\n");
    }
	
	return 0;
}
