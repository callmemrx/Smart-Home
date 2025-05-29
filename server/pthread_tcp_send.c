#include "data_global.h"
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include "data_list.h"
#include <sys/time.h>

#define BUFFER_SIZE 1024

extern pthread_cond_t cond_tcp_send;
extern pthread_mutex_t mutex_cmdlist;

extern cmd_cache_list cmd_cache_head, cmd_cache_tail;
extern int client_fd;

void *pthread_tcp_send (void *arg)
{
	cmd_cache_list cmd_cache_p;
	printf ("pthread_tcp_send is ok\n");

	while (1) {
		pthread_mutex_lock(&mutex_cmdlist);
		pthread_cond_wait(&cond_tcp_send, &mutex_cmdlist);
		if (client_fd == -1) {
        pthread_mutex_unlock(&mutex_cmdlist);
        break;
		}
		
		//while判断 防止条件变量空唤醒
        while ((cmd_cache_p = GetCacheNode(cmd_cache_head, &cmd_cache_tail)) != NULL) {
            char* command = cmd_cache_p->data;
            if (send(client_fd, command, strlen(command), 0) < 0) {
                perror("send error");
                free(cmd_cache_p);
                continue;
            }
			printf("Sending command: %s successful\n", command);
            free(cmd_cache_p);
        }
        pthread_mutex_unlock(&mutex_cmdlist);
	}
	return NULL;
}
