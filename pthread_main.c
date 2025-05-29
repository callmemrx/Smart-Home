#include "data_global.h"
#include "cmd_cache.h"

extern pthread_cond_t cond_tcp_send;

extern pthread_mutex_t mutex_cmdlist;

extern int msgid;
extern struct env_info_array all_info_RT; 

extern cmd_cache_list cmd_cache_head, cmd_cache_tail;

void *pthread_main (void *arg)
{
	key_t key;
	ssize_t msgsize;
	struct msg msgbuf;

	if ((key = ftok ("/home/book/app", 'g')) < 0)
	{
		perror ("ftok msgqueue");
		exit (-1);
	}
	if ((msgid = msgget (key, IPC_CREAT | 0666)) < 0)
	{
		if(errno == EEXIST)
		{
			if ((msgid = msgget(key, 0666)) < 0) {
                perror("msgget msgid");
                exit(EXIT_FAILURE);
				}
		} 
		else
		{
			perror ("msgget msgid");
			exit (-1);
		}
	}

	cmd_cache_head = CreateEmptyCacheList ();
	cmd_cache_tail = cmd_cache_head;
	
	printf ("pthread_main is ok\n");
	while (1)
	{
		bzero (&msgbuf, sizeof (msgbuf));
		msgsize = msgrcv (msgid, &msgbuf, sizeof (msgbuf) - sizeof (long), MAIN, 0);
		if (msgsize < 0) {
            perror("msgrcv error");
            break; // 退出循环
        }
		
		printf ("get cmd = %s\n", msgbuf.text);
		
		pthread_mutex_lock (&mutex_cmdlist);	
		if (InsertCacheNode(&cmd_cache_tail, msgbuf.text) == -1) {
			printf("插入节点失败，内存不足\n");
		}else{
			printf("Inserting command: %s\n", msgbuf.text);
		}
		pthread_mutex_unlock (&mutex_cmdlist);
		pthread_cond_signal (&cond_tcp_send);
	}
	return NULL;
}

