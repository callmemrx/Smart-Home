#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <sys/shm.h>
#include "data_global.h"

int main() 
{ 
	key_t key;
	int msgid;
	struct msg msg_buf;
	
	// 连接消息队列
	if((key = ftok("/home/book/app", 'g')) < 0){
		perror("ftok");
		exit(1);
	}

	if((msgid = msgget(key, 0666)) < 0){
		perror("msgget");
		exit(1);
	}	

	while(1){
		printf("1. data on\n");
		printf("2. data off\n");
		printf("3. Turn camera on\n");
		printf("4. Turn camera off\n");
		printf("5. Exit\n");
		printf("Enter your choice: ");

		int choice;
		scanf("%d", &choice);

		switch(choice){
			case 1:
				bzero(msg_buf.text, sizeof(msg_buf.text));
				strcpy(msg_buf.text, "data_on");
				msg_buf.type = 1;
				msgsnd(msgid, &msg_buf, sizeof(msg_buf)-sizeof(long), 0);
				printf("Command sent: DATA ON\n");
				break;

			case 2:
				bzero(msg_buf.text, sizeof(msg_buf.text));
				strcpy(msg_buf.text, "data_off");
				msg_buf.type = 1;
				msgsnd(msgid, &msg_buf, sizeof(msg_buf)-sizeof(long), 0);
				printf("Command sent: DATA OFF\n");
				break;

			case 3:
				bzero(msg_buf.text, sizeof(msg_buf.text));
				strcpy(msg_buf.text, "camera_on");
				msg_buf.type = 1;
				msgsnd(msgid, &msg_buf, sizeof(msg_buf)-sizeof(long), 0);
				printf("Command sent: camera on\n");
				break;

			case 4:
				bzero(msg_buf.text, sizeof(msg_buf.text));
				strcpy(msg_buf.text, "camera_off");
				msg_buf.type = 1;
				msgsnd(msgid, &msg_buf, sizeof(msg_buf)-sizeof(long), 0);
				printf("Command sent: camera off\n");
				break;

			case 5:
				exit(0);

			default:
				printf("Invalid choice!\n");
		}

		// 等待一下让状态更新
		sleep(1);
	}
	return 0;
} 
