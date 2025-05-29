CC=gcc
CFLAGS1=-Wall -c -g 
CFLAGS2= -lpthread
OBJS=main.o data_global.o data_list.o pthread_main.o pthread_refresh.o cmd_cache.o pthread_tcp_send.o pthread_tcp_rcv.o

server :$(OBJS)
	$(CC) -o $@ $^ $(CFLAGS2)
$(OBJS):%.o:%.c
	$(CC) $(CFLAGS1) $< -o $@

.PHONY:clean
clean:
	rm *.o server

