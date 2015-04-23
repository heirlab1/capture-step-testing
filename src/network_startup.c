/*
 * network_startup.c
 *
 *  Created on: Feb 26, 2015
 *      Author: mul8
 */
#include "network_startup.h"
//#ifndef NETWORK_STARTUP_H
//#define NETWORK_STARTUP_H

//#include <stdio.h>
//#include <errno.h>
//#include <sys/socket.h>
//#include <resolv.h>
//#include <arpa/inet.h>
//#include <errno.h>

#define MAXBUF	1024
int clientfd;

void startup(int port) {
	int sockfd;
	struct sockaddr_in self;
	struct sockaddr_in client_addr;
	int addrlen=sizeof(client_addr);

	/*---Create streaming socket---*/
	if ( (sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0 )
	{
		perror("Socket");
//		exit(errno);
		return;
	}

	/*---Initialize address/port structure---*/
	bzero(&self, sizeof(self));
	self.sin_family = AF_INET;
	self.sin_port = htons(port);
	self.sin_addr.s_addr = INADDR_ANY;

	/*---Assign a port number to the socket---*/
	if ( bind(sockfd, (struct sockaddr*)&self, sizeof(self)) != 0 )
	{
		perror("socket--bind");
//		exit(errno);
		return;
	}

	/*---Make it a "listening socket"---*/
	if ( listen(sockfd, 20) != 0 )
	{
		perror("socket--listen");
//		exit(errno);
		return;
	}

	clientfd = accept(sockfd, (struct sockaddr*)&client_addr, &addrlen);

	printf("Socket Connected\n");

	close(sockfd);

}

void send_message(char *message, int num_bytes) {
	send(clientfd, message, num_bytes, 0);
}

int receive_message(char *buffer) {
	// TODO buffer might not work, so watch out
	return (recv(clientfd, buffer, MAXBUF, 0));
}

void shutdown_socket() {
	close(clientfd);
}

//#endif /* NETWORK_STARTUP_H */
