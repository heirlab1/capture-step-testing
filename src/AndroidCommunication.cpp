//============================================================================
// Name        : Stand_Demo.cpp
// Author      : Kellen Carey
// Version     :
// Copyright   : 
// Description : Hello World in C++, Ansi-style
//============================================================================
#include "AndroidCommunication.h"
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include "network_startup.h"
#include <math.h>
#include <pthread.h>
#include <vector>
#include <string>
using namespace std;

#define PI 3.14159265

extern "C" void startup(int);
extern "C" int receive_message(char *message);
extern "C" void send_message(char *message, int num_bytes);
extern "C" void shutdown_socket();

static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
static std::string myString = "";

std::string AndroidCommunication::getString() {
	pthread_mutex_lock(&mutex);
	std::string result = myString;
	pthread_mutex_unlock(&mutex);

	return result;
}

void AndroidCommunication::sendString(std::string toSend) {
	pthread_mutex_lock(&mutex);
	char buffer[2048];
	toSend.copy(buffer, toSend.size() , 0);

	printf("\rBuffer = %s\n", buffer);

	send_message(buffer, toSend.size());
	pthread_mutex_unlock(&mutex);
}

void AndroidCommunication::resetString() {
	pthread_mutex_lock(&mutex);
	myString = "";
	pthread_mutex_unlock(&mutex);
}


void * AndroidCommunication::run(void * args) {

	startup(9999);

	char buffer[1024];
	while (1) {
		int num_bytes = receive_message(buffer);

		char message[num_bytes+1];


		for (int i = 0; i < num_bytes; i++) {
			printf("%c", buffer[i]);
			message[i] = buffer[i];
		}
		message[num_bytes] = '\0';
		pthread_mutex_lock(&mutex);
		myString = string(message);

		myString = myString.substr(0, num_bytes);
		pthread_mutex_unlock(&mutex);
		bzero(buffer, sizeof(buffer));

	}

	shutdown_socket();


	cout << "Stand" << endl; // prints Stand
	return 0;
}
