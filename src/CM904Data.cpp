/*
 * CM904Data.cpp
 *
 *  Created on: Feb 19, 2015
 *      Author: mul8
 */

#include "CM904Data.h"
#include <pthread.h>

namespace CM904DATA {

pthread_mutex_t cm904_mutex = PTHREAD_MUTEX_INITIALIZER;

int values[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

int get(int index) {
	// TODO Might need mutex here?
	int result;
//	pthread_mutex_lock(&imu_mutex);
	result = values[index];
//	pthread_mutex_unlock(&imu_mutex);
	return result;
//	return (values[index]);

}

void set(int index, int value) {
	// TODO Might need mutex here?
//	pthread_mutex_lock(&imu_mutex);
	values[index] = value;
//	pthread_mutex_unlock(&imu_mutex);
}
} /* namespace CM904DATA */

