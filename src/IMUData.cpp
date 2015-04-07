/*
 * IMUData.cpp
 *
 *  Created on: Feb 19, 2015
 *      Author: Kellen Carey
 */

#include "IMUData.h"
#include <pthread.h>

namespace IMU {

pthread_mutex_t imu_mutex = PTHREAD_MUTEX_INITIALIZER;

double values[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

double get(int index) {
	// TODO Might need mutex here?
	double result;
//	pthread_mutex_lock(&imu_mutex);
	result = values[index];
//	pthread_mutex_unlock(&imu_mutex);
	return result;
//	return (values[index]);

}

void set(int index, double value) {
	// TODO Might need mutex here?
//	pthread_mutex_lock(&imu_mutex);
	values[index] = value;
//	pthread_mutex_unlock(&imu_mutex);
}
} /* namespace IMU */
