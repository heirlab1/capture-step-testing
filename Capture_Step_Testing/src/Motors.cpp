/*
 * Motors.cpp
 *
 *  Created on: Feb 5, 2015
 *      Author: mul8
 */
#include "Motors.h"


double motor_position_array[24];

pthread_mutex_t motor_mutex = PTHREAD_MUTEX_INITIALIZER;

void Motors::initialize() {
	for (int i = 0; i < 24; i++) {
		motor_position_array[i] = 0.0;
	}
}

double Motors::getMotorPosition(int motor) {
	pthread_mutex_lock(&motor_mutex);
	double result = motor_position_array[motor-1];
	pthread_mutex_unlock(&motor_mutex);
	return result;
}

void Motors::setMotorPosition(int motor, double pos) {
	pthread_mutex_lock(&motor_mutex);
	motor_position_array[motor-1] = pos;
	pthread_mutex_unlock(&motor_mutex);
}


