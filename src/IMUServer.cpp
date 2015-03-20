/*
 * IMUServer.cpp
 *
 *  Created on: Feb 24, 2015
 *      Author: mul8
 */

#include "IMUServer.h"
#include <ctime>
#include <pthread.h>
#include "mutexes.h"
ReadIMU imu_device;
double timeout = 1.0/7.0;

double getUnixTime() {
	struct timespec tv;

	if (clock_gettime(CLOCK_REALTIME, &tv) != 0) {
		return 0;
	}

	return (((double)tv.tv_sec) + (tv.tv_nsec / 1000000000.0));
}
//namespace IMU_Server {
//IMU_Server::IMU_Server() {
//	// TODO Auto-generated constructor stub
//}
//
//IMU_Server::~IMU_Server() {
//	// TODO Auto-generated destructor stub
//	imu_device.close_port();
//}

void* IMU_Server::run(void * args) {
	double last_time = getUnixTime();

	Gyro gyro;
	Accl accl;
	Cmps cmps;

	while (1) {

		if ((getUnixTime() - last_time) > timeout) {
//			pthread_mutex_lock(&serial_port_mutex);
			imu_device.read_serial_xyz();
//			pthread_mutex_unlock(&serial_port_mutex);
			gyro = imu_device.get_gyro();
			accl = imu_device.get_accl();
			cmps = imu_device.get_cmps();
			IMU::set(IMU::ACCELEROMETER_X, accl.x);
			IMU::set(IMU::ACCELEROMETER_Y, accl.y);
			IMU::set(IMU::ACCELEROMETER_Z, accl.z);

			IMU::set(IMU::GYROSCOPE_X, gyro.x);
			IMU::set(IMU::GYROSCOPE_Y, gyro.y);
			IMU::set(IMU::GYROSCOPE_Z, gyro.z);

			IMU::set(IMU::COMPASS_X, cmps.x);
			IMU::set(IMU::COMPASS_Y, cmps.y);
			IMU::set(IMU::COMPASS_Z, cmps.z);
		}
		pthread_testcancel();
	}
}

//}
