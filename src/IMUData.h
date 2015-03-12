/*
 * IMUData.h
 *
 *  Created on: Feb 19, 2015
 *      Author: mul8
 */

#ifndef IMUDATA_H_
#define IMUDATA_H_

namespace IMU {

enum names {ACCELEROMETER_X = 0, ACCELEROMETER_Y, ACCELEROMETER_Z,
		GYROSCOPE_X, GYROSCOPE_Y, GYROSCOPE_Z,
		COMPASS_X, COMPASS_Y, COMPASS_Z};

double get(int index);

void set(int index, double value);

} /* namespace IMU */

#endif /* IMUDATA_H_ */
