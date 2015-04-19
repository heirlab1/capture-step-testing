/*
 * CM904Data.h
 *
 *  Created on: Feb 19, 2015
 *      Author: mul8
 */

#ifndef CM904DATA_H_
#define CM904DATA_H_

namespace CM904DATA {

enum names {ACCELEROMETER_X = 0, ACCELEROMETER_Y, ACCELEROMETER_Z,
		GYROSCOPE_X, GYROSCOPE_Y, GYROSCOPE_Z,
		COMPASS_X, COMPASS_Y, COMPASS_Z, BATTERY_VOLTAGE, BUTTON_MAP};

int get(int index);

void set(int index, int value);

} /* namespace IMU */

#endif /* CM904DATA_H_ */

