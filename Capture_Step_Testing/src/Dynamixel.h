/*
 * Dynamixel.h
 *
 *  Created on: Dec 8, 2014
 *      Author: unicorn
 */

#ifndef DYNAMIXEL_H_
#define DYNAMIXEL_H_

#include "dynamixel.h"

namespace Dynamixel {
	void init();
	void setMotorPosition(int, double, int);
	void setMotorPositionInt(int, int);
	void enableMotor(int);
	int getZeroPose(int);
	void close();
}

#endif /* DYNAMIXEL_H_ */
