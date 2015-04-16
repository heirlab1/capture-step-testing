/*
 * Dynamixel.h
 *
 *  Created on: Dec 8, 2014
 *      Author: Kellen Carey
 */

#ifndef DYNAMIXEL_H_
#define DYNAMIXEL_H_

#include "dynamixel.h"
#include <vector>

namespace Dynamixel {
	void init();
	void setMotorPosition(int, double, int, double);
	void setMotorPositionInt(int, int);
	void enableMotor(int);
	int getZeroPose(int);
	void close();
	bool addToSyncwrite(int id, std::vector<int> newData);
	bool setSyncwriteStartAddress(int startAddress);
	bool setSyncwriteEachLength(int eachLength);
	bool sendSyncWrite();
	void saveConfig();
	void setInitialPose(int motor, int adjustment);
	double getMotorPosition(int motor);
}

#endif /* DYNAMIXEL_H_ */
