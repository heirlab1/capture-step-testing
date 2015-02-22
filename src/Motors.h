/*
 * Motors.h
 *
 *  Created on: Feb 5, 2015
 *      Author: mul8
 */

#ifndef MOTORS_H_
#define MOTORS_H_

#include <pthread.h>

namespace Motors {

	void initialize();

	double getMotorPosition(int motor);
	void setMotorPosition(int motor, double position);



};




#endif /* MOTORS_H_ */
