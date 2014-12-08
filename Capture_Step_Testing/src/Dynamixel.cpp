/*
 * Dynamixel.cpp
 *
 *  Created on: Dec 8, 2014
 *      Author: unicorn
 */

#include "Dynamixel.h"
#include "dynamixel.h"
#include <iostream>

#define P_GOAL_POSITION		30
#define P_MOVING_SPEED		32
#define P_ENABLE			24
#define P_PRESENT_POSITION	36

#define PI 3.14159265

int initial_poses[] = {650, 3436, 1065, 2029,
		2495, 2159, 503, 1548, 2588, 1051,
		1992, 1058
};

void Dynamixel::init() {
	dxl_initialize(0, 0);
}

void Dynamixel::close() {
	dxl_terminate();
}

int Dynamixel::getZeroPose(int motor) {
	if (motor > 0 && motor <= 12) {
		return initial_poses[motor-1];
	}
	else {
		return dxl_read_word(motor, P_PRESENT_POSITION);
	}
}

void Dynamixel::setMotorPositionInt(int motor, int position) {
	dxl_write_word(motor, P_MOVING_SPEED, 50);
	dxl_write_word(motor, P_GOAL_POSITION, position);
}

void Dynamixel::setMotorPosition(int motor, double angle, int speed = 75) {
	// Convert angle to motor positions
	int motor_positions = (int)(angle/(2.0*PI) * 4096.0);
//	std::cout << "Motor " << motor << ": " << motor_positions << std::endl;
	int zero_position = getZeroPose(motor);
	switch (motor) {
	case 1:
		dxl_write_word(motor, P_MOVING_SPEED, speed);
		dxl_write_word(motor, P_GOAL_POSITION, zero_position + motor_positions);
		break;
	case 2:
		dxl_write_word(motor, P_MOVING_SPEED, speed);
		dxl_write_word(motor, P_GOAL_POSITION, zero_position - motor_positions);
		break;
	case 3:
		dxl_write_word(motor, P_MOVING_SPEED, speed);
		dxl_write_word(motor, P_GOAL_POSITION, zero_position + motor_positions);
		break;
	case 4:
		dxl_write_word(motor, P_MOVING_SPEED, speed);
		dxl_write_word(motor, P_GOAL_POSITION, zero_position - motor_positions);
		break;
	case 5:
		dxl_write_word(motor, P_MOVING_SPEED, speed);
		dxl_write_word(motor, P_GOAL_POSITION, zero_position - motor_positions);
		break;
	case 6:
		dxl_write_word(motor, P_MOVING_SPEED, speed);
		dxl_write_word(motor, P_GOAL_POSITION, zero_position + motor_positions);
		break;
	case 7:
		dxl_write_word(motor, P_MOVING_SPEED, speed);
		dxl_write_word(motor, P_GOAL_POSITION, zero_position + motor_positions);
		break;
	case 8:
		dxl_write_word(motor, P_MOVING_SPEED, speed);
		dxl_write_word(motor, P_GOAL_POSITION, zero_position - motor_positions);
		break;
	case 9:
		dxl_write_word(motor, P_MOVING_SPEED, speed);
		dxl_write_word(motor, P_GOAL_POSITION, zero_position + motor_positions);
		break;
	case 10:
		dxl_write_word(motor, P_MOVING_SPEED, speed);
		dxl_write_word(motor, P_GOAL_POSITION, zero_position + motor_positions);
		break;
	case 11:
		dxl_write_word(motor, P_MOVING_SPEED, speed);
		dxl_write_word(motor, P_GOAL_POSITION, zero_position + motor_positions);
		break;
	case 12:
		dxl_write_word(motor, P_MOVING_SPEED, speed);
		dxl_write_word(motor, P_GOAL_POSITION, zero_position - motor_positions);
		break;
	default:
		dxl_write_byte(motor, P_ENABLE, true);
		break;
	/*case 13:
		break;
	case 14:
		break;
	case 15:
		break;
	case 16:
		break;
	case 17:
		break;
	case 18:
		break;
	case 19:
		break;
	case 20:
		break;*/
	}
}

void Dynamixel::enableMotor(int motor) {
	dxl_write_byte(motor, P_ENABLE, true);
}
