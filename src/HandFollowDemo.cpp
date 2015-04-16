/*
 * HandFollowDemo.cpp
 *
 *  Created on: Apr 16, 2015
 *      Author: parzival
 */

#include "HandFollowDemo.h"
#include "Dynamixel.h"
#include <math.h>
#include <pthread.h>
#include <stdio.h>

#define PI				3.141597

const int RIGHT_SHOULDER_OUT		= 14;
const int RIGHT_SHOULDER_FORWARD	= 16;
const int RIGHT_ELBOW				= 18;
const int HEAD_LEFT_RIGHT			= 19;
const int HEAD_UP_DOWN				= 20;

const double UPPER_ARM 			= 5.25;
const double LOWER_ARM 			= 8;
const double FIXED_SHOULDER 	= 1.5;
const double NECK_SHOULDER 		= 3.5;
const double CAMERA_HEIGHT		= 7.0;
double last_clock				= 0.0;
double period					= 1;


double get_distance_from_body_front(double distance);
double get_distance_from_body_side(double distance, double added_theta);
double get_distance_from_shoulder_to_hand();
double get_angle_from_shoulder_to_hand(double distance);
double get_head_left_right(double forward, double out);

double get_distance_from_shoulder_to_hand() {
	double theta_elbow = Dynamixel::getMotorPosition(RIGHT_ELBOW);
	double result = sqrt((UPPER_ARM*UPPER_ARM) + (LOWER_ARM*LOWER_ARM) - 2*(UPPER_ARM)*(LOWER_ARM)*cos(PI-theta_elbow));
	return result;
}

double get_angle_from_shoulder_to_hand(double distance = 0) {
	if (distance == 0) {
		distance = get_distance_from_shoulder_to_hand();
	}
	double theta = acos(((distance*distance) + (UPPER_ARM*UPPER_ARM) - (LOWER_ARM*LOWER_ARM))/(2*(distance)*(UPPER_ARM)));
	return theta;
}

double get_distance_from_body_front(double distance = 0) {
	double theta_shoulder_out = Dynamixel::getMotorPosition(RIGHT_SHOULDER_OUT);
	if (distance == 0) {
		distance = get_distance_from_shoulder_to_hand();
	}
	double result = FIXED_SHOULDER + (distance*sin(theta_shoulder_out));
	return result;
}

double get_distance_from_body_side(double distance = 0, double added_theta = 0) {
	double theta_shoulder_front_back = Dynamixel::getMotorPosition(RIGHT_SHOULDER_FORWARD);
	if (distance == 0) {
		distance = get_distance_from_shoulder_to_hand();
	}
	if (added_theta == 0) {
		added_theta = get_angle_from_shoulder_to_hand(distance);
	}
	double theta_effective = theta_shoulder_front_back + added_theta;
	double result = distance*sin(theta_effective);
	return result;
}

double get_head_left_right(double forward = 0, double out = 0) {
	if (forward == 0) {
		forward = get_distance_from_body_side(0, 0);
	}
	if (out == 0) {
		out = get_distance_from_body_front(0);
	}

	out += NECK_SHOULDER;
	double result = atan(forward / out);
	printf("\rForward: %f\tOut: %f\tResult: %f", forward, out, result);
	return result;
}

double get_head_up_down(double distance = 0, double forward = 0) {
	if (distance == 0) {
		distance = get_distance_from_shoulder_to_hand();
	}
	if (forward == 0) {
		forward = get_distance_from_body_front(distance);
	}
	double theta_forward = asin(forward / distance);
	double theta_calculated = acos(CAMERA_HEIGHT / distance);
	double result = PI/2.0 - (theta_forward + theta_calculated);

	printf("\rUp forward: %f\tCalculated: %f\t result: %f", theta_forward, theta_calculated, result);
	return result;
}


namespace HandFollow {
double getUnixTime() {
	struct timespec tv;

	if (clock_gettime(CLOCK_REALTIME, &tv) != 0) {
		return 0;
	}

	return (((double)tv.tv_sec) + (tv.tv_nsec / 1000000000.0));
}

void* run(void*) {
	last_clock = getUnixTime();
	while (1) {
		pthread_testcancel();
		if ((getUnixTime() - last_clock) > period) {
			Dynamixel::setMotorPosition(HEAD_LEFT_RIGHT, get_head_left_right(0, 0), 50, -1.0);
			Dynamixel::setMotorPosition(HEAD_UP_DOWN, get_head_up_down(), 50, -1.0);
			Dynamixel::setSyncwriteEachLength(4);
			Dynamixel::setSyncwriteStartAddress(30);
			Dynamixel::sendSyncWrite();
			last_clock = getUnixTime();
		}
	}

}

} /* namespace HandFollow */
