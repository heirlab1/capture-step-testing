//============================================================================
// Name        : Capture_Step_Testing.cpp
// Author      : Kellen Carey
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include "iostream"
#include "Dynamixel.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <math.h>

using namespace cv;

#define PI 3.14159265

int pos[20] = {};
int left_leg_length = 35;
int right_leg_length = 35;


int main() {
	std::cout << "Begin testing Capture Step" << std::endl;

	cvNamedWindow("Ball");

	for (int i = 0; i < 20; i++) {
		pos[i] = Dynamixel::getZeroPose(i+1);
	}
//		cvCreateTrackbar("Motor 1", "Ball", &pos[0], 4095, NULL);
//		cvCreateTrackbar("Motor 2", "Ball", &pos[1], 4095, NULL);
//		cvCreateTrackbar("Motor 3", "Ball", &pos[2], 4095, NULL);
//		cvCreateTrackbar("Motor 4", "Ball", &pos[3], 4095, NULL);
//		cvCreateTrackbar("Motor 5", "Ball", &pos[4], 4095, NULL);
//		cvCreateTrackbar("Motor 6", "Ball", &pos[5], 4095, NULL);
//		cvCreateTrackbar("Motor 7", "Ball", &pos[6], 4095, NULL);
//		cvCreateTrackbar("Motor 8", "Ball", &pos[7], 4095, NULL);
//		cvCreateTrackbar("Motor 9", "Ball", &pos[8], 4095, NULL);
//		cvCreateTrackbar("Motor 10", "Ball", &pos[9], 4095, NULL);
//		cvCreateTrackbar("Motor 11", "Ball", &pos[10], 4095, NULL);
//		cvCreateTrackbar("Motor 12", "Ball", &pos[11], 4095, NULL);
	cvCreateTrackbar("Left Leg Length", "Ball", &left_leg_length, 35, NULL);
	cvCreateTrackbar("Right Leg Length", "Ball", &right_leg_length, 35, NULL);

//	}

	cvWaitKey(80);

	Dynamixel::init();
	for (int i = 1; i < 13; i++) {
		Dynamixel::setMotorPosition(i, 0.0, 50);
		Dynamixel::setSyncwriteEachLength(4);
		Dynamixel::setSyncwriteStartAddress(30);
		Dynamixel::sendSyncWrite();
//		Dynamixel::enableMotor(i);
	}

	while (cvWaitKey(80) != 'c') {
		// Deal with Right Leg first
		double theta_knee = PI - acos((614.5-((double)right_leg_length*(double)right_leg_length))/610.5);
		double theta_hip = acos(((((double)right_leg_length*(double)right_leg_length) + 70)/(37*(double)right_leg_length)));
		double theta_foot = acos((((double)right_leg_length*(double)right_leg_length) - 70)/(33*(double)right_leg_length));
		std::cout << "Moving Knee to angle: " << theta_knee << std::endl;
		Dynamixel::setSyncwriteEachLength(4);
		Dynamixel::setSyncwriteStartAddress(30);
		Dynamixel::setMotorPosition(3, theta_hip, 100);
		Dynamixel::setMotorPosition(7, theta_knee, 100);
		Dynamixel::setMotorPosition(9, theta_foot, 100);

		// Deal with the Left Leg second
//		theta_knee = PI - acos((614.5-((double)right_leg_length*(double)right_leg_length))/610.5);
//		theta_hip = acos(((((double)right_leg_length*(double)right_leg_length) + 70)/(37*(double)right_leg_length)));
//		theta_foot = acos((((double)right_leg_length*(double)right_leg_length) - 70)/(33*(double)right_leg_length));
//		std::cout << "Moving Knee to angle: " << theta_knee << std::endl;
		Dynamixel::setMotorPosition(4, theta_hip, 100);
		Dynamixel::setMotorPosition(8, theta_knee, 100);
		Dynamixel::setMotorPosition(10, theta_foot, 100);

		Dynamixel::sendSyncWrite();

	}
}
