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
#include <ctime>
#include <stdio.h>
//#include "config_parser.h"
#include <sstream>
#include<sys/types.h>
#include <unistd.h>
#include <pthread.h>
#include "Vision.h"
#include "Motors.h"
#include "Walk.h"
#include "BallFollower.h"

using namespace cv;

Vision vis;
WalkEngine::Walk walk;
BallFollow::BallFollower ballFollower(walk);


//double getUnixTime(void);
//void setLegLengths(int leg,int length);
//std::vector<double> getLegThetas(double amplitude);
//void loadConfigs();
//void saveConfigs();
//std::vector<double> raiseLeg(double height, double leg_length, double theta_one);
//int run();

void *walk_thread_function(void *arg) {
	walk.run();

	pthread_exit(0);
}

void *vision(void *arg) {
//	vis.init();
	while (1) {
		vis.setAction(CENTER_BALL);
		vis.nextFrame();
	}

	pthread_exit(0);
}

void *follow(void *arg) {
	while (1) {
		ballFollower.run();
	}

	pthread_exit(0);
}

void loadConfigs() {
//	std::ifstream f("src/main_config.ini");
//	f >> main_config;
//	f.close();
//
//	std::istringstream(main_config["frequency"]) >> frequency;
//	std::istringstream(main_config["amplitude"]) >> amplitude;
//	std::istringstream(main_config["samples"]) >> samples;
//	std::istringstream(main_config["fudge_factor"]) >> fudge_factor;
//	std::istringstream(main_config["fudge_2"]) >> fudge_2;
//	std::istringstream(main_config["ankle_sway_percentage"]) >> ankle_sway_percentage;
}

void saveConfig() {
//		std::ostringstream value_convert;
//		value_convert << frequency;
//		config["frequency"] =  value_convert.str();
//
//
//	std::ofstream out("src/config.ini");
//	out << config;
//	out.close();
}

void init() {
	std::cout << "Begin testing Capture Step" << std::endl;

//	cvNamedWindow("Walk Parameters");

	//	for (int i = 0; i < 20; i++) {
	//		pos[i] = Dynamixel::getZeroPose(i+1);
	//	}
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
//	cvCreateTrackbar("Frequency", "Walk Parameters", &frequency, 10000, NULL);
//	cvCreateTrackbar("Amplitude", "Walk Parameters", &amplitude, 100, NULL);
//	cvCreateTrackbar("Fudge Factor HIPS", "Walk Parameters", &fudge_factor, 1000, NULL);
//	cvCreateTrackbar("Fudge Factor ANKLES", "Walk Parameters", &fudge_2, 1000, NULL);
//	cvCreateTrackbar("Ankle SWAY", "Walk Parameters", &ankle_sway_percentage, 1000, NULL);
//	cvCreateTrackbar("Zero Distance", "Walk Parameters", &LEG_CENTER, 34, NULL);
//	cvCreateTrackbar("Hip Multiplier", "Walk Parameters", &hip_multiplier, 10, NULL);
//	cvCreateTrackbar("Ankle Multiplier", "Walk Parameters", &ankle_multiplier, 10, NULL);
//	cvCreateTrackbar("Height Step", "Walk Parameters", &height, 100, NULL);
//	cvCreateTrackbar("Walk Straight", "Walk Parameters", &straight, 2, NULL);
	//	}

	cvWaitKey(80);

	Dynamixel::init();
	for (int i = 1; i < 13; i++) {
		Dynamixel::setMotorPosition(i, 0.0, -1, 1);
		//		Dynamixel::enableMotor(i);
	}

	Dynamixel::setMotorPosition(16, 0.0, 25, -1);
	Dynamixel::setMotorPosition(15, 0.0, 25, -1);
	Dynamixel::setMotorPosition(17, 0.0, 25, -1);
	Dynamixel::setMotorPosition(18, 0.0, 25, -1);
	Dynamixel::setMotorPosition(23, 0.0, 25, -1);
	Dynamixel::setMotorPosition(24, 0.0, 25, -1);
	Dynamixel::setMotorPosition(13, 0.0, -1, 0.5);
	Dynamixel::setMotorPosition(14, 0.0, -1, 0.5);
	Dynamixel::setSyncwriteEachLength(4);
	Dynamixel::setSyncwriteStartAddress(30);
	Dynamixel::sendSyncWrite();

	vis.init();
}


int main() {
	init();

	BallFollow::BallFollower ballFollower(walk);

	pthread_t walking;
	pthread_attr_t attr;
	pthread_t vision_thread;
	pthread_attr_t vision_attr;
	pthread_t ball_follower;
	pthread_attr_t ball_attr;

	pthread_attr_init(&attr);
	pthread_attr_init(&vision_attr);
	pthread_attr_init(&ball_attr);

	pthread_create(&walking, &attr, walk_thread_function, 0);
	pthread_create(&vision_thread, &vision_attr, vision, 0);
	pthread_create(&ball_follower, &ball_attr, follow, 0);

	pthread_join(walking, NULL);

	pthread_cancel(vision_thread);

	pthread_join(vision_thread, NULL);

	printf("Joined Vision\n");

	pthread_cancel(ball_follower);

	printf("Ball Follower Cancelled\n");

	// Don't need to worry about joining here because we are exiting the program
	// However, it is slightly worrisome why this causes the program to hang
	pthread_join(ball_follower, NULL);

	printf("Ball Follower Joined\n");

	return 0;
}
