//============================================================================
// Name        : Capture_Step_Testing.cpp
// Author      : Kellen Carey
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
//#include "IMUServer.h"
//#include "IMUData.h"
#include "Joystick.h"
#include "CM904Server.h"
#include "CM904.h"
#include "AndroidCommunication.h"

//#define VISION

using namespace cv;

#ifdef VISION
Vision vis;
#endif
WalkEngine::Walk walk;
BallFollow::BallFollower ballFollower(walk);
//static	pthread_mutex_t serial_port_mutex = PTHREAD_MUTEX_INITIALIZER;

void *walk_thread_function(void *arg) {
	walk.run();

	pthread_exit(0);
}

#ifdef VISION
void *vision(void *arg) {
//	vis.init();
	while (1) {
		vis.setAction(CENTER_BALL);
		vis.nextFrame();
	}

	pthread_exit(0);
}
#endif

void *follow(void *arg) {
	while (1) {
		ballFollower.run();
		pthread_testcancel();
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
#ifdef VISION
	vis.init();
#endif
}


int main() {
	pthread_t android;
	pthread_attr_t android_attr;
	pthread_attr_init(&android_attr);

	pthread_t joystick_server;
	pthread_attr_t joystick_attr;

	pthread_attr_init(&joystick_attr);
	BallFollow::BallFollower ballFollower(walk);

	pthread_t walking;
	pthread_attr_t attr;

#ifdef VISION
	pthread_t vision_thread;
	pthread_attr_t vision_attr;
	pthread_attr_init(&vision_attr);
#endif
	pthread_t ball_follower;
	pthread_attr_t ball_attr;

//	pthread_t imu_server;
//	pthread_attr_t imu_attr;

	pthread_t cm904_server;
	pthread_attr_t cm904_attr;

	pthread_attr_init(&attr);
	pthread_attr_init(&ball_attr);
//	pthread_attr_init(&imu_attr);
	pthread_attr_init(&cm904_attr);

	pthread_create(&android, &android_attr, AndroidCommunication::run, 0);

	while (1) {
		std::string command = "";
		while ((command = AndroidCommunication::getString()) == ""); // Busy wait until we have something from the Android
		AndroidCommunication::resetString();
		if (command == "walk") {
			// Start walk threads
			AndroidCommunication::sendString("toSay Walking");
			printf("\rWalking\n");
			init();
			// Create threads
			pthread_create(&walking, &attr, walk_thread_function, 0);
			pthread_create(&cm904_server, &cm904_attr, CM904Server::run, 0);

			// Join threads
			pthread_join(walking, NULL);
			pthread_cancel(cm904_server);
			pthread_join(cm904_server, NULL);

		} else if (command == "track hand") {
			// Start track hand
			AndroidCommunication::sendString("toSay Tracking Hand");
			printf("\rTracking hand\n");
			init();

		} else if (command == "follow ball") {
			// Start ball following
			AndroidCommunication::sendString("toSay Following Ball");
			printf("\rFollowing ball\n");
			init();

			// Create threads
			pthread_create(&walking, &attr, walk_thread_function, 0);
			pthread_create(&cm904_server, &cm904_attr, CM904Server::run, 0);
			pthread_create(&ball_follower, &ball_attr, follow, 0);

			// Join threads
			pthread_join(walking, NULL);
			pthread_cancel(cm904_server);
			pthread_join(cm904_server, NULL);
			pthread_cancel(ball_follower);
			pthread_join(ball_follower, NULL);
		} else if (command == "stand") {
			AndroidCommunication::sendString("toSay Standing");
			printf("Standing");
			init();
		} else {
			String tosay = "toSay Sorry, I don't know how to " + command;
			AndroidCommunication::sendString(tosay);
		}
	}





	pthread_create(&joystick_server, &joystick_attr, Joystick::run, 0);

//	while (Joystick::joy.buttons[Joystick::Y_BUTTON] != BUTTON_PRESSED);

	init();



	// Busy wait
//	while (Joystick::joy.buttons[Joystick::X_BUTTON] != BUTTON_PRESSED);

	pthread_create(&walking, &attr, walk_thread_function, 0);
#ifdef VISION

//	while (Joystick::joy.buttons[Joystick::START_BUTTON] != BUTTON_PRESSED);
	pthread_create(&vision_thread, &vision_attr, vision, 0);
#endif
	pthread_create(&ball_follower, &ball_attr, follow, 0);
	pthread_create(&cm904_server, &cm904_attr, CM904Server::run, 0);

	pthread_join(walking, NULL);
#ifdef VISION
	pthread_cancel(vision_thread);

	pthread_join(vision_thread, NULL);
#endif

	printf("Joined Vision\n");

	pthread_cancel(ball_follower);
//	pthread_cancel(joystick_server);

	printf("Ball Follower Cancelled\n");


	pthread_join(ball_follower, NULL);

//	pthread_cancel(imu_server);
	pthread_cancel(cm904_server);

//	pthread_join(imu_server, NULL);
	pthread_join(cm904_server, NULL);

	printf("Ball Follower Joined\n");

	return 0;
}
