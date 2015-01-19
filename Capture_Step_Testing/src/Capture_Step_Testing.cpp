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

using namespace cv;

#define PI 3.14159265
int LEG_CENTER = 32;

std::vector<double> sin_values;
std::vector<std::vector<double> > left_leg_values;
std::vector<std::vector<double> > right_leg_values;
int frequency = 12;
int amplitude = 5;
int samples = 128;
double last_clock = 0;
int fudge_factor = 0;
int fudge_2 = 0;
int ankle_sway_percentage = 25;
int hip_multiplier = 2;
int ankle_multiplier = 1;

//configuration::data main_config;

bool testing = true;

enum THETAS {HIP = 0, KNEE = 1, ANKLE = 2};

double getUnixTime(void);
std::vector<double> getLegThetas(double legLength);
void loadConfigs();
void saveConfigs();


int main() {
	std::cout << "Begin testing Capture Step" << std::endl;

	cvNamedWindow("Ball");

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
	cvCreateTrackbar("Frequency", "Ball", &frequency, 1000, NULL);
	cvCreateTrackbar("Amplitude", "Ball", &amplitude, 100, NULL);
	cvCreateTrackbar("Fudge Factor HIPS", "Ball", &fudge_factor, 1000, NULL);
	cvCreateTrackbar("Fudge Factor ANKLES", "Ball", &fudge_2, 1000, NULL);
	cvCreateTrackbar("Ankle SWAY", "Ball", &ankle_sway_percentage, 1000, NULL);
	cvCreateTrackbar("Zero Distance", "Ball", &LEG_CENTER, 34, NULL);
	cvCreateTrackbar("Hip Multiplier", "Ball", &hip_multiplier, 10, NULL);
	cvCreateTrackbar("Ankle Multiplier", "Ball", &ankle_multiplier, 10, NULL);
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
	Dynamixel::setSyncwriteEachLength(4);
	Dynamixel::setSyncwriteStartAddress(30);
	Dynamixel::sendSyncWrite();

	last_clock = getUnixTime();
	int current_sin_index = 0;
	sin_values.resize(samples);
	for (unsigned i = 0; i < sin_values.size(); i++) {
		sin_values[i] = (sin(2.0*PI*((double)i/sin_values.size())) + cos(4.0*PI*((double)i/sin_values.size())));
	}
	int last_sin_size = sin_values.size();

	right_leg_values.resize(last_sin_size);
	left_leg_values.resize(last_sin_size);
	for (int i = 0; i < last_sin_size; i++) {
		right_leg_values[i].resize(3);
		left_leg_values[i].resize(3);
		right_leg_values[i] = getLegThetas((double)LEG_CENTER + (double)(amplitude/20.0)*sin_values[i]);
		left_leg_values[i] = getLegThetas((double)LEG_CENTER - (double)(amplitude/20.0)*sin_values[i]);
		//		printf("Right Leg Values for %i: %f, %f, %f\n", i, right_leg_values[i][HIP], right_leg_values[i][KNEE], right_leg_values[i][ANKLE]);
	}
	char wait_key = (char)cvWaitKey(80);
	while (wait_key != 'c') {
		if (testing) {
			// Check to see if it's been long enough to update the motor positions
			// Taking 20 discrete moments along the sine curve yields the 5/freq.
			// As the freq is on a scale from 0 - 100, with 100 corresponding to 1 Hz.
			double ellapsed_time = getUnixTime()-last_clock;
			if (ellapsed_time > 1.0/(((double)frequency)/*16*/)) {
				//		if (wait_key == 'n') {

//				std::cout << "Ellpased Time should be: " << 1.0/(((double)frequency)/*(double)sin_values.size()*/);
//				std::cout << "\t Ellapsed Time is: " << ellapsed_time << std::endl;
//				std::cout << "Iteration: " << current_sin_index << std::endl;
				Dynamixel::setSyncwriteEachLength(4);
				Dynamixel::setSyncwriteStartAddress(30);
							std::vector<double> rightLegValues = getLegThetas((double)LEG_CENTER + (double)(amplitude/20.0)*sin_values[current_sin_index]);
//							std::vector<double> leftLegValues = getLegThetas((double)LEG_CENTER - (double)(amplitude/20.0)*sin_values[current_sin_index]);
							std::vector<double> leftLegValues = getLegThetas((double)LEG_CENTER + (double)(amplitude/20.0)*sin_values[(current_sin_index + samples/2) % samples]);
//							printf("Amplitude: %i\n", amplitude);
				if (current_sin_index < (samples*3/4) - 1 && current_sin_index > (samples*1/4) - 1) {
									Dynamixel::setMotorPosition(3, rightLegValues[HIP] + ((double)fudge_factor/1000.0 * sin_values[current_sin_index]), -1, 1.0/(((double)frequency)/*16*/));
//					Dynamixel::setMotorPosition(3, right_leg_values[current_sin_index][HIP] + ((double)fudge_factor/1000.0), 500);
									Dynamixel::setMotorPosition(9, rightLegValues[ANKLE] - ((double)fudge_factor/1000. * sin_values[current_sin_index]), -1, 1.0/(((double)frequency)/*16*/));
//					Dynamixel::setMotorPosition(9, right_leg_values[current_sin_index][ANKLE] + ((double)fudge_2/1000.0), 500);
				}
				else {
									Dynamixel::setMotorPosition(3, rightLegValues[HIP] - ((double)fudge_factor/1000.0 * sin_values[current_sin_index]), -1, 1.0/(((double)frequency)/*16*/));
//					Dynamixel::setMotorPosition(3, right_leg_values[current_sin_index][HIP], 500);
									Dynamixel::setMotorPosition(9, rightLegValues[ANKLE] + ((double)fudge_factor/1000.0 * sin_values[current_sin_index]), -1, 1.0/(((double)frequency)/*16*/));
//					Dynamixel::setMotorPosition(9, right_leg_values[current_sin_index][ANKLE], 500);
				}
//				Dynamixel::setMotorPosition(7, right_leg_values[current_sin_index][KNEE], 500);
							Dynamixel::setMotorPosition(7, rightLegValues[KNEE], -1, 1.0/(((double)frequency)/*16*/));


				if (current_sin_index > (samples*3/4) - 1 || current_sin_index < (samples*1/4) - 1) {
									Dynamixel::setMotorPosition(4, leftLegValues[HIP] + ((double)fudge_factor/1000.0 * sin_values[current_sin_index]), -1, 1.0/(((double)frequency)/*16*/));
//					Dynamixel::setMotorPosition(4, left_leg_values[current_sin_index][HIP] + ((double)fudge_factor/1000.0), 500);
									Dynamixel::setMotorPosition(10,leftLegValues[ANKLE] - ((double)fudge_factor/1000.0 * sin_values[current_sin_index]), -1, 1.0/(((double)frequency)/*16*/));
//					Dynamixel::setMotorPosition(10,left_leg_values[current_sin_index][ANKLE] + ((double)fudge_2/1000.0), 500);
				}
				else {
									Dynamixel::setMotorPosition(4, leftLegValues[HIP] - ((double)fudge_factor/1000.0 * sin_values[current_sin_index]), -1, 1.0/(((double)frequency)/*16*/));
//					Dynamixel::setMotorPosition(4, left_leg_values[current_sin_index][HIP], 500);
									Dynamixel::setMotorPosition(10,leftLegValues[ANKLE] + ((double)fudge_factor/1000. * sin_values[current_sin_index]), -1, 1.0/(((double)frequency)/*16*/));
//					Dynamixel::setMotorPosition(10,left_leg_values[current_sin_index][ANKLE], 500);
				}
							Dynamixel::setMotorPosition(8, leftLegValues[KNEE], -1, 1.0/(((double)frequency)/*16*/));
//				Dynamixel::setMotorPosition(8, left_leg_values[current_sin_index][KNEE], 500);

//				printf("Ankle SWAY: %f\n", ((double)ankle_sway_percentage/1000.0));
				double ankle_theta = ((double)ankle_sway_percentage/1000.0) * atan((double)amplitude*sin_values[current_sin_index]/6.0);
				Dynamixel::setMotorPosition(11, -1*ankle_multiplier*ankle_theta, -1, 1.0/(((double)frequency)/*16*/));
				Dynamixel::setMotorPosition(12, ankle_multiplier*ankle_theta, -1, 1.0/(((double)frequency)/*16*/));
				Dynamixel::setMotorPosition(5, hip_multiplier*ankle_theta, -1, 1.0/(((double)frequency)/*16*/));
				Dynamixel::setMotorPosition(6, -1*hip_multiplier*ankle_theta, -1, 1.0/(((double)frequency)/*16*/));


				//			std::cout << "Made it!" << std::endl;

				//			if (last_sin_size != samples) {
				//				sin_values.resize(samples);
				//				for (unsigned i = 0; i < sin_values.size(); i++) {
				//					sin_values[i] = sin(2.0*PI*((double)i/sin_values.size()));
				//				}
				//			}

				current_sin_index = (current_sin_index + 1) % samples;

				if (current_sin_index == samples) {
					current_sin_index = 0;
				}

				//			std::cout << "Iteration: " << current_sin_index << std::endl;

				last_clock = getUnixTime();
			}

		}
		else {
			for (int i = 0; i < 12; i++) {
				Dynamixel::setMotorPosition(i+1, 0.0, 100, -1);
			}
		}


		Dynamixel::sendSyncWrite();


		if (wait_key == 's') {
			Dynamixel::saveConfig();
			std::cout << "Saved Config!" << std::endl;
		}

		if (wait_key == 't') {
			testing = !testing;
		}

		wait_key = (char)cvWaitKey(1);
		//		Dynamixel::sendSyncWrite();

	}
}

std::vector<double> getLegThetas(double legLength) {
	std::vector<double> result;
	result.resize(3);
	result[HIP] = acos(((((double)legLength*(double)legLength) + 70)/(37*(double)legLength)));
	result[KNEE] = PI - acos((614.5-((double)legLength*(double)legLength))/610.5);
	result[ANKLE] = acos((((double)legLength*(double)legLength) - 70)/(33*(double)legLength));

	return result;
}

double getUnixTime() {
	struct timespec tv;

	if (clock_gettime(CLOCK_REALTIME, &tv) != 0) {
		return 0;
	}

	return (((double)tv.tv_sec) + (tv.tv_nsec / 1000000000.0));
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
