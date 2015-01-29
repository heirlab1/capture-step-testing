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
#define E  2.718
int LEG_CENTER = 32;

std::vector<double> sin_values;
std::vector<std::vector<double> > left_leg_values;
std::vector<std::vector<double> > right_leg_values;
int frequency = 60;
int amplitude = 6;
int samples = 64;
int height = 26;
double last_clock = 0;
int fudge_factor = 4;
int fudge_2 = 0;
int ankle_sway_percentage = 25;
int hip_multiplier = 2;
int ankle_multiplier = 1;

//configuration::data main_config;

bool testing = true;

enum THETAS {LEFT_HIP = 0, RIGHT_HIP = 1, LEFT_ANKLE = 2, RIGHT_ANKLE = 3};
enum RAISE_LEG {L_ONE = 0, THETA_TWO = 1, THETA_ANKLE = 2};

double getUnixTime(void);
void setLegLengths(int leg,int length);
std::vector<double> getLegThetas(double amplitude);
void loadConfigs();
void saveConfigs();
std::vector<double> raiseLeg(double height, double leg_length, double theta_one);


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
	cvCreateTrackbar("Frequency", "Ball", &frequency, 10000, NULL);
	cvCreateTrackbar("Amplitude", "Ball", &amplitude, 100, NULL);
	cvCreateTrackbar("Fudge Factor HIPS", "Ball", &fudge_factor, 1000, NULL);
	cvCreateTrackbar("Fudge Factor ANKLES", "Ball", &fudge_2, 1000, NULL);
	cvCreateTrackbar("Ankle SWAY", "Ball", &ankle_sway_percentage, 1000, NULL);
	cvCreateTrackbar("Zero Distance", "Ball", &LEG_CENTER, 34, NULL);
	cvCreateTrackbar("Hip Multiplier", "Ball", &hip_multiplier, 10, NULL);
	cvCreateTrackbar("Ankle Multiplier", "Ball", &ankle_multiplier, 10, NULL);
	cvCreateTrackbar("Height Step", "Ball", &height, 100, NULL);
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
//		if (i < sin_values.size()/4 || i > sin_values.size()*3/4) {
//			sin_values[i] = (sin(2.0*PI*((double)i/sin_values.size())) + cos(4.0*PI*((double)i/sin_values.size())) - PI/16.0);
		sin_values[i] = (sin(2.0*PI*((double)i/sin_values.size())));
//		}
//		else {
//			sin_values[i] = (2 * exp(-9*(i - PI/2.0)) - 1);
//		}
		Dynamixel::sendSyncWrite();
	}

	setLegLengths(0, LEG_CENTER);

	char wait_key = (char)cvWaitKey(80);
	while (wait_key != 'c') {
		if (testing) {
			// Check to see if it's been long enough to update the motor positions
			// Taking 20 discrete moments along the sine curve yields the 5/freq.
			// As the freq is on a scale from 0 - 100, with 100 corresponding to 1 Hz.
			double ellapsed_time = getUnixTime()-last_clock;
			if (ellapsed_time > 1.0/(((double)frequency)/*16*/)) {
				//		if (wait_key == 'n') {

				Dynamixel::setSyncwriteEachLength(4);
				Dynamixel::setSyncwriteStartAddress(30);
				std::vector<double> legValues = getLegThetas((double)(amplitude)*sin_values[current_sin_index]);

				if (sin_values[current_sin_index] < -0.85) {
					// Raise the right leg
					std::vector<double> modified = raiseLeg(height/10.0, LEG_CENTER, legValues[RIGHT_HIP]);
					Dynamixel::setMotorPosition(5, legValues[RIGHT_HIP] + 15*modified[THETA_TWO], -1, 1.0/(((double)frequency)));
					Dynamixel::setMotorPosition(11, (PI/2.0 - modified[THETA_ANKLE])*-1, -1, 1.0/(((double)frequency)/*16*/));
					Dynamixel::setMotorPosition(6, legValues[LEFT_HIP], -1, 1.0/(((double)frequency)/*16*/));
					Dynamixel::setMotorPosition(12,legValues[LEFT_ANKLE], -1, 1.0/(((double)frequency)/*16*/));
					Dynamixel::setMotorPosition(3, -1*sin_values[current_sin_index]/((double)fudge_factor) +
							acos((((modified[L_ONE]*modified[L_ONE]) + 70)/(37*modified[L_ONE]))), -1, 1.0/((double)frequency));
					Dynamixel::setMotorPosition(9, sin_values[current_sin_index]/(2*(double)fudge_factor) +
							acos(((modified[L_ONE]*modified[L_ONE]) - 70)/(33*modified[L_ONE])), -1, 1.0/((double)frequency));

					printf("%f\n", sin_values[current_sin_index]/((double)fudge_factor) +
							acos((((modified[L_ONE]*modified[L_ONE]) + 70)/(37*modified[L_ONE]))));
					setLegLengths(-1, (int)modified[L_ONE]);
					setLegLengths(1, LEG_CENTER);

				}
				else if (sin_values[current_sin_index] > 0.85) {
					// Raise the left leg
					std::vector<double> modified = raiseLeg(height/10.0, LEG_CENTER, legValues[RIGHT_HIP]);
					Dynamixel::setMotorPosition(5, legValues[RIGHT_HIP], -1, 1.0/(((double)frequency)));
					Dynamixel::setMotorPosition(11, legValues[RIGHT_ANKLE], -1, 1.0/(((double)frequency)/*16*/));
					Dynamixel::setMotorPosition(6, legValues[LEFT_HIP] + 15*modified[THETA_TWO], -1, 1.0/(((double)frequency)/*16*/));
					Dynamixel::setMotorPosition(12,legValues[THETA_ANKLE], -1, 1.0/(((double)frequency)/*16*/));
					Dynamixel::setMotorPosition(4, sin_values[current_sin_index]/((double)fudge_factor) +
							acos((((modified[L_ONE]*modified[L_ONE]) + 70)/(37*modified[L_ONE]))), -1, 1.0/((double)frequency));
					Dynamixel::setMotorPosition(10, -1*sin_values[current_sin_index]/(2*(double)fudge_factor) +
							acos(((modified[L_ONE]*modified[L_ONE]) - 70)/(33*modified[L_ONE])), -1, 1.0/((double)frequency));


					printf("%f\n", -1*sin_values[current_sin_index]/((double)fudge_factor) +
							acos((((modified[L_ONE]*modified[L_ONE]) + 70)/(37*modified[L_ONE]))));

					setLegLengths(1, (int)modified[L_ONE]);
					setLegLengths(-1, LEG_CENTER);
				}
				else {
					// Perform the normal leg operations
					Dynamixel::setMotorPosition(5, legValues[RIGHT_HIP], -1, 1.0/(((double)frequency)/*16*/));
					Dynamixel::setMotorPosition(11, legValues[RIGHT_ANKLE], -1, 1.0/(((double)frequency)/*16*/));
					Dynamixel::setMotorPosition(6, legValues[LEFT_HIP], -1, 1.0/(((double)frequency)/*16*/));
					Dynamixel::setMotorPosition(12,legValues[LEFT_ANKLE], -1, 1.0/(((double)frequency)/*16*/));
					setLegLengths(0, LEG_CENTER);
				}

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

std::vector<double> getLegThetas(double amplitude) {
	std::vector<double> result;
//	printf("Amplitude: %f\n", amplitude);
	result.resize(4);
	if (amplitude > 0) {
		double theta = PI/2.0 - acos(amplitude/(double)LEG_CENTER);
		result[LEFT_HIP] = theta;
		result[RIGHT_HIP] = -1*theta;
		result[LEFT_ANKLE] = -1*theta;
		result[RIGHT_ANKLE] = theta;
//		printf("Theta: %f\n", theta);
	}
	else {
		double theta = PI/2.0 - acos(-1*amplitude/(double)LEG_CENTER);
		result[LEFT_HIP] = -1*theta;
		result[RIGHT_HIP] = theta;
		result[LEFT_ANKLE] = theta;
		result[RIGHT_ANKLE] = -1*theta;
//		printf("Theta: %f\n", theta);
	}

	return result;
}

std::vector<double> raiseLeg(double height, double leg_length, double theta_one) {
	/* Calculate the values needed to raise a leg straight up while body is on an angle
	 * height is the height to raise the foot vertically
	 * leg_legth is the initial length of the leg
	 * theta_one is the angle at which the leg is in relation to the center of mass
	 *
	 * l_one is the new leg length
	 * theta_two is the (additional) angle for the hip (should be added on to the pre-existing angle)
	 * theta_ankle is the new ankle theta (can be used straight up)
	 */
	std::vector<double> result;
	result.resize(3);
	double l_one = sqrt(height*height + leg_length*leg_length - 2*height*leg_length*cos(theta_one));
	double theta_two = acos((l_one*l_one + leg_length*leg_length - height*height)/(2*l_one*leg_length));
	double theta_ankle = PI - theta_one - theta_two - PI/2.0;
	result[L_ONE] = l_one;
	result[THETA_TWO] = theta_two;
	result[THETA_ANKLE] = theta_ankle;

	return result;
}

void setLegLengths(int leg, int legLength) {
	if (leg == 0) {
		std::vector<double> result;
		result.resize(3);
		result[0] = acos(((((double)legLength*(double)legLength) + 70)/(37*(double)legLength)));
		result[1] = PI - acos((614.5-((double)legLength*(double)legLength))/610.5);
		result[2] = acos((((double)legLength*(double)legLength) - 70)/(33*(double)legLength));

		Dynamixel::setMotorPosition(3, result[0], -1, 1.0/(((double)frequency)/*16*/));
		Dynamixel::setMotorPosition(9, result[2], -1, 1.0/(((double)frequency)/*16*/));
		Dynamixel::setMotorPosition(7, result[1], -1, 1.0/(((double)frequency)/*16*/));


		Dynamixel::setMotorPosition(4, result[0], -1, 1.0/(((double)frequency)/*16*/));
		Dynamixel::setMotorPosition(10,result[2], -1, 1.0/(((double)frequency)/*16*/));
		Dynamixel::setMotorPosition(8, result[1], -1, 1.0/(((double)frequency)/*16*/));

	}
	else if (leg < 0) {
		// RIGHT LEG
		std::vector<double> result;
		result.resize(3);
		result[0] = acos(((((double)legLength*(double)legLength) + 70)/(37*(double)legLength)));
		result[1] = PI - acos((614.5-((double)legLength*(double)legLength))/610.5);
		result[2] = acos((((double)legLength*(double)legLength) - 70)/(33*(double)legLength));

	//	Dynamixel::setMotorPosition(3, result[0], -1, 1.0/(((double)frequency)/*16*/));
//		Dynamixel::setMotorPosition(9, result[2], -1, 1.0/(((double)frequency)/*16*/));
		Dynamixel::setMotorPosition(7, result[1], -1, 1.0/(((double)frequency)/*16*/));
	}
	else {
		// LEFT_LEG
		std::vector<double> result;
		result.resize(3);
		result[0] = acos(((((double)legLength*(double)legLength) + 70)/(37*(double)legLength)));
		result[1] = PI - acos((614.5-((double)legLength*(double)legLength))/610.5);
		result[2] = acos((((double)legLength*(double)legLength) - 70)/(33*(double)legLength));

	//	Dynamixel::setMotorPosition(4, result[0], -1, 1.0/(((double)frequency)/*16*/));
		//Dynamixel::setMotorPosition(10,result[2], -1, 1.0/(((double)frequency)/*16*/));
		Dynamixel::setMotorPosition(8, result[1], -1, 1.0/(((double)frequency)/*16*/));
	}

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
