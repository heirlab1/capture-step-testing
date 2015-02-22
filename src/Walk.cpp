/*
 * Walk.cpp
 *
 *  Created on: Feb 12, 2015
 *      Author: mul8
 */

#include "Walk.h"
#include "Dynamixel.h"
#include "Motors.h"
#include <stdio.h>
#include <math.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <pthread.h>

namespace WalkEngine {
#define PI 3.14159265
#define E  2.718
int LEG_CENTER = 32;

pthread_mutex_t modifier_mutex = PTHREAD_MUTEX_INITIALIZER;

std::vector<double> sin_values;
std::vector<std::vector<double> > left_leg_values;
std::vector<std::vector<double> > right_leg_values;
int frequency = 60;
int amplitude = 5;
int samples = 64;
int height = 26;
double last_clock = 0;
//int fudge_factor = 4;
int fudge_factor = 16;
int fudge_2 = 0;
int ankle_sway_percentage = 25;
int hip_multiplier = 2;
int ankle_multiplier = 1;
int straight = 1;
double forward_back_offset = 0.0;
int next_straight = 1;
int next_fudge_factor = 16;

//configuration::data main_config;

bool testing = true;

enum THETAS {LEFT_HIP = 0, RIGHT_HIP = 1, LEFT_ANKLE = 2, RIGHT_ANKLE = 3};
enum RAISE_LEG {L_ONE = 0, THETA_TWO = 1, THETA_ANKLE = 2};


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

Walk::Walk() {
	// TODO Auto-generated constructor stub

}

Walk::~Walk() {
	// TODO Auto-generated destructor stub
}

void Walk::run() {

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

				double head_up = Motors::getMotorPosition(24);
				double head_left = Motors::getMotorPosition(23);
				forward_back_offset = 0.0;

				if (head_left < PI/-6.0) {
					Dynamixel::setMotorPosition(13, 0.0, -1, 0.5);
					Dynamixel::setMotorPosition(14, 0.0, -1, 0.5);
//					if (current_sin_index == 0) {
//						straight = 2;
//						fudge_factor = 5;
//					}
				}
				else if (head_left > PI/6.0) {
					Dynamixel::setMotorPosition(13, 0.0, -1, 0.5);
					Dynamixel::setMotorPosition(14, 0.0, -1, 0.5);
//					if (current_sin_index == 0) {
//						straight = 0;
//						fudge_factor = 5;
//					}
				}
				else {
					if (head_up < PI/-4.0) {
						Dynamixel::setMotorPosition(13, PI/-4.0, -1, 0.5);
						Dynamixel::setMotorPosition(14, PI/-4.0, -1, 0.5);
					} else {
					Dynamixel::setMotorPosition(13, PI/-6.0, -1, 0.5);
					Dynamixel::setMotorPosition(14, PI/-6.0, -1, 0.5);
					}
//					if (current_sin_index == 0) {
//						straight = 1;
//						fudge_factor = 4;
//					}
				}

				if (head_up < PI/-6.0) {
					forward_back_offset = 0.01;
				}

				if (current_sin_index == 0) {
					pthread_mutex_lock(&modifier_mutex);
					straight = next_straight;
					fudge_factor = next_fudge_factor;
					pthread_mutex_unlock(&modifier_mutex);
				}

				if (sin_values[current_sin_index] < -0.85) {
					// Raise the right leg
					std::vector<double> modified = raiseLeg(height/10.0, LEG_CENTER, legValues[RIGHT_HIP]);
					Dynamixel::setMotorPosition(5, legValues[RIGHT_HIP] + 15*modified[THETA_TWO], -1, 1.0/(((double)frequency)));
					Dynamixel::setMotorPosition(11, (PI/2.0 - modified[THETA_ANKLE])*-1, -1, 1.0/(((double)frequency)/*16*/));
					Dynamixel::setMotorPosition(6, legValues[LEFT_HIP], -1, 1.0/(((double)frequency)/*16*/));
					Dynamixel::setMotorPosition(12,legValues[LEFT_ANKLE], -1, 1.0/(((double)frequency)/*16*/));

					// If walking straight
					if (straight == 1) {
						Dynamixel::setMotorPosition(3, -1*sin_values[current_sin_index]/((double)fudge_factor) /* - forward_back_offset */+
								acos((((modified[L_ONE]*modified[L_ONE]) + 70)/(37*modified[L_ONE]))), -1, 1.0/((double)frequency));
						Dynamixel::setMotorPosition(9, sin_values[current_sin_index]/(4*(double)fudge_factor) +
								acos(((modified[L_ONE]*modified[L_ONE]) - 70)/(33*modified[L_ONE])), -1, 1.0/((double)frequency));

						Dynamixel::setMotorPosition(1, 0.0, -1, 1.0/((double)frequency));
						Dynamixel::setMotorPosition(2, 0.0, -1, 1.0/((double)frequency));
					}
					// If turning Left
					else if (straight == 0) {
						Dynamixel::setMotorPosition(3, /*-1*sin_values[current_sin_index]/((double)fudge_factor)*/ -0.1 +
								acos((((modified[L_ONE]*modified[L_ONE]) + 70)/(37*modified[L_ONE]))), -1, 1.0/((double)frequency));
						Dynamixel::setMotorPosition(9, /*sin_values[current_sin_index]/(2*(double)fudge_factor)*/ -0.1 +
								acos(((modified[L_ONE]*modified[L_ONE]) - 70)/(33*modified[L_ONE])), -1, 1.0/((double)frequency));

						Dynamixel::setMotorPosition(1, sin_values[current_sin_index]/((double)fudge_factor*3.0), -1, 1.0/((double)frequency));
						Dynamixel::setMotorPosition(2, sin_values[current_sin_index]/((double)fudge_factor*3.0), -1, 1.0/((double)frequency));
					}
					// Turning right
					else {
						Dynamixel::setMotorPosition(3, /*-1*sin_values[current_sin_index]/((double)fudge_factor)*/ -0.1 +
								acos((((modified[L_ONE]*modified[L_ONE]) + 70)/(37*modified[L_ONE]))), -1, 1.0/((double)frequency));
						Dynamixel::setMotorPosition(9, /*sin_values[current_sin_index]/(2*(double)fudge_factor)*/ -0.1 +
								acos(((modified[L_ONE]*modified[L_ONE]) - 70)/(33*modified[L_ONE])), -1, 1.0/((double)frequency));

						Dynamixel::setMotorPosition(1, -1*sin_values[current_sin_index]/((double)fudge_factor*3.0), -1, 1.0/((double)frequency));
						Dynamixel::setMotorPosition(2, -1*sin_values[current_sin_index]/((double)fudge_factor*3.0), -1, 1.0/((double)frequency));

					}

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

					// If walking straight
					if (straight == 1) {
						Dynamixel::setMotorPosition(4, sin_values[current_sin_index]/((double)fudge_factor) + //forward_back_offset +
								acos((((modified[L_ONE]*modified[L_ONE]) + 70)/(37*modified[L_ONE]))), -1, 1.0/((double)frequency));
						Dynamixel::setMotorPosition(10, -1*sin_values[current_sin_index]/(2*(double)fudge_factor) +
								acos(((modified[L_ONE]*modified[L_ONE]) - 70)/(33*modified[L_ONE])), -1, 1.0/((double)frequency));

						Dynamixel::setMotorPosition(1, 0.0, -1, 1.0/((double)frequency));
						Dynamixel::setMotorPosition(2, 0.0, -1, 1.0/((double)frequency));
					}
					// Turning Left
					else if (straight == 0) {
						Dynamixel::setMotorPosition(4, /*sin_values[current_sin_index]/((double)fudge_factor)*/ -0.1 +
								acos((((modified[L_ONE]*modified[L_ONE]) + 70)/(37*modified[L_ONE]))), -1, 1.0/((double)frequency));
						Dynamixel::setMotorPosition(10, /* -1*sin_values[current_sin_index]/(2*(double)fudge_factor)*/ -0.1 +
								acos(((modified[L_ONE]*modified[L_ONE]) - 70)/(33*modified[L_ONE])), -1, 1.0/((double)frequency));

						Dynamixel::setMotorPosition(1, sin_values[current_sin_index]/((double)fudge_factor*3.0), -1, 1.0/((double)frequency));
						Dynamixel::setMotorPosition(2, sin_values[current_sin_index]/((double)fudge_factor*3.0), -1, 1.0/((double)frequency));
					}
					// Turning Right
					else {
						Dynamixel::setMotorPosition(4, /*sin_values[current_sin_index]/((double)fudge_factor)*/ -0.1 +
								acos((((modified[L_ONE]*modified[L_ONE]) + 70)/(37*modified[L_ONE]))), -1, 1.0/((double)frequency));
						Dynamixel::setMotorPosition(10, /* -1*sin_values[current_sin_index]/(2*(double)fudge_factor)*/ -0.1 +
								acos(((modified[L_ONE]*modified[L_ONE]) - 70)/(33*modified[L_ONE])), -1, 1.0/((double)frequency));

						Dynamixel::setMotorPosition(1, -1*sin_values[current_sin_index]/((double)fudge_factor*3.0), -1, 1.0/((double)frequency));
						Dynamixel::setMotorPosition(2, -1*sin_values[current_sin_index]/((double)fudge_factor*3.0), -1, 1.0/((double)frequency));
					}


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
//					Dynamixel::setMotorPosition(1, 0.0, -1, 1.0/((double)frequency));
//					Dynamixel::setMotorPosition(2, 0.0, -1, 1.0/((double)frequency));
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
			printf("Saved Config!\n");
		}

		if (wait_key == 't') {
			testing = !testing;
		}

		wait_key = (char)cvWaitKey(1);
		//		Dynamixel::sendSyncWrite();

	}

}

void Walk::turn_left() {
	pthread_mutex_lock(&modifier_mutex);
	next_straight = 0;
	next_fudge_factor = 5;
	pthread_mutex_unlock(&modifier_mutex);
}
void Walk::turn_right() {
	pthread_mutex_lock(&modifier_mutex);
	next_straight = 2;
	next_fudge_factor = 5;
	pthread_mutex_unlock(&modifier_mutex);
}
void Walk::walk_straight() {
	pthread_mutex_lock(&modifier_mutex);
	next_straight = 1;
	next_fudge_factor = 4;
	pthread_mutex_unlock(&modifier_mutex);
}

} /* namespace WalkEngine */
