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
#include "IMUData.h"
#include "Joystick.h"

#include <iostream>
#include <fstream>

static double acc_x_averages[] = {
		949,
		633.5,
		-239.3333333,
		-1325,
		-1707.833333,
		-1340.833333,
		-870.1666667,
		-297.6666667,
		547.5,
		1343.833333,
		1217.333333,
		-105.3333333,
		-1080.5,
		-1948.833333,
		-2689,
		-2967.5,
		-3839,
		-3597.5,
		-265.1666667,
		1673.166667,
		680.8333333,
		241.1666667,
		-2590.166667,
		-886.3333333,
		-1130.166667,
		-1353.5,
		3317.5,
		2033.166667,
		1786.166667,
		-1089.5,
		-2502.833333,
		-3410.166667,
		-4243.5,
		-3792.833333,
		-3850.666667,
		-3164.833333,
		-1809,
		-2506.5,
		-3411.333333,
		-3933.666667,
		-4944.833333,
		-4965,
		-4715.833333,
		-4043,
		-3136,
		-2102.833333,
		-460.1666667,
		2890.333333,
		3296.333333,
		436.6666667,
		-1288.5,
		-3187.833333,
		-1501.166667,
		-654.1666667,
		-683.6666667,
		191.8333333,
		634.6666667,
		532.3333333,
		-2358,
		-2557.833333,
		-2388.666667,
		-2162.333333,
		1352.333333,
		1031.333333
};
double acc_x_std_dev = 1761.733855;

double acc_y_averages[] = {
		-5646.833333,
		-5264.833333,
		-5533.333333,
		-5520,
		-5748.666667,
		-5623.5,
		-4885.333333,
		-4458.5,
		-4834.5,
		-4802.833333,
		-4946,
		-4624.166667,
		-4737.833333,
		-4102.333333,
		-3813.166667,
		-4488,
		-3616.666667,
		-3820.666667,
		-5341.666667,
		-3567.333333,
		-5045.333333,
		-5715.5,
		-4444,
		-3810.333333,
		-3252.333333,
		5876.166667,
		-6835,
		11135.83333,
		1562.666667,
		782.3333333,
		925.3333333,
		12035.83333,
		401,
		640.6666667,
		-5340.166667,
		-3425.333333,
		-3266.333333,
		-3703.666667,
		-4279,
		-4105.666667,
		-4640.666667,
		-4477.666667,
		-4308.5,
		-4488,
		-2758.666667,
		-3655,
		-2089.833333,
		-1852.333333,
		-2014.833333,
		-4164.5,
		-4875.5,
		-5589,
		-5855.166667,
		-5909.333333,
		-6160,
		-5263.5,
		-5457.833333,
		-6344.333333,
		-6681.666667,
		-5215.166667,
		-5405.833333,
		4733,
		-7096,
		-6588

};
double acc_y_std_dev = 4250.321031/2.0;

double acc_z_averages[] = {
		-12136.33333,
		-12472.5,
		-12726.5,
		-12650.16667,
		-13005.83333,
		-13246.66667,
		-13489.5,
		-13742.5,
		-13828,
		-13578,
		-13399,
		-13060.83333,
		-12931,
		-10321.5,
		-9670.666667,
		-13675.5,
		-13881.66667,
		-13349,
		-14742.83333,
		-17965.5,
		-18275.5,
		-16889.83333,
		-15217.33333,
		-12438.83333,
		-14694.33333,
		-15397.66667,
		-15605.5,
		-12938.66667,
		-8970.166667,
		-9444.166667,
		-11782.5,
		-13502.16667,
		-14129.33333,
		-13392.16667,
		-13690.16667,
		-14106.33333,
		-14243.83333,
		-13243.5,
		-13563.5,
		-13233.16667,
		-13071.83333,
		-13331,
		-13716.66667,
		-13645.66667,
		-13589.33333,
		-11273.66667,
		-9362.666667,
		-13200,
		-14155.83333,
		-17651.66667,
		-18843.5,
		-18577.83333,
		-17233.16667,
		-16154.16667,
		-14973.66667,
		-14153.83333,
		-15010.33333,
		-15337,
		-14270.83333,
		-9165.666667,
		-6780.333333,
		-9554.333333,
		-11406,
		-11178
};
double acc_z_std_dev = 1976.129093;

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
//	init_sdl();
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

	double acc_x;
	double acc_y;
	double acc_z;

	double gyro_x;
	double gyro_y;
	double gyro_z;

	double cmps_x;
	double cmps_y;
	double cmps_z;

	std::ofstream out("Datas.txt");

	if (out.is_open()) {
//		printf("OPENEND THE FILE!!!!!!!!\n");
	}
	else {
//		printf(" FAILED TO OPEN THE FILE\n\n");
	}

	//	out << "Stp\tACC_X\tACC_Y\tACC_Z\tGYRO_X\tGYRO_Y\tGYRO_Z\tCMPS_X\tCMPS_Y\tCMPS_\n";

	while (wait_key != 'c') {
		if (testing) {
			// Check to see if it's been long enough to update the motor positions
			// Taking 20 discrete moments along the sine curve yields the 5/freq.
			// As the freq is on a scale from 0 - 100, with 100 corresponding to 1 Hz.
			double ellapsed_time = getUnixTime()-last_clock;
			if (ellapsed_time > 1.0/(((double)frequency)/*16*/)) {
				//		if (wait_key == 'n') {

				// TODO Get values from joystick and print to console
//				if (Joystick::joystick)

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

//					printf("%f\n", sin_values[current_sin_index]/((double)fudge_factor) +
//							acos((((modified[L_ONE]*modified[L_ONE]) + 70)/(37*modified[L_ONE]))));
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


//					printf("%f\n", -1*sin_values[current_sin_index]/((double)fudge_factor) +
//							acos((((modified[L_ONE]*modified[L_ONE]) + 70)/(37*modified[L_ONE]))));

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
				//				acc_x = IMU::get(IMU::ACCELEROMETER_X);
				//				acc_y = IMU::get(IMU::ACCELEROMETER_Y);
				//				acc_z = IMU::get(IMU::ACCELEROMETER_Z);
				//
				//				gyro_x = IMU::get(IMU::GYROSCOPE_X);
				//				gyro_y = IMU::get(IMU::GYROSCOPE_Y);
				//				gyro_z = IMU::get(IMU::GYROSCOPE_Z);
				//
				//				cmps_x = IMU::get(IMU::COMPASS_X);
				//				cmps_y = IMU::get(IMU::COMPASS_Y);
				//				cmps_z = IMU::get(IMU::COMPASS_Z);
				//
				////				out << current_sin_index << "\t" << acc_x << "\t" << acc_y << "\t" << acc_z << "\t"
				////						<< gyro_x << "\t" << gyro_y << "\t" << gyro_z << "\t"
				////						<< cmps_x << "\t" << cmps_y << "\t" << cmps_z << "\n";
				//
				//				if ((acc_x - acc_x_averages[current_sin_index]) > acc_x_std_dev) {
				//					// TODO Let user know we've exceeded std deviation
				//					Dynamixel::setMotorPosition(15, PI/4, -1, 0.25);
				//					// FIXME Testing to see if setting initial poses works or not
				//					Dynamixel::setInitialPose(5, -5);
				//					Dynamixel::setInitialPose(6, -5);
				////					printf("HAILP!!!!!!\n");
				//				} else if ((acc_x - acc_x_averages[current_sin_index]) < (-1*acc_x_std_dev)) {
				//					// TODO Let user know we've exceeded std deviation
				//					Dynamixel::setMotorPosition(15, PI/4, -1, 0.25);
				//					// FIXME Testing to see if setting initial poses works or not
				//					Dynamixel::setInitialPose(5, 5);
				//					Dynamixel::setInitialPose(6, 5);
				////					printf("HAILP!!!!!!\n");
				//				}
				//				if ((acc_y - acc_y_averages[current_sin_index]) > acc_y_std_dev) {
				//					Dynamixel::setMotorPosition(16, PI/4, -1, 0.25);
				//					// FIXME Testing to see if setting initial poses works or not
				//					Dynamixel::setInitialPose(3, -5);
				//					Dynamixel::setInitialPose(4, 5);
				//					Dynamixel::setInitialPose(3, (int)(acc_y-acc_y_averages[current_sin_index])/-100000.0);
				//					Dynamixel::setInitialPose(4, (int)(acc_y-acc_y_averages[current_sin_index])/100000.0);
				////					printf("HAILP!!!!!!\n");
				//				} else if ((acc_y - acc_y_averages[current_sin_index]) < (-1*acc_y_std_dev/2.0)) {
				//					Dynamixel::setMotorPosition(16, PI/4, -1, 0.25);
				//					// FIXME Testing to see if setting initial poses works or not
				////					Dynamixel::setInitialPose(3, 15);
				////					Dynamixel::setInitialPose(4, -15);
				//					Dynamixel::setInitialPose(3, (int)(acc_y-acc_y_averages[current_sin_index])/-2000.0);
				//					Dynamixel::setInitialPose(4, (int)(acc_y-acc_y_averages[current_sin_index])/2000.0);
				////					printf("HAILP!!!!!!\n");
				//				}
				////				if ((acc_z - acc_z_averages[current_sin_index]) > acc_z_std_dev) {
				//					Dynamixel::setMotorPosition(17, PI/4, -1, 0.25);
				//					Dynamixel::setMotorPosition(18, PI/-4, -1, 0.25);
				//					// FIXME Testing to see if setting initial poses works or not
				//					Dynamixel::setInitialPose(3, -5);
				//					Dynamixel::setInitialPose(4, 5);
				//					printf("HAILP!!!!!!\n");
				//				} else if ((acc_z - acc_z_averages[current_sin_index]) < (-1*acc_z_std_dev)) {
				//					Dynamixel::setMotorPosition(17, PI/4, -1, 0.25);
				//					Dynamixel::setMotorPosition(18, PI/-4, -1, 0.25);
				//					// FIXME Testing to see if setting initial poses works or not
				//					Dynamixel::setInitialPose(3, 5);
				//					Dynamixel::setInitialPose(4, -5);
				//					printf("HAILP!!!!!!\n");
				//				}

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


		//		acc_x = IMU::get(IMU::ACCELEROMETER_X);
		//		acc_y = IMU::get(IMU::ACCELEROMETER_Y);
		//		acc_z = IMU::get(IMU::ACCELEROMETER_Z);
		//
		//		gyro_x = IMU::get(IMU::GYROSCOPE_X);
		//		gyro_y = IMU::get(IMU::GYROSCOPE_Y);
		//		gyro_z = IMU::get(IMU::GYROSCOPE_Z);
		//
		//		cmps_x = IMU::get(IMU::COMPASS_X);
		//		cmps_y = IMU::get(IMU::COMPASS_Y);
		//		cmps_z = IMU::get(IMU::COMPASS_Z);
		//
		//		out << current_sin_index << "\t" << acc_x << "\t" << acc_y << "\t" << acc_z << "\t"
		//				<< gyro_x << "\t" << gyro_y << "\t" << gyro_z << "\t"
		//				<< cmps_x << "\t" << cmps_y << "\t" << cmps_z << "\n";

		//		printf("Got %f for accelerometer x!!!!\n", test);


		if (wait_key == 's') {
			Dynamixel::saveConfig();
			printf("Saved Config!\n");
		}

		if (wait_key == 't') {
			testing = !testing;
		}
		if (wait_key == 'r') {
			for (int i = 15; i < 19; i++) {
				Dynamixel::setMotorPosition(i, 0.0, -1, 0.25);
			}
			Dynamixel::sendSyncWrite();
		}

		wait_key = (char)cvWaitKey(1);
		//		Dynamixel::sendSyncWrite();

	}

	out.close();

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
