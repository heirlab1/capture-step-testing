/*
 * Dynamixel.cpp
 *
 *  Created on: Dec 8, 2014
 *      Author: unicorn
 */

#include "Dynamixel.h"
#include "dynamixel.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <vector>
#include <ctime>
#include "config_parser.h"
#include <fstream>
#include <sstream>

#define P_GOAL_POSITION		30
#define P_MOVING_SPEED		32
#define P_ENABLE			24
#define P_PRESENT_POSITION	36

#define PI 3.14159265

int initial_poses[] = {650, 3436, 1005, 2079,
		2495, 2159, 503, 1548, 2638, 1101,
		1992, 1058
};

int pos[12];

configuration::data config;

// Variables for the SyncWrite function
std::vector<int> ids;
std::vector<int> data;
int start_address = 0;
int each_length = 0;

void Dynamixel::init() {
	dxl_initialize(0, 0);

	std::ifstream f("src/config.ini");
	f >> config;
	f.close();

	cvNamedWindow("Initial Positions");

	std::map<std::string, std::string>::iterator it;

	for (it = config.begin(); it != config.end(); it++) {

		if (it->first == "motor_1_zero") {
			std::istringstream(it->second) >> pos[0];
		}
		else if (it->first == "motor_2_zero") {
			std::istringstream(it->second) >> pos[1];
		}
		else if (it->first == "motor_3_zero") {
			std::istringstream(it->second) >> pos[2];
		}
		else if (it->first == "motor_4_zero") {
			std::istringstream(it->second) >> pos[3];
		}
		else if (it->first == "motor_5_zero") {
			std::istringstream(it->second) >> 	pos[4];
		}
		else if (it->first == "motor_6_zero") {
			std::istringstream(it->second) >> 	pos[5];
		}
		else if (it->first == "motor_7_zero") {
			std::istringstream(it->second) >> pos[6];
		}
		else if (it->first == "motor_8_zero") {
			std::istringstream(it->second) >> pos[7];
		}
		else if (it->first == "motor_9_zero") {
			std::istringstream(it->second) >> pos[8];
		}
		else if (it->first == "motor_10_zero") {
			std::istringstream(it->second) >> pos[9];
		}
		else if (it->first == "motor_11_zero") {
			std::istringstream(it->second) >> pos[10];
		}
		else if (it->first == "motor_12_zero") {
			std::istringstream(it->second) >> pos[11];
		}
	}
	cvCreateTrackbar("Motor 1", "Initial Positions", &pos[0], 4095, NULL);
	cvCreateTrackbar("Motor 2", "Initial Positions", &pos[1], 4095, NULL);
	cvCreateTrackbar("Motor 3", "Initial Positions", &pos[2], 4095, NULL);
	cvCreateTrackbar("Motor 4", "Initial Positions", &pos[3], 4095, NULL);
	cvCreateTrackbar("Motor 5", "Initial Positions", &pos[4], 4095, NULL);
	cvCreateTrackbar("Motor 6", "Initial Positions", &pos[5], 4095, NULL);
	cvCreateTrackbar("Motor 7", "Initial Positions", &pos[6], 4095, NULL);
	cvCreateTrackbar("Motor 8", "Initial Positions", &pos[7], 4095, NULL);
	cvCreateTrackbar("Motor 9", "Initial Positions", &pos[8], 4095, NULL);
	cvCreateTrackbar("Motor 10", "Initial Positions", &pos[9], 4095, NULL);
	cvCreateTrackbar("Motor 11", "Initial Positions", &pos[10], 4095, NULL);
	cvCreateTrackbar("Motor 12", "Initial Positions", &pos[11], 4095, NULL);
}

void Dynamixel::saveConfig() {

	for (int i = 0; i < 12; i++) {
		std::ostringstream key_convert;
		std::ostringstream value_convert;
		key_convert << (i+1);
		std::string key = "motor_";
		key.append(key_convert.str());
		key.append("_zero");
		value_convert << pos[i];
		std::cout << "Key: " << key << "\tValue: " << value_convert.str() << std::endl;
			config[key] =  value_convert.str();
	}

	std::ofstream out("src/config.ini");
	out << config;
	out.close();
	init();
}

void Dynamixel::close() {
	dxl_terminate();
}

int Dynamixel::getZeroPose(int motor) {
	if (motor > 0 && motor <= 12) {
		//		return initial_poses[motor-1];
		return pos[motor-1];
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
	std::vector<int> newData;
	switch (motor) {
	case 1:
		//		dxl_write_word(motor, P_MOVING_SPEED, speed);
		//		dxl_write_word(motor, P_GOAL_POSITION, zero_position + motor_positions);
		newData.push_back(dxl_get_lowbyte(zero_position + motor_positions));
		newData.push_back(dxl_get_highbyte(zero_position + motor_positions));
		//		addToSyncwrite(motor, data);
		break;
	case 2:
		//		dxl_write_word(motor, P_MOVING_SPEED, speed);
		//		dxl_write_word(motor, P_GOAL_POSITION, zero_position - motor_positions);
		newData.push_back(dxl_get_lowbyte(zero_position - motor_positions));
		newData.push_back(dxl_get_highbyte(zero_position - motor_positions));
		break;
	case 3:
		//		dxl_write_word(motor, P_MOVING_SPEED, speed);
		//		dxl_write_word(motor, P_GOAL_POSITION, zero_position + motor_positions);
		newData.push_back(dxl_get_lowbyte(zero_position + motor_positions));
		newData.push_back(dxl_get_highbyte(zero_position + motor_positions));
		break;
	case 4:
		//		dxl_write_word(motor, P_MOVING_SPEED, speed);
		//		dxl_write_word(motor, P_GOAL_POSITION, zero_position - motor_positions);
		newData.push_back(dxl_get_lowbyte(zero_position - motor_positions));
		newData.push_back(dxl_get_highbyte(zero_position - motor_positions));
		break;
	case 5:
		//		dxl_write_word(motor, P_MOVING_SPEED, speed);
		//		dxl_write_word(motor, P_GOAL_POSITION, zero_position - motor_positions);
		newData.push_back(dxl_get_lowbyte(zero_position - motor_positions));
		newData.push_back(dxl_get_highbyte(zero_position - motor_positions));
		break;
	case 6:
		//		dxl_write_word(motor, P_MOVING_SPEED, speed);
		//		dxl_write_word(motor, P_GOAL_POSITION, zero_position + motor_positions);
		newData.push_back(dxl_get_lowbyte(zero_position + motor_positions));
		newData.push_back(dxl_get_highbyte(zero_position + motor_positions));
		break;
	case 7:
		//		dxl_write_word(motor, P_MOVING_SPEED, speed);
		//		dxl_write_word(motor, P_GOAL_POSITION, zero_position + motor_positions);
		newData.push_back(dxl_get_lowbyte(zero_position + motor_positions));
		newData.push_back(dxl_get_highbyte(zero_position + motor_positions));
		break;
	case 8:
		//		dxl_write_word(motor, P_MOVING_SPEED, speed);
		//		dxl_write_word(motor, P_GOAL_POSITION, zero_position - motor_positions);
		newData.push_back(dxl_get_lowbyte(zero_position - motor_positions));
		newData.push_back(dxl_get_highbyte(zero_position - motor_positions));
		break;
	case 9:
		//		dxl_write_word(motor, P_MOVING_SPEED, speed);
		//		dxl_write_word(motor, P_GOAL_POSITION, zero_position + motor_positions);
		newData.push_back(dxl_get_lowbyte(zero_position + motor_positions));
		newData.push_back(dxl_get_highbyte(zero_position + motor_positions));
		break;
	case 10:
		//		dxl_write_word(motor, P_MOVING_SPEED, speed);
		//		dxl_write_word(motor, P_GOAL_POSITION, zero_position + motor_positions);
		newData.push_back(dxl_get_lowbyte(zero_position + motor_positions));
		newData.push_back(dxl_get_highbyte(zero_position + motor_positions));
		break;
	case 11:
		//		dxl_write_word(motor, P_MOVING_SPEED, speed);
		//		dxl_write_word(motor, P_GOAL_POSITION, zero_position + motor_positions);
		newData.push_back(dxl_get_lowbyte(zero_position + motor_positions));
		newData.push_back(dxl_get_highbyte(zero_position + motor_positions));
		break;
	case 12:
		//		dxl_write_word(motor, P_MOVING_SPEED, speed);
		//		dxl_write_word(motor, P_GOAL_POSITION, zero_position - motor_positions);
		newData.push_back(dxl_get_lowbyte(zero_position - motor_positions));
		newData.push_back(dxl_get_highbyte(zero_position - motor_positions));
		break;
	default:
		dxl_write_byte(motor, P_ENABLE, true);
		return;
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
	newData.push_back(dxl_get_lowbyte(speed));
	newData.push_back(dxl_get_highbyte(speed));

	addToSyncwrite(motor, newData);
}

void Dynamixel::enableMotor(int motor) {
	dxl_write_byte(motor, P_ENABLE, true);
}


bool Dynamixel::addToSyncwrite(int id, std::vector<int> newData) {
	//std::cout << "Adding ID " << id << ": ";
	ids.push_back(id);
	for (unsigned i = 0; i < newData.size(); i++) {
		data.push_back(newData[i]);
		//std::cout << newData[i] << " ";
	}
	//std::cout << std::endl;
	return true;
}

bool Dynamixel::setSyncwriteStartAddress(int startAddress) {
	start_address = startAddress;
	return true;
}

bool Dynamixel::setSyncwriteEachLength(int eachLength) {
	each_length = eachLength;
	return true;
}

bool Dynamixel::sendSyncWrite() {
	int idsLength= ids.size();

	for (unsigned i = 0; i < data.size(); i++) {
		//std::cout << data[i] << " ";
	}
	// std::cout << std::endl;

	dxl_set_txpacket_id(BROADCAST_ID);//ID of destination
	dxl_set_txpacket_instruction(INST_SYNC_WRITE);// packet type
	dxl_set_txpacket_parameter(0, start_address);//which data to manipulate. speed, goal pos, ect
	dxl_set_txpacket_parameter(1, each_length);//how long the instruction will be. 2 for word, 1 for byte,

	for(int i=0; i<idsLength; i++ )
	{
		// Set the ID number
		dxl_set_txpacket_parameter(2+(each_length+1)*i, ids[i]);
		//std::cout << "ID " << ids[i] << ": ";
		// Set the data values
		for (int j = 1; j < each_length+1; j++) {
			dxl_set_txpacket_parameter(2+(each_length+1)*i + j, data[(each_length)*i + (j-1)]);
			//std::cout << data[(each_length) * i + j-1] << " ";
		}
		//std::cout << std::endl;
		//             if (each_length == 1) {
		//                     dxl_set_txpacket_parameter(2+2*i+1, data[i]);
		//             }
		//
		//             else if (each_length == 2) {
		//                     dxl_set_txpacket_parameter(2+3*i+1, data[2*i]);
		//                     dxl_set_txpacket_parameter(2+3*i+2, data[2*i + 1]);
		//             }
	}
	//std::cout << "made it here" << std::endl;

	dxl_set_txpacket_length((each_length+1)*idsLength+4);//(2+1) for writing a word, (1+1) for writing a byte, 4 is a constant

	dxl_txrx_packet();//sends the packet

	ids.resize(0);
	data.resize(0);
	if (dxl_get_result( ) == COMM_TXSUCCESS) {
		return true;
	}
	return false;

}
