/*
 * Dynamixel.cpp
 *
 *  Created on: Dec 8, 2014
 *      Author: unicorn
 */

#include "Dynamixel.h"
#include "dynamixel.h"
#include <iostream>
#include <vector>
#include <ctime>

#define P_GOAL_POSITION		30
#define P_MOVING_SPEED		32
#define P_ENABLE			24
#define P_PRESENT_POSITION	36

#define PI 3.14159265

int initial_poses[] = {650, 3436, 1005, 2079,
		2495, 2159, 503, 1548, 2638, 1101,
		1992, 1058
};

// Variables for the SyncWrite function
std::vector<int> ids;
std::vector<int> data;
int start_address = 0;
int each_length = 0;

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
