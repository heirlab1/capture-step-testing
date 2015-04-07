/*
 * Author: Adam Stroud
 */
#ifndef READ_IMU_H_
#define READ_IMU_H_

#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <vector>
#include <math.h>
#include <fstream>
#include <unistd.h>


#define X_AXIS	0
#define Y_AXIS	1
#define Z_AXIS	2

#define GYROSCOPE_HEADER	71
#define ACCELEROMETER_HEADER	65
#define COMPASS_HEADER	67

#define USB_SERIAL_PORT "/dev/ttyACM0" // (Linux Ubuntu Mac Book)


using namespace std;
//char sensor_buf[1023];


struct Gyro{
	int x;
	int y;
	int z;
};

struct Accl{
	int x;
	int y;
	int z;
};

struct Cmps{
	int x;
	int y;
	int z;
};


class ReadIMU{
public:

	ReadIMU();
	virtual ~ReadIMU();
	int init_serial_input (char *);
	void read_serial_xyz ();
	void printValues();
	int readLeftHand();
	int readRightHand();
	Gyro get_gyro();
	Accl get_accl();
	Cmps get_cmps();
	void close_port();
};

#endif /*READ_IMU_H*/
