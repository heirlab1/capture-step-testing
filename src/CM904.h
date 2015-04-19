#ifndef CM904_H_
#define CM904_H_

#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <sstream>
#include <vector>
#include <math.h>
#include <fstream>

#define X_AXIS	0
#define Y_AXIS	1
#define Z_AXIS	2

#define GYROSCOPE_HEADER	71
#define ACCELEROMETER_HEADER	65
#define COMPASS_HEADER	67


using namespace std;

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

struct System{
	int batt_voltage;
	int button_map;
};


class CM904{
public:

	CM904();
	virtual ~CM904();
	void readSerial();
	int init_serial_input(char *);
	void	writeToCM904(char[3]);
	void readSysData();
	void sysCmd(bool, bool, int);
	void printValues();
	Gyro get_gyro();
	Accl get_accl();
	Cmps get_cmps();
	System get_system();
	void close_port();
};

#endif /*CM904_H_*/
