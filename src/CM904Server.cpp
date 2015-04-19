/*
 * CM904Server.cpp
 *
 *  Created on: Feb 24, 2015
 *      Author: mul8
 */

#include "CM904Server.h"
#include "CM904Data.h"
#include <ctime>

CM904 cm904;

double timeout = 1.0/100.0;
char sys_cmd_packet[3];


double getUnixTime() {
	struct timespec tv;

	if (clock_gettime(CLOCK_REALTIME, &tv) != 0) {
		return 0;
	}

	return (((double)tv.tv_sec) + (tv.tv_nsec / 1000000000.0));
}


void* CM904Server::run(void * args) {
	double last_time = getUnixTime();
	int motor_power;
	int motor_pwr_counter;
	int LED_power;
	FILE *cm904_port;

	Gyro gyro;
	Accl accl;
	Cmps cmps;
	System sys;

	sys_cmd_packet[0] = 0x5A;
	sys_cmd_packet[1]= motor_power;
	sys_cmd_packet[2]= LED_power;


	while (1) {



		//usleep(10000); //we were using this to set the frequency. the condition
		// below will do a system coordinated pause before we get the next
		

		if ((getUnixTime() - last_time) > timeout) {
			/*	this will control cm904 LEDs and motor power	*/

			if(motor_pwr_counter == timeout*10){

			/*Toggle power power for testing purposes*/
				if(motor_power){
					motor_power = false;
				}
				else{
					motor_power = true;
				}
				motor_pwr_counter = 0;
			}
			motor_pwr_counter++;

			sys_cmd_packet[1] = motor_power;

			// TODO How long does it take to open the port, versus what is the impact of having the port constantly open
//			cm904_port = fopen(CM904_USB_PORT, "w");
//			fprintf(cm904_port,"%s",sys_cmd_packet);
//			fclose(cm904_port);
			cm904.writeToCM904(sys_cmd_packet);
			/*this loop will fetch cm904 system data packet. gyro, accl, cmpes, sys data*/
			for(int i = 0; i<4; i++){
				cm904.readSysData();
			}

			gyro = cm904.get_gyro();	//gets 3 gyro axes
			accl = cm904.get_accl();	//gets 3 accelerometer axes
			cmps = cm904.get_cmps();	//gets 3 compas axes
			sys = cm904.get_system();	//gets battery voltage & button map

			CM904DATA::set(CM904DATA::ACCELEROMETER_X, accl.x);
			CM904DATA::set(CM904DATA::ACCELEROMETER_Y, accl.y);
			CM904DATA::set(CM904DATA::ACCELEROMETER_Z, accl.z);

			CM904DATA::set(CM904DATA::GYROSCOPE_X, gyro.x);
			CM904DATA::set(CM904DATA::GYROSCOPE_Y, gyro.y);
			CM904DATA::set(CM904DATA::GYROSCOPE_Z, gyro.z);

			CM904DATA::set(CM904DATA::COMPASS_X, cmps.x);
			CM904DATA::set(CM904DATA::COMPASS_Y, cmps.y);
			CM904DATA::set(CM904DATA::COMPASS_Z, cmps.z);

			CM904DATA::set(CM904DATA::BATTERY_VOLTAGE, sys.batt_voltage);
			CM904DATA::set(CM904DATA::BUTTON_MAP, sys.button_map);

			if(sys.button_map){
				if(sys.button_map & 1){
					printf("\rGot Button 1!!!\n\n");
				}
				if(sys.button_map & 2){
					printf("\rGot Button 2!!!\n\n");
				}
				if(sys.button_map & 4){
					printf("\rGot Button 3!!!\n\n");
				}
				if(sys.button_map & 8){
					printf("\rGot Button 4!!!\n\n");
				}
//				usleep(250000);
			}
			last_time = getUnixTime();

		}
		pthread_testcancel();
	}

}

//}

