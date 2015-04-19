
#include "CM904.h"
//#include "CM904Data.h"


//TODO for some reason, comm only works when using ttyACM1, not ttyACM0
#define SYS_DATA_CODE	0x5A
#define SYS_CMD_CODE	0x64
#define CM904_SERIAL_PORT	"/dev/ttyACM1"

Gyro gyro;
Accl accl;
Cmps cmps;
System sys;

int port_fd;
int global_count;
char *global_port_name;

char packet[5];

std::string currentLine= " ";

CM904::CM904 () {

	FILE *cm904_port;

	//std::cout<<"new: try $ stty -F /dev/ttyACM<*> cs8 9600 ignbrk -brkint -icrnl -imaxbel -opost -onlcr -isig -icanon -iexten -echo -echoe -echok -echoctl -echoke noflsh -ixon -crtscts"<<std::endl;

//	if (cm904.init_serial_input(CM904_SERIAL_PORT) == -1)  {
	if (init_serial_input(CM904_SERIAL_PORT) == -1) {
		std::cout << "Couldn't open USB port" << std::endl;
	}
}
CM904::~CM904 () {
}

int CM904::init_serial_input(char *port_id){
	struct termios options;
	global_port_name = port_id;
	std::cout << "Made Termios" << std::endl;

	port_fd = open(port_id, O_RDWR | O_NOCTTY | O_NDELAY);

	//std::cout << "Opened fd USB port" << std::endl;

	if (port_fd == -1){
		return port_fd;
	}	
	fcntl(port_fd, F_SETFL, 0);    // clear all flags on descriptor, enable direct I/O
	
	tcgetattr(port_fd, &options);   // read serial port options
	// enable receiver, set 8 bit data, ignore control lines
	options.c_cflag |= (CLOCAL | CREAD | CS8);
	// disable parity generation and 2 stop bits
	options.c_cflag &= ~(PARENB | CSTOPB);
	// set the new port options
	tcsetattr(port_fd, TCSANOW, &options);


	return port_fd;
}


void CM904::readSerial(){

	char rx_packet[128];
	std::vector<char> line;
	int err = 0;

	line.resize(20);
	err = read(port_fd, &rx_packet,57);
	if(err>0){
		printf("%s\n",rx_packet);

	}


}

void CM904::writeToCM904(char msg[3]){

	write(port_fd, msg, 3);

}

/*This function is called by a main program or another program in order to
 * read from the cm904*/
void CM904::readSysData(){
	char sensor_buf;
	std::vector<char> line;
	int to_number = 0;

		line.resize(20);

		read(port_fd, &sensor_buf, 1);

		//printf("entering\n");
		if(sensor_buf=='G'){
			//printf("G\n");
			read(port_fd, &sensor_buf, 1);

			//found the header, now moving on to the 3 axis values
			if(sensor_buf=='\n'){

				//first line is x, second is y, third is z
				for( int q = 0; q < 3; q++){

					//look through a bunch of characters
					for(int i = 0; i<20; i++){
						read(port_fd, &sensor_buf, 1);

						//newline means that this axis number is over
						if(sensor_buf=='\n'){
							i=20;	//this will cause the for loop to exit
							to_number = 0;
							line.resize(i+1);
							std::string str_holder(line.begin(), line.end());//convert the character array to a string
							std::stringstream(str_holder)>>to_number; //cast the string to a number
							//							printf("%c", line[j]);
							//printf("%i", to_number);

							switch(q){
							case X_AXIS:
//								CM904DATA::set(CM904DATA::GYROSCOPE_X, to_number);
								gyro.x = to_number;
								break;
							case Y_AXIS:
//								CM904DATA::set(CM904DATA::GYROSCOPE_Y, to_number);
								gyro.y = to_number;
								break;
							case Z_AXIS:
								gyro.z = to_number;
//								CM904DATA::set(CM904DATA::GYROSCOPE_Z, to_number);
								break;
							}
							//printf("\n");
						}
						//if it wasn't a newline, then keep adding to the char array
						else{
							line[i]= sensor_buf;
						}
					}
				}
			}
		}

		else if(sensor_buf=='A'){
			//printf("A\n");
			read(port_fd, &sensor_buf, 1);

			//found the header, now moving on to the 3 axis values
			if(sensor_buf=='\n'){

				//first line is x, second is y, third is z
				for( int q = 0; q < 3; q++){

					//look through a bunch of characters
					for(int i = 0; i<20; i++){
						read(port_fd, &sensor_buf, 1);

						//newline means that this axis number is over
						if(sensor_buf=='\n'){
							i=20;	//this will cause the for loop to exit
							to_number = 0;
							line.resize(i+1);
							std::string str_holder(line.begin(), line.end());
							std::stringstream(str_holder)>>to_number;
							//printf("%c", line[j]);
							//printf("%i", to_number);

							switch(q){
							case X_AXIS:
//								CM904DATA::set(CM904DATA::ACCELEROMETER_X, to_number);
								accl.x = to_number;
								break;
							case Y_AXIS:
//								CM904DATA::set(CM904DATA::ACCELEROMETER_Y, to_number);
								accl.y = to_number;
								break;
							case Z_AXIS:
//								CM904DATA::set(CM904DATA::ACCELEROMETER_Z, to_number);
								accl.z = to_number;
								break;
							}
							//printf("\n");
						}
						//if it wasn't a newline, then keep adding to the char array
						else{
							line[i]= sensor_buf;
						}
					}
				}
			}
		}
		else if(sensor_buf=='C'){
			//printf("C\n");
			read(port_fd, &sensor_buf, 1);

			//found the header, now moving on to the 3 axis values
			if(sensor_buf=='\n'){

				//first line is x, second is y, third is z
				for( int q = 0; q < 3; q++){

					//look through a bunch of characters
					for(int i = 0; i<20; i++){
						read(port_fd, &sensor_buf, 1);

						//newline means that this axis number is over
						if(sensor_buf=='\n'){
							i=20;	//this will cause the for loop to exit
							to_number = 0;
							line.resize(i+1);
							std::string str_holder(line.begin(), line.end());
							std::stringstream(str_holder)>>to_number;
							//printf("%c", line[j]);
							//printf("%i", to_number);

							switch(q){
							case X_AXIS:
//								CM904DATA::set(CM904DATA::COMPASS_X, to_number);
								cmps.x = to_number;
								break;
							case Y_AXIS:
//								CM904DATA::set(CM904DATA::COMPASS_Y, to_number);
								cmps.y = to_number;
								break;
							case Z_AXIS:
//								CM904DATA::set(CM904DATA::COMPASS_Z, to_number);
								cmps.z = to_number;
								break;
							}
							//printf("\n");
						}
						//if it wasn't a newline, then keep adding to the char array
						else{
							line[i]= sensor_buf;
						}
					}
				}
			}
		}

		else if(sensor_buf=='D'){
			//printf("C\n");
			read(port_fd, &sensor_buf, 1);

			//found the header, now moving on to the 3 axis values
			if(sensor_buf=='\n'){

				//first line is x, second is y, third is z
				for( int q = 0; q < 2; q++){

					//look through a bunch of characters
					for(int i = 0; i<20; i++){
						read(port_fd, &sensor_buf, 1);

						//newline means that this axis number is over
						if(sensor_buf=='\n'){
							i=20;	//this will cause the for loop to exit
							to_number = 0;
							line.resize(i+1);
							std::string str_holder(line.begin(), line.end());
							std::stringstream(str_holder)>>to_number;
							//printf("%c", line[j]);
							//printf("%i", to_number);

							switch(q){
							case 0:
								//							printf("%i\n", to_number);
//								CM904DATA::set(CM904DATA::BATTERY_VOLTAGE, to_number);
								sys.button_map= to_number;
								break;
							case 1:
								//							printf("%i\n", to_number);
//								CM904DATA::set(CM904DATA::BUTTON_MAP, to_number);
								sys.batt_voltage= to_number;
								break;
							case 2:
								//							printf("%i\n\n", to_number);
								break;

							}
							//printf("\n");
						}
						//if it wasn't a newline, then keep adding to the char array
						else{
							line[i]= sensor_buf;
						}
					}
				}
			}
		}

}

void CM904::sysCmd( bool motor_power, bool LED_power, int LED_speed){
//	FILE *cm904_port;
//
//	packet[0] =0x5A;
//	packet[1]= motor_power;
//	packet[2]= LED_power;
//	packet[3] =  LED_speed & 0xFF;
//	packet[4] = (LED_speed & 0xFF00) >> 8;
//
//	cm904_port = fopen(global_port_name, "w");
//	fprintf(cm904_port,"%s",packet);
//	fclose(cm904_port);

}
void CM904::printValues(){
	std::cout<< gyro.x<<"\t"<<gyro.y<<"\t"<<gyro.z<<"\t"<<accl.x<<"\t"<<accl.y<<"\t"<<accl.z<<"\t"<<cmps.x<<"\t"<<cmps.y<<"\t"<<cmps.z<<"\n"<<sys.batt_voltage<<"\n"<<sys.button_map<<"\n";

}
/*This set of functions allows a main loop or other program to access the data
 * that we read from the CM904*/
void CM904::close_port() {
	close(port_fd);
}

Gyro CM904::get_gyro() {
	return (gyro);
}

Accl CM904::get_accl() {
	return (accl);
}

Cmps CM904::get_cmps() {
	return (cmps);
}
System CM904::get_system() {
	return (sys);
}

