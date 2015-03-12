#include "ReadIMU.h"
#include "IMUData.h"



Gyro gyro;
Accl accl;
Cmps cmps;
int number = 0;
int fd;

 ReadIMU:: ReadIMU(){

	 	 fd = init_serial_input(USB_SERIAL_PORT);

		std::cout<<"\nTo run program properly:"<<std::endl;
		std::cout<<"-Make sure Arduino is on ttyACM1"<<std::endl;
		std::cout<<"-Make sure you init_serial_input(USB_SERIAL_PORT); in the start of main()"<<std::endl;
		std::cout<<"-If it is not, unplug USB and plug it into the other port"<<std::endl;
		std::cout<<"-Do $ sudo chmod 777 /dev/ttyACM*\n"<<std::endl;
		std::cout<<"new: try $ stty -F /dev/ttyACM<*> cs8 9600 ignbrk -brkint -icrnl -imaxbel -opost -onlcr -isig -icanon -iexten -echo -echoe -echok -echoctl -echoke noflsh -ixon -crtscts"<<std::endl;
		std::cout<<"-It is important that the size of the packet sent from the arduino be exacly the size of its primitive data types\n"<<std::endl;

		if (fd == -1)  {
			std::cout << "Couldn't open USB port" << std::endl;
		}


 }

 ReadIMU::~ReadIMU(){

 }

int ReadIMU::init_serial_input (char * port) {
	int fd = 0;
	struct termios options;

	std::cout << "Made Termios" << std::endl;

	fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);

	std::cout << "Opened fd USB port" << std::endl;

	if (fd == -1)
		return fd;
	fcntl(fd, F_SETFL, 0);    // clear all flags on descriptor, enable direct I/O
	tcgetattr(fd, &options);   // read serial port options
	// enable receiver, set 8 bit data, ignore control lines
	options.c_cflag |= (CLOCAL | CREAD | CS8);
	// disable parity generation and 2 stop bits
	options.c_cflag &= ~(PARENB | CSTOPB);
	// set the new port options
	tcsetattr(fd, TCSANOW, &options);
	return fd;
}

int ReadIMU::readLeftHand(){
	char sensor_buf;
	std::vector<char> line;
	int to_number = 0;

	line.resize(20);
	read(fd, &sensor_buf, 1);


	if(sensor_buf=='B'){
		read(fd, &sensor_buf, 1);
		if(sensor_buf=='1'){
			read(fd, &sensor_buf, 1);
			if(sensor_buf=='\n'){
				printf("got the button!\n");
				return 1;
				}
			}
		}
	}

int ReadIMU::readRightHand(){
	char sensor_buf;
	std::vector<char> line;
	int to_number = 0;

	line.resize(20);
	read(fd, &sensor_buf, 1);


	if(sensor_buf=='B'){
		read(fd, &sensor_buf, 1);
		if(sensor_buf=='2'){
			read(fd, &sensor_buf, 1);
			if(sensor_buf=='\n'){
				printf("got the button!\n");
				return 1;
				}
			}
		}
	}


void  ReadIMU::read_serial_xyz () {
	char sensor_buf;
	std::vector<char> line;
	int to_number = 0;

//	FILE *file;
//	getchar();
//	file = fopen(USB_SERIAL_PORT, "w");
//	fprintf(file,"%c",'A');
//	fclose(file);
//	sleep(.05);


	line.resize(20);
	read(fd, &sensor_buf, 1);

	//printf("entering\n");
	if(sensor_buf=='G'){
		//printf("G\n");
		read(fd, &sensor_buf, 1);

		//found the header, now moving on to the 3 axis values
		if(sensor_buf=='\n'){

			//first line is x, second is y, third is z
			for( int q = 0; q < 3; q++){

				//look through a bunch of characters
				for(int i = 0; i<20; i++){
					read(fd, &sensor_buf, 1);

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
							gyro.x = to_number;
							break;
						case Y_AXIS:
							gyro.y = to_number;
							break;
						case Z_AXIS:
							gyro.z = to_number;
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
		read(fd, &sensor_buf, 1);

		//found the header, now moving on to the 3 axis values
		if(sensor_buf=='\n'){

			//first line is x, second is y, third is z
			for( int q = 0; q < 3; q++){

				//look through a bunch of characters
				for(int i = 0; i<20; i++){
					read(fd, &sensor_buf, 1);

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
							accl.x = to_number;
							break;
						case Y_AXIS:
							accl.y = to_number;
							break;
						case Z_AXIS:
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
		read(fd, &sensor_buf, 1);

		//found the header, now moving on to the 3 axis values
		if(sensor_buf=='\n'){

			//first line is x, second is y, third is z
			for( int q = 0; q < 3; q++){

				//look through a bunch of characters
				for(int i = 0; i<20; i++){
					read(fd, &sensor_buf, 1);

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
							cmps.x = to_number;
							break;
						case Y_AXIS:
							cmps.y = to_number;
							break;
						case Z_AXIS:
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
}

void ReadIMU::printValues(){
	std::cout<< gyro.x<<"\t"<<gyro.y<<"\t"<<gyro.z<<"\t"<<accl.x<<"\t"<<accl.y<<"\t"<<accl.z<<"\t"<<cmps.x<<"\t"<<cmps.y<<"\t"<<cmps.z<<"\n";

}

void ReadIMU::close_port() {
	close(fd);
}

Gyro ReadIMU::get_gyro() {
	return (gyro);
}

Accl ReadIMU::get_accl() {
	return (accl);
}

Cmps ReadIMU::get_cmps() {
	return (cmps);
}

