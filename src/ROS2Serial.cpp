#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h> 
#include <string.h>
#include <Tracking_with_Quuppa/AZTag.h>
#include <vector>
#include <ros/time.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>

int open_port();
void dputchar(char c,int fd);
void sendCommands(const int& controlX, const int& controlY, int fd);

// Opens serial port. Currently ttyS2 which is the db9 connector
int open_port(void) {
	int fd; // file description for the serial port
	fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
	struct termios options;
	tcgetattr(fd,&options);
	cfsetispeed(&options, B115200);
	cfsetospeed(&options, B115200);
	tcsetattr(fd,TCSANOW, &options);
	if(fd == -1) {
		ROS_ERROR("Port could not be opened");
	}
	else {
		fcntl(fd, F_SETFL, 0);
		ROS_INFO("port is open. \n");
	}
	return (fd);
}

// sends a character to that port
void dputchar(char c,int fd) {
	unsigned char send_byte[] = {c};
	ssize_t k = write(fd,send_byte,1);
	if(k == -1) printf("ERROR SENDING\n");
	usleep(2000);
}

// send the full list of commands to the servo board with a 2ms delay between chars
void sendCommands(const int& controlX, const int& controlY, int fd) {
	// Temp value for pulling characters out of command
	int intTemp;
	static char digits[10] = {'0','1','2','3','4','5','6','7','8','9'};
	static char esc = (unsigned char) 27;
	// Send esc
	dputchar(esc,fd);
	// Sending roll info
	dputchar('p',fd);
	intTemp = controlX / 100 % 10;
	dputchar(digits[intTemp],fd);
	intTemp = controlX / 10 % 10;
	dputchar(digits[intTemp],fd);
	intTemp = controlX % 10;
	dputchar(digits[intTemp],fd);
	// Sending pitch info
	dputchar('r', fd);
	intTemp = controlY / 100 % 10;
	dputchar(digits[intTemp],fd);
	intTemp = controlY / 10 % 10;
	dputchar(digits[intTemp],fd);
	intTemp = controlY % 10;
	dputchar(digits[intTemp],fd);
}
