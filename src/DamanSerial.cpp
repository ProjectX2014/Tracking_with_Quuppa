#include "DamanSerial.h"

bool debugSerial = true;	// sends prints from serial function

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
		printf("ERROR: Port could not be opened. \n");
	}
	else {
		fcntl(fd, F_SETFL, 0);
		printf("port is open. \n");
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
//	if(debugSerial)	printf("\np");
	intTemp = controlX / 100 % 10;
//	if(debugSerial)printf("%c", digits[intTemp]);
	dputchar(digits[intTemp],fd);
	intTemp = controlX / 10 % 10;
//	if(debugSerial)printf("%c", digits[intTemp]);
	dputchar(digits[intTemp],fd);
	intTemp = controlX % 10;
//	if(debugSerial)printf("%c", digits[intTemp]);
	dputchar(digits[intTemp],fd);
	// Sending pitch info
	dputchar('r', fd);
//	if(debugSerial)printf("r");
	intTemp = controlY / 100 % 10;
//	if(debugSerial)printf("%c", digits[intTemp]);
	dputchar(digits[intTemp],fd);
	intTemp = controlY / 10 % 10;
//	if(debugSerial)printf("%c", digits[intTemp]);
	dputchar(digits[intTemp],fd);
	intTemp = controlY % 10;
//	if(debugSerial)printf("%c", digits[intTemp]);
	dputchar(digits[intTemp],fd);
}

