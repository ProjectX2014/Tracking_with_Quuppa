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
#include <string.h>
#include <iostream>
#include <sstream>

const float PI = 3.14159265359;

const int NEUTRAL = 127;
const int MAXX = 312;
const int MAXY = 233;
const int MAXSERVO = 254;
const int FILTERSIZE = 50;

int open_port(); // Open the fd port for serial
void dputchar(char c,int fd); // Send a character to the port
void sendCommands(const int& gimballPitch, const int& gimballYaw, const int& copterYaw, int fd); // Send the full list of commands to PIC
//void readCommands(int& oldGimballPitch, int& oldGimballYaw, bool& i7Control, const int& fd);	// Read last commands and flight state
double lowPass(const double& input, const double& alpha, const bool& ROLL_TRUE);
void servoControl(float, float, float, int&, int&, int&, const bool&); // Find control values based on robot state
void mixSignals(void); // later can be some form of sensor fusion

// RPY variables of the marker from quupa
float trackerRoll  = 0.0;
float trackerPitch = 0.0; 
float trackerYaw   = 0.0;

// Post-filtering xy values
float posX = 0.0;
float posY = 0.0;
float posZ = 0.0;

// ROS message variables
geometry_msgs::Vector3 controlRPY;
geometry_msgs::Vector3 oldRPY;

// Filter time coefficients
double yawTime;
double pitchTime;

// Controller gains (from launch file)
double KpR, KpP, KpY, KdR, KdP, KdY;

// Read the Quuppa position from quuppa node
void quupa_rpy_callback(const geometry_msgs::Vector3& msg) {
	trackerRoll  = msg.z;
	trackerPitch = msg.x;
	trackerYaw   = msg.y;
}

// Old roll and pitch values which will come from serial
int oldGimballYaw = NEUTRAL; int oldGimballPitch = NEUTRAL;

int main(int argc, char** argv) {
	ros::init(argc, argv, "Tracker");
	ros::NodeHandle node;
	ros::Rate loop_rate(100);

	// Look for filter tiems in launch file
	if(node.getParam("/yawTime", yawTime)) {;} else ROS_ERROR("No yaw time horizon.");
	if(node.getParam("/pitchTime", pitchTime)) {;} else ROS_ERROR("No pitch time horizon.");

	// Get controller gains from launch
	if(node.getParam("/rollP", KpR)) {;} else ROS_ERROR("No Roll P Gain.");
	if(node.getParam("/rollD", KdR)) {;} else ROS_ERROR("No Roll D Gain.");
	if(node.getParam("/pitchP", KpP)) {;} else ROS_ERROR("No Pitch P Gain.");
	if(node.getParam("/pitchD", KdP)) {;} else ROS_ERROR("No Pitch D Gain.");
	if(node.getParam("/yawP", KpY)) {;} else ROS_ERROR("No Yaw P Gain.");
	if(node.getParam("/yawD", KdY)) {;} else ROS_ERROR("No Yaw D Gain.");

	ros::Subscriber quupaRPY_sub = node.subscribe("MarkerRPY", 1, quupa_rpy_callback);
	ros::Publisher  control_pub = node.advertise<geometry_msgs::Vector3>("controlVector",1);

	// Set up what will be control outputs to be sent over serial
	int gimballYaw, gimballPitch, copterYaw;
	gimballYaw = NEUTRAL; gimballPitch = NEUTRAL; copterYaw = NEUTRAL;

	// Initialize variables to be read from serial (eventually)
	bool i7Control;

	// Open port for serial communications
	int portNo = open_port();

	// Set up filters based on time coefficients from launch file
	double dt = 1.0/50.0;
	double yawAlpha = dt / (yawTime+dt);
	double pitchAlpha = dt / (pitchTime+dt);
	ROS_INFO("Yaw Alpha = %f", yawAlpha);
	ROS_INFO("Pitch Alpha = %f", pitchAlpha);
	oldRPY.x = 0.0; oldRPY.y = 0.0; oldRPY.z = 0.0;

	// Main loop to run
	while(ros::ok()) {
//		tcflush(portNo, TCIOFLUSH); // clear buffer

/*		// Find the filtered pitch and yaw values
		trackerPitch = pitchAlpha*trackerPitch + (1.0-pitchAlpha)*oldRPY.y;
		trackerYaw   = yawAlpha*trackerYaw   + (1.0-yawAlpha)*oldRPY.z;*/
		trackerPitch = lowPass(trackerPitch, pitchAlpha, false);
		trackerYaw   = lowPass(trackerYaw, yawAlpha, true);

		// Set old RPY values for filtering on next loop
		oldRPY.x = trackerRoll; oldRPY.y = trackerPitch; oldRPY.z = trackerYaw;

		// Determine the three control values based on current tracker position and robot state
//		readCommands(oldGimballPitch, oldGimballYaw, i7Control, portNo);				
		servoControl(trackerPitch, trackerYaw, trackerRoll, gimballYaw, gimballPitch, copterYaw, false);
		// Publish these for monitoring. not used by another node (currently)
		controlRPY.x = gimballYaw; controlRPY.y = gimballPitch; controlRPY.z = copterYaw;
		control_pub.publish(controlRPY);

		// Send command s to the serial port selected in portNo
		//sendCommands(gimballPitch, gimballYaw, copterYaw, portNo);
		sendCommands(gimballYaw, gimballPitch, copterYaw, portNo);

		// Send all the publishers out and wait if necessary
		ros::spinOnce();
		loop_rate.sleep();
	}
}

// Determine control outputs based on tracker position inputs and robot state
void servoControl(float rollIn, float pitchIn, float yawIn, int& controlRoll, int& controlPitch, int& controlYaw, const bool& i7Control) {
	// Set some bound on the marker angles
	static double bound = 1.0*PI/180.0;

	static double oldErrorRoll = 0.0; static double oldErrorPitch = 0.0; static double oldErrorYaw = 0.0;
	
	if(abs(rollIn) < bound) { oldErrorRoll = 0.0;  }
	if(abs(pitchIn) < bound) { oldErrorPitch = 0.0;}

	static int oldControlRoll = NEUTRAL;
	static int oldControlPitch = NEUTRAL;
	static int oldControlYaw = NEUTRAL;
	controlRoll = oldControlRoll + (KpR*rollIn + KdR*(rollIn - oldErrorRoll));
	controlPitch = oldControlPitch + (KpP*pitchIn + KdP*(pitchIn - oldErrorPitch));
	controlYaw = NEUTRAL + (KpY*pitchIn + KdY*(pitchIn - oldErrorPitch));
//	controlYaw = NEUTRAL + KpY*rollIn + KdY*(rollIn - oldErrorYaw);

	ROS_INFO("Yaw = %d", controlYaw);

//	oldErrorRoll = rollIn; oldErrorPitch = pitchIn; oldErrorYaw = rollIn;
	oldErrorRoll = rollIn; oldErrorPitch = pitchIn; oldErrorYaw = pitchIn;


	if(controlRoll > MAXSERVO) {
		controlRoll = MAXSERVO;
		oldErrorRoll = 0.0;		
	}
	if(controlRoll <= 0) {
		controlRoll = 1;
		oldErrorRoll = 0.0;
	}
	if(controlPitch > MAXSERVO) {
		controlPitch = MAXSERVO;
		oldErrorPitch = 0.0;
	}
	if(controlPitch <= 0) {
		controlPitch = 1;
		oldErrorPitch = 0.0;
	}
	if(controlYaw > MAXSERVO) {
		controlYaw = MAXSERVO;
		oldErrorYaw = 0.0;
	}
	if(controlYaw <= 0) {
		controlYaw = 1;
		oldErrorYaw = 0.0;
	}
//	if(i7Control)
//		controlRoll = NEUTRAL;
	oldControlRoll = controlRoll; oldControlPitch = controlPitch; oldControlYaw = controlYaw;
} 
	
// Opens serial port. Currently ttyS0 for default DB9 connector
int open_port(void) {
	int fd; // file description for the serial port
	//fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
	fd = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
//	fd = open("/dev/ttyS0", O_WRONLY | O_NOCTTY | O_NDELAY | O_NONBLOCK);
	struct termios options;
	tcgetattr(fd,&options);
	cfsetispeed(&options, B115200);
	cfsetospeed(&options, B115200);
	tcsetattr(fd,TCSANOW, &options);
	if(fd == -1) {
		ROS_ERROR("Port could not be opened");
		close(fd);
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
void sendCommands(const int& gimballPitch, const int& gimballYaw, const int& copterYaw, int fd) {
	// Temp value for pulling characters out of command
	int intTemp;
	static char digits[10] = {'0','1','2','3','4','5','6','7','8','9'};
	static char esc = (unsigned char) 27;
	// Send esc
	dputchar(esc,fd);
	// Sending pitch info
	dputchar('p',fd);
	intTemp = gimballPitch / 100 % 10;
	dputchar(digits[intTemp],fd);
	intTemp = gimballPitch / 10 % 10;
	dputchar(digits[intTemp],fd);
	intTemp = gimballPitch % 10;
	dputchar(digits[intTemp],fd);
	// Sending roll info
	dputchar('r', fd);
	intTemp = gimballYaw / 100 % 10;
	dputchar(digits[intTemp],fd);
	intTemp = gimballYaw / 10 % 10;
	dputchar(digits[intTemp],fd);
	intTemp = gimballYaw % 10;
	dputchar(digits[intTemp],fd);
	// Sending yaw info
	dputchar('y',fd);
	intTemp = copterYaw / 100 % 10;
	dputchar(digits[intTemp],fd);
	intTemp = copterYaw / 10% 10;
	dputchar(digits[intTemp],fd);
	intTemp = copterYaw % 10;
	dputchar(digits[intTemp],fd);
}

double lowPass(const double& input, const double& alpha, const bool& ROLL_TRUE) {
	static std::vector<double> rollInputVec(FILTERSIZE, 0.0);
	static std::vector<double> pitchInputVec(FILTERSIZE,0.0);
	static std::vector<double> output(FILTERSIZE,0.0);
	if(ROLL_TRUE) {
		rollInputVec.erase(rollInputVec.begin());
		rollInputVec.push_back(input);
		output[0] = rollInputVec[0];
		for(int i=1; i < rollInputVec.size(); i++) {
			output[i] = alpha*rollInputVec[i] + (1.0-alpha)*output[i-1];
		}
	}
	else {
		pitchInputVec.erase(pitchInputVec.begin());
		pitchInputVec.push_back(input);
		output[0] = pitchInputVec[0];
		for(int i=1; i < pitchInputVec.size(); i++) {
			output[i] = alpha*pitchInputVec[i] + (1.0 - alpha)*output[i-1];
		}
	}
	return output.back();
}


/*
// if c==0 then i7 controls yaw, i7Control = true
// else if c == 1 then RC controls yaw, i7Control = false
void readCommands(int& oldGimballYaw, int& oldGimballPitch, bool& i7Control, const int& fd) {

	char buf[8];
	int n = read(fd, buf, sizeof buf);

	if(buf[6] == 13 && buf[7] == 10) {
		unsigned int yaw;
		std::stringstream ssYaw;
		ssYaw << std::hex << buf[0] << buf[1];
		ssYaw >> yaw;

		unsigned int pitch;
		std::stringstream ssPitch;
		ssPitch << std::hex << buf[2] << buf[3];
		ssPitch >> pitch;

		unsigned int control;
		std::stringstream ssI;
		ssI << std::hex << buf[4] << buf[5];
		ssI >> control;

		oldGimballYaw = yaw;
		oldGimballPitch = pitch;
		if(control == 0)
			i7Control = true;
		else
			i7Control = false;
	}
//	else
//		printf("Failed test\n");
//	printf("oldGimballYaw = %d, oldGimballPitch = %d, i7Control = %d\n", oldGimballYaw, oldGimballPitch, i7Control);
}
*/
