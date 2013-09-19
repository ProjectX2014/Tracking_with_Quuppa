#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h> 
#include <string.h>
#include <Tracking_with_Quuppa/AZTag.h>
#include <vector>
#include <ros/time.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3.h>

const float PI = 3.14159265359;

// RPY variables of the marker
float trackerRoll  = 0.0;
float trackerPitch = 0.0; 
float trackerYaw   = 0.0;

// RPY for publishing
geometry_msgs::Vector3 rpyPub;

// Marker azimuth and zenith
float trackerAzm;
float trackerZen;

const float zenBound = 2.0*PI/180.0;

void marker_callback(const Tracking_with_Quuppa::AZTag& msg) {
	trackerAzm = msg.azm[0]*PI/180.0;
	trackerZen = msg.zen[0]*PI/180.0;
	if (trackerZen < zenBound) {
		trackerZen = 0.0;
		trackerAzm = 0.0;
	}
}
	
int main(int argc, char** argv) {
	ros::init(argc, argv, "QuuppaReader");
	ros::NodeHandle node;
	ros::Rate loop_rate(50);

	ros::Subscriber marker_sub;
	marker_sub = node.subscribe("Quuppa_AZ", 1, marker_callback);
	
	ros::Publisher markerRPY_pub;	
	markerRPY_pub  = node.advertise<geometry_msgs::Vector3>("MarkerRPY", 1);

	while(ros::ok()) {
		trackerPitch = tan(trackerZen)*cos(trackerAzm); 
		trackerRoll  = tan(trackerZen)*sin(trackerAzm); 
		ROS_INFO("R: %f, P: %f, Y: %f", trackerRoll, trackerPitch, trackerYaw);
		rpyPub.x = trackerRoll;
		rpyPub.y = trackerPitch;
		rpyPub.z = trackerYaw;
		markerRPY_pub.publish(rpyPub);

		ros::spinOnce();
		loop_rate.sleep();
	}
}
