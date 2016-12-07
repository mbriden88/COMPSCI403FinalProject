#include <algorithm>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

int main(int argc, char **argv) {
	ros::init(argc, argv, "compsci403_final_project");
	ros::NodeHandle n;

	//services n shit

	ros::spin();

	return 0;
}