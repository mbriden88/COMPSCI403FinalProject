#include <algorithm>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

int main(int argc, char **argv) {
	ros::init(argc, argv, "compsci403_final_project");
	ros::NodeHandle n;

	//services n shit

	// deal with input
	ros::ServiceServer pointCloudFromDepthImage = n.advertiseService()
	ros::ServiceServer changeToRobotReference = n.advertiseService()

	// identify obstacles
	ros::ServiceServer identifyObstacles = n.advertiseService()

	// find person to follow
	ros::ServiceServer findPerson = n.advertiseService()

	// determine if person is moving
	ros::ServiceServer checkPersonMotion = n.advertiseService()

	// follow person - main function
	ros:: ServiceServer followPerson = n.advertiseService()



	ros::spin();

	return 0;
}