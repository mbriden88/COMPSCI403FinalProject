#include <algorithm>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

// Kinect Intrinsics
float a = 3.008;
float b = -0.002745;
float px = 320;
float py = 240;
float fx = 588.446;
float fy = -564.227;

void depthImageToPointCloud(sensor_msgs::Image image, sensor_msgs::PointCloud &point_cloud){
    point_cloud.header = image.header;
    for (unsigned int y = 0; y < image.height; ++y) {
        for (unsigned int x = 0; x < image.width; ++x) {
      		uint16_t byte0 = image.data[2 * (x + y * image.width) + 0];
      		uint16_t byte1 = image.data[2 * (x + y * image.width) + 1];
      		if (!image.is_bigendian) {
        		std::swap(byte0, byte1);
      		}
      		// Combine the two bytes to form a 16 bit value, and disregard the
      		// most significant 4 bits to extract the lowest 12 bits.
      		const uint16_t raw_depth = ((byte0 << 8) | byte1) & 0x7FF;
      		// Reconstruct 3D point from x, y, raw_depth.
      		geometry_msgs::Point32 point;
      		// Modify the following lines of code to do the right thing, with the
      		// correct parameters.
      		point.z = 1/(a + (b * raw_depth));
      		point.x = ((x - px)/fx)*point.z;
      		point.y = ((y - py)/fy)*point.z;
      		point_cloud.points.push_back(point);
    	}
    }
}

void changePointCloudToReferenceFrameOfRobt(){

}

void findPerson(){

}

void calculatePathToPerson(){

}

void avoidObstacles(){

}

void PersonFollowerCallback(const sensor_msgs::Image& depth_image){
	sensor_msgs::PointCloud point_cloud;
	depthImageToPointCloud(depth_image, point_cloud);
	


}

int main(int argc, char **argv) {
	ros::init(argc, argv, "compsci403_final_project");
	ros::NodeHandle n;

	ros::Subscriber person_follower_subscriber =
      n.subscribe("/Cobot/Kinect/Depth", 1, PersonFollowerCallback);

	ros::spin();

	return 0;
}