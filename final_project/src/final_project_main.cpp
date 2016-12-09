#include <algorithm>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

//Robot Attributes
float robotRadius = 0.18;
float robotHeight = 0.36;
float epsilon = 0.15;

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

void changePointCloudToReferenceFrameOfRobot(sensor_msgs::PointCloud point_cloud, vector<Vector3f> &filtered_point_cloud){
	//TODO: potentially need to get R and T from a service
	Matrix3f R = MatrixXf::Identity(3,3);
  Vector3f T(0.13, 0, 0.305);
	for (size_t i = 0; i < point_cloud.size(); ++i) {
    Vector3f point = R * point_cloud[i] + T;
    if(point(2) < robotHeight && point(2) > epsilon){
      filtered_point_cloud.push_back(point);
    }
  }
}

void findPerson(){

}

void calculatePathToPerson(){

}

void avoidObstacles(){

}

void PersonFollowerCallback(const sensor_msgs::Image& depth_image){
	// change depth image to point cloud
	sensor_msgs::PointCloud point_cloud;
	depthImageToPointCloud(depth_image, point_cloud);
	// change point cloud to reference frame of robot
  vector<Vector3f> filtered_point_cloud;
  changePointCloudToReferenceFrameOfRobot(point_cloud, filtered_point_cloud);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "compsci403_final_project");
	ros::NodeHandle n;

	ros::Subscriber person_follower_subscriber =
    n.subscribe("/Cobot/Kinect/Depth", 1, PersonFollowerCallback);

	ros::spin();

	return 0;
}