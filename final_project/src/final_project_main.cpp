#include <algorithm>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <math.h>

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <opencv2/features2d.hpp>

using Eigen::Matrix3f;
using Eigen::MatrixXf;
using Eigen::MatrixXd;
using Eigen::Vector3f;
using Eigen::Vector2f;
using geometry_msgs::Point32;
using geometry_msgs::Point;
using geometry_msgs::Twist;
using sensor_msgs::LaserScan;
using sensor_msgs::PointCloud;
using std::cout;
using std::vector;
using cv::Algorithm;
using cv::Feature2D;
using cv::SimpleBlobDetector;
using namespace std;
using namespace cv;
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

void depthImageToPointCloud(sensor_msgs::Image image, vector<Vector3f> &point_cloud){
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
      Vector3f point;
      // Modify the following lines of code to do the right thing, with the
      // correct parameters.
      point.z() = 1/(a + (b * raw_depth));
      point.x() = ((x - px)/fx)*point.z();
      point.y() = ((y - py)/fy)*point.z();
      point_cloud.push_back(point);
    }
  }
}

void changePointCloudToReferenceFrameOfRobot(vector<Vector3f> point_cloud, vector<Vector3f> &filtered_point_cloud){
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

Vector2f findPerson(vector<Vector3f> image){
  //TODO: use blob detection to find person
  cv::SimpleBlobDetector::Params params_for_human_blob;
  params_for_human_blob.filterByArea = true;
 
  // may need these due to default settings
  params_for_human_blob.filterByInertia = false;
  params_for_human_blob.filterByConvexity = false;
  params_for_human_blob.filterByColor = false;
  params_for_human_blob.filterByCircularity = false;
  
  // may need to change these to find human
  // params_for_human_blob.minArea = 
  // params_for_human_blob.maxArea = 
  Ptr<SimpleBlobDetector> blob_detector = SimpleBlobDetector::create(params_for_human_blob);
  vector<cv::KeyPoint> points_of_human;
  blob_detector->detect(image, points_of_human);
  Vector2f point_to_return(points_of_human[0].pt.x, points_of_human[0].pt.y);
  return point_to_return;
}

void calculatePathToPerson(){
  //TODO: calculate the free path length to the point representing the person

}

void avoidObstacles(){
  //TODO: dynamic window approach to avoid obstacles and return best velocity commands
}

void PersonFollowerCallback(const sensor_msgs::Image& depth_image){
  // change depth image to point cloud
  vector<Vector3f> point_cloud;
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