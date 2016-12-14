//  Some code taken from Joydeep Biswas solution code for assignment 5, CS403.
//  These functions are marked as such.

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
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

// #include <pcl/point_types.h>
// #include <pcl_ros/point_cloud.h>
// #include "cob_3d_mapping_common/point_types.h"
// #include "cob_3d_segmentation/impl/fast_segmentation.hpp"
// #include "cob_3d_features/organized_normal_estimation_omp.h"

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
using nav_msgs::Odometry;
// using cv::Algorithm;
// using cv::Feature2D;
// using cv::SimpleBlobDetector;
using namespace std;
// using namespace cv;

// person-cluster stuff
// typedef cob_3d_segmentation::FastSegmentation<
//   pcl::PointXYZRGB,
//   pcl::Normal,
//   PointLabel> Segmentation3d;
// typedef cob_3d_features::OrganizedNormalEstimationOMP<
//   pcl::PointXYZRGB,
//   pcl::Normal,
//   PointLabel> NormalEstimation;

// Vector3f cloud_cb(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& input) {
//     pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
//   pcl::PointCloud<PointLabel>::Ptr labels(new pcl::PointCloud<PointLabel>);

//   // Segmentation requires estimated normals for every 3d point
//   NormalEstimation one;
//   // labels are first used to mark NaN and border points as a
//   // preparation for the segmantation
//   one.setOutputLabels(labels);
//   // sets the pixelwindow size, radial step step_size and window step size
//   one.setPixelSearchRadius(8,2,2);
//   // sets the threshold for border point determination
//   one.setSkipDistantPointThreshold(8);
//   one.setInputCloud(input);
//   one.compute(*normals);

//   Segmentation3d seg;
//   seg.setNormalCloudIn(normals);
//   // labels are now assigned according to the cluster a point belongs to
//   seg.setLabelCloudInOut(labels);
//   // defines the method how seed points are initialized (SEED_RANDOM | SEED_LINEAR)
//   seg.setSeedMethod(cob_3d_segmentation::SEED_RANDOM);
//   seg.setInputCloud(input);
//   seg.compute();

//   Segmentation3d::ClusterHdlPtr cluster_handler = seg.clusters();
//   // use .begin() and .end() to iterate over all computed clusters
  
//   int size = 10000;
//   Vector3f center;
//   Segmentation3d::ClusterPtr c = cluster_handler->begin();
//   for(; c != cluster_handler->end(); ++c) {
//     if (c->size() < size) {
//       size = c->size();
//       center = c->getCentroid();
//     }
//   }

//   return center;
// }


//Robot Attributes
float robotRadius = 0.18;
float robotHeight = 0.36;
float epsilon = 0.15;
float AmaxLin = 1 * 0.5; // m/s^2
float AmaxRot = 1 * 2.0; // rad/s^2
float VMaxLin = 0.5; // m/s
float VMaxRot = 1.5; // rad/s
float DelT = 0.02; // s

// reduce likelyhood of getting stray points from obstacles
float robotHeightBonus = 0.05;

// Kinect Intrinsics
float a = 3.008;
float b = -0.002745;
float px = 320;
float py = 240;
float fx = 588.446;
float fy = -564.227;

// Laser Scan Parameters
float minAngle = -28.0 * M_PI/ 180;
float maxAngle = 28.0 * M_PI/ 180;
float stepSize = 1.0 * M_PI/ 180;
float maxRange = 4.0;
float minRange = 0.8;
int laserScanLength = 57;

// optimization function coeffecients
float v_coeff = 1;
float w_coeff = -0.003; // -5
float free_path_coeff = 1; //10
float position_coeff = 1;

Odometry last_odometry;
// last known velocities
// Vector2f lastVel(0.0, 0.0);

ros::Publisher velocity_command_publisher_;

// last known person position
Vector2f lastPersonPosition(0.0, 0.0);

// ideal distance to maintain from human in meters
float idealDistance = 1.0;

//Step 1
void depthImageToPointCloud(sensor_msgs::Image image, sensor_msgs::PointCloud &point_cloud){
  point_cloud.header = image.header;
  for (unsigned int y = 0; y < image.height; ++y) {
    for (unsigned int x = 0; x < image.width; ++x) {
      uint16_t byte0 = image.data[2 * (x + y * image.width) + 0];
      uint16_t byte1 = image.data[2 * (x + y * image.width) + 1];
      if (!image.is_bigendian) {
        std::swap(byte0, byte1);
      }
      
      const uint16_t raw_depth = ((byte0 << 8) | byte1) & 0x7FF;
      
      geometry_msgs::Point32 point;
      
      point.z = 1/(a + (b * raw_depth));
      point.x = ((x - px)/fx)*point.z;
      point.y = ((y - py)/fy)*point.z;
      point_cloud.points.push_back(point);
    }
  }
}

//Step 2
void changePointCloudToReferenceFrameOfRobot(sensor_msgs::PointCloud point_cloud, vector<Vector3f> &filtered_point_cloud){

  //TODO: potentially need to get R and T from a service
  Matrix3f R = MatrixXf::Identity(3,3);
  Vector3f T(0.13, 0, 0.305);
  for (size_t i = 0; i < point_cloud.points.size(); ++i) {
    Vector3f point_cloud_point(point_cloud.points[i].x, point_cloud.points[i].y, point_cloud.points[i].z);
    Vector3f point = R * point_cloud_point + T;
    if(point(2) > epsilon){
      filtered_point_cloud.push_back(point);
    }
  }
}

//Step 3
void InitializeDynamicWindow(MatrixXf& win_v, MatrixXf& win_w, const Vector2f& V0){
  int size = win_v.rows();
  int center_ind = (size - 1)/2;
  float v_step = AmaxLin * DelT / ((size - 1)/2);
  float w_step = AmaxRot * DelT / ((size - 1)/2);
  float v_bias = 0.1;


  for(int i = 0; i < size; ++i){
    for(int j = 0; j < size; ++j){
      float v = (i - center_ind) * v_step + V0(0);
      float w = (j - center_ind) * w_step + V0(1);
      v = (v > VMaxLin)? VMaxLin: v;
      v = (v < v_bias)? v_bias: v;
      w = (w > VMaxRot)? VMaxRot: w;
      win_v(i,j) = v;
      win_w(i,j) = w;
    }
  }

}

vector<Vector3f> updateImageOnlyPointsAboveHeight(vector<Vector3f> original_image){
  vector<Vector3f> image;
  for (size_t i = 0; i < original_image.size(); ++i) {
    Vector3f point_tmp = original_image[i];
    if(point_tmp(2) > robotHeight + robotHeightBonus){
      image.push_back(point_tmp);
    }
  }
  return image;
}


//Step 5
void PointCloudToLaserScanHelper(const vector<Vector3f>& point_cloud, vector<float>& ranges){
  float min_angle = -28.0 * M_PI/ 180;
  // float max_angle = 28.0 * M_PI/ 180;
  float step_size = 1.0 * M_PI/ 180;
  float max_range = 4.0;
  float min_range = 0.8;
  size_t laser_length = 28 - (-28) + 1;


  // Calculate the angle of all the points in the point cloud in the polar coordinate system (projected to the ground plane)
  vector<float> angles(point_cloud.size());
  for(size_t i = 0; i < point_cloud.size(); ++i){
    angles[i] = atan2(point_cloud[i](1), point_cloud[i](0));
  }

  float ang = min_angle;
  for(size_t i = 0; i < laser_length; ++i){
    // Define the query slice for current laser beam
    float start_range = ang - step_size/2;
    float end_range = ang + step_size/2;

    // The minimum distance reading in the current slice
    float min_dist = max_range;

    // Fill the laser scan with exhastive search. A hash table could be used for faster results.
    for (size_t j = 0; j < point_cloud.size(); ++j){
      if(angles[j] >= start_range && angles[j] < end_range){
        float dist = sqrt(point_cloud[j](0) * point_cloud[j](0) + point_cloud[j](1) * point_cloud[j](1));
        if(dist < max_range && dist > min_range && dist < min_dist){
          min_dist = dist;
        }
      }
    }
    ranges.push_back(min_dist);
    ang += step_size;

  }
}

// Step 6
void LaserScanToPoint(const vector<float>& ranges, vector<Vector2f>& points){
  for(size_t i =0; i < ranges.size(); ++i){
    float angle = minAngle + i * stepSize;
    Vector2f point(cos(angle) * ranges[i], sin(angle) * ranges[i]);
    points.push_back(point);
  }
}

Vector2f estimatePositionChange(const Vector2f& vel) {
  Vector2f estimatedPosition;
  float R = vel(0) / vel(1);
  float angleChange = vel(1) * DelT;
  float magnitude = 2 * R * sin(angleChange / 2);
  estimatedPosition(0) = cos(angleChange) * magnitude;
  estimatedPosition(0) = sin(angleChange) * magnitude;
  return estimatedPosition;
}

//Step 4
Vector2f findPerson(vector<Vector3f> orignial_image){  

  Vector2f person_position;

  vector<Vector3f> cluster1;
  vector<Vector3f> cluster2;
  vector<Vector3f> cluster3;

  Vector3f center1;
  Vector3f center2;
  Vector3f center3;

  vector<Vector3f> centers;
  centers.push_back(center1);
  centers.push_back(center2);
  centers.push_back(center3);

  vector<Vector3f> newCenters;
  Vector3f newCenter1;
  Vector3f newCenter2;
  Vector3f newCenter3;
  newCenters.push_back(newCenter1);
  newCenters.push_back(newCenter2);
  newCenters.push_back(newCenter3);

  // generate initial random centers
  for (size_t i = 0; i < centers.size(); i++) {
    centers[i].x() = rand() % 5;
    centers[i].y() = rand() % 9 + (-4);
    centers[i].z() = rand() % 7;
  }

  while (true) {
    // check each point
    for (size_t i = 0; i < orignial_image.size(); i++) {
      int bestCenter;
      float minDistance = 10000;
      float currentDistance;
      // find the closest center
      for (size_t j = 0; j < centers.size(); j++) {
        Vector3f pointToCenter = orignial_image[i] - centers[j];
        currentDistance = sqrt(pointToCenter(0) * pointToCenter(0) + pointToCenter(1) * pointToCenter(1) + pointToCenter(2) * pointToCenter(2));
        if (currentDistance < minDistance) {
          minDistance = currentDistance;
          bestCenter = j;
        }
      }
      // add the point to the best cluster
      switch(bestCenter) {
        case 0 : cluster1.push_back(orignial_image[i]);
          break;
        case 1 : cluster2.push_back(orignial_image[i]);
          break;
        case 2 : cluster3.push_back(orignial_image[i]);
          break;
      }
    }

    // zero out the new centers
    for (Vector3f cent : newCenters) {
      cent.x() = 0;
      cent.y() = 0;
      cent.z() = 0;
    }

    // find new centers
    for (Vector3f point : cluster1) {
      newCenter1 += point;
    }
    newCenter1 /= cluster1.size();

    for (Vector3f point : cluster2) {
      newCenter2 += point;
    }
    newCenter2 /= cluster2.size();

    for (Vector3f point : cluster3) {
      newCenter3 += point;
    }
    newCenter3 /= cluster3.size();

    // if nothing has changed, exit loop
    if (centers[0] == newCenters[0] && centers[1] == newCenters[1] && centers[2] == newCenters[2]) {
      break;
    }

    // otherwise update centers
    for (size_t i = 0; i < centers.size(); i++) {
      centers[i] = newCenters[i];
    }
  }


  // vector<Vector3f> cutImage = updateImageOnlyPointsAboveHeight(orignial_image);
  // vector<float>ranges;
  // PointCloudToLaserScanHelper(cutImage, ranges);
  // float angle = 0;
  // int lasersUsed = 0;
  // float avRange = 0;
  // for (size_t i = 0; i < ranges.size(); i++) {
  //   if (ranges[i] < maxRange) {
  //     angle += minAngle + (stepSize * i);
  //     avRange += ranges[i];
  //     lasersUsed++;
  //   }
  // }
  // if (lasersUsed == 0) {
  //   Vector2f estimatedPersonPositionChange;
  //   float v0 = last_odometry.twist.twist.linear.x;
  //   float w0 = last_odometry.twist.twist.angular.z;
  //   Vector2f vel(v0, w0);
  //   estimatedPersonPositionChange = estimatePositionChange(vel);
  //   return lastPersonPosition - estimatedPersonPositionChange;
  // }
  // angle /= lasersUsed;
  // avRange /= lasersUsed;
  // Vector2f point_to_return;
  // point_to_return(0) = cos(angle) * avRange;
  // point_to_return(1) = sin(angle) * avRange;
  // ROS_INFO("point of human x: %f y: %f", point_to_return.x(), point_to_return.y());

  // return point_to_return; 
  return person_position;
}

// Step 9
bool FreePathLength(float& free_path_length, const Vector2f& point, const Vector2f& vel) {
  int is_obstacle = false;
  free_path_length = maxRange;

  if(vel(1) == 0){
    free_path_length = point(0) - robotRadius;
    is_obstacle = true;
    return is_obstacle;
  }
  // radius of rotation
  float R = fabs(vel(0)/vel(1)); 
  Vector2f center(0, 0);
  center(1) = (vel(1) > 0)? R: -R;
  Vector2f center_to_obstacle(point(0) - center(0), point(1) - center(1));
  float dist_to_center = sqrt(center_to_obstacle(0) * center_to_obstacle(0) + center_to_obstacle(1) * center_to_obstacle(1));

  
  if(dist_to_center <= (R + robotRadius) && dist_to_center >= (R - robotRadius)){
    is_obstacle = true;
    float obstacle_angle = atan2(center_to_obstacle(1), center_to_obstacle(0));
    free_path_length = (obstacle_angle < 0)?( M_PI/2 + obstacle_angle) * R: ( M_PI/2 - obstacle_angle) * R;
    float delta = fabs(dist_to_center - R);
    float correction = sqrt(robotRadius * robotRadius - delta * delta);
    free_path_length -= correction;
  }

  return is_obstacle;
}

// Step 7
// bool freePathToGoalState(const Vector2f vel, const Vector2f person_position, const float angle, Vector2f &goalState) {

//   float personMag = sqrt((person_position(0) * person_position(0)) + (person_position(1) * person_position(1)));
//   float goalStateMag = personMag - idealDistance;
//   goalState = Vector2f(cos(angle) * goalStateMag, sin(angle) * goalStateMag);
//   float pathLength;
//   if (FreePathLength(pathLength, goalState, vel)) {
//     // if there is an obstacle in the way
//     float goalAngleRelativeToPerson = (angle > M_PI) ? (angle - M_PI) : (angle + M_PI);
//     // try other points on the circle
//     for (int i = 1; i < 11; i++) {
//       // angle to new point on circle in positive direction
//       float newAngle = goalAngleRelativeToPerson + (i * M_PI / 10);
//       goalState = Vector2f(cos(newAngle) * idealDistance, sin(newAngle) * idealDistance) + person_position;
//       if (!FreePathLength(pathLength, goalState, vel)) {
//         return true;
//       }
//       // check in negative direction
//       goalState = Vector2f(cos(newAngle) * idealDistance, sin(newAngle) * idealDistance) + person_position;
//       if (!FreePathLength(pathLength, goalState, vel)) {
//         return true;
//       }
//       // if there is no clear path to the circle around the person, attempt to bypass
//       // the obstacle by making the person the goal and relying on optimization function
//       goalState =  person_position;
//       return false;
//     }

//   } else{
//     return true;
//   }
// }



// Step 10
float getScore(const Vector2f& vel, const float v_coeff, const float w_coeff, const float free_path_coeff, const Vector2f& person_position, const float free_path_length) {
  float angle = atan2(person_position(1), person_position(0));

  // calculate goal state information
  float personMag = sqrt((person_position(0) * person_position(0)) + (person_position(1) * person_position(1)));
  float goalStateMag = personMag - idealDistance;
  Vector2f goalState = Vector2f(cos(angle) * goalStateMag, sin(angle) * goalStateMag);
  // calculate robot position information
  Vector2f estimatedRobotPosition;
  estimatedRobotPosition = estimatePositionChange(vel);
  Vector2f difference = estimatedRobotPosition - goalState;
  float distanceFromGoal = sqrt(difference(0) * difference(0) + difference(1) + difference(1));
  return (v_coeff * vel(0)) + (w_coeff * fabs(vel(1) - angle)) + (free_path_coeff * free_path_length) - (position_coeff * distanceFromGoal);

}

// float getScoreSamePos(const Vector2f& vel, const float v_coeff, const float w_coeff, const float free_path_coeff, const float free_path_length) {
//   return (v_coeff * vel(0)) + (w_coeff * fabs(vel(1))) + (free_path_coeff * free_path_length);

// }

// Step 8
// void GetBestCommand(const MatrixXf& win_v, const MatrixXf& win_w, const vector<Vector2f>& obstacles, MatrixXf& scores, MatrixXd& is_admissible, Vector2f& index_best, Vector2f& person_position){

//   size_t size = win_v.rows();
//   Vector2f current_best_index(-1, -1);
//   float current_best_score = -1000000;
//   Vector2f vel;
//   MatrixXf free_path_m(size, size);

//   for(size_t i = 0; i < size; ++i){
//     for(size_t j = 0; j < size; ++j){
//       vel(0) = win_v(i,j);
//       vel(1) = win_w(i,j);
//       float free_path_length = maxRange;

//       // Go over all the obstacle points in the laser scan to find the free path length
//       for(size_t k = 0; k < obstacles.size(); ++k){
//         float free_path_length_current;
//         FreePathLength(free_path_length_current, obstacles[k], vel);
        
//         if(free_path_length_current < free_path_length){
//           free_path_length = free_path_length_current;
//         }
//       }

//       // Check if the current velocity is admissible
//       bool is_admissible_tmp = true;

//       if(vel(0) > sqrt(2 * AmaxLin * free_path_length)){
//         is_admissible_tmp = false;
//       }
      
//       is_admissible(i,j) = (is_admissible_tmp == true)? 1: 0;

//       // Calculate the score for current velocity
//       // scores(i, j) = (v_coeff * vel(0)) + (w_coeff * fabs(vel(1))) + (free_path_coeff * free_path_length);
//       scores(i, j) = getScore(vel, v_coeff, w_coeff, free_path_coeff, person_position, free_path_length);

//       // Keep the best score so far
//       if(is_admissible_tmp == 1 && scores(i, j) > current_best_score){
//         current_best_score = scores(i, j);
//         current_best_index << i, j;
//       }
//       free_path_m(i,j) = free_path_length;

//     }
//   }
//   index_best = current_best_index;
// }

void GetBestCommand(const MatrixXf& win_v, const MatrixXf& win_w, const vector<Vector2f>& obstacles, MatrixXf& scores, MatrixXd& is_admissible, Vector2f& index_best, Vector2f& person_position){
  size_t size = win_v.rows();
  Vector2f current_best_index(-1, -1);
  float current_best_score = -1000000;
  Vector2f vel;
  MatrixXf free_path_m(size, size);

  for(size_t i = 0; i < size; ++i){
    for(size_t j = 0; j < size; ++j){
      vel(0) = win_v(i,j);
      vel(1) = win_w(i,j);
      float free_path_length = maxRange;

      // Go over all the obstacle points in the laser scan to find the free path length
      for(size_t k = 0; k < obstacles.size(); ++k){
        float free_path_length_current;
        FreePathLength(free_path_length_current, obstacles[k], vel);
        
        if(free_path_length_current < free_path_length){
          free_path_length = free_path_length_current;
        }
      }

      // Check if the current velocity is admissible
      bool is_admissible_tmp = true;

      if(vel(0) > sqrt(2 * AmaxLin * free_path_length)){
        is_admissible_tmp = false;
      }
      
      is_admissible(i,j) = (is_admissible_tmp == true)? 1: 0;

      // Calculate the score for current velocity
      // scores(i, j) = (v_coeff * vel(0)) + (w_coeff * fabs(vel(1))) + (free_path_coeff * free_path_length);
      scores(i, j) = getScore(vel, v_coeff, w_coeff, free_path_coeff, person_position, free_path_length);

      // Keep the best score so far
      if(is_admissible_tmp == 1 && scores(i, j) > current_best_score){
        current_best_score = scores(i, j);
        current_best_index << i, j;
      }
      free_path_m(i,j) = free_path_length;

    }
  }
  index_best = current_best_index;

}




//*************************************************************************************
// *************** Function taken from CS403 Assignment5 solution code ****************
// *************** (with some modification) *******************************************
//*************************************************************************************
// bool GetCommandVelService(
//     compsci403_assignment5::GetCommandVelSrv::Request& req,
//     compsci403_assignment5::GetCommandVelSrv::Response& res) {

//   vector<Vector3f> point_cloud;
//   // The input v0 and w0 are each vectors. The x component of v0 is the linear 
//   // velocity towards forward direction and the z component of w0 is the
//   // rotational velocity around the z axis, i.e. around the center of rotation
//   // of the robot and in counter-clockwise direction
//   const Vector2f V(req.v0.x, req.w0.z);

//   for (unsigned int y = 0; y < req.Image.height; ++y) {
//     for (unsigned int x = 0; x < req.Image.width; ++x) {
//       // Add code here to only process only every nth pixel

//       uint16_t byte0 = req.Image.data[2 * (x + y * req.Image.width) + 0];
//       uint16_t byte1 = req.Image.data[2 * (x + y * req.Image.width) + 1];
//       if (!req.Image.is_bigendian) {
//         std::swap(byte0, byte1);
//       }
//       // Combine the two bytes to form a 16 bit value, and disregard the
//       // most significant 4 bits to extract the lowest 12 bits.
//       const uint16_t raw_depth = ((byte0 << 8) | byte1) & 0x7FF;
//       // Reconstruct 3D point from x, y, raw_depth using the camera intrinsics and add it to your point cloud.

//       float X;
//       float Y;
//       float Z;

//       Get3DPointFromDisparity(x, y, raw_depth, &X, &Y, &Z);
//       Vector3f point(Z, -X, Y);
//       point_cloud.push_back(point);
//     }
//   }

//   // Filter the point cloud to remove the ground 
//   Matrix3f R = MatrixXf::Identity(3,3);
//   Vector3f T(0, 0, 0);
//   vector<Vector3f> filtered_point_cloud;
//   ObstaclePointCloudHelper(R, T, point_cloud, filtered_point_cloud);

//   // Use your code from part 3 to convert the point cloud to a laser scan
//   vector<float> ranges;
//   PointCloudToLaserScanHelper(filtered_point_cloud, ranges);

//   // Convert the laser scan to 2D points in cartesian coordinate system
//   vector<Vector2f> obstacles;
//   LaserScanToPoint(ranges, obstacles);


//   // Implement dynamic windowing approach to find the best velocity command for next time step
//   int win_size = 41;
//   MatrixXf win_v(win_size, win_size);
//   MatrixXf win_w(win_size, win_size);
//   MatrixXf scores(win_size, win_size);
//   MatrixXd is_admissible(win_size, win_size);
//   Vector2f index_best;
//   InitializeDynamicWindow(win_v, win_w, V);
//   GetBestCommand(win_v, win_w, obstacles, scores, is_admissible, index_best);


//   // Return the best velocity command
//   // Cv is of type Point32 and its x component is the linear velocity towards forward direction
//   // you do not need to fill its other components
//   // Take the best velocity based on the cost function contingent upon there has been any admissible velocities
//   if(index_best(0) != -1) 
//     res.Cv.x = win_v(index_best(0), index_best(1));
//   else 
//     res.Cv.x = 0;
  
//   // Cw is of type Point32 and its z component is the rotational velocity around z axis
//   // you do not need to fill its other components
//   if(index_best(0) != -1) 
//     res.Cw.z = win_w(index_best(0), index_best(1));
//   else
//     res.Cw.z = 0; 

//   return true;
// }


void OdometryCallback(const nav_msgs::Odometry& odometry) {
  last_odometry = odometry;
}

void PersonFollowerCallback(const sensor_msgs::Image& depth_image){
  // change depth image to point cloud
  sensor_msgs::PointCloud point_cloud;

  depthImageToPointCloud(depth_image, point_cloud);
  // change point cloud to reference frame of robot
  vector<Vector3f> filtered_point_cloud;
  changePointCloudToReferenceFrameOfRobot(point_cloud, filtered_point_cloud);

  // current velocities
  float v0 = last_odometry.twist.twist.linear.x;
  float w0 = last_odometry.twist.twist.angular.z;
  Vector2f V(v0, w0);

  // create dynamic window
  int win_size = 41;
  MatrixXf win_v(win_size, win_size);
  MatrixXf win_w(win_size, win_size);
  MatrixXf scores(win_size, win_size);
  MatrixXd is_admissible(win_size, win_size);
  Vector2f index_best;
  InitializeDynamicWindow(win_v, win_w, V);
  
  // get person position
  lastPersonPosition = findPerson(filtered_point_cloud);

  //angle of person
  float angle = atan2(lastPersonPosition(1), lastPersonPosition(0));

  // goal state
  Vector2f goal_state;
  // get laser scan
  vector<Vector2f> obstacles;
  vector<float> ranges;
  PointCloudToLaserScanHelper(filtered_point_cloud, ranges);
  LaserScanToPoint(ranges, obstacles);
  // get x/y coordinate of goal state
  // float personMag = sqrt((person_position(0) * person_position(0)) + (person_position(1) * person_position(1)));
  // float goalStateMag = personMag - idealDistance;
  // goalState = Vector2f(cos(angle) * goalStateMag, sin(angle) * goalStateMag);

  // float goalAngleRelativeToPerson = (angle > M_PI) ? (angle - M_PI) : (angle + M_PI);
  // int count = 0;
  // float newAngle;

  // while (index_best(0) == -1 && count < 10) {
  //   GetBestCommand(win_v, win_w, obstacles, scores, is_admissible, index_best, goalState);
  //   count++;
  //   newAngle = goalAngleRelativeToPerson + (count * M_PI / 20);
  //   goalState = Vector2f(cos(newAngle) * idealDistance, sin(newAngle) * idealDistance) + person_position;
  //   GetBestCommand(win_v, win_w, obstacles, scores, is_admissible, index_best, goalState);
  // }
  // // check other side of semi-circle
  // count = 0;
  // while (index_best(0) == -1 && count < 10) {
  // count++;
  // newAngle = goalAngleRelativeToPerson - (count * M_PI / 20);
  // goalState = Vector2f(cos(newAngle) * idealDistance, sin(newAngle) * idealDistance) + person_position;
  // GetBestCommand(win_v, win_w, obstacles, scores, is_admissible, index_best, goalState);
  // }

  // // if no admissible command to get to circle, go towards person
  // if (index_best(0) == -1) {
    // goalState = person_position;
  GetBestCommand(win_v, win_w, obstacles, scores, is_admissible, index_best, lastPersonPosition);
  // }

  Twist command_vel;

  // make sure a command was returned
  if (index_best(0) != -1) {
    command_vel.linear.x = win_v(index_best(0), index_best(1));
    command_vel.angular.z = win_w(index_best(0), index_best(1));
  }
  else {
    command_vel.linear.x = 0;
    command_vel.angular.z = 0;
  }
  
  ROS_INFO("Velcoity commands v: %f w: %f", command_vel.linear.x, command_vel.angular.z);
  velocity_command_publisher_.publish(command_vel);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "compsci403_final_project");
  ros::NodeHandle n;

  ros::Subscriber person_follower_subscriber =
    n.subscribe("/Cobot/Kinect/Depth", 1, PersonFollowerCallback);
  ros::Subscriber odometry_subscriber = n.subscribe("/odom", 1, OdometryCallback);

	// deal with input
	// ros::ServiceServer pointCloudFromDepthImage = n.advertiseService()
	// ros::ServiceServer changeToRobotReference = n.advertiseService()

	// // identify obstacles
	// ros::ServiceServer identifyObstacles = n.advertiseService()

	// // find person to follow
	// ros::ServiceServer findPerson = n.advertiseService()

	// // determine if person is moving
	// ros::ServiceServer checkPersonMotion = n.advertiseService()

	// // follow person - main function
	// ros:: ServiceServer followPerson = n.advertiseService()

  // ros::ServiceServer service1 = n.advertiseService(
  //     "/COMPSCI403/GetCommandVel", getBestCommandService);

  velocity_command_publisher_ = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
	ros::spin();
  return 0;
}