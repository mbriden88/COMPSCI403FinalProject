//  Some code taken from Joydeep Biswas solution code for assignment 5, CS403.
//  These functions are marked as such.


#include <algorithm>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

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

// optimization function coeffecients
float v_coeff = 1;
float w_coeff = -0.003; // -5
float free_path_coeff = 1; //10

// last known velocities
Vector2f lastVel(0.0, 0.0);

// last known person position
Vector2f lastPersonPosition(0.0, 0.0);

// ideal distance to maintain from human in meters
float idealDistance = 1.0;

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

//*************************************************************************************
// *************** Function taken from CS403 Assignment5 solution code ****************
//*************************************************************************************
// Helper function to find the free path length
bool FreePathLength(float* free_path_length, const Vector2f& point, const Vector2f& vel) {
  int is_obstacle = false;
  *free_path_length = gMaxRange;

  if(vel(1) == 0){
    *free_path_length = point(0) - gRobotRadius;
    is_obstacle = true;
    return is_obstacle;
  }
  // radius of rotation
  float R = fabs(vel(0)/vel(1)); 
  Vector2f center(0, 0);
  center(1) = (vel(1) > 0)? R: -R;
  Vector2f center_to_obstacle(point(0) - center(0), point(1) - center(1));
  float dist_to_center = sqrt(center_to_obstacle(0) * center_to_obstacle(0) + center_to_obstacle(1) * center_to_obstacle(1));

  
  if(dist_to_center <= (R + gRobotRadius) && dist_to_center >= (R - gRobotRadius)){
    is_obstacle = true;
    float obstacle_angle = atan2(center_to_obstacle(1), center_to_obstacle(0));
    *free_path_length = (obstacle_angle < 0)?( M_PI/2 + obstacle_angle) * R: ( M_PI/2 - obstacle_angle) * R;
    float delta = fabs(dist_to_center - R);
    float correction = sqrt(gRobotRadius * gRobotRadius - delta * delta);
    *free_path_length -= correction;
  }

  return is_obstacle;
}

//*************************************************************************************
// *************** Function taken from CS403 Assignment5 solution code ****************
//*************************************************************************************
void InitializeDynamicWindow(MatrixXf& win_v, MatrixXf& win_w, const Vector2f& V0){
  int size = win_v.rows();
  int center_ind = (size - 1)/2;
  float v_step = gAmaxLin * gDelT / ((size - 1)/2);
  float w_step = gAmaxRot * gDelT / ((size - 1)/2);
  float v_bias = 0.1;


  for(int i = 0; i < size; ++i){
    for(int j = 0; j < size; ++j){
      float v = (i - center_ind) * v_step + V0(0);
      float w = (j - center_ind) * w_step + V0(1);
      v = (v > gVMaxLin)? gVMaxLin: v;
      v = (v < v_bias)? v_bias: v;
      w = (w > gVMaxRot)? gVMaxRot: w;
      win_v(i,j) = v;
      win_w(i,j) = w;
    }
  }

}

//*************************************************************************************
// *************** Function taken from CS403 Assignment5 solution code ****************
// *************** (with some modification) *******************************************
//*************************************************************************************
// Evaluate a cost function over the dynamic window and return the index of the best cell
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
      float free_path_length = gMaxRange;

      // Go over all the obstacle points in the laser scan to find the free path length
      for(size_t k = 0; k < obstacles.size(); ++k){
        float free_path_length_current;
        FreePathLength(&free_path_length_current, obstacles[k], vel);
        
        if(free_path_length_current < free_path_length){
          free_path_length = free_path_length_current;
        }
      }

      // Check if the current velocity is admissible
      bool is_admissible_tmp = true;

      if(vel(0) > sqrt(2 * gAmaxLin * free_path_length)){
        is_admissible_tmp = false;
      }
      
      is_admissible(i,j) = (is_admissible_tmp == true)? 1: 0;

      // Calculate the score for current velocity
      // scores(i, j) = (v_coeff * vel(0)) + (w_coeff * fabs(vel(1))) + (free_path_coeff * free_path_length);
      scores(i, j) = getScore(vel, v_coeff, w_coeff, free_path_coeff, person_position);

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
bool GetCommandVelService(
    compsci403_assignment5::GetCommandVelSrv::Request& req,
    compsci403_assignment5::GetCommandVelSrv::Response& res) {

  vector<Vector3f> point_cloud;
  // The input v0 and w0 are each vectors. The x component of v0 is the linear 
  // velocity towards forward direction and the z component of w0 is the
  // rotational velocity around the z axis, i.e. around the center of rotation
  // of the robot and in counter-clockwise direction
  const Vector2f V(req.v0.x, req.w0.z);

  for (unsigned int y = 0; y < req.Image.height; ++y) {
    for (unsigned int x = 0; x < req.Image.width; ++x) {
      // Add code here to only process only every nth pixel

      uint16_t byte0 = req.Image.data[2 * (x + y * req.Image.width) + 0];
      uint16_t byte1 = req.Image.data[2 * (x + y * req.Image.width) + 1];
      if (!req.Image.is_bigendian) {
        std::swap(byte0, byte1);
      }
      // Combine the two bytes to form a 16 bit value, and disregard the
      // most significant 4 bits to extract the lowest 12 bits.
      const uint16_t raw_depth = ((byte0 << 8) | byte1) & 0x7FF;
      // Reconstruct 3D point from x, y, raw_depth using the camera intrinsics and add it to your point cloud.

      float X;
      float Y;
      float Z;

      Get3DPointFromDisparity(x, y, raw_depth, &X, &Y, &Z);
      Vector3f point(Z, -X, Y);
      point_cloud.push_back(point);
    }
  }

  // Filter the point cloud to remove the ground 
  Matrix3f R = MatrixXf::Identity(3,3);
  Vector3f T(0, 0, 0);
  vector<Vector3f> filtered_point_cloud;
  ObstaclePointCloudHelper(R, T, point_cloud, filtered_point_cloud);

  // Use your code from part 3 to convert the point cloud to a laser scan
  vector<float> ranges;
  PointCloudToLaserScanHelper(filtered_point_cloud, ranges);

  // Convert the laser scan to 2D points in cartesian coordinate system
  vector<Vector2f> obstacles;
  LaserScanToPoint(ranges, obstacles);


  // Implement dynamic windowing approach to find the best velocity command for next time step
  int win_size = 41;
  MatrixXf win_v(win_size, win_size);
  MatrixXf win_w(win_size, win_size);
  MatrixXf scores(win_size, win_size);
  MatrixXd is_admissible(win_size, win_size);
  Vector2f index_best;
  InitializeDynamicWindow(win_v, win_w, V);
  GetBestCommand(win_v, win_w, obstacles, scores, is_admissible, index_best);


  // Return the best velocity command
  // Cv is of type Point32 and its x component is the linear velocity towards forward direction
  // you do not need to fill its other components
  // Take the best velocity based on the cost function contingent upon there has been any admissible velocities
  if(index_best(0) != -1) 
    res.Cv.x = win_v(index_best(0), index_best(1));
  else 
    res.Cv.x = 0;
  
  // Cw is of type Point32 and its z component is the rotational velocity around z axis
  // you do not need to fill its other components
  if(index_best(0) != -1) 
    res.Cw.z = win_w(index_best(0), index_best(1));
  else
    res.Cw.z = 0; 

  return true;
}



float getScore(const Vector2f& vel, const float v_coeff, const float w_coeff, const float free_path_coeff, const Vector2f& person_position) {
  float angle = atan2(person_position(1), person_position(0));
  Vector2f goalState = goalState(person_position, angle);

  return (v_coeff * vel(0)) + (w_coeff * fabs(vel(1) - angle)) + (free_path_coeff * free_path_length);

}

Vector2f getGoalState(const Vector2f& person_position, const angle) {
  float personMag = sqrt(person_position(0) * person_position(0) + person_position(1) * person_position(1));
  float goalStateMag = personMag - idealDistance;
  return Vector2f(cos(angle) * goalStateMag, sin(angle) * goalStateMag);
}

void changePointCloudToReferenceFrameOfRobot(sensor_msgs::PointCloud point_cloud, vector<Vector3f> &filtered_point_cloud){
  //TODO: potentially need to get R and T from a service
  Matrix3f R = MatrixXf::Identity(3,3);
  Vector3f T(0.13, 0, 0.305);
  for (size_t i = 0; i < point_cloud.size(); ++i) {
    Vector3f point = R * point_cloud[i] + T;
    filtered_point_cloud.push_back(point);
  }
}

void findPerson(){
  //TODO: use RANSAC plane fitting to find person and return point in middle of plane
}

void calculatePathToPerson(Point32 goalState, currentVW){
  //TODO: calculate the free path length to the point representing the person
  MatrixXf win_v;
  MatrixXf win_w;
  InitializeDynamicWindow(win_v, win_w, currentVW);


}

void avoidObstacles(){
  //TODO: dynamic window approach to avoid obstacles and return best velocity commands
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

  ros::ServiceServer service1 = n.advertiseService(
      "/COMPSCI403/GetCommandVel", getBestCommandService);



	ros::spin();

  return 0;
}