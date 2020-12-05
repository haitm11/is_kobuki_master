#ifndef KOBUKI_CONTROLLER_H
#define KOBUKI_CONTROLLER_H

#include <cmath>
#include <cstdlib>
#include <ctime>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/CliffEvent.h>
#include <kobuki_msgs/MotorPower.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <ros/ros.h>
#include <ecl/threads.hpp>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <time.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <unistd.h>
#include <string>
#include <tf/tf.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Empty.h>

class KobukiController {
private:
  // ROS handle
  ros::NodeHandle nh;

  // Subscribers
  ros::Subscriber odometry_sub, laser_sub, amcl_sub, map_metadata_sub, map_sub;

  // Publishers
  ros::Publisher cmd_vel_pub, reset_odom_pub, amcl_pub;

  // Message
  geometry_msgs::Twist twist;
  geometry_msgs::Pose2D current_pose;


  // Custom variable

  // Use when moveTo
  float cellSize;
  int rows, cols;

  double PI;
  double PI_MOVE;
  float obsDetecDistance;
  bool isObs;
  bool goAnble;
  bool runAgain;
  float angularSpeed;
  float linearSpeed;
  int currentCellX;
  int currentCellY;
  float amclPoseX;
  float amclPoseY;
  float currentDirectionByAMCL;
  bool initMap;
  int mapHeight;
  int mapWidth;
  double mapResolution;
  double mapOriginX;
  double mapOriginY;
  
  int **mapArr;
  int up;
  int left;
  int down;
  int right;
  int flag;

  std::vector<int> mapData; 
  
  ecl::Thread thread;

public:
  KobukiController(ros::NodeHandle nodehandle) {
    nh = nodehandle;
    PI = 3.1415926535897;
    PI_MOVE = 3.0645;
    obsDetecDistance = 0.28;   // luon luon phai lon hon ban kinh robot
    angularSpeed = PI / 6;
    linearSpeed = 0.1;
    initMap = true;
    isObs = false;
    goAnble = false;
    flag = 1;
    runAgain = false;
  }

  // Declare methods
  bool init();

  void turnToCell(int r, int c);
  void turn(float degree);
  void go(float distance);
  float computeHypotenuse(float a, float b);

  void laserHandle(const sensor_msgs::LaserScanConstPtr laser);
  // Xu li cac su kien tuong ung
  void odometryHandle(const nav_msgs::OdometryConstPtr& odometry);
  void amclHandle(const geometry_msgs::PoseWithCovarianceStampedConstPtr amclpose);
   // Khoi tao map
  void initializeMap(const nav_msgs::MapMetaDataConstPtr msg);
  void setMapData(const nav_msgs::OccupancyGridConstPtr msg);

  void moveWithCCD();
  void printMapWithCCD();
  bool checkNearObsReal(int r, int c);

  // Chuyen mapdata tu 1 chieu sang 2 chieu
  int** oneArrToTwoArr(std::vector<int> mapData, int width, int height);
  int findLineDown(int **mapArr, int width, int height);
  int findLineUp(int **mapArr, int width, int height);
  int findLineLeft(int **mapArr, int width, int height);
  int findLineRight(int **mapArr, int width, int height);

  float getDistanceFromCellToCell(int rowBegin, int colBegin, int rowEnd, int colEnd, float cellSize);
  float getAngleFromCellToCell(int rowBegin, int colBegin, int rowEnd, int colEnd);
  void setVisitedReal(int r, int c);
  void genePathFromCellToCell(int rowBegin, int colBegin, int rowEnd, int colEnd);
  int checkStraight(int row1, int col1, int row2, int col2, int row3, int col3);
  

};

#endif
