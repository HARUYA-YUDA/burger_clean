#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <std_msgs/Bool.h>
#include <sstream>
#include <iostream>
#include "arm.hpp"
#include <vector>
#include "waypoints.hpp"
#include "std_srvs/Trigger.h"

using namespace std;

const int INITIAL_POSE = 0;
const int CLEAN_RIGHT_POSE = 1;
const int CLEAN_LEFT_POSE = 2;

//arm
inline void arm_initial(class Arm arm, std_srvs::Trigger srv);
inline void clean_right(class Arm arm, ros::NodeHandle *n, ros::ServiceClient enable_srv, ros::ServiceClient disable_srv, ros::ServiceClient set_cleaning_srv, std_srvs::Trigger srv);

int main(int argc, char **argv)
{
  ROS_INFO("Start");

  //magic, don't think about here.
  ros::init(argc, argv, "sample");
  ros::NodeHandle nh;
  //Service
  ros::ServiceClient set_cleaning_srv = nh.serviceClient<std_srvs::Trigger>("set_cleaning");
  ros::ServiceClient set_normal = nh.serviceClient<std_srvs::Trigger>("set_normal");
  ros::ServiceClient enable_srv = nh.serviceClient<std_srvs::Trigger>("enable_all");
  ros::ServiceClient disable_srv = nh.serviceClient<std_srvs::Trigger>("disable_all");
  ros::Rate rate(10);
  std_srvs::Trigger srv;

  Arm arm(&nh);
  //error
  arm.armPos(cleanpose[INITIAL_POSE]);
  while(ros::ok() && arm.moveCheck()){
    ros::spinOnce();
    arm.cycle();
  }
  
  clean_right(arm, &nh, enable_srv, disable_srv, set_cleaning_srv, srv);
  ROS_INFO("called service");

  //initializing way point.
  // ROS cooridnata: forward is X axis,  left direction is y axis, and up is z axis.


//main while
//---------------------------------------------------
//---------------------------------------------------


  ROS_INFO("Mission complete!");
  sleep(3); // [s]
  return 0;
}

inline void arm_initial(class Arm arm, std_srvs::Trigger srv)
{
  arm.armPos(cleanpose[INITIAL_POSE]);
  while(ros::ok() && arm.moveCheck()){
    ros::spinOnce();
    arm.cycle();
  }
}
inline void clean_right(class Arm arm, ros::NodeHandle *n, ros::ServiceClient enable_srv, ros::ServiceClient disable_srv, ros::ServiceClient set_cleaning_srv, std_srvs::Trigger srv)
{
  arm.armPos(cleanpose[CLEAN_RIGHT_POSE]);
  while(ros::ok() && arm.moveCheck()){
    ros::spinOnce();
    arm.cycle_clean();
  }
  if(disable_srv.call(srv)){
  }
  else{
    ROS_ERROR("Failed to call service disable_all");
  }
  if(set_cleaning_srv.call(srv)){
  }
  else{
    ROS_ERROR("Failed to call service disable_all");
  }
  if(enable_srv.call(srv)){
  }
  else{
    ROS_ERROR("Failed to call service disable_all");
  }
}
