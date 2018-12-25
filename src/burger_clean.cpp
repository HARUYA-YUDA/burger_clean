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

geometry_msgs::Pose2D current_pose;
ros::Publisher pose2d_pub;
ros::Publisher twist_pub;
geometry_msgs::Twist twist;

const double rot_vel = 0.3;    // angular velocity [rad/s]
const double linear_vel = 0.2; // linear  velocity [m/s]
const double kp_linear  = 0.75;  // proportional gain for lenear velocity
const double kp_angular = 1.0;  // proportional gain for lenear velocity     
const double ki_angular = 0.05;  // proportional gain for lenear velocity     
const double rot_vel_i = 0.1;
const int INITIAL_POSE = 0;
const int CLEAN_RIGHT_POSE = 1;
const int CLEAN_LEFT_POSE = 2;

//Vehicle
void odomCallback(const nav_msgs::OdometryConstPtr& msg);
void gotoDirection(int no);
void gotoPosition(int no);
void gotoWaypoint(int no);

//arm
inline void arm_initial(class Arm arm, std_srvs::Trigger srv);
inline void clean_right(class Arm arm, ros::NodeHandle *n, ros::ServiceClient enable_srv, ros::ServiceClient disable_srv, ros::ServiceClient set_cleaning_srv, std_srvs::Trigger srv);

int main(int argc, char **argv)
{
  double old_pose=0;

  ROS_INFO("Start");

  //magic, don't think about here.
  ros::init(argc, argv, "burger_clean");
  ros::NodeHandle nh;
  //Subscriber
  ros::Subscriber odom_sub = nh.subscribe("odom", 1, odomCallback);
  //Publisher
  twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1000);
  pose2d_pub = nh.advertise<geometry_msgs::Pose2D>("roomba_pose2d", 1);
  //Service
  ros::ServiceClient set_cleaning_srv = nh.serviceClient<std_srvs::Trigger>("set_cleaning");
  ros::ServiceClient set_normal = nh.serviceClient<std_srvs::Trigger>("set_normal");
  ros::ServiceClient enable_srv = nh.serviceClient<std_srvs::Trigger>("enable_all");
  ros::ServiceClient disable_srv = nh.serviceClient<std_srvs::Trigger>("disable_all");
  ros::Rate rate(10);
  std_srvs::Trigger srv;

  Arm arm(&nh);
  arm_initial(arm, srv);

  //initializing way point.
  // ROS cooridnata: forward is X axis,  left direction is y axis, and up is z axis.
  int next_wp = 0; // Next waypoint
  int next_ap = 0; // Next armpoint
  sleep(1);
  //initializing arm position 
   
  arm.armPos(armpose[11]);
  while(ros::ok() && arm.moveCheck()){
    ros::spinOnce();
    arm.cycle();    
  }


//main while
//---------------------------------------------------
  while (ros::ok() && waypoint[next_wp].x != 999) {
    ros::spinOnce();
    //way point running
    ROS_INFO("Go to WP %d",next_wp);
    gotoWaypoint(next_wp);
    arm.armPos(armpose[next_ap]);
    
    while(ros::ok() && arm.moveCheck()){
      ros::spinOnce();
      arm.cycle();    
    }

  next_wp++;
  next_ap++;
  }
//---------------------------------------------------


  twist.angular.z = 0;
  twist.linear.x  = 0;
  twist_pub.publish(twist);
  ROS_INFO("Mission complete!");
  sleep(3); // [s]
  return 0;
}

void odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
  // linear position
  current_pose.x = msg->pose.pose.position.x;
  current_pose.y = msg->pose.pose.position.y;

  // quaternion to RPY conversion
  tf::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  // angular position
  current_pose.theta = yaw;
  pose2d_pub.publish(current_pose);
}

// Face to the direction
void gotoDirection(int no)
{
  double thresh = 1.0 * M_PI/180.0;
  double diff = thresh + 1; // make diff is bigger than thresh

  ros::Rate loop_rate(100);
  double angle_tmp_i = 0;
  twist.linear.x = 0; // linear velocity is 0, i.e., only rotate
  while (fabs(diff) > thresh) {
    diff = waypoint[no].yaw - current_pose.theta;
    if (diff >   M_PI) diff -= 2 * M_PI;
    else if (diff < - M_PI) diff += 2 * M_PI;
    // cout << "dir diff:" << diff << endl;
    // p
    double angle_tmp_p = kp_angular * diff;
    // i
    angle_tmp_i += ki_angular*diff;
    if (angle_tmp_i >= rot_vel_i ) angle_tmp_i=rot_vel_i;
    else if (angle_tmp_i <= -rot_vel_i) angle_tmp_i=-rot_vel_i;

    double angle_tmp = angle_tmp_p + angle_tmp_i;
    if (angle_tmp >= rot_vel ) angle_tmp=rot_vel;
    else if (angle_tmp <= -rot_vel) angle_tmp=-rot_vel;
 
    twist.angular.z =  angle_tmp;

    twist_pub.publish(twist);
    ros::spinOnce();
    loop_rate.sleep();
  }
  twist.linear.x  = linear_vel;
  twist.angular.z = 0;
  twist_pub.publish(twist);
  ros::spinOnce();
}

// Go to the next waypoint
void gotoPosition(int no) {
  double thresh = 0.01; // [m]
  // initial diff_dist should be bigger than thresh
  double diff_dist = thresh + 1; 
  twist.linear.x = linear_vel;

  ros::Rate loop_rate(100);

  double angle_tmp_i = 0;
  while (fabs(diff_dist) > thresh) {
    double diff_x = waypoint[no].x - current_pose.x;
    double diff_y = waypoint[no].y - current_pose.y;
    double diff_theta = atan2(diff_y, diff_x) - current_pose.theta;
    diff_dist = sqrt(diff_x * diff_x + diff_y * diff_y);
    cout << "gotoPosition" << diff_dist << endl;
    cout << "diff dist:" << diff_dist << "[m]" << endl;
    cout << "current_pose.x:" << current_pose.x << "[m]" << endl;
    cout << "current_pose.y:" << current_pose.y << "[m]" << endl;
    cout << "current_pose.theta:" << current_pose.theta << "[rad]" << endl;
    if (diff_theta >   M_PI) diff_theta -= 2 * M_PI;
    else if (diff_theta < - M_PI) diff_theta += 2 * M_PI;
    //maximum velosity
    double angle_tmp_p = kp_angular * diff_theta;
    // i
    angle_tmp_i += ki_angular*diff_theta;
    if (angle_tmp_i >= rot_vel_i ) angle_tmp_i=rot_vel_i;
    else if (angle_tmp_i <= -rot_vel_i) angle_tmp_i=-rot_vel_i;

    double angle_tmp = angle_tmp_p + angle_tmp_i;
    if (angle_tmp >= rot_vel ) angle_tmp=rot_vel;
    else if (angle_tmp <= -rot_vel) angle_tmp=-rot_vel;
 
    twist.angular.z =  angle_tmp;

   //maximum velosity
    double dist_tmp = kp_linear * diff_dist;
    if (dist_tmp >= linear_vel ) dist_tmp=linear_vel;
    else if (dist_tmp <= -linear_vel) dist_tmp=-linear_vel;
    twist.linear.x  = dist_tmp;

    twist_pub.publish(twist);
    ros::spinOnce();
    loop_rate.sleep();
  }
  twist.angular.z = 0;
  twist.linear.x  = 0;
  twist_pub.publish(twist);
  ros::spinOnce();
  ROS_INFO("Arrived at WP %d",no);
  sleep(1); // [s]
}

void gotoWaypoint(int no)
{
  gotoPosition(no);
  gotoDirection(no);
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
