#ifndef GROUPER_LIDAR_H
#define GROUPER_LIDAR_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <dynamixel_control/MotorState.h>
#include <dynamixel_control/MotorControl.h>

//#ifndef PI
#define PI 3.14159
//#endif

namespace grouper_lidar
{
enum GROUPER_LIDAR_STATE
{
  IDLE,
  INIT,
  CW,
  CCW
};
class GrouperLidar
{
 public:
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
  ros::ServiceClient motor_control_;
  ros::ServiceServer grouper_depthmap_control_;
  ros::Publisher grouper_depthmap_;
  ros::Publisher grouper_pointcloud_;
  ros::Subscriber motor_state_;
  
  std:string motor_name_;
  GROUPER_LIDAR_STATE operating_state_;

  dynamixel_control:MotorControl motor_srv_;

  float motor_goal_position_;
  float motor_present_position_;
 public:
  GrouperLidar();
  ~GrouperLidar();
  bool grouperLidarLoop(void);
 private:
  void initGrouperLidar(void);
  void resetMotorPosition(void);
  bool grouperLidarControlCallback(grouper_lidar::GrouperLidarControl::Request  &req
                                   grouper_lidar::GrouperLidarControl::Response &res);
  void motorStateCallback(const dynamixel_control::MotorState::ConstPtr& msg);
  bool motorReachGoal();
}
}

#endif //GROUPER_LIDAR_H
