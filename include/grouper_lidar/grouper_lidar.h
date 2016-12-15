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
#define MOTOR_BOARD_CW  -3.14159
#define MOTOR_BOARD_CCW -1.60761
#define MOTOR_TOLERANT 3
#define SCAN_LINES 200
#define LIDAR_POINTS 1081
#define OUTPUT_ROW 200
#define OUTPUT_COL 200
#define LIDAR_STEP 2

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
  ros::Publisher grouper_intensity_;
  ros::Publisher grouper_pointcloud_;
  ros::Subscriber motor_state_;
  ros::Subscriber lidar_scan_;
  
  std:string motor_name_;
  GROUPER_LIDAR_STATE operating_state_;

  dynamixel_control:MotorControl motor_srv_;

  double motor_goal_position_;
  double motor_present_position_;
  
  float lidar_ranges_[];
  float lidar_intensities_[];
  
  float ranges_matrix_[SCAN_LINES][LIDAR_POINTS];
  float intensities_matrix_[SCAN_LINES][LIDAR_POINTS];

  uint16_t ranges_img[OUTPUT_ROW][OUTPUT_COL];
  uint16_t intensities_img[OUTPUT_ROW][OUTPUT_COL];

  uint32_t msg_seq;

  float scan_position_matrix_[SCAN_LINES];
  int present_scan_line_;
  
  
 public:
  GrouperLidar();
  ~GrouperLidar();
  bool grouperLidarLoop(void);
 private:
  void initGrouperLidar(void);
  void resetMotorPosition(void);
  void moveMotor(double value);
  bool motorReachGoal();
  void LogLidar();
  void output1Frame();
  bool grouperLidarControlCallback(grouper_lidar::GrouperLidarControl::Request  &req
                                   grouper_lidar::GrouperLidarControl::Response &res);
  void motorStateCallback(const dynamixel_control::MotorState::ConstPtr& msg);
  void lidarScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
}
}

#endif //GROUPER_LIDAR_H
