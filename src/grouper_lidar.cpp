#include <grouper_lidar/grouper_lidar.h>

using namespace grouper_lidar

GrouperLidar::GrouperLidar()
    :nh_priv_("~"),
     motor_name_("")
{
  nh_priv_.param<std::string>("motor_name_", motor_name_, "radar_tilt");
  
  motor_control_ = nh_.serviceClient<dynamixel_control:MotorControl>("/dynamixel_control/motor_control");
  grouper_depthmap_control_ = nh_.advertiseService("/grouper_lidar/capture_control", &GrouperLidar::grouperLidarControlCallback, this);
//  grouper_depthmap_ = nh_.advertise<senser_msgs::Image>("/grouper_lidar/depthmap_image", 10);
//  depthmap_image output
//  grouper_pointcloud_ = nh.advertise<sensor_msgs::PointCloud2>( "/grouper_lidar/point_cloud", 10);
//  pc output;
  motor_state_ = nh_.subscribe("/dynamixel_control/motor_state", 10, chatterCallback, this);
  operating_state_ = IDLE;
  motor_srv_.motor_name = "motor_name_";
  motor_srv_.control_type = "position";
  motor_srv_.unit = "rad"
}

GrouperLidar::~GrouperLidar()
{
//TODO:destructor
}

bool GrouperLidar::grouperLidarLoop(void)
{
  switch(operating_state_)
  {
    case IDLE:
      
    break;
    case INIT:
      motor_srv_.value = -PI;
      if (motor_control_.call(motor_srv_))
      {
        ROS_DEBUG("motor to %f", motor_srv_.response.value);
      }
      else
      {
        ROS_ERROR("Failed to call motor service");
      }
      motor_goal_position_ = motor_srv_.response.value;
      if (motorReachGoal())
      {
        operating_state_=CCW;
      }
        
    break;
    case CW:
      
    break;
    case CCW:
      
    break;
    default:
      
    break;
  }
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "grouper_lidar");
  GrouperLidar grouper_lidar;

  ros::Rate loop_rate(25);

  while (ros::ok())
  {
    grouper_lidar.grouperLidarLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
