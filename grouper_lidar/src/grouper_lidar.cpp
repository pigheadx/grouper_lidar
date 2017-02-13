#include <grouper_lidar/grouper_lidar.h>

using namespace grouper_lidar;

GrouperLidar::GrouperLidar()
    :nh_priv_("~"),
     motor_name_("")
{
  nh_priv_.param<std::string>("motor_name_", motor_name_, "radar_tilt");
  initGrouperLidar();
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
      moveMotor(MOTOR_BOARD_CCW);
      if (motorReachGoal())
      {
        present_scan_line_ = 0;
        operating_state_=CW;
      }
    break;
    case CW:
      if(motorReachGoal())
      {
        LogLidar();
        output1Frame();
        if(present_scan_line_>=SCAN_LINES-1)
        {
          //output1Frame();
          operating_state_=CCW;
        }
        else
        {
          present_scan_line_++;
          moveMotor(MOTOR_BOARD_CCW-(present_scan_line_*(MOTOR_BOARD_CCW-MOTOR_BOARD_CW)/(SCAN_LINES-1)));
        }
      }
      else
      {
        ROS_ERROR("motor didn't reach goal");
      }
    break;
    case CCW:
      if(motorReachGoal())
      {
        LogLidar();
        output1Frame();
        if(present_scan_line_<=0)
        {
          //output1Frame();
          operating_state_=CW;
        }
        else
        {
          present_scan_line_--;
          moveMotor(MOTOR_BOARD_CCW-(present_scan_line_*(MOTOR_BOARD_CCW-MOTOR_BOARD_CW)/(SCAN_LINES-1)));
        }
      }
      else
      {
        ROS_ERROR("motor didn't reach goal");
      }
    break;
    default:
      
    break;
  }
}

void GrouperLidar::initGrouperLidar(void)
{
  motor_control_ = nh_.serviceClient<dynamixel_control_msgs::MotorControl>("dynamixel_control/motor_control");

  motor_state_ = nh_.subscribe("dynamixel_control/motor_state", 10, &GrouperLidar::motorStateCallback, this);
  lidar_scan_ = nh_.subscribe("scan", 10, &GrouperLidar::lidarScanCallback, this);

  grouper_depthmap_ = nh_.advertise<sensor_msgs::Image>("grouper_lidar/depthmap_image", 10);
  grouper_intensity_ = nh_.advertise<sensor_msgs::Image>("grouper_lidar/intensity_image", 10);
  grouper_pointcloud_ = nh_.advertise<sensor_msgs::PointCloud2>( "grouper_lidar/point_cloud", 10);

  grouper_depthmap_control_ = nh_.advertiseService("grouper_lidar/capture_control", &GrouperLidar::grouperLidarControlCallback, this);

  operating_state_ = IDLE;
  motor_srv_.request.motor_name = motor_name_;
  motor_srv_.request.control_type = "position";
  motor_srv_.request.unit = "rad";
  motor_goal_position_ = 0;
  motor_present_position_ = 0;
  memset(ranges_matrix_,0,sizeof(ranges_matrix_));
  memset(intensities_matrix_,0,sizeof(intensities_matrix_));
  memset(scan_position_matrix_,0,sizeof(scan_position_matrix_));
  memset(ranges_img,0,sizeof(ranges_img));
  memset(intensities_img,0,sizeof(intensities_img));
  present_scan_line_ = 0;
  msg_seq = 0;
}

void GrouperLidar::moveMotor(double value)
{
  motor_srv_.request.value = value;
  if (motor_control_.call(motor_srv_))
  {
    ROS_DEBUG("motor to %f", motor_srv_.response.value);
    motor_goal_position_ = motor_srv_.response.value;
  }
  else
  {
    ROS_ERROR("Failed to call motor service");
  }

}

bool GrouperLidar::motorReachGoal()
{
  return (abs(motor_present_position_-motor_goal_position_) < MOTOR_TOLERANT);
}

void GrouperLidar::LogLidar()
{
  memcpy(ranges_matrix_[present_scan_line_],lidar_ranges_,LIDAR_POINTS*sizeof(float));
  memcpy(intensities_matrix_[present_scan_line_],lidar_intensities_,LIDAR_POINTS*sizeof(float));
  scan_position_matrix_[present_scan_line_] = motor_present_position_;
  for(int i=0; i<OUTPUT_COL; i++)
  {
    ranges_img[present_scan_line_][i]=ranges_matrix_[present_scan_line_][(LIDAR_POINTS/2)+LIDAR_STEP*((OUTPUT_COL/2)-i)];
    intensities_img[present_scan_line_][i]=intensities_matrix_[present_scan_line_][(LIDAR_POINTS/2)+LIDAR_STEP*((OUTPUT_COL/2)-i)];
  }
}

void GrouperLidar::output1Frame()
{
  sensor_msgs::Image depthmap_msg;
  depthmap_msg.header.seq = msg_seq;
  depthmap_msg.header.stamp = ros::Time::now();
  depthmap_msg.header.frame_id = "depthmap";
  depthmap_msg.height = OUTPUT_ROW;
  depthmap_msg.width = OUTPUT_COL;
  depthmap_msg.encoding = "32FC1";
  depthmap_msg.is_bigendian = false;
  depthmap_msg.step = OUTPUT_COL*sizeof(float);
  depthmap_msg.data.resize(OUTPUT_ROW*depthmap_msg.step);
//  for(int i=0; i<OUTPUT_ROW; i++)
//  {
//    for(int j=0; j<OUTPUT_COL; j++)
//    {
//      depthmap_msg.data[i*depthmap_msg.step+j] = ranges_img[i][j];
//    }
//  }
  memcpy(&depthmap_msg.data[0], &ranges_img[0][0], OUTPUT_ROW*depthmap_msg.step);
  grouper_depthmap_.publish(depthmap_msg);

  sensor_msgs::Image intensity_msg;  
  intensity_msg.header.seq = msg_seq;
  intensity_msg.header.stamp = ros::Time::now();
  intensity_msg.header.frame_id = "intensity";
  intensity_msg.height = OUTPUT_ROW;
  intensity_msg.width = OUTPUT_COL;
  intensity_msg.encoding = "32FC1";
  intensity_msg.is_bigendian = false;
  intensity_msg.step = OUTPUT_COL*sizeof(float);
  intensity_msg.data.resize(OUTPUT_ROW*intensity_msg.step);
//  for(int i=0; i<OUTPUT_ROW; i++)
//  {
//    for(int j=0; j<OUTPUT_COL; j++)
//    {
//      intensity_msg.data[i*intensity_msg.step+j] = intensities_img[i][j];
//    }
//  }
  memcpy(&intensity_msg.data[0], &intensities_img[0][0], OUTPUT_ROW*intensity_msg.step);
  grouper_intensity_.publish(intensity_msg);

  sensor_msgs::PointCloud2 pc_msg;
  pcl::PointCloud< pcl::PointXYZI > pc;
  pc.width  = LIDAR_POINTS;
  pc.height = SCAN_LINES;
  pc.is_dense = false;
  pc.points.resize( pc.width * pc.height );
  for(int i = 0; i < pc.height; ++i ) { 
    const double phi = (-1)*PI + scan_position_matrix_[i]*(2*PI)/MOTOR_POINTS;
    const double x_mod = LIDAR_HEIGHT*cos(phi+0.5*PI);
    const double z_mod = LIDAR_HEIGHT*sin(phi+0.5*PI);
    for(int j = 0; j < pc.width; ++j ) {
      const int k = pc.width * i + j;
      const double theta = (LIDAR_ANGLE_MIN) + j*((LIDAR_ANGLE_MAX)-(LIDAR_ANGLE_MIN))/(LIDAR_POINTS-1);
      const double x_pre = ranges_matrix_[i][j]*cos(theta);
      pc.points[k].x = x_pre*cos(phi)+x_mod;
      pc.points[k].y = ranges_matrix_[i][j]*sin(theta);
      pc.points[k].z = x_pre*sin(phi)+z_mod;
      pc.points[k].intensity = intensities_matrix_[i][j];
    }
  }
  pcl::toROSMsg(pc, pc_msg);
  pc_msg.header.seq = msg_seq;
  pc_msg.header.stamp = ros::Time::now();
  pc_msg.header.frame_id = "pc";
  grouper_pointcloud_.publish(pc_msg);

  msg_seq++;
}

bool GrouperLidar::grouperLidarControlCallback(grouper_lidar_msgs::GrouperLidarControl::Request  &req,
                                               grouper_lidar_msgs::GrouperLidarControl::Response &res)
{
  res.ans=false;
  if(req.command=="start")
  {
    operating_state_ = INIT;
    res.ans = true;
  }
  if(req.command=="stop")
  {
    operating_state_ = IDLE;
    res.ans = true;
  }
  if(req.command=="reset")
  {
    operating_state_ = IDLE;
    res.ans = true;
  }
  return res.ans;
}

void GrouperLidar::motorStateCallback(const dynamixel_control_msgs::MotorState::ConstPtr& msg)
{
  if(msg->header.frame_id==motor_name_)
  {
    motor_present_position_=msg->present_position;
    ROS_DEBUG("motor present %f", motor_present_position_);
  }
}

void GrouperLidar::lidarScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  memcpy(&lidar_ranges_[0],&msg->ranges[0],LIDAR_POINTS*sizeof(float));
  memcpy(&lidar_intensities_[0],&msg->intensities[0],LIDAR_POINTS*sizeof(float));
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "grouper_lidar");
  GrouperLidar grouper_lidar;

  ros::Rate loop_rate(50);

  while (ros::ok())
  {
    grouper_lidar.grouperLidarLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
