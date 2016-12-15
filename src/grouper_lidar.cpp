#include <grouper_lidar/grouper_lidar.h>

using namespace grouper_lidar

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
      moveMotor(MOTOR_BOARD_CCW)
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
        if(present_scan_line_>=SCAN_LINES-1)
        {
          output1Frame();
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
        if(present_scan_line_<=0)
        {
          output1Frame();
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
  motor_control_ = nh_.serviceClient<dynamixel_control:MotorControl>("/dynamixel_control/motor_control");

  motor_state_ = nh_.subscribe("/dynamixel_control/motor_state", 10, motorStateCallback, this);
  lidar_scan_ = nh_.subscribe("scan", 10, lidarScanCallback, this);

  grouper_depthmap_ = nh_.advertise<senser_msgs::Image>("/grouper_lidar/depthmap_image", 10);
  grouper_intensity_ = nh_.advertise<senser_msgs::Image>("/grouper_lidar/intensity_image", 10);
  grouper_pointcloud_ = nh.advertise<sensor_msgs::PointCloud2>( "/grouper_lidar/point_cloud", 10);

  grouper_depthmap_control_ = nh_.advertiseService("/grouper_lidar/capture_control", &GrouperLidar::grouperLidarControlCallback, this);

  operating_state_ = IDLE;
  motor_srv_.motor_name = motor_name_;
  motor_srv_.control_type = "position";
  motor_srv_.unit = "rad"
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
  motor_srv_.value = value;
  if (motor_control_.call(motor_srv_))
  {
    ROS_DEBUG("motor to %f", motor_srv_.response.value);
  }
  else
  {
    ROS_ERROR("Failed to call motor service");
  }
  motor_goal_position_ = motor_srv_.response.value;
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
    ranges_img[present_scan_line_][i]=(int)ranges_matrix_[present_scan_line_][(LIDAR_POINTS/2)+LIDAR_STEP*(i-(OUTPUT_COL/2))]/60*65535;
    intensities_img[present_scan_line_][i]=(int)intensities_matrix_[present_scan_line_][(LIDAR_POINTS/2)+LIDAR_STEP*(i-(OUTPUT_COL/2))]/60*65535;
  }
}

void GrouperLidar::output1Frame()
{
  sensor_msgs::Image depthmap_msg;
  depthmap_msg.header.seq = msg_seq;
  depthmap_msg.header.stamp = ros::Time::now();
  depthmap_msg.frame_id = "depthmap";
  depthmap_msg.height = OUTPUT_ROW;
  depthmap_msg.width = OUTPUT_COL;
  depthmap_msg.encoding = "mono16";
  depthmap_msg.is_bigendian = false;
  depthmap_msg.step = OUTPUT_COL*sizeof(uint16_t);
  depthmap_msg.data = ranges_img;
  grouper_depthmap_.publish(depthmap_msg);

  sensor_msgs::Image intensity_msg;  
  intensity_msg.header.seq = msg_seq;
  intensity_msg.header.stamp = ros::Time::now();
  intensity_msg.frame_id = "intensity";
  intensity_msg.height = OUTPUT_ROW;
  intensity_msg.width = OUTPUT_COL;
  intensity_msg.encoding = "mono16";
  intensity_msg.is_bigendian = false;
  intensity_msg.step = OUTPUT_COL*sizeof(uint16_t);
  intensity_msg.data = intensities_img;
  grouper_intensity_.publish(intensity_msg);

  sensor_msgs::PointCloud2 pc_msg;
  pcl::PointCloud< pcl::PointXYZI > pc;
  pc.width  = LIDAR_POINTS;
  pc.height = SCAN_LINES;
  pc.is_dense = false;
  pc.points.resize( pc.width * pc.height );
  for(int i = 0; i < pc.height; ++i ) {
    for(int j = 0; j < pc.width; ++j ) {
      const int k = pc.width * i + j;
      pc.points[k].x = ;
      pc.points[k].y = ;
      pc.points[k].z = ;
      pc.points[k].intensity = intensities_matrix_[i][j];
    }
  }
  pcl::toROSMsg(pc, pc_msg);
  pc_msg.header.seq = msg_seq;
  pc_msg.header.stamp = ros::Time::now();
  pc_msg.frame_id = "pc";
  grouper_pointcloud_.publish(pc_msg);

  msg_seq++;
}

bool GrouperLidar::grouperLidarControlCallback(grouper_lidar::GrouperLidarControl::Request  &req
                                               grouper_lidar::GrouperLidarControl::Response &res)
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

void GrouperLidar::motorStateCallback(const dynamixel_control::MotorState::ConstPtr& msg)
{
  if(msg.header.frame_id==motor_name_)
  {
    motor_present_position_=msg.present_position;
    ROS_DEBUG("motor present %f", motor_present_position_);
  }
}

void GrouperLidar::lidarScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  lidar_ranges_ = msg.ranges;
  lidar_intensities_ = msg.intensities;
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
