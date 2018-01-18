#include <stdint.h>
#include <queue>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "dobot_api/DobotDll.h"

geometry_msgs::Twist desired__;
ros::Time timeout__;

void velocityCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  desired__ = *msg;
  timeout__ = ros::Time::now() + ros::Duration(0.5);
}

uint64_t enqueue(const geometry_msgs::Twist& msg)
{
  float vel = sqrt(pow(msg.linear.x, 2) + pow(msg.linear.y, 2) + pow(msg.linear.z, 2));
  
  CPParams params;
  params.planAcc = 200;
  params.juncitionVel = 1000*vel;
  params.acc = 200;
  params.realTimeTrack = 0;
  
  CPCmd cmd;
  cmd.cpMode = CPRelativeMode;
  if (vel > 0)
  {
    // Relative position after 10ms at this speed
    cmd.x = 10*msg.linear.x;
    cmd.y = 10*msg.linear.y;
    cmd.z = 10*msg.linear.z;
  }
  else
    cmd.x = cmd.y = cmd.z = 0;
  cmd.velocity = 0;
  
  uint64_t index;
  SetCPParams(&params, false, &index);
  SetCPCmd(&cmd, true, &index);
  
  return index;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dobot_controller");
  ros::NodeHandle nh;
  
  std::string device;
  nh.param<std::string>("device", device, "/dev/ttyUSB0");
  
  int queue_depth;
  nh.param<int>("queue_depth", queue_depth, 2);
  
  // Connect Dobot before starting the service
  int result = ConnectDobot(device.c_str(), 115200, 0, 0);
  switch (result)
  {
    case DobotConnect_NoError:
      break;
    case DobotConnect_NotFound:
      ROS_ERROR("Dobot not found");
      return 1;
    case DobotConnect_Occupied:
      ROS_ERROR_STREAM("Invalid port name " << device << " or Dobot is occupied by another application");
      return 2;
    default:
      break;
  }
  
  ros::Subscriber sub = nh.subscribe("cmd_vel", 10, velocityCallback);
  
  SetQueuedCmdClear();
  SetQueuedCmdStartExec();
  
  uint64_t index;
  GetQueuedCmdCurrentIndex(&index);

  ros::Rate rate(1000);
  while (ros::ok())
  {
    if (ros::Time::now() < timeout__)
    {
      uint64_t current_index;
      GetQueuedCmdCurrentIndex(&current_index);
      
      while (index-current_index < 2*queue_depth)
        index = enqueue(desired__);
    }
    
    ros::spinOnce();
    rate.sleep();
  }
  
  SetQueuedCmdForceStopExec();
  
  return 0;
}
