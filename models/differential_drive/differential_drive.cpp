#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->linear.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("cmd_vel", 1, cmd_vel_callback);
  ros::spin();

  return 0;
}