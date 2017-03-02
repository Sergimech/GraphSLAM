#include <ros/ros.h>
#include <common/Registration.h>

void scanner_callback(const sensor_msgs::LaserScan& input) {

}

ros::Publisher scanner_pub;

int main(int argc, char** argv) {
  ros::init(argc, argv, "scanner");
  ros::NodeHandle n;

  ros::Subscriber scanner_sub = n.subscribe("/base_scan", 1, scanner_callback);

  scanner_pub  = n.advertise<common::Registration>("/scanner/registration", 1);

  ros::spin();
  return 0;
}
