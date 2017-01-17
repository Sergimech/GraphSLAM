#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <math.h>

class Odometry {
public:
  Odometry() {
    odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 50);
    vel_sub = n.subscribe("/cmd_vel", 50, &Odometry::callback, this);
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    vx = 0.0;
    vy = 0.0;
    vth = 0.0;
    x = 0.0;
    y = 0.0;
    th = 0.0;
  }

  void initiate() {
    while(n.ok()) {
      current_time = ros::Time::now();
      double delta_t = (current_time - last_time).toSec();
      double delta_x = (vx * cos(th) - vy * sin(th)) * delta_t;
      double delta_y = (vx * sin(th) + vy * cos(th)) * delta_t;
      double delta_th = vth * delta_t;
      
      x += delta_x;
      y += delta_y;
      th += delta_th;

      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

      geometry_msgs::TransformStamped odom_trans;
      odom_trans.header.stamp = current_time;
      odom_trans.header.frame_id = "odom";
      odom_trans.child_frame_id = "base_link";

      odom_trans.transform.translation.x = x;
      odom_trans.transform.translation.y = y;
      odom_trans.transform.translation.z = 0.0;
      odom_trans.transform.rotation = odom_quat;

      odom_broadcaster.sendTransform(odom_trans);

      nav_msgs::Odometry odom;
      odom.header.stamp = current_time;
      odom.header.frame_id = "odom";

      odom.pose.pose.position.x = x;
      odom.pose.pose.position.y = y;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation = odom_quat;

      odom.child_frame_id = "base_link";
      odom.twist.twist.linear.x = vx;
      odom.twist.twist.linear.y = vy;
      odom.twist.twist.angular.z = vth;

      odom_pub.publish(odom);

      last_time = current_time;
      ros::spinOnce();
    }
  }

private:
  ros::NodeHandle n;
  ros::Publisher odom_pub;
  ros::Subscriber vel_sub;
  tf::TransformBroadcaster odom_broadcaster;
  ros::Time current_time, last_time;
  double vx, vy, vth;
  double x, y, th;
  
  void callback(const geometry_msgs::Twist::ConstPtr& input) {
    vx = input->linear.x;
    vy = input->linear.y;
    vth = input->angular.z;
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "odometry");
  Odometry odom;
  odom.initiate();
  return 0;
}
