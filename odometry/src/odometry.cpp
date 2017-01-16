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
    wheel_base = 0.35;
    wheel_radius = 0.5;
    ticks_per_rotation = 360;
    right_wheel_encoder_count = 0;
    left_wheel_encoder_count = 0;
    right_wheel_distance = 0.0;
    left_wheel_distance = 0.0;
    center_wheel_distance = 0.0;
    linear_vel = 0.0;
    angular_vel = 0.0;
    x = 0.0;
    y = 0.0;
    th = 0.0;
  }

  void initiate() {
    ros::Rate r(1.0);

    while(n.ok()) {
      current_time = ros::Time::now();
      double delta_t = (current_time - last_time).toSec();

      double right_wheel_velocity = right_wheel_vel(linear_vel, angular_vel);
      double left_wheel_velocity = left_wheel_vel(linear_vel, angular_vel);

      right_wheel_update(right_wheel_velocity, delta_t);
      left_wheel_update(left_wheel_velocity, delta_t);
 
      x += ( center_wheel_distance * cos( th ) );
      y += ( center_wheel_distance * sin( th ) );
      th += ( ( right_wheel_distance - left_wheel_distance ) / wheel_base );

      ROS_INFO("x[%f], y[%f], th[%f]", x, y, th);

      odom_pub.publish(odom);

      last_time = current_time;
      ros::spinOnce();
      r.sleep();
    }
  }

private:
  ros::NodeHandle n;
  ros::Publisher odom_pub;
  ros::Subscriber vel_sub;
  tf::TransformBroadcaster odom_broadcaster;
  ros::Time current_time, last_time;
  double wheel_base, wheel_radius, linear_vel, angular_vel, left_wheel_encoder_count, right_wheel_encoder_count, right_wheel_distance, left_wheel_distance, center_wheel_distance, x, y, th;
  int ticks_per_rotation;

  double right_wheel_vel(double linear_velocity, double angular_velocity) {
    return ( 2 * linear_velocity + angular_velocity * wheel_base ) / 2 * wheel_radius;
  }

  double left_wheel_vel(double linear_velocity, double angular_velocity) {
    return ( 2 * linear_velocity - angular_velocity * wheel_base ) / 2 * wheel_radius;
  }

  void right_wheel_update(double right_wheel_velocity, double time) {
    right_wheel_encoder_count += ( ( right_wheel_velocity * time ) / ( 2 * M_PI * wheel_radius ) ) * ticks_per_rotation;  
    right_wheel_distance += ( ( right_wheel_velocity * time ) / ( 2 * M_PI * wheel_radius ) );  
    center_wheel_update();
  }

  void left_wheel_update(double left_wheel_velocity, double time) {
    left_wheel_encoder_count += ( ( left_wheel_velocity * time ) / ( 2 * M_PI * wheel_radius ) ) * ticks_per_rotation;
    left_wheel_distance += ( ( left_wheel_velocity * time ) / ( 2 * M_PI * wheel_radius ) );
    center_wheel_update();
  }

  void center_wheel_update() {
    center_wheel_distance = ( left_wheel_distance + right_wheel_distance ) / 2;
  }

  void callback(const geometry_msgs::Twist::ConstPtr& input) {
    linear_vel = input->linear.x;
    angular_vel = input->angular.z;
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "odometry");
  Odometry odom;
  odom.initiate();
  return 0;
}
