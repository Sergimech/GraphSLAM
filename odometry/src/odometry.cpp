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
    wheel_base = 0.225;
    wheel_radius = 0.03;
    ticks_per_rotation = 2199.6;
    current_right_wheel_encoder_count = 0;
    current_left_wheel_encoder_count = 0;
    last_right_wheel_encoder_count = 0;
    last_left_wheel_encoder_count = 0;
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
      double delta_time = (current_time - last_time).toSec();
      double right_wheel_velocity = right_wheel_vel(linear_vel, angular_vel);
      double left_wheel_velocity = left_wheel_vel(linear_vel, angular_vel);

      ROS_INFO("##########################");
      ROS_INFO("delta_time[%f], right_wheel_velocity[%f], left_wheel_velocity[%f]", delta_time, right_wheel_velocity, left_wheel_velocity);
      
      right_wheel_update(right_wheel_velocity, delta_time);
      left_wheel_update(left_wheel_velocity, delta_time);

      double delta_tick_right = current_right_wheel_encoder_count - last_right_wheel_encoder_count;
      double delta_tick_left = current_left_wheel_encoder_count - last_left_wheel_encoder_count;

      ROS_INFO("delta_tick_right[%f], delta_tick_left[%f]", delta_tick_right, delta_tick_left);

      double right_wheel_distance = wheel_distance(delta_tick_right);
      double left_wheel_distance = wheel_distance(delta_tick_left);

      ROS_INFO("right_wheel_distance[%f], left_wheel_distance[%f]", right_wheel_distance, left_wheel_distance);

      double center_wheel_distance = ( right_wheel_distance + left_wheel_distance ) / 2;

      ROS_INFO("center_wheel_distance[%f]", center_wheel_distance);
 
      double delta_x = ( center_wheel_distance * cos( th ) );
      double delta_y = ( center_wheel_distance * sin( th ) );
      double delta_th = ( ( right_wheel_distance - left_wheel_distance ) / wheel_base );

      ROS_INFO("delta_x[%f], delta_y[%f], delta_th[%f]", delta_x, delta_y, delta_th);

      x += ( center_wheel_distance * cos( th ) );
      y += ( center_wheel_distance * sin( th ) );
      th += ( ( right_wheel_distance - left_wheel_distance ) / wheel_base );

      ROS_INFO("x[%f], y[%f], th[%f]", x, y, th);

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
      odom.twist.twist.linear.x = linear_vel;
      odom.twist.twist.linear.y = 0.0;
      odom.twist.twist.angular.z = angular_vel;

      odom_pub.publish(odom);

      last_time = current_time;
      last_right_wheel_encoder_count = current_right_wheel_encoder_count;
      last_left_wheel_encoder_count = current_left_wheel_encoder_count;

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
  double wheel_base, wheel_radius;
  double linear_vel, angular_vel;
  double current_left_wheel_encoder_count, current_right_wheel_encoder_count, last_left_wheel_encoder_count, last_right_wheel_encoder_count;
  double x, y, th;
  double ticks_per_rotation;

  double right_wheel_vel(double linear_velocity, double angular_velocity) {
    return ( ( 2 * linear_velocity ) + ( angular_velocity * wheel_base ) ) / ( 2 * wheel_radius );
  }

  double left_wheel_vel(double linear_velocity, double angular_velocity) {
    return ( ( 2 * linear_velocity ) + ( angular_velocity * wheel_base ) ) / ( 2 * wheel_radius );
  }

  double wheel_distance(int delta_tick) {
    return ( 2 * M_PI * wheel_radius ) * ( delta_tick / ticks_per_rotation );
  }

  void right_wheel_update(double right_wheel_velocity, double time) {
    current_right_wheel_encoder_count += ( ( right_wheel_velocity * time ) / ( 2 * M_PI * wheel_radius ) ) * ticks_per_rotation;  
  }

  void left_wheel_update(double left_wheel_velocity, double time) {
    current_left_wheel_encoder_count += ( ( left_wheel_velocity * time ) / ( 2 * M_PI * wheel_radius ) ) * ticks_per_rotation;
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
