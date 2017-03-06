#include <math.h>
#include <vector>

#include <Eigen/Core>

#include <ros/ros.h>

#include <common/Odometry.h>
#include <common/Pose2DWithCovariance.h>
#include <common/OdometryBuffer.h>

#include <geometry_msgs/Twist.h>

double vx, vy, vth, Delta_x, Delta_y, Delta_th;
std::vector<common::Odometry> buffer_odom;

common::Pose2DWithCovariance create_Pose2DWithCovariance_msg(double x, double y, double th, Eigen::MatrixXd m) {
  common::Pose2DWithCovariance output;

  output.pose.x = x;
  output.pose.y = y;
  output.pose.theta = th;
  
  for(int i = 0; i < m.rows(); i++) {
    for(int j = 0; j < m.cols(); j++) {
      output.covariance[( i * m.rows() ) + j] = m(i, j);
    }
  }

  return output;
}

// JS: This method is used by more than one node.
//      It could be placed in package 'common'
//      and also renamed to 'between(pose1, pose2)'
common::Pose2DWithCovariance pose_transform(common::Pose2DWithCovariance start_pose,
					    common::Pose2DWithCovariance end_pose) {
  common::Pose2DWithCovariance transform;
  double t_start_th = start_pose.pose.theta;
  double t_end_th = end_pose.pose.theta;
  double cos_th = cos ( t_start_th );
  double sin_th = sin ( t_start_th );
  double dx = end_pose.pose.x - start_pose.pose.x;
  double dy = end_pose.pose.y - start_pose.pose.y;
  double dth = t_end_th - t_start_th;
  dth = std::fmod( dth + M_PI, 2 * M_PI ) - M_PI;
  transform.pose.x = ( cos_th * dx ) + ( sin_th * dy );
  transform.pose.y = ( -1 * sin_th * dx ) + ( cos_th * dy );
  transform.pose.theta = dth;

  return transform;
  
}

void vel_callback(const geometry_msgs::Twist::ConstPtr& input) {
  vx = input->linear.x;
  vy = input->linear.y;
  vth = input->angular.z;
}

void add_to_buffer(common::Odometry input) {
  if(buffer_odom.size() > 1000) {
    buffer_odom.clear();
  }

  buffer_odom.push_back(input);
}

bool odometry_buffer_request(common::OdometryBuffer::Request &req, common::OdometryBuffer::Response &res) {
  int buffer_search_position = 0;
  int t_start_buffer_position = 0;
  int t_end_buffer_position = 0;
  int t_start = (int) req.t_start.toSec();
  int t_end = (int) req.t_end.toSec();
  bool t_start_found = false;
  bool t_end_found = false;
  std::vector<common::Odometry> odometry_buffer_frozen = buffer_odom;

  for(int i = 0; i < buffer_odom.size(); i++) {
    buffer_search_position = (int) odometry_buffer_frozen[i].ts.toSec();

    if(buffer_search_position == t_start) {
      t_start_buffer_position = i;
      t_start_found = true;
    }

    if(buffer_search_position == t_end) {
      t_end_buffer_position = i;
      t_end_found = true;
    }
  }

  if(t_start_found && t_end_found) {
    common::Pose2DWithCovariance t_start_pose = odometry_buffer_frozen[t_start_buffer_position].pose;
    common::Pose2DWithCovariance t_end_pose = odometry_buffer_frozen[t_end_buffer_position].pose;
    res.delta = pose_transform(t_start_pose, t_end_pose);
    return true;
  }

  return false;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "odometry");
  ros::NodeHandle n;

  ros::Subscriber vel_sub = n.subscribe("/cmd_vel", 1, vel_callback);

  ros::Publisher odom_pub = n.advertise<common::Odometry>("/odometry/odometry", 1);

  ros::Time current_time = ros::Time::now();
  ros::Time last_time = current_time;
  ros::Rate loop_rate(100);

  vx = 0.0;
  vy = 0.0;
  vth = 0.0;

  Delta_x = 0.0;
  Delta_y = 0.0;
  Delta_th = 0.0;

  while(ros::ok()) {
    current_time = ros::Time::now();
    double delta_t = ( current_time - last_time ).toSec();
    double delta_x = vx * delta_t;
    double delta_y = vy * delta_t;
    double delta_th = vth * delta_t;

    double Delta_th_cos = cos ( Delta_th );
    double Delta_th_sin = sin ( Delta_th );

    Delta_x += ( delta_x * Delta_th_cos ) - ( delta_y * Delta_th_sin );
    Delta_y += ( delta_y * Delta_th_sin ) + ( delta_y * Delta_th_cos );
    Delta_th += delta_th;
    Delta_th = std::fmod( Delta_th + M_PI, 2 * M_PI ) - M_PI;

    Eigen::Matrix3d J_D_D = Eigen::Matrix3d::Zero(3, 3);
    J_D_D(0, 0) = 1;
    J_D_D(0, 2) = ( -1 * delta_x * Delta_th_sin ) - ( delta_y * Delta_th_cos );
    J_D_D(1, 1) = 1;
    J_D_D(1, 2) = ( delta_x * Delta_th_cos ) - ( delta_y * Delta_th_sin );
    J_D_D(2, 2) = 1;

    Eigen::Matrix3d J_D_d = Eigen::Matrix3d::Zero(3, 3);
    J_D_d(0, 0) = Delta_th_cos;
    J_D_d(0, 1) = -1 * Delta_th_sin;
    J_D_d(1, 0) = Delta_th_sin;
    J_D_d(1, 1) = Delta_th_cos;
    J_D_d(2, 2) = 1;

    common::Odometry delta_odom;
    delta_odom.ts = current_time;
    delta_odom.pose = create_Pose2DWithCovariance_msg(delta_x, delta_y, delta_th, J_D_d);

    common::Odometry Delta_odom;
    Delta_odom.ts = current_time;
    Delta_odom.pose = create_Pose2DWithCovariance_msg(Delta_x, Delta_y, Delta_th, J_D_D);
    
    add_to_buffer(delta_odom);
    odom_pub.publish(Delta_odom);

    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::spin();
  return 0;
}
