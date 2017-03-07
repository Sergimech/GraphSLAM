#include <math.h>
#include <vector>
#include <deque>

#include <Eigen/Core>

#include <ros/ros.h>

#include <common/Odometry.h>
#include <common/Pose2DWithCovariance.h>
#include <common/OdometryBuffer.h>

#include <geometry_msgs/Twist.h>

// JS: David, I guess this file is the result of a copypaste from HW1.
// There are many things not needed, and I would like to use the logic explained in the Drive document
// Therefore I have put many messages here, with indications.
// Please feel free to consult me

double vx, vy, vth;
double k_d_d = 0.1, k_r_d = 0.1, k_r_r = 0.1; // TODO migrate to rosparams
//, Delta_x, Delta_y, Delta_th; // JS: We do not need Delta
std::deque<common::Odometry> buffer_odom; // JS: changed vector --> deque for an efficient pop_front().

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
//    -  It should be placed in package 'common'
//    -  and also renamed to 'between(pose1, pose2)'
//    -  see the document in Drive !!
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
    // buffer_odom.clear(); // JS: clear() is not a good policy. Use a deque for the buffer and use pop_front() instead.
      buffer_odom.pop_front();
  }

  buffer_odom.push_back(input);
}

// JS: Rename and adjust API to 'between(keyframe , keyframe)' as in the document
// JS: or, otherwise, just leave as a buffer request, in such case return only one odometry instance, not the increment between two instances.
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
    res.delta = pose_transform(t_start_pose, t_end_pose); // JS: we want one pose returned, not the pose increment
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

  // JS: we are not using Delta
//  Delta_x = 0.0;
//  Delta_y = 0.0;
//  Delta_th = 0.0;

  while(ros::ok()) {
    current_time = ros::Time::now();
    double delta_t = ( current_time - last_time ).toSec();
    double delta_x = vx * delta_t;
    double delta_y = vy * delta_t;
    double delta_th = vth * delta_t;

    // JS: we are not using Delta
//    double Delta_th_cos = cos ( Delta_th );
//    double Delta_th_sin = sin ( Delta_th );

    // JS: we are not using Delta
//    Delta_x += ( delta_x * Delta_th_cos ) - ( delta_y * Delta_th_sin );
//    Delta_y += ( delta_y * Delta_th_sin ) + ( delta_y * Delta_th_cos );
//    Delta_th += delta_th;
//    Delta_th = std::fmod( Delta_th + M_PI, 2 * M_PI ) - M_PI;

    // JS: we are not integrating covariances through Jacobians, but through constants k_d_d, k_r_d and k_r_r.
//    Eigen::Matrix3d J_D_D = Eigen::Matrix3d::Zero(3, 3);
//    J_D_D(0, 0) = 1;
//    J_D_D(0, 2) = ( -1 * delta_x * Delta_th_sin ) - ( delta_y * Delta_th_cos );
//    J_D_D(1, 1) = 1;
//    J_D_D(1, 2) = ( delta_x * Delta_th_cos ) - ( delta_y * Delta_th_sin );
//    J_D_D(2, 2) = 1;

    // JS: we are not integrating covariances through Jacobians, but through constants k_d_d, k_r_d and k_r_r.
//    Eigen::Matrix3d J_D_d = Eigen::Matrix3d::Zero(3, 3);
//    J_D_d(0, 0) = Delta_th_cos;
//    J_D_d(0, 1) = -1 * Delta_th_sin;
//    J_D_d(1, 0) = Delta_th_sin;
//    J_D_d(1, 1) = Delta_th_cos;
//    J_D_d(2, 2) = 1;

    // JS: we are not publishing delta
//    common::Odometry delta_odom;
//    delta_odom.ts = current_time;
//    delta_odom.pose = create_Pose2DWithCovariance_msg(delta_x, delta_y, delta_th, J_D_d);

    // JS: we are not using Delta
//    common::Odometry Delta_odom;
//    Delta_odom.ts = current_time;
//    Delta_odom.pose = create_Pose2DWithCovariance_msg(Delta_x, Delta_y, Delta_th, J_D_D);

    // JS: we need to integrate the pose, of type Pose2DWithCovariance:
    // JS: we need to create a Odometry message with the integrated pose and the timestamp, consult the Drive document
    // JS: we need to publish the last pose, of type Pose2D, with (x, y, th)
    // JS: proceed as follows (consult the Drive doc, and use compose() for the integration of 'delta' onto the pose (x,y,th).
    //   - Integrate pose (below a copypaste from HW1):
    //    //### Integrate Global Pose ###
    //    x  += delta_x * cos ( th ) - delta_y * sin ( th );
    //    y  += delta_x * sin ( th ) + delta_y * cos ( th );
    //    th += delta_th;
    //    th  = std::fmod( th + M_PI, 2 * M_PI) - M_PI;
    //   - Compute covariance, using k_d_d, k_r_d, and k_r_r as in scanner.cpp
    //   - Construct odometry message
    //   - Bufferize odometry message
    //   - Publish Pose2D(x,y,z)


    // JS: we are not buffering 'delta'
//    add_to_buffer(delta_odom);
    
    //JS: we are not publishing 'Delta'
//    odom_pub.publish(Delta_odom);

    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::spin();
  return 0;
}
