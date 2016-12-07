#include "scanmatcher.h"
#include <iostream>
#include <stdlib.h>
#include <cmath>
#include <math.h>

ScanMatcher::ScanMatcher() {
  new_pose_t.setIdentity();
  //
  input.laser[0] = 0.0;
  input.laser[1] = 0.0;
  input.laser[2] = 0.0;
  // General input parameters
  input.sigma = 0.01;
  input.use_corr_tricks = 1;
  input.restart = 0;
  input.restart_threshold_mean_error = 0.01;
  input.restart_dt = 1.0;
  input.restart_dtheta = 0.1;
  input.clustering_threshold = 0.25;
  input.orientation_neighbourhood = 20;
  input.use_point_to_line_distance = 1;
  input.do_alpha_test = 0;
  input.do_alpha_test_thresholdDeg = 20.0;
  input.outliers_maxPerc = 0.90;
  input.outliers_adaptive_order = 0.7;
  input.outliers_adaptive_mult = 2.0;
  input.do_visibility_test = 0;
  input.outliers_remove_doubles = 1;
  input.do_compute_covariance = 1;
  input.debug_verify_tricks = 0;
  input.use_ml_weights = 0;
  input.use_sigma_weights = 0;
  //
  min_laser_range = 0.05;
  max_laser_range = 4.5;
}
;

double ScanMatcher::convertScantoDLP(sensor_msgs::LaserScan& scan, LDP& ldp){
  unsigned int numberOfScans = scan.ranges.size();
  ldp = ld_alloc_new(numberOfScans);
  double invalid_scans = 0;
  //
  for(unsigned int i = 0; i < numberOfScans; i++) {
    //Set range to -1 if if it exceeds the bounds of the laser scanner.
    double range = scan.ranges[i];
    if(range > min_laser_range && range < max_laser_range) {
      ldp->valid[i] = 1;
      ldp->readings[i] = range;
    } else {
      ldp->valid[i] = 0;
      ldp->readings[i] = -1;
      invalid_scans++;
    }
    //Set angle
    ldp->theta[i] = scan.angle_min + i * scan.angle_increment;
    ldp->cluster[i] = -1;
  }
  //
  ldp->min_theta = ldp->theta[0];
  ldp->max_theta = ldp->theta[numberOfScans - 1];
  //
  ldp->odometry[0] = 0.0;
  ldp->odometry[1] = 0.0;
  ldp->odometry[2] = 0.0;
  ldp->true_pose[0] = 0.0;
  ldp->true_pose[1] = 0.0;
  ldp->true_pose[2] = 0.0;
  ldp->estimate[0] = 0.0;
  ldp->estimate[1] = 0.0;
  ldp->estimate[2] = 0.0;
  return invalid_scans / ((double) numberOfScans);
}
;

bool ScanMatcher::processScan(LDP& ldp, LDP& ref_ldp, double change_x, double change_y, double change_theta, double mean[], double covariance[][3], double outp[], double& error){
  input.laser_ref = ref_ldp;
  input.laser_sens = ldp;
  //
  tf::Transform change_t;
  createTfFromXYTheta(change_x, change_y, change_theta, change_t);
  change_t = change_t * (new_pose_t * ref_pose_t.inverse());
  //Set initial estimate of input
  input.first_guess[0] = change_t.getOrigin().getX();
  input.first_guess[1] = change_t.getOrigin().getY();
  input.first_guess[2] = tf::getYaw(change_t.getRotation()); 
  // Scan matching by ICP
  sm_icp(&input, &output);
  if(output.valid) {
    // Raw output
    outp[0] = output.x[0];
    outp[1] = output.x[1];
    outp[2] = output.x[2];
    //
    error = output.error;
    //
    tf::Transform output_t;
    createTfFromXYTheta(output.x[0], output.x[1], output.x[2], output_t);
    new_pose_t = ref_pose_t * output_t;
    //Set the new pose determined by the matching
    mean[0] = new_pose_t.getOrigin().getX();
    mean[1] = new_pose_t.getOrigin().getY();
    mean[2] = tf::getYaw(new_pose_t.getRotation());
    //
    // ROS_INFO("Error %2.4f, nvalid %d", error, output.nvalid);
    //
    if(input.do_compute_covariance == 1) {
      //Set covariance
      unsigned int rows = output.cov_x_m->size1, cols = output.cov_x_m->size2;
      for(unsigned int i = 0; i < cols; i++) {
        for(unsigned int j = 0; j < rows; j++) {
          covariance[i][j] = gsl_matrix_get(output.cov_x_m, i, j);
        }
      }
    }
  }
  //
  ld_free(ref_ldp);
  ld_free(ldp);
  //Return
  if(output.valid) {
    return true;
  } else {
    return false;
  }
}
;

void ScanMatcher::createTfFromXYTheta(double x, double y, double theta, tf::Transform& t) {
  t.setOrigin(tf::Vector3(x, y, 0.0));
  tf::Quaternion q;
  q.setRPY(0.0, 0.0, theta);
  t.setRotation(q);
}
;

bool ScanMatcher::graphScanMatch(LaserScan& scan_to_match, GraphPose& new_pose, LaserScan& reference_scan, GraphPose& ref_pose, double mean[3], double covariance[][3], double outp[], double& error) {
  LDP ref_ldp;
  convertScantoDLP(reference_scan, ref_ldp);
  LDP current_ldp;
  convertScantoDLP(scan_to_match, current_ldp);
  // ROS_INFO("Invalid: %f", invalid_rate);
  // Transforms for the new pose and reference pose
  // createTfFromXYTheta(new_pose.x, new_pose.y, new_pose.theta, new_pose_t);
  new_pose_t.setIdentity();
  createTfFromXYTheta(ref_pose.x, ref_pose.y, ref_pose.theta, ref_pose_t);
  // All scans should be between this interval
  input.min_reading = min_laser_range;
  input.max_reading = max_laser_range;
  // Allow more distance grom the solution as the scan-matching distance is higher
  input.max_iterations = 15;
  input.epsilon_xy = 0.000001;
  input.epsilon_theta = 0.000001;
  input.do_compute_covariance = 1;
  input.max_angular_correction_deg = 100.0;
  input.max_linear_correction = .8;
  input.max_correspondence_dist = .8;
  
  //Calculate change in position
  double dx = new_pose.x - ref_pose.x;
  double dy = new_pose.y - ref_pose.y;
  double dt = new_pose.theta - ref_pose.theta;
  //
  if (dt >= PI) {
      dt -= 2 * PI;
  } else if (dt < -PI) {
      dt += 2 * PI;
  }
  bool result = processScan(current_ldp, ref_ldp, dx, dy, dt, mean, covariance, outp, error);
  return result;
};

bool ScanMatcher::scanMatch(LaserScan& scan_to_match, double change_x, double change_y, double change_theta, GraphPose& prev_pose, LaserScan& reference_scan, GraphPose& ref_pose, double mean[3], double& error) {
  LDP ref_ldp;
  convertScantoDLP(reference_scan, ref_ldp);
  LDP current_ldp;
  convertScantoDLP(scan_to_match, current_ldp);
  // Transforms for the new pose and reference pose
  // createTfFromXYTheta(prev_pose.x, prev_pose.y, prev_pose.theta, new_pose_t);
  createTfFromXYTheta(ref_pose.x, ref_pose.y, ref_pose.theta, ref_pose_t);
  // All scans should be between this interval
  input.min_reading = min_laser_range;
  input.max_reading = max_laser_range;
  input.max_iterations = 15;
  input.epsilon_xy = 0.0000001;
  input.epsilon_theta = 0.0000001;
  input.do_compute_covariance = 0;
  input.max_angular_correction_deg = 30.0;
  input.max_linear_correction = 0.3;
  input.max_correspondence_dist = 0.5;
  //
  double covariance[3][3], output[3];
  bool result = processScan(current_ldp, ref_ldp, change_x, change_y, change_theta, mean, covariance, output, error);
  return result;
};