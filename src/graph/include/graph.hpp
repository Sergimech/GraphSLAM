#include <vector>
#include <math.h>
#include <iostream>

#include <ros/ros.h>

#include <common/Factor.h>
#include <common/Keyframe.h>
#include <common/ClosestKeyframe.h>
#include <common/LastKeyframe.h>
#include <common/Registration.h>
#include <common/Pose2DWithCovariance.h>
#include <common/OdometryBuffer.h>

#include <gtsam/inference/Key.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

common::Pose2DWithCovariance compose(common::Pose2DWithCovariance input_1, common::Pose2DWithCovariance input_2) {
  common::Pose2DWithCovariance output;
  double input_1_th = input_1.pose.theta;
  double input_2_th = input_2.pose.theta;
  double cos_th = cos( input_1_th );
  double sin_th = sin( input_1_th );
  double dx = input_2.pose.x - input_1.pose.x;
  double dy = input_2.pose.y - input_1.pose.y;
  double dth = input_2_th - input_1_th;
  dth = std::fmod(dth + M_PI, 2 * M_PI) - M_PI;
  output.pose.x = ( cos_th * dx ) + ( sin_th * dy );
  output.pose.y = ( -sin_th * dx ) + ( cos_th *dy );
  output.pose.theta = dth;

  return output;
}
