#include <vector>

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

gtsam::NonlinearFactorGraph graph;
gtsam::Values initial;
common::Pose2DWithCovariance pose_opt;
std::vector<common::Registration> registrations;

int keyframe_IDs;

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

void new_factor(common::Registration input) {
  input.keyframe_new.id_2 = keyframe_IDs++;
  registrations.push_back(input);
  gtsam::noiseModel::Diagonal::shared_ptr delta_Model =
    gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) <<
					 input.factor_new.delta.covariance[0],
					 input.factor_new.delta.covariance[4],
					 input.factor_new.delta.covariance[9]));

  common::Pose2DWithCovariance pose_new = compose(input.keyframe_new.pose_opti, input.factor_new.delta);
  
  initial.insert(input.id_2,
		 gtsam::Pose2(pose_new.pose.x,
			      pose_new.pose.y,
			      pose_new.pose.theta));
  
  graph.push_back(gtsam::BetweenFactor<gtsam::Pose2>(input.id_1,
						     input.id_2,
						     gtsam::Pose2(input.factor_new.delta.pose.x,
								  input.factor_new.delta.pose.y,
								  input.factor_new.delta.pose.theta), delta_Model));
}

void loop_factor(common::Registrationn input) {
  gtsam::noiseModel::Diagonal::shared_ptr delta_Model =
    gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) <<
					 input.factor_loop.delta.covariance[0],
					 input.factor_loop.delta.covariance[4],
					 input.factor_loop.delta.covariance[9]));
  
  graph.add(gtsam::BetweenFactor<gtsam::Pose2>(input.id_1,
					       input.id_2,
					       gtsam::Pose2(input.factor_loop.delta.pose.x,
							    input.factor_loop.delta.pose.y,
							    input.factor_loop.delta.pose.theta), delta_Model));
}

bool last_keyframe(common::LastKeyframe::Request &req, common::LastKeyframe::Response &res) {
  if(!registrations.empty()) {
    res.keyframe_last = registrations[registrations.size() - 1].keyframe_new;
    return true;
  }
  
  return false;
}

bool closest_keyframe(common::ClosestKeyframe::Request &req, common::ClosestKeyframe::Response &res) {
  std::vector<double> distances;

  for(int i = 0; i < registrations.size() - 10; i++) {
    double x1 = req.;
    double x2 = ;
    double y1 = ;
    double y2 = ;
    distances.push_back(sqrt( pow( )))
  }
  
  return true;
}

void registration_callback(const common::Registration& input) {
  if(input.keyframe_flag) {
    new_factor(input);
  }

  if(input.loop_closure_flag) {
    loop_factor(input);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "graph");
  ros::NodeHandle n;

  keyframe_IDs = 0;

  gtsam::Diagonal::shared_ptr priorNoise priorNoise =
    noiseModel::Diagonal::Sigmas((gtsam::Vector(3) <<
				  0.3,
				  0.3,
				  0.1));
  graph.push_back(gtsam::PriorFactor<gtsam::Pose2>(1,
						   gtsam::Pose2(0, 0, 0),
						   priorNoise));
  
  ros::Subscriber registration_sub = n.subscribe("/scanner/registration", 1, registration_callback);
  ros::ServiceServer last_keyframe_service = n.advertiseService("/graph/last_keyframe", last_keyframe);

  ros::spin();
  return 0;
}
