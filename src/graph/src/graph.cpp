#include <graph.hpp>

gtsam::NonlinearFactorGraph graph;
gtsam::Values initial;
common::Pose2DWithCovariance pose_opt;
std::vector<common::Keyframe> keyframes; // JS: this vector will be continuously resized. Better use std::deque?
int keyframe_IDs;

void new_factor(common::Registration input) {
  input.factor_new.id_2 = keyframe_IDs;
  input.keyframe_new.id = keyframe_IDs++;

  Eigen::MatrixXd Q(3, 3);
  Q << input.factor_new.delta.covariance[0],
    input.factor_new.delta.covariance[1],
    input.factor_new.delta.covariance[2],
    input.factor_new.delta.covariance[3],
    input.factor_new.delta.covariance[4],
    input.factor_new.delta.covariance[5],
    input.factor_new.delta.covariance[6],
    input.factor_new.delta.covariance[7],
    input.factor_new.delta.covariance[8];
  gtsam::noiseModel::Gaussian::shared_ptr delta_Model = gtsam::noiseModel::Gaussian::Covariance( Q );

  common::Pose2DWithCovariance pose_new = compose(input.keyframe_last.pose_opti, input.factor_new.delta);

  keyframes.push_back(input.keyframe_new);

  initial.insert(input.factor_new.id_2,
		 gtsam::Pose2(pose_new.pose.x,
			      pose_new.pose.y,
			      pose_new.pose.theta));
  
  graph.add(gtsam::BetweenFactor<gtsam::Pose2>(input.factor_new.id_1,
					       input.factor_new.id_2,
					       gtsam::Pose2(input.factor_new.delta.pose.x,
							    input.factor_new.delta.pose.y,
							    input.factor_new.delta.pose.theta), delta_Model));
}

void loop_factor(common::Registration input) {
  Eigen::MatrixXd Q(3, 3);
  Q << input.factor_new.delta.covariance[0],
    input.factor_new.delta.covariance[1],
    input.factor_new.delta.covariance[2],
    input.factor_new.delta.covariance[3],
    input.factor_new.delta.covariance[4],
    input.factor_new.delta.covariance[5],
    input.factor_new.delta.covariance[6],
    input.factor_new.delta.covariance[7],
    input.factor_new.delta.covariance[8];
  gtsam::noiseModel::Gaussian::shared_ptr delta_Model = gtsam::noiseModel::Gaussian::Covariance( Q );
  graph.add(gtsam::BetweenFactor<gtsam::Pose2>(input.factor_loop.id_1,
					       input.factor_loop.id_2,
					       gtsam::Pose2(input.factor_loop.delta.pose.x,
							    input.factor_loop.delta.pose.y,
							    input.factor_loop.delta.pose.theta), delta_Model));

}

void solve() {
  gtsam::Values poses_opti = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();
  poses_opti.print();
  gtsam::Marginals marginals(graph, poses_opti);
  marginals.print();

  for(int i = 0; i < keyframes.size(); i++) {
    //keyframes[i].pose_opti.pose = poses_opti.at(keyframes[i].id);
    //keyframes[i].pose_opti.covariance = marginals.marginalCovariance(keyframes[i].id);
  }

  // JS: make initial take the last solution, so that next iteration is simpler:
  initial = poses_opti;
}

bool last_keyframe(common::LastKeyframe::Request &req, common::LastKeyframe::Response &res) {
  if(!keyframes.empty()) {
    res.keyframe_last = keyframes.back();
    return true;
  }
  
  return false;
}

bool closest_keyframe(common::ClosestKeyframe::Request &req, common::ClosestKeyframe::Response &res) {
  if(!keyframes.empty()) {
    std::vector<double> distances;

    for(int i = 0; i < keyframes.size() - 10; i++) {
      double x1 = req.keyframe_last.pose_opti.pose.x;
      double y1 = req.keyframe_last.pose_opti.pose.y;
      double x2 = keyframes[i].pose_opti.pose.x;
      double y2 = keyframes[i].pose_opti.pose.y;
      distances.push_back( sqrt( pow( x2 - x1, 2 ) + pow( y2 - y1, 2 ) ) );
    }

    int minimum_keyframe_index = 0;
    for(int i = 0; i < distances.size(); i++) {
      if(distances[i] < distances[minimum_keyframe_index]) {
	minimum_keyframe_index = i;
      }
    }

    res.keyframe_closest = keyframes[minimum_keyframe_index];
    return true;
  }
  
  return false;
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

  gtsam::noiseModel::Gaussian::shared_ptr priorNoise =
    gtsam::noiseModel::Gaussian::Covariance((gtsam::Vector(3) << 0.3, 0.3, 0.1));
  graph.push_back(gtsam::PriorFactor<gtsam::Pose2>(1, gtsam::Pose2(0, 0, 0), priorNoise));
  
  ros::Subscriber registration_sub = n.subscribe("/scanner/registration", 1, registration_callback);
  ros::ServiceServer last_keyframe_service = n.advertiseService("/graph/last_keyframe", last_keyframe);
  ros::ServiceServer closest_keyframe_service = n.advertiseService("/graph/closest_keyframe", closest_keyframe);

  ros::spin();
  return 0;
}
