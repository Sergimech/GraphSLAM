#include <graph.hpp>

gtsam::NonlinearFactorGraph graph;
gtsam::Values initial;
common::Pose2DWithCovariance pose_opt;
std::vector<common::Keyframe> keyframes; // JS: this vector will be continuously resized. Better use std::deque?
int keyframe_IDs;

void new_factor(common::Registration input) {
  input.keyframe_new.id = keyframe_IDs;
  input.factor_new.id_2 = keyframe_IDs++;
  ROS_INFO("NEW FACTOR ID=%d CREATION STARTED.", input.keyframe_new.id);

  common::Pose2DWithCovariance pose_new = compose(input.keyframe_last.pose_opti, input.factor_new.delta);

  input.keyframe_new.pose_opti = pose_new;
  keyframes.push_back(input.keyframe_new);
  
  initial.insert(input.keyframe_new.id, gtsam::Pose2(pose_new.pose.x, pose_new.pose.y, pose_new.pose.theta));

  Eigen::MatrixXd Q = covariance_to_eigen(input.factor_new);
  gtsam::noiseModel::Gaussian::shared_ptr delta_Model = gtsam::noiseModel::Gaussian::Covariance( Q );

  graph.add(gtsam::BetweenFactor<gtsam::Pose2>(input.factor_new.id_1,
					       input.factor_new.id_2,
					       gtsam::Pose2(input.factor_new.delta.pose.x,
							    input.factor_new.delta.pose.y,
							    input.factor_new.delta.pose.theta), delta_Model));
  ROS_INFO("NEW FACTOR ID=%d CREATION FINISHED.", input.keyframe_new.id);
}

void loop_factor(common::Registration input) {
  ROS_INFO("LOOP FACTOR ID_1=%d ID_2=%d CREATION STARTED.", input.factor_loop.id_1, input.factor_loop.id_1);
  Eigen::MatrixXd Q = covariance_to_eigen(input.factor_loop);
  gtsam::noiseModel::Gaussian::shared_ptr delta_Model = gtsam::noiseModel::Gaussian::Covariance( Q );
  graph.add(gtsam::BetweenFactor<gtsam::Pose2>(input.factor_loop.id_1,
					       input.factor_loop.id_2,
					       gtsam::Pose2(input.factor_loop.delta.pose.x,
							    input.factor_loop.delta.pose.y,
							    input.factor_loop.delta.pose.theta), delta_Model));
  ROS_INFO("LOOP FACTOR ID_1=%d ID_2=%d CREATION FINISHED.", input.factor_loop.id_1, input.factor_loop.id_1);
}

void solve() {
  ROS_INFO("SOLVE STARTED.");
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
  ROS_INFO("SOLVE FINISHED.");
}

bool last_keyframe(common::LastKeyframe::Request &req, common::LastKeyframe::Response &res) {
  ROS_INFO("LAST KEYFRAME SERVICE STARTED.");
  if(!keyframes.empty()) {
    res.keyframe_last = keyframes.back();
    ROS_INFO("LAST KEYFRAME ID=%d SERVICE FINISHED.", keyframes.back().id);
    return true;
  }

  ROS_INFO("LAST KEYFRAME SERVICE FINISHED. No keyframes available.");
  return false;
}

bool closest_keyframe(common::ClosestKeyframe::Request &req, common::ClosestKeyframe::Response &res) {
  ROS_INFO("CLOSEST KEYFRAME SERVICE STARTED.");
  if(!keyframes.empty()) {
    std::vector<double> distances;

    if(keyframes.size() > 10) {
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
      ROS_INFO("CLOSEST KEYFRAME ID=%d SERVICE FINISHED.", keyframes[minimum_keyframe_index].id);
      return true;
    } else {
      ROS_INFO("CLOSEST KEYFRAME SERVICE FINISHED. Not enought keyframes.");
      return false;
    }
  }

  ROS_INFO("CLOSEST KEYFRAME SERVICE FINISHED. No keyframes available.");
  return false;
}

void registration_callback(const common::Registration& input) {
  ROS_INFO("###REGISTRATION CALLBACK STARTED.###");
  if(!keyframes.empty()) {
    ROS_INFO("No. of Keyframes = %lu", keyframes.size() );
    ROS_INFO("ID of Last Keyframe = %d", keyframes.back().id);
    ROS_INFO("ts of Last Keyframe = %f", keyframes.back().ts.toSec());
    ROS_INFO("height+width of pointcloud of Last Keyframe = %u %u",
	     keyframes.back().pointcloud.height,
	     keyframes.back().pointcloud.width);
  }
  if(input.keyframe_flag) {
    new_factor(input);
  }

  if(input.loop_closure_flag) {
    loop_factor(input);
  }
  ROS_INFO("###REGISTRATION CALLBACK FINISHED.###");
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "graph");
  ros::NodeHandle n;

  keyframe_IDs = 0;

  Eigen::MatrixXd Q(3, 3);
  Q.Zero(3, 3);
  Q(0, 0) = 0.1;
  Q(1, 1) = 0.1;
  Q(2, 2) = 0.1;
  
  gtsam::noiseModel::Gaussian::shared_ptr priorNoise = gtsam::noiseModel::Gaussian::Covariance( Q );
  graph.push_back(gtsam::PriorFactor<gtsam::Pose2>(1, gtsam::Pose2(0, 0, 0), priorNoise));
  
  ros::Subscriber registration_sub = n.subscribe("/scanner/registration", 1, registration_callback);
  ros::ServiceServer last_keyframe_service = n.advertiseService("/graph/last_keyframe", last_keyframe);
  ros::ServiceServer closest_keyframe_service = n.advertiseService("/graph/closest_keyframe", closest_keyframe);

  ros::spin();
  return 0;
}
