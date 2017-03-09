#include <graph.hpp>
//#include <scanner/scanner.hpp>

gtsam::NonlinearFactorGraph graph;
gtsam::Values initial;
common::Pose2DWithCovariance pose_opt;
std::vector<common::Keyframe> keyframes; // JS: Better use map<key,Keyframe> where key = ID, as in gtsam::Values
int keyframe_IDs; // Simple ID factory.

// #### TUNING CONSTANTS START
double sigma_xy_prior = 0.1; // TODO migrate to rosparams
double sigma_th_prior = 0.1; // TODO migrate to rosparams
int keyframes_to_skip_in_loop_closing = 10; // TODO migrate to rosparams
// #### TUNING CONSTANTS END

void prior_factor(common::Registration input)
{
//	ROS_INFO("PRIOR FACTOR STARTED");
    // Advance keyframe ID factory
    keyframe_IDs++;

    // Define prior state and noise model
    double x_prior = 0;
    double y_prior = 0;
    double th_prior = 0;

    Eigen::MatrixXd Q(3, 3);
    Q.setZero();
    Q(0, 0) = sigma_xy_prior * sigma_xy_prior;
    Q(1, 1) = sigma_xy_prior * sigma_xy_prior;
    Q(2, 2) = sigma_th_prior * sigma_th_prior;

    gtsam::Pose2 pose_prior(x_prior, y_prior, th_prior);
    gtsam::noiseModel::Gaussian::shared_ptr noise_prior = gtsam::noiseModel::Gaussian::Covariance(Q);

    // Define new KF
    input.keyframe_new.id = keyframe_IDs;
    // input.keyframe_new.pose_odom = // TODO: get odometry pose from odometry_pose service.
    //input.keyframe_new.pose_opti = create_Pose2DWithCovariance_msg(x_prior, y_prior, th_prior, Q); // TODO fix this
    keyframes.push_back(input.keyframe_new);

    // Add factor and prior to the graph
    graph.add(gtsam::PriorFactor<gtsam::Pose2>(input.keyframe_new.id, pose_prior, noise_prior));
    initial.insert(input.keyframe_new.id, pose_prior);

    ROS_INFO("PRIOR FACTOR ID=%d CREATED. %lu KF, %lu Factor.", input.keyframe_new.id, keyframes.size(), graph.nrFactors());
}

void new_factor(common::Registration input)
{
//    ROS_INFO("NEW FACTOR ID=%d CREATION STARTED.", input.keyframe_new.id);

    // Advance keyframe ID factory
    keyframe_IDs++;

    // Compute new KF pose
    common::Pose2DWithCovariance pose_new_msg = compose(input.keyframe_last.pose_opti, input.factor_new.delta);
    gtsam::Pose2 pose_new(pose_new_msg.pose.x, pose_new_msg.pose.y, pose_new_msg.pose.theta);

    // Define new KF
    input.keyframe_new.id = keyframe_IDs;
    input.keyframe_new.pose_opti = pose_new_msg;
    // input.keyframe_new.pose_odom = // TODO: get odometry pose from odometry_pose service.
    keyframes.push_back(input.keyframe_new);

    // Define new factor
    input.factor_new.id_2 = input.keyframe_new.id;
    Eigen::MatrixXd Q = covariance_to_eigen(input.factor_new);
    gtsam::noiseModel::Gaussian::shared_ptr noise_delta = gtsam::noiseModel::Gaussian::Covariance(Q);

    // Add factor and state to the graph
    initial.insert(input.keyframe_new.id, pose_new);
    graph.add(gtsam::BetweenFactor<gtsam::Pose2>(input.factor_new.id_1,
                                                 input.factor_new.id_2,
                                                 gtsam::Pose2(input.factor_new.delta.pose.x,
                                                              input.factor_new.delta.pose.y,
                                                              input.factor_new.delta.pose.theta),
                                                 noise_delta));

    ROS_INFO("NEW FACTOR %d-->%d CREATED. %lu KFs, %lu Factors", input.keyframe_last.id, input.keyframe_new.id, keyframes.size(), graph.nrFactors());
}

void loop_factor(common::Registration input)
{
//    ROS_INFO("LOOP FACTOR %d-->%d STARTED.", input.factor_loop.id_1, input.factor_loop.id_2);

    // Define new factor
    Eigen::MatrixXd Q = covariance_to_eigen(input.factor_loop);
    gtsam::noiseModel::Gaussian::shared_ptr noise_delta = gtsam::noiseModel::Gaussian::Covariance(Q);

    // Add factor to the graph
    graph.add(gtsam::BetweenFactor<gtsam::Pose2>(input.factor_loop.id_1,
                                                 input.factor_loop.id_2,
                                                 gtsam::Pose2(input.factor_loop.delta.pose.x,
                                                              input.factor_loop.delta.pose.y,
                                                              input.factor_loop.delta.pose.theta),
                                                 noise_delta));
    ROS_INFO("LOOP FACTOR %d-->%d CREATED. %lu KFs, %lu Factors", input.factor_loop.id_1, input.factor_loop.id_2, keyframes.size(), graph.nrFactors());
}

void solve() {
  ROS_INFO("SOLVE STARTED.");
  //graph.print();
  //initial.print();
  gtsam::Values poses_opti = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();
  gtsam::Marginals marginals(graph, poses_opti);

  for(int i = 0; i < keyframes.size(); i++) {
    keyframes[i].pose_opti.pose.x = poses_opti.at<gtsam::Pose2>(keyframes[i].id).x();
    keyframes[i].pose_opti.pose.y = poses_opti.at<gtsam::Pose2>(keyframes[i].id).y();
    keyframes[i].pose_opti.pose.theta = poses_opti.at<gtsam::Pose2>(keyframes[i].id).theta();
    Eigen::MatrixXd pose_opti_covariance = marginals.marginalCovariance(keyframes[i].id);
    keyframes[i].pose_opti = eigen_to_covariance(keyframes[i].pose_opti, pose_opti_covariance);
  }

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

    if(keyframes.size() > keyframes_to_skip_in_loop_closing) {
      for(int i = 0; i < keyframes.size() - keyframes_to_skip_in_loop_closing; i++) {
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
      ROS_INFO("CLOSEST KEYFRAME SERVICE FINISHED. Not enough keyframes.");
      return false;
    }
  }

  ROS_INFO("CLOSEST KEYFRAME SERVICE FINISHED. No keyframes available.");
  return false;
}

void registration_callback(const common::Registration& input) {
  ROS_INFO("###REGISTRATION CALLBACK STARTED.###");

  if(input.first_frame_flag) {
      prior_factor(input);
  }

  else if(input.keyframe_flag) {
      new_factor(input);

      if(input.loop_closure_flag) {
          loop_factor(input);
      }
      solve();
  }

  ROS_INFO("###REGISTRATION CALLBACK FINISHED.###");
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "graph");
  ros::NodeHandle n;
  
  // Init ID factory
  keyframe_IDs = 0;

  ros::Subscriber registration_sub = n.subscribe("/scanner/registration", 1, registration_callback);
  ros::ServiceServer last_keyframe_service = n.advertiseService("/graph/last_keyframe", last_keyframe);
  ros::ServiceServer closest_keyframe_service = n.advertiseService("/graph/closest_keyframe", closest_keyframe);

  ros::spin();
  return 0;
}
