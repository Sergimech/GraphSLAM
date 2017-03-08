#include <scanner.hpp>

ros::Publisher registration_pub;
ros::Publisher pointcloud_debug_pub;
ros::ServiceClient keyframe_last_client;
ros::ServiceClient keyframe_closest_client;

// Tuning constants:
const double converged_fitness_threshold = 0.15; // TODO migrate to rosparams
double k_disp_disp = 0.1, k_rot_disp = 0.1, k_rot_rot = 0.1; // TODO migrate to rosparams
int keyframe_IDs;

sensor_msgs::PointCloud2 scan_to_pointcloud(sensor_msgs::LaserScan input) {
  ROS_INFO("SCAN TO POINTCLOUD STARTED.");
  laser_geometry::LaserProjection projector;
  sensor_msgs::PointCloud2 output;
  projector.projectLaser(input, output);

  ROS_INFO("SCAN TO POINTCLOUD FINISHED.");
  return output;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr format_pointcloud(sensor_msgs::PointCloud2 input) {
  ROS_INFO("FORMAT POINTCLOUD STARTED.");
  pcl::PCLPointCloud2 pcl2_pointcloud;
  pcl_conversions::toPCL(input, pcl2_pointcloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl2_pointcloud, *output);
  
  ROS_INFO("FORMAT POINTCLOUD FINISHED.");
  return output;
}

common::Registration gicp(sensor_msgs::PointCloud2 input_1, sensor_msgs::PointCloud2 input_2) {
  ROS_INFO("GICP STARTED.");
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_1 = format_pointcloud(input_1);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_2 = format_pointcloud(input_2);
  
  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;

  gicp.setInputSource(pointcloud_1);
  gicp.setInputTarget(pointcloud_2);

  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_transform(new pcl::PointCloud<pcl::PointXYZ>);
  gicp.align(*pointcloud_transform);

  bool converged = gicp.hasConverged();
  double converged_fitness = gicp.getFitnessScore();
  Eigen::Matrix4f transform = gicp.getFinalTransformation();
  common::Registration output;

  if(converged) {
    if(converged_fitness > converged_fitness_threshold) {
      geometry_msgs::Pose2D transform_Delta = make_Delta(transform);
      Eigen::MatrixXd covariance_Delta = compute_covariance(k_disp_disp, k_rot_disp, k_rot_rot, transform_Delta);
      common::Pose2DWithCovariance Delta = create_Pose2DWithCovariance_msg(transform_Delta, covariance_Delta);
    
      output.factor_new.delta = Delta;
      output.factor_loop.delta = Delta;
      output.keyframe_flag = true;
    } else {
      output.keyframe_flag = false;
    }
  } else {
    output.keyframe_flag = false;
  }

  ROS_INFO("GICP FINISHED.");
  return output;
}

void scanner_callback(const sensor_msgs::LaserScan& input) {
  common::LastKeyframe keyframe_last_request;
  bool keyframe_last_request_returned = keyframe_last_client.call(keyframe_last_request);

  if(keyframe_last_request_returned) {
    common::Registration output;
    sensor_msgs::PointCloud2 input_pointcloud = scan_to_pointcloud(input);
    sensor_msgs::PointCloud2 keyframe_last_pointcloud = keyframe_last_request.response.keyframe_last.pointcloud;
    keyframe_IDs = keyframe_last_request.response.keyframe_last.id + 1;
    
    ROS_INFO("GICP registration_last STARTED");
    common::Registration registration_last = gicp(input_pointcloud, keyframe_last_pointcloud);
    ROS_INFO("GICP registration_last FINISHED");
    
    output.keyframe_flag = registration_last.keyframe_flag;
    output.loop_closure_flag = false;
    output.keyframe_new.id = keyframe_IDs;
    output.keyframe_new.ts = input.header.stamp;
    output.factor_new.id_1 = keyframe_last_request.response.keyframe_last.id;
    output.factor_new.id_2 = output.keyframe_new.id;
    output.keyframe_new.pointcloud = input_pointcloud;
    output.factor_new.delta = registration_last.factor_new.delta;
          
    common::ClosestKeyframe keyframe_closest_request;
    keyframe_closest_request.request.keyframe_last = keyframe_last_request.response.keyframe_last;
    bool keyframe_closest_request_returned = keyframe_closest_client.call(keyframe_closest_request);

    if(keyframe_closest_request_returned) {
      sensor_msgs::PointCloud2 keyframe_closest_pointcloud =
	keyframe_closest_request.response.keyframe_closest.pointcloud;
      ROS_INFO("GICP registration_closest STARTED");
      common::Registration registration_closest = gicp(keyframe_closest_pointcloud, keyframe_last_pointcloud);
      ROS_INFO("GICP registration_closest FINISHED");

      ROS_INFO("factor_loop.id_1 = %d, factor_loop.id_2 = %d",
	       keyframe_last_request.response.keyframe_last.id,
	       keyframe_IDs);
      output.factor_loop.id_1 = keyframe_last_request.response.keyframe_last.id;
      output.factor_loop.id_2 = keyframe_IDs;
      output.factor_loop.delta = registration_closest.factor_loop.delta;
      output.loop_closure_flag = registration_closest.keyframe_flag;
    }
    
    registration_pub.publish(output);
  }

  if(!keyframe_last_request_returned) {
    common::Registration output;
    sensor_msgs::PointCloud2 input_pointcloud = scan_to_pointcloud(input);
    output.keyframe_new.id = keyframe_IDs;
    output.keyframe_flag = true;
    output.loop_closure_flag = false;
    output.keyframe_new.pointcloud = input_pointcloud;
    registration_pub.publish(output);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "scanner");
  ros::NodeHandle n;

  ros::Subscriber scanner_sub = n.subscribe("/base_scan", 1, scanner_callback);
  keyframe_IDs = 1;

  registration_pub  = n.advertise<common::Registration>("/scanner/registration", 1);
  pointcloud_debug_pub = n.advertise<sensor_msgs::PointCloud2>("/scanner/debug_pointcloud", 1);

  keyframe_last_client = n.serviceClient<common::LastKeyframe>("/graph/last_keyframe");
  keyframe_closest_client = n.serviceClient<common::ClosestKeyframe>("/graph/closest_keyframe");

  ros::spin();
  return 0;
}
