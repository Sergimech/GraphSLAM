#include "sm_node.h"
//
SMNode::SMNode(ros::NodeHandle& nh) {
	//
	pose_publisher = nh.advertise<graphslam::pose_laser>("pose_laser", 1);
	last_pose_publisher = nh.advertise<PoseStamped>("last_pose", 1);
	//
	first_scan = true;
	// Set the initial pose for the graph and scanmatcher to 0,0,0
	cur_sm_pose.x = 0.;
	cur_sm_pose.y = 0.;
	cur_sm_pose.theta = 0;
	prev_sm_pose = cur_sm_pose;
	est_sm_pose = cur_sm_pose;
	//
	sm_pose_updated = true;
	sm_odom_updated = false;
	//
	if(!nh.hasParam("sm_node/scan_topic"))
    	ROS_WARN("No param named 'sm_node/scan_topic'");

    nh.param("sm_node/max_laser_range", max_laser_range, 4.5);
    nh.param("sm_node/min_laser_range", min_laser_range, 0.1);
    //
    matcher.min_laser_range = min_laser_range;
    matcher.max_laser_range = max_laser_range;

	if(!nh.getParam("sm_node/scan_topic", scan_topic))
    	scan_topic = "base_scan";
	// Subscribe to odom an laser scan messages
	laserScan_Sub = nh.subscribe(scan_topic, 1, &SMNode::laserScan_callback, this);
	odometry_Sub = nh.subscribe("odom", 1, &SMNode::odom_callback, this);
	//
	// set up parent and child frames
	tf_map_to_odom_.frame_id_ = std::string("map");
	tf_map_to_odom_.child_frame_id_ = std::string("odom");
}
;

double dx = 0, dy = 0, dt = 0;
void SMNode::laserScan_callback(const LaserScan::ConstPtr& msg){	
	// ROS_INFO("Graphslam scanmatching!");
	// Check if we will perform scanmatching
	if(!first_scan) {
		double mean[3], error;
		LaserScan scan = *msg, ref_scan = *cur_sm_scan;
		bool result = matcher.scanMatch(scan, dx, dy, dt, cur_sm_pose, ref_scan, prev_sm_pose, mean, error);		
		// ROS_INFO("SM Error %f", error);
		if(result) {
			cur_sm_pose.x = mean[0];
			cur_sm_pose.y = mean[1];
			cur_sm_pose.theta = mean[2];
		} else {
			// In this case, cur_sm_pose will have the value updated by the odometry
			// ROS_WARN("Scanmatching in GraphSlam failed.");
			cur_sm_pose = est_sm_pose;
			ROS_WARN("Using estimated pose: x %f y: %f t: %f", cur_sm_pose.x, cur_sm_pose.y, cur_sm_pose.theta);
		}
		//
		cur_msg.pose.header.stamp = ros::Time().now();
		cur_msg.pose.header.frame_id = "/map";
		cur_msg.pose.pose.position.x = cur_sm_pose.x;
		cur_msg.pose.pose.position.y = cur_sm_pose.y;
		cur_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(cur_sm_pose.theta);
		cur_msg.scan = *msg;
		// Check if we should update the reference scan to a new frame
		if(distance(cur_sm_pose.x, prev_sm_pose.x, cur_sm_pose.y, prev_sm_pose.y) > 0.1 
			|| abs(rot_distance(cur_sm_pose.theta, prev_sm_pose.theta)) > 0.15) {
			prev_sm_pose = cur_sm_pose;
			cur_sm_scan = msg;
		}
		prev_sm_odom = cur_odom;

		// Publish the lase scanmatching pose
		pose_publisher.publish(cur_msg);
		last_pose_publisher.publish(cur_msg.pose);
		//
		if(sm_odom_updated) {
			tf_map_to_odom_.stamp_ = ros::Time::now();
			tf_map_to_odom_.setOrigin(tf::Vector3(cur_sm_pose.x - cur_odom.position.x, cur_sm_pose.y - cur_odom.position.y, 0));
			double theta = rot_distance(cur_sm_pose.theta, tf::getYaw(cur_odom.orientation));
			tf_map_to_odom_.setRotation(tf::createQuaternionFromYaw(theta));
			tf_br_.sendTransform(tf::StampedTransform(tf_map_to_odom_, ros::Time::now(), "map", "odom"));
		}
	}
	// Store the first reference scan for later use
	if(first_scan) {
		first_scan = false;
		cur_sm_scan = msg;
		//
		tf_map_to_odom_.stamp_ = ros::Time::now();
		tf_map_to_odom_.setOrigin(tf::Vector3(0,0,0));
		tf_map_to_odom_.setRotation(tf::createQuaternionFromYaw(0));
		tf_br_.sendTransform(tf::StampedTransform(tf_map_to_odom_, ros::Time::now(), "map", "odom"));
	}
}
;

void SMNode::odom_callback(const nav_msgs::Odometry& msg){
	if(!sm_odom_updated) {
		prev_sm_odom = msg.pose.pose;
		init_odom = msg.pose.pose;
		cur_odom = msg.pose.pose;
		sm_odom_updated = true;
	} else {
		cur_odom = msg.pose.pose;
		double new_x = cur_odom.position.x, new_y = cur_odom.position.y;
		double prev_theta = tf::getYaw(prev_sm_odom.orientation), new_theta = tf::getYaw(cur_odom.orientation);
		dx = new_x - prev_sm_odom.position.x;
		dy = new_y - prev_sm_odom.position.y;
		dt = new_theta - prev_theta;
		//
		double drot1 = atan2(new_y - prev_sm_odom.position.y, new_x - prev_sm_odom.position.x) - prev_theta;
		double dtrans = distance(prev_sm_odom.position.x, new_x, prev_sm_odom.position.y, new_y);
		double drot2 = new_theta - prev_theta - drot1;
		//
		est_sm_pose.x = cur_sm_pose.x + dtrans * cos(cur_sm_pose.theta + drot1);
		est_sm_pose.y = cur_sm_pose.y + dtrans * sin(cur_sm_pose.theta + drot1);
		est_sm_pose.theta = cur_sm_pose.theta + drot1 + drot2;	
		//
		//est_sm_pose.x = est_sm_pose.x + dtrans * cos(est_sm_pose.theta + drot1);
		//est_sm_pose.y = est_sm_pose.y + dtrans * sin(est_sm_pose.theta + drot1);
		//est_sm_pose.theta = est_sm_pose.theta + drot1 + drot2;
		//prev_sm_odom = cur_odom;
	}
}
;

float SMNode::distance(float x1, float x2, float y1, float y2) {
	return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}
;

float SMNode::rot_distance(float theta1, float theta2) {
	float rot_dist = theta1 - theta2;
    if (rot_dist >= PI) {
        rot_dist -= 2 * PI;
    } else if (rot_dist < -PI) {
        rot_dist += 2 * PI;
    }
	return rot_dist;
}
;
/*
void SMNode::spin() {
	//ros::Rate rate(100); // Specify the FSM loop rate in Hz
	while (ros::ok()) { // Keep spinning loop until user presses Ctrl+C
		ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
		if(sm_pose_updated) {
	
			//ROS_INFO("Pose published");
		}
	}
}
;
*/
int main(int argc, char **argv){
	ros::init(argc, argv, "sm_node");
	ros::NodeHandle n;
	SMNode sm_node(n);
	ros::spin();
	return 0;
}
;