#ifndef GRAPHSLAM_H
#define GRAPHSLAM_H
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/Marker.h"
#include "pose_laser.h"
//
#include "graph.h"
#include "scanmatcher.h"
#include "graphnodes.h"

#ifndef PI
#define PI 3.14159265359
#endif

using namespace std;
using namespace geometry_msgs;
using namespace sensor_msgs;

class GraphSlam {
public:
	GraphSlam(ros::NodeHandle& nh);
	~GraphSlam();
	void spin();
private:
	ros::Subscriber laserScan_Sub, odometry_Sub, pose_Sub;
	ros::Publisher map_publish, pose_publish, graph_publish, pose_publisher;
	// The last pose and corresponding scan
	Pose prev_graph_pose;
	LaserScan cur_scan;
	nav_msgs::OccupancyGrid cur_map;
	double resolution, min_node_dist, min_node_rot, range_t, min_laser_range, max_laser_range;
	string scan_topic;
	int solve_after_nodes, solve_iterations;
	//
	GraphPose cur_sm_pose;
	bool odom_updated, scan_updated, first_scan;
	//
	Graph* graph;
	//
	float distance(float x1, float x2, float y1, float y2);
	float rot_distance(float theta1, float theta2);
	void laserScan_callback(const LaserScan::ConstPtr& msg);
	void pose_callback(const graphslam::pose_laser& msg);
	void drawPoses();
	void drawScans();
	void rosToGraphPose(const Pose& ros_pose, GraphPose& g_pose);
};
#endif
