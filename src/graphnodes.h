#ifndef GRAPHNODES_H
#define GRAPHNODES_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/OccupancyGrid.h"

using namespace std;
using namespace geometry_msgs;

struct GraphPose {
	double x, y, theta;
};

struct ScanGrid {
	// The size, based on the range of the laserscanner
	int width, height;
	// The bounds in the real map
	int ymax, ymin, xmax, xmin;
	// The grid, stored in row-major order
	vector<double> grid;
};

// A node in the graph, contains its own little occupancygrid to later be combined with all nodes in the map
struct Node {
	unsigned int id;
	// This will be the true estimate of the pose
	GraphPose graph_pose;
	sensor_msgs::LaserScan laser_scan;
	// The occupancygrid for the scan at this position
	ScanGrid scan_grid;
};

// An edge represents the connection between two nodes
struct Edge {
	unsigned int parent_id, child_id;
	//
	double mean[3];
	double covariance[3][3];
};
#endif