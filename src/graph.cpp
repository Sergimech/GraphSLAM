#include "graph.h"

using namespace std;

const int N_INF = -9999999;
const int P_INF = 9999999;

typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >  SlamBlockSolver;
typedef g2o::LinearSolverCSparse< SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

Graph::Graph(double resolution, double range_threshold, double max_range, double min_range) {
    //ROS_INFO("Graph entering constructor");
	this->resolution = resolution;
	this->range_threshold = range_threshold;
    this->idCounter = 1;
    //
    this->min_range = min_range;
    this->max_range = max_range;
};

Graph::~Graph(){
    node_list.clear();
    edge_list.clear();
}

void Graph::addNode(GraphPose pose, const sensor_msgs::LaserScan& scan){
    //ROS_INFO("Graph entering addNode");
	Node * n = new Node();
    n->id = this->idCounter;
    idCounter++;
    // set the true pose to the odometry initially
    n->graph_pose = pose;
    // Store the scans
	n->laser_scan = scan;
    //
    if(node_list.size() > 0) {
        double mean[3], output[3];
        double covariance[3][3];
        double error;
        // Add the link between the current and the previous node
        Edge * e = new Edge();
        e->child_id = n->id;
        e->parent_id = last_node->id;
        GraphPose last_pose = last_node->graph_pose;
        ScanMatcher matcher;

        bool result = false;
        try {
            result = matcher.graphScanMatch(n->laser_scan, n->graph_pose, last_node->laser_scan, last_node->graph_pose, mean, covariance, output, error);
        } catch (int e) {
            // nothing
        }
	    // ROS_INFO("Graph SM Error %f", error);
        // Match the new node's scans to previous scans and add edges according
        if(result) {
            ROS_INFO("Using sm result for graph node.");
        // if(false) {
            // Update the node's position according to the result from the scan-match
            //n->graph_pose.x = mean[0];
            //n->graph_pose.y = mean[1];
            //n->graph_pose.theta = mean[2];
            //
            g2o::SE2 se_new(n->graph_pose.x, n->graph_pose.y, n->graph_pose.theta);
            g2o::SE2 se_prev(last_pose.x, last_pose.y, last_pose.theta);
            g2o::SE2 transf = se_prev.inverse() * se_new;
            //
            e->mean[0] = transf[0];
            e->mean[1] = transf[1];
            e->mean[2] = transf[2];
            // Store the covariance at the node
            memcpy(e->covariance, covariance, sizeof(double) * 9);
        } else {
            ROS_WARN("Error scan matching while storing node, using default values!");
            g2o::SE2 se_new(n->graph_pose.x, n->graph_pose.y, n->graph_pose.theta);
            g2o::SE2 se_prev(last_pose.x, last_pose.y, last_pose.theta);
            g2o::SE2 transf = se_prev.inverse() * se_new;
            //
            e->mean[0] = transf[0];
            e->mean[1] = transf[1];
            e->mean[2] = transf[2];
            for(unsigned int i = 0; i < 3; i++) {
                for(unsigned int j = 0; j < 3; j++) {
                        e->covariance[i][j] = 0.;
                }
            }
            // Use a covariance estimate
            e->covariance[0][0] = 0.025;
            e->covariance[1][1] = 0.025;
            e->covariance[2][2] = 0.009;
        }
        //
	    edge_list.push_back(e);
        node_list.push_back(n);
        last_node = n;
        if(distance(pose.x, last_pose.x, pose.y, last_pose.y) > 0) {
             addNearbyConstraints(8, 1, 2, 0.2, 0.2);
        } 
    } else {
        node_list.push_back(n);
        last_node = n;
    }
    //
    n->scan_grid = scanToOccGrid(scan, n->graph_pose);
    //
    ROS_INFO("Added node: %d, graph pose x: %f, y: %f t: %f.", n->id, n->graph_pose.x, n->graph_pose.y, n->graph_pose.theta);
}
;

void Graph::addNearbyConstraints(int close_limit, int step_size, double dist_limit, double min_dist_delta, double min_angle_delta) {
    GraphPose last_pose = last_node->graph_pose;
    sensor_msgs::LaserScan last_scan = last_node->laser_scan;
    unsigned int last_id = last_node->id;
    // ROS_INFO("Searching nearby constraints. id: %d.", last_id);
    // Look for nodes that are nearby
    double distance, dt, dx, dy, error;
    Node* n;
    for(int i = 0; i < (int)(node_list.size() - close_limit); i += step_size) {
        n = node_list[i];
        if (n->id == last_node->id)
            continue;
        //
        dx = last_pose.x - n->graph_pose.x;
        dy = last_pose.y - n->graph_pose.y;
        dt = last_pose.theta - n->graph_pose.theta;
        if (dt >= PI) {
          dt -= 2 * PI;
        } else if (dt < -PI) {
          dt += 2 * PI;
        }
        distance = sqrt(pow(dx, 2) + pow(dy, 2));
        // The nodes are too far apart
        if(distance > dist_limit)
            continue;
        // Don't add a new constraint if one already exists
        bool exists = false;
        for(unsigned int j = 0; j < edge_list.size(); j++) {
            if(edge_list[j]->child_id == last_id && edge_list[j]->parent_id == n->id) {
                exists = true;
                break;
            }
        }
        double mean[3], output[3];
        double cov[3][3];
        try {
            ScanMatcher matcher;
            if(!exists && matcher.graphScanMatch(last_scan, last_pose, n->laser_scan, n->graph_pose, mean, cov, output, error)) {
                // ROS_INFO("SM Error %f", error);
                // Check if the calculated distance corresponds with the distance mean calculated by the scanmatcher
                if (abs(dx - output[0]) <= min_dist_delta && abs(dy - output[1]) <= min_dist_delta && abs(dt - output[2]) < min_angle_delta) {
                    // Add the link between the current and the previous node
                    Edge * e = new Edge();
                    e->child_id = last_id;
                    e->parent_id = n->id;
                    //
                    // memcpy(e->mean, output, sizeof(double) * 3);
                    g2o::SE2 se_prev(n->graph_pose.x, n->graph_pose.y, n->graph_pose.theta);
                    g2o::SE2 se_new(last_pose.x, last_pose.y, last_pose.theta);
                    g2o::SE2 transf = se_prev.inverse() * se_new;
                    //
                    e->mean[0] = transf[0];
                    e->mean[1] = transf[1];
                    e->mean[2] = transf[2];
                    memcpy(e->covariance, cov, sizeof(double) * 9);
                    edge_list.push_back(e);
                    // ROS_INFO("Found a link between nodes.");
                }
            }
        } catch (int e) {
            //
        }
    }
}
;

// Combine the scan-grids in the nodes into a complete map!
void Graph::generateMap(nav_msgs::OccupancyGrid& cur_map) {
    //ROS_INFO("Graph entering generateMap");
    // First we need to know the outer-bounds of the map
    double xmax = N_INF, xmin = P_INF, ymax = N_INF, ymin = P_INF;
    Node * node;
    for (unsigned int i = 0; i < node_list.size(); i++)
    {
        node = node_list[i];
        float x = node->graph_pose.x, y = node->graph_pose.y;
        if (xmax < x + node->scan_grid.xmax * resolution)
            xmax = x + node->scan_grid.xmax * resolution;

        if (xmin > x - node->scan_grid.xmin * resolution)
            xmin = x - node->scan_grid.xmin * resolution;

        if (ymax < y + node->scan_grid.ymax * resolution)
            ymax = y + node->scan_grid.ymax * resolution;

        if (ymin > y - node->scan_grid.ymin * resolution)
            ymin = y - node->scan_grid.ymin * resolution;
    }
    //ROS_INFO("Graph map bounds: %f, %f, %f, %f", xmin, xmax, ymin, ymax);
    // Map size
    int map_height = round((ymax - ymin) / resolution);
    int map_width = round((xmax - xmin) / resolution);
    // Increase the size of the map slightly so we don't run into any rounding errors
    // Why 2? Because 1 and 0 didn't work :)
    map_width += 2;
    map_height += 2;
    unsigned int map_size = (unsigned int)(map_height * map_width);
    // ROS_INFO("Graph map size: %d", map_size);
    //
    cur_map.header.frame_id = "/map";
    cur_map.header.stamp = ros::Time().now();
    cur_map.info.height = map_height;
    cur_map.info.width = map_width;
    cur_map.info.resolution = resolution;
    //
    cur_map.data.resize(map_size);
    geometry_msgs::Pose origin;
    origin.orientation.x = 0.;
    origin.orientation.y = 0.;
    origin.orientation.z = 0.;
    origin.orientation.w = 1.;
    origin.position.x = xmin;
    origin.position.y = ymin;
    cur_map.info.origin = origin;
    // This vector counts how many times a map-position was seen in the graph
    // Later we use this to compute the certainty that a position is an obstacle
    vector<unsigned int> pos_seen (map_size, 0);
    // These will count how many times a position was blocked/free. All others will be unknown
    vector<unsigned int> pos_fr (map_size, 0);
    vector<unsigned int> pos_blocked (map_size, 0);
    //ROS_INFO("Graph combining local grids");
    // Go through all nodes' scan-grids and maintain the position information for each
    for (unsigned int i = 0; i < node_list.size(); i++)
    {
        node = node_list[i];
        float x = node->graph_pose.x, y = node->graph_pose.y;
        int node_x = round((x - xmin) / resolution) - node->scan_grid.xmin;
        int node_y = round((y - ymin) / resolution) - node->scan_grid.ymin;
        // Go through the local map, and count the occupancies for the global map
        for (int j = 0; j < node->scan_grid.height; j++) {
            for (int k = 0; k < node->scan_grid.width; k++) {
                // Index in the global map
                int global_index = (map_width * (node_y + j)) + (node_x + k);
                // The value of the local grid
                int local_index = (j * node->scan_grid.width) + k;
                int value = node->scan_grid.grid[local_index];
                //
                if (value == 100) { // Position is blocked
                    pos_blocked[global_index]++;
                    pos_seen[global_index]++;
                }
                if (value == 0) {// Position is unoccupied
                    pos_fr[global_index]++;
                    pos_seen[global_index]++;
                }
            }
        }
    }
    // ROS_INFO("Graph determining occupied/free/unknown");
    // Now, we can update the global map according to what we saw in the local maps
    for(unsigned int i = 0; i < map_size; i++) {
        // Haven't seen this one occupied nor free
        if(pos_seen[i] == 0) { 
            cur_map.data[i] = -1;
            continue;
        }
        // Check if we are certain enough that a position is blocked
        if ((double)pos_blocked[i] / (double)pos_seen[i] >= .25)
            cur_map.data[i] = 100;
        else
            cur_map.data[i] = 0;
    }
    //ROS_INFO("Graph finished generateMap");
}
;

// Take a scan as input, generate a small, local occupancy grid
ScanGrid Graph::scanToOccGrid(const sensor_msgs::LaserScan& scan, GraphPose& pose) {
    //ROS_INFO("Graph entering scanToOccGrid");
	ScanGrid new_grid;
	double angle_incr = scan.angle_increment;
	// First we need to know the size and bounds of the grid.
    double xmax = N_INF,xmin = P_INF,ymax = N_INF,ymin = P_INF;
    double scan_angle, min_angle = scan.angle_min;
    int num_scans = scan.ranges.size();
    //ROS_INFO("Graph Generating local occ grid at x: %f, y: %f t: %f.", pose.x, pose.y, pose.theta);
    // Get the scan positions the bound the grid
    for (int i = 0; i < num_scans; i++)
    {
        scan_angle = pose.theta + min_angle + i * angle_incr;
        if (ymax < pose.y + scan.ranges[i] * sin(scan_angle))
            ymax = pose.y + scan.ranges[i] * sin(scan_angle);
        if (xmax < pose.x + scan.ranges[i] * cos(scan_angle))
            xmax = pose.x + scan.ranges[i] * cos(scan_angle);
        if (ymin > pose.y + scan.ranges[i] * sin(scan_angle))
            ymin = pose.y + scan.ranges[i] * sin(scan_angle);
        if (xmin > pose.x + scan.ranges[i] * cos(scan_angle))
            xmin = pose.x + scan.ranges[i] * cos(scan_angle);
    }
    // Initialize the grid, set the bounds relative to the odometry
    new_grid.ymax = round((ymax - pose.y) / resolution);
    new_grid.ymin = round((pose.y - ymin) / resolution);
    new_grid.xmax = round((xmax - pose.x) / resolution);
    new_grid.xmin = round((pose.x - xmin) / resolution);
    //ROS_INFO("Graph local occ grid bounds: xmax: %f, ymax: %f, xmin: %f, ymin: %f.", ymax, xmax, ymin, xmin);
    // Size of the grid can be computed with the bounds
    new_grid.height = new_grid.ymax + new_grid.ymin;
    new_grid.width =  new_grid.xmax + new_grid.xmin;
    // Prevent rounding errors
    new_grid.height++;
    new_grid.width++;
    // Set the grid to the correct size
    int grid_size = new_grid.height * new_grid.width;
    new_grid.grid.resize(grid_size);
    // Set each cell to unknown space, later we will it with obstacles and known space
    for (int i= 0; i < grid_size; i++) {
        new_grid.grid[i] = -1;
    }
    //
   	double measurement, theta, x, y;
   	int row, col;
    for (int i = 0; i < num_scans; i++)
    {
        measurement = scan.ranges[i];
        // Check if out of range, dont place an obstacle on the grid in this case
        if (measurement > max_range || measurement < min_range)
            continue;
        // Determine and set the location of the object in the local grid
        theta = pose.theta + min_angle + i * angle_incr;
        x = measurement * cos(theta);
        y = measurement * sin(theta);
        col = new_grid.ymin + round(y / resolution);
        row = new_grid.xmin + round(x / resolution);
        // 
        new_grid.grid[col * new_grid.width + row] = 100;
    }
    // Use triangulation to fill the grid covered by the scan with known space
    double range, scan_dist, scan_theta, scan_end;
    int index, scan_index;
    for (int i = 0; i < new_grid.height; i++) {
        for (int j = 0; j < new_grid.width; j++) {
            index = i * new_grid.width + j;
            // We can skip positions we know are occupied
            if (new_grid.grid[index] == 100)
                continue;
            scan_theta = atan2(i - new_grid.ymin, j - new_grid.xmin) - pose.theta - scan.angle_min;
            scan_theta -= floor(scan_theta / PI / 2) * (PI * 2);
            // Check if the scan is out of bounds
            scan_index = round(scan_theta / scan.angle_increment);
            if (scan_index < 0|| scan_index >= num_scans) {
                new_grid.grid[index] = -1;
                continue;
            }
            //
            range = scan.ranges[scan_index];
            scan_dist = sqrt(pow(j - new_grid.xmin, 2) + pow(i - new_grid.ymin, 2));
            scan_end = range - (scan_dist * resolution);
            // If the end of the scan is outside the bounds of the grid, it will be unknown
            // For instance, it will be behind a wall. Otherwise, it is in between the robot and the wall.
            if(scan_end > 0)
                new_grid.grid[index] = 0;
            else
                new_grid.grid[index] = -1;
        }
    }
    //ROS_INFO("Graph finished scanToOccGrid");
    return new_grid;
}
;

void Graph::solve(unsigned int iterations){
    ROS_INFO(":: SOLVING! ::");
    //Setup solver
    g2o::SparseOptimizer sparseOptimizer;
    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering(false);
    SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
    g2o::OptimizationAlgorithmGaussNewton* solverGauss = 
            new g2o::OptimizationAlgorithmGaussNewton(blockSolver);
    sparseOptimizer.setAlgorithm(solverGauss);

    //Convert pose nodes to g2o node structure and add in the graph.
    for(unsigned int i = 0; i < node_list.size(); i ++){
        Node* curNode = node_list[i];
        // ROS_INFO("Curnode id: %d", curNode->id);
        // Convert the node.
        GraphPose graph_pose = curNode->graph_pose;
        g2o::SE2 converted_pose(graph_pose.x, graph_pose.y, graph_pose.theta);        
        // Create the vertex to put in the graph.
        g2o::VertexSE2* vertex = new g2o::VertexSE2;
        vertex->setId(curNode->id);
        // ROS_INFO("Converted node id: %d", vertex->id());
        vertex->setEstimate(converted_pose);
        // Add to the graph
        sparseOptimizer.addVertex(vertex);
    }

    // Set one pose fixed, to reduce complexity. This pose wont be changed by the optimizer.
    sparseOptimizer.vertex(1)->setFixed(true);

    // Convert the edges to g2o edges and add them in the graph
    for(unsigned int i = 0; i < edge_list.size(); i++) {
        // ROS_INFO("Adding edge: %d", i);
        Edge* edge = edge_list[i];
        //ROS_INFO("Edge parent id: %d, child id: %d", edge->parent_id, edge->child_id);
        //Actually make the edge for the optimizer.
        g2o::EdgeSE2* graph_edge = new g2o::EdgeSE2;
        graph_edge->vertices()[0] = sparseOptimizer.vertex(edge->parent_id);
        graph_edge->vertices()[1] = sparseOptimizer.vertex(edge->child_id);
        //
        g2o::SE2 se_mean(edge->mean[0], edge->mean[1], edge->mean[2]);
        graph_edge->setMeasurement(se_mean);
        Matrix3d cov;
        cov = MatrixXd::Zero(3,3);
        for(unsigned int i = 0; i < 3; i++) {
            for(unsigned int j = 0; j < 3; j++) {
              cov(i, j) = edge->covariance[i][j];
              // ROS_INFO("Covariance[%d]: %f", i+j, covariance[i][j]);
            }
        }
        graph_edge->setInformation(cov.inverse());
        //Add edge to optimizer
        sparseOptimizer.addEdge(graph_edge);
    }

    //Optimize!
    sparseOptimizer.setVerbose(false);
    sparseOptimizer.initializeOptimization();
    sparseOptimizer.optimize(iterations);

    //Convert the solved poses back
    for(unsigned int i = 0; i < node_list.size(); i++){
        GraphPose* currentPose = &(node_list[i]->graph_pose);
        g2o::SE2 optimized_pose = ((g2o::VertexSE2*) sparseOptimizer.vertex(node_list[i]->id))->estimate();
        //
        if(rot_distance(currentPose->theta, optimized_pose[2]) != 0. || distance(currentPose->x, optimized_pose[0], currentPose->y, optimized_pose[1]) > 0.) {
            ROS_INFO("Node %d, pose before optimize: x %f, y %f, t %f", node_list[i]->id, currentPose->x, currentPose->y, currentPose->theta);
            ROS_INFO("Node %d, pose after optimize: x %f, y %f, t %f", node_list[i]->id, optimized_pose[0], optimized_pose[1], optimized_pose[2]);
        }
        currentPose->x = optimized_pose[0];
        currentPose->y = optimized_pose[1];
        currentPose->theta = optimized_pose[2];
    }
    sparseOptimizer.clear();
};

float Graph::distance(float x1, float x2, float y1, float y2) {
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}
;

float Graph::rot_distance(float theta1, float theta2) {
    float rot_dist = theta1 - theta2;
    if (rot_dist >= PI) {
        rot_dist -= 2 * PI;
    } else if (rot_dist < -PI) {
        rot_dist += 2 * PI;
    }
    return rot_dist;
}
;

void Graph::createTfFromXYTheta(double x, double y, double theta, tf::Transform& t) {
  t.setOrigin(tf::Vector3(x, y, 0.0));
  tf::Quaternion q;
  q.setRPY(0.0, 0.0, theta);
  t.setRotation(q);
}
;
