#include <ros/ros.h>
#include <vector>
#include <common/Keyframe.h>
#include <common/Keyframes.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <costmap_2d/costmap_2d_ros.h>

common::Keyframes keyframes;
ros::Publisher marker_pub;
ros::Publisher pointcloud_pub;

void create_arrow(common::Keyframe keyframe) {
  uint32_t shape = visualization_msgs::Marker::ARROW;
  int id = keyframe.id;;
  double x = keyframe.pose_opti.pose.x;
  double y = keyframe.pose_opti.pose.y;
  double th = keyframe.pose_opti.pose.theta;
  visualization_msgs::Marker marker;
  
  marker.header.frame_id = "/odom";
  marker.header.stamp = ros::Time::now();
  marker.ns = "arrows";
  marker.id = id;
  marker.type = shape;
  marker.action = visualization_msgs::Marker::ADD;
      
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromYaw(th);
  marker.pose.orientation = quaternion;
  marker.scale.x = 0.5;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.r = 251.0f;
  marker.color.g = 13.0f;
  marker.color.b = 13.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration(5);
  marker_pub.publish(marker);
}

void create_scan(common::Keyframe keyframe) {
  uint32_t shape = visualization_msgs::Marker::SPHERE;
  int id = keyframe.id;
  double x = keyframe.pose_opti.pose.x;
  double y = keyframe.pose_opti.pose.y;
  double th = keyframe.pose_opti.pose.theta;
  visualization_msgs::Marker marker;

  for(int i = 0; i < keyframe.scan.ranges.size(); i++) {
    marker.header.frame_id = "/odom";
    marker.header.stamp = ros::Time::now();
    marker.ns = "arrow_scan";
    marker.id = id + 1 + i;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;

    double point_th = keyframe.scan.angle_min + (keyframe.scan.angle_increment * i);
    double point_x = keyframe.scan.ranges[i] * sin( point_th + 1.57) ;
    double point_y = -1 * keyframe.scan.ranges[i] * cos( point_th + 1.57);
      
    marker.pose.position.x = point_x;
    marker.pose.position.y = point_y;
    geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromYaw(point_th);
    marker.pose.orientation = quaternion;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 251.0f;
    marker.color.g = 13.0f;
    marker.color.b = 13.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(5);
    marker_pub.publish(marker);
  }
}

void keyframes_callback(const common::Keyframes& input) {
  keyframes = input;
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(20);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1000);
  ros::Subscriber keyframe_sub = n.subscribe("/graph/keyframes", 1, keyframes_callback);

  while(ros::ok()) {
    for(int i = 0; i < keyframes.keyframes.size(); i++) {
      create_arrow(keyframes.keyframes[i]);
    }
  
    for(int i = 0; i < keyframes.keyframes.size(); i++) {
      //      create_scan(keyframes.keyframes[i]);
    }
    
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}

