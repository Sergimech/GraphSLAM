cmake_minimum_required(VERSION 2.8.3)
project(common)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  )

add_message_files(
  FILES
  Factor.msg
  Keyframe.msg
  Odometry.msg
  Registration.msg
  Pose2DWithCovariance.msg
  )

add_service_files(
  FILES
  LastKeyframe.srv
  ClosestKeyframe.srv
  OdometryBuffer.srv
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  )



catkin_package(
  DEPENDS message_runtime
)
