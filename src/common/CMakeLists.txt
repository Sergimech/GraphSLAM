cmake_minimum_required(VERSION 2.8.3)
project(common)

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  std_msgs
  costmap_2d
  sensor_msgs
  geometry_msgs
  message_generation
  visualization_msgs
  )

add_message_files(
  FILES
  Factor.msg
  Keyframe.msg
  Keyframes.msg
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

include_directories(${catkin_INCLUDE_DIRS})

add_executable(markers src/basic_shapes.cpp)
target_link_libraries(markers ${catkin_LIBRARIES})
add_dependencies(markers common_gencpp)
