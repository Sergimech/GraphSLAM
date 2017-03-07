cmake_minimum_required(VERSION 2.8.3)
project(scanner)

find_package(catkin REQUIRED COMPONENTS
  tf
  pcl_ros
  pcl_conversions
  laser_geometry
  cmake_modules
  sensor_msgs
  geometry_msgs
  roscpp
  common
  )

find_package(
  Eigen3 REQUIRED
  )

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  )

add_executable(scanner src/scanner.cpp)
target_link_libraries(scanner ${catkin_LIBRARIES} ${EIGEN_LIBRARIES})
add_dependencies(scanner common_gencpp)
