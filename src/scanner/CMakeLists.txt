cmake_minimum_required(VERSION 2.8.3)
project(scanner)

find_package(catkin REQUIRED COMPONENTS
  tf
  pcl_ros
  pcl_conversions
  laser_geometry
  cmake_modules
  sensor_msgs
  roscpp
  common
  )

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(scanner src/scanner.cpp)
target_link_libraries(scanner ${catkin_LIBRARIES})
add_dependencies(scanner common_gencpp)
