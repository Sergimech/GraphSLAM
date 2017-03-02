cmake_minimum_required(VERSION 2.8.3)
project(odometry)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  )

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(odometry src/odometry.cpp)
target_link_libraries(odometry ${catkin_LIBRARIES})
add_dependencies(odometry common_gencpp)
