cmake_minimum_required(VERSION 2.8.3)
project(graph_slam)
add_definitions(-std=c++11)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake_modules")

find_package(CSparse REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(G2O REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs tf nav_msgs visualization_msgs)

if ( NOT CSPARSE_FOUND )
  message(FATAL_ERROR "Package CSparse not found!")
endif( NOT CSPARSE_FOUND )

if ( NOT EIGEN3_FOUND )
  message(FATAL_ERROR "Package Eigen3 not found!")
endif( NOT EIGEN3_FOUND )

if ( NOT G2O_FOUND )
  message(FATAL_ERROR "Package g2o not found!")
endif( NOT G2O_FOUND )

include_directories(${CSPARSE_INCLUDE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${G2O_INCLUDE_DIR})
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(graph src/graph.cpp)
add_library(scanmatcher src/scanmatcher.cpp)

add_executable(graphslam src/graphslam.cpp)
add_executable(sm_node src/sm_node.cpp)

target_link_libraries(graphslam graph ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${CSPARSE_LIBRARIES} ${G2O_LIBRARIES} )
target_link_libraries(sm_node scanmatcher ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${CSPARSE_LIBRARIES} ${G2O_LIBRARIES} )
target_link_libraries(graph scanmatcher g2o_core g2o_stuff g2o_types_slam2d g2o_solver_cholmod g2o_solver_pcg g2o_solver_csparse cxsparse g2o_incremental g2o_csparse_extension csparse ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${CSPARSE_LIBRARIES} ${G2O_LIBRARIES} )
target_link_libraries(scanmatcher "${PROJECT_SOURCE_DIR}/lib64/libcsm.so")

catkin_package()