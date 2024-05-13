cmake_minimum_required(VERSION 3.0.2)
project(visualization_utils)


set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES visualization_utils
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs visualization_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)


add_library(visualization_utils
  src/map_visualizer.cpp
  src/traj_visualizer.cpp
)
target_link_libraries(
  visualization_utils
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)


add_executable(map_test_node
  src/map_test_node.cpp
  src/map_visualizer.cpp
)
target_link_libraries(
  map_test_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)


add_executable(traj_test_node
  src/traj_test_node.cpp
  src/traj_visualizer.cpp
)
target_link_libraries(
  traj_test_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)


