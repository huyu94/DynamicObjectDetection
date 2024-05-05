cmake_minimum_required(VERSION 3.0.2)
project(data_analyzer)

## Compile as C++11, supported in ROS Kinetic and newer
ADD_COMPILE_OPTIONS(-std=c++14 )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  roslib
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES data_analyzer
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS} 

)


add_executable(data_analyzer src/data_analyzer.cpp)
target_link_libraries(
  data_analyzer
  ${catkin_LIBRARIES}

)
