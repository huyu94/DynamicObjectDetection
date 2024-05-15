cmake_minimum_required(VERSION 3.0.2)
project(ros_test)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  geometry_msgs
  message_generation
)


add_message_files(
  FILES
  Bspline.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_test
 CATKIN_DEPENDS roscpp rospy std_msgs visualization_msgs message_runtime
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)


add_executable(ros_test_node src/test_node.cpp)
target_link_libraries(ros_test_node ${catkin_LIBRARIES})