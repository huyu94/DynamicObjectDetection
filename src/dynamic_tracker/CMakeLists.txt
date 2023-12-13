cmake_minimum_required(VERSION 3.0.2)
project(dynamic_tracker)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  message_filters
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dynamic_tracker
  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  )
include_directories(
  /usr/local/include/munkres
)
set(MunkresLIB /usr/local/lib/libmunkres.a)


add_library(dynamic_tracker
  src/dynamic_tracker.cpp
  src/dbscan_cluster.cpp
  src/tracking_pool.cpp

  )
target_link_libraries(dynamic_tracker
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${MunkresLIB}
  )

add_executable(main_node
  src/main_node.cpp
  src/dynamic_tracker.cpp
  src/dbscan_cluster.cpp
  src/tracking_pool.cpp
)
target_link_libraries(main_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${MunkresLIB}
)


add_executable(test_node
  src/test_node.cpp
)
target_link_libraries(test_node
  ${catkin_LIBRARIES}
  ${MunkresLIB}
)

add_executable(cluster_test_node
  src/cluster_test_node.cpp
  src/cluster_tester.cpp
  src/dbscan_cluster.cpp

)

target_link_libraries(cluster_test_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${MunkresLIB}
)




