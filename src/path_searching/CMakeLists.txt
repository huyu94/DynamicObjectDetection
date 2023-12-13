cmake_minimum_required(VERSION 3.0.2)
project(path_searching)
set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  plan_env
  cv_bridge
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES path_searching
  CATKIN_DEPENDS plan_env
#  DEPENDS system_lib
)

include_directories( 
  SYSTEM 
  include 
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  /usr/local/include/munkres
)

add_library( path_searching 
    src/astar.cpp
    src/topo_rpm.cpp
    )
target_link_libraries( path_searching
    ${catkin_LIBRARIES} 
    ${MunkresLIB}
    )  
# add_dependencies(path_searching plan_env)