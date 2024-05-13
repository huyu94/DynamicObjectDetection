cmake_minimum_required(VERSION 3.0.2)
project(trajectories)


set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")


find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
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
  INCLUDE_DIRS include
  LIBRARIES trajectories
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs message_runtime
#  DEPENDS system_lib
)


add_library(trajectories
  src/non_uniform_bspline.cpp
  src/polynomial_traj.cpp
  src/uniform_bspline.cpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

