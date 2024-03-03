cmake_minimum_required(VERSION 3.0.2)
project(obj_predictor)


set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")



find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  
  
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES obj_predictor
 CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS OpenCV
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})


add_library(obj_predictor
  src/obj_predictor.cpp 
)
target_link_libraries(obj_predictor
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
)  


add_executable(obj_generator
  src/obj_generator.cpp 
)
target_link_libraries(obj_generator 
  ${catkin_LIBRARIES}
)