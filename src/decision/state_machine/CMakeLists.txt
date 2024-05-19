cmake_minimum_required(VERSION 3.0.2)
project(state_machine)



find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  quadrotor_msgs
  plan_manager
)

include_directories(/usr/local/include/munkres)
set(MunkresLIB
        /usr/local/lib/libmunkres.a
        )


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES decision
 CATKIN_DEPENDS roscpp rospy std_msgs plan_manager
#  DEPENDS system_lib
)
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}
  src/fsm.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)


add_executable(state_machine_node 
  src/state_machine_node.cpp 
)

  target_link_libraries(state_machine_node
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )