cmake_minimum_required(VERSION 3.0.2)
project(plan_manager)


set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-dev")



find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  plan_env
  visualization_utils
  quadrotor_msgs
  trajectories
)



catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES plan_manager
  CATKIN_DEPENDS roscpp std_msgs plan_env visualization_utils quadrotor_msgs trajectories
#  DEPENDS system_lib
)
include_directories(/usr/local/include/munkres)
set(MunkresLIB
    /usr/local/lib/libmunkres.a
    )

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  SYSTEM
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

# file(GLOB_RECURSE SOURCES "src/**/*.cpp")
# message("Sources: ${SOURCES}")
add_library(plan_manager
  src/plan_manager.cpp
  src/path_searching/astar.cpp
  src/path_searching/kinodynamic_astar.cpp
  src/path_searching/topo_prm.cpp
  src/uniform_bspline_opt/bspline_optimizer.cpp
  )

target_link_libraries(
  plan_manager
  ${catkin_LIBRARIES}
  ${MunkresLIB}
  # ${PCL_LIBRARIES}
)



add_executable(test_node
  src/test_node.cpp
  src/plan_manager.cpp
  src/path_searching/astar.cpp
  src/path_searching/kinodynamic_astar.cpp
  src/path_searching/topo_prm.cpp
  src/uniform_bspline_opt/bspline_optimizer.cpp
)
# add_executable(plan_manager_node src/plan_manager_node.cpp)

target_link_libraries(test_node
  ${catkin_LIBRARIES}
  ${MunkresLIB}
)

add_executable(traj_server src/traj_server.cpp)
target_link_libraries(traj_server ${catkin_LIBRARIES} ${MunkresLIB})
# add_dependencies(traj_server ${${PROJECT_NAME}_EXPORTED_TARGETS})

