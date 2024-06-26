cmake_minimum_required(VERSION 2.8.3)
project(plan_env)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++14 )
add_compile_options(-w)
# add_compile_options(-Wno-unused-variable)
# add_compile_options(-Wno-maybe-uninitialized)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")


find_package(OpenCV REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  message_filters
  roslib
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES plan_env
 CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

include_directories(/usr/local/include/munkres)
set(MunkresLIB
        /usr/local/lib/libmunkres.a
        )
# link_directories(${PCL_LIBRARY_DIRS})

# file(GLOB_RECURSE plan_env_SRC
#     "src/*.cpp"
# )

add_library( plan_env 
    ${plan_env_SRC}
    include/plan_env/ikd-Tree/ikd_Tree.cpp
    src/grid_map.cpp
    src/kalman_filter.cpp
    src/raycast.cpp
    src/tracker_pool.cpp
    src/env_manager.cpp
    src/map_visualizer.cpp
    src/pos_checker.cpp
)

target_link_libraries( plan_env
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    ${MunkresLIB}
    ${OpenCV_LIBS}
)  

add_executable(env_node
    src/env_node.cpp
    include/plan_env/ikd-Tree/ikd_Tree.cpp
    src/grid_map.cpp
    src/kalman_filter.cpp
    src/raycast.cpp
    src/tracker_pool.cpp
    src/env_manager.cpp
    src/map_visualizer.cpp
)

target_link_libraries(env_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${MunkresLIB}
    ${OpenCV_LIBS}
)

add_executable(test_stCapsule
    src/test_stCapsule.cpp
)

target_link_libraries(test_stCapsule
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

