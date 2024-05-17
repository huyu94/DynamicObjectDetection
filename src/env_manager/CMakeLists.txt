cmake_minimum_required(VERSION 2.8.3)
project(env_manager)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++14 )
add_compile_options(-w)
# add_compile_options(-Wno-unused-variable)
# add_compile_options(-Wno-maybe-uninitialized)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")


find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  message_filters
  roslib
  visualization_utils
)

include_directories(/usr/local/include/munkres)
set(MunkresLIB
        /usr/local/lib/libmunkres.a
        )

catkin_package(
 INCLUDE_DIRS include 
 LIBRARIES env_manager 
 CATKIN_DEPENDS roscpp std_msgs visualization_utils
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


# link_directories(${PCL_LIBRARY_DIRS})


add_library(${PROJECT_NAME} 
    include/${PROJECT_NAME}/ikd-Tree/ikd_Tree.cpp
    src/grid_map.cpp
    src/kalman_filter.cpp
    src/raycast.cpp
    src/tracker_pool.cpp
    src/env_manager.cpp
    src/pos_checker.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    ${MunkresLIB}
    ${OpenCV_LIBS}
)

add_executable(test_node
    src/test_node.cpp
)

target_link_libraries(test_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${MunkresLIB}
    ${OpenCV_LIBS}
    ${PROJECT_NAME}
)





add_executable(test_stCapsule
    src/test_stCapsule.cpp
)

target_link_libraries(test_stCapsule
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

