cmake_minimum_required(VERSION 2.8.3)
project(plan_env)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(OpenCV REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  cv_bridge
  message_filters
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

file(GLOB plan_env_SRC
    "src/*.cpp"
)
add_library( plan_env 
    ${plan_env_SRC}
    # src/grid_map.cpp 
    # src/raycast.cpp
)
target_link_libraries( plan_env
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    ${MunkresLIB}
)  
# add_dependencies(plan_env munkres)

# add_executable(dsp_map_node 
#     src/dsp_map_node.cpp
#     src/dsp_map.cpp
#     src/dsp_map_math.cpp
#     src/particle_map.cpp
#     )
# target_link_libraries(dsp_map_node
#     ${catkin_LIBRARIES} 
#     ${PCL_LIBRARIES}
#     ${MunkresLIB}
#     )  
# add_dependencies(plan_env munkres)

