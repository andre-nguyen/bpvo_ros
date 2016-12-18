cmake_minimum_required(VERSION 2.8.3)
project(bpvo_ros)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -march=native")

find_package(catkin_simple REQUIRED)
catkin_simple()

find_package(Eigen3 REQUIRED)
find_package(PkgConfig)
find_package(yaml_cpp_catkin QUIET)
if(${yaml_cpp_catkin_FOUND})
  message(STATUS "Found yaml_cpp_catkin, using instead of system library.")
  set(YamlCpp_LIBRARIES ${yaml_cpp_catkin_LIBRARIES})
  set(YamlCpp_INCLUDE_DIRS ${yaml_cpp_catkin_INCLUDE_DIRS})
else()
  message(STATUS "No yaml_cpp_catkin, using yaml-cpp system library instead.")
  pkg_check_modules(YamlCpp REQUIRED yaml-cpp>=0.5)
endif()

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###########
## Build ##
###########

link_directories(/usr/local/lib)
include_directories(/usr/local/include/bpvo ${EIGEN3_INCLUDE_DIR} ${YamlCpp_INCLUDE_DIRS})

cs_add_library(bpvo_ros src/bpvo_ros.cpp)
target_link_libraries(bpvo_ros
 ${catkin_LIBRARIES}
 ${YamlCpp_LIBRARIES}
 bpvo
)

cs_add_executable(bpvo_ros_node src/bpvo_ros_node.cpp)
target_link_libraries(bpvo_ros_node
 ${catkin_LIBRARIES}
 ${YamlCpp_LIBRARIES}
 bpvo
 bpvo_ros
)
