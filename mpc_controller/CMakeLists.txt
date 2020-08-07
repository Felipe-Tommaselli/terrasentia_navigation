cmake_minimum_required(VERSION 3.0.2)
project(mpc_controller)

set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf2
)

find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mpc_dev
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs std_msgs tf2
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  /usr/local/include
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

link_directories(/usr/local/lib)

add_executable(mpc_node
  src/mpc_node.cpp
  src/mpc.cpp
  #include/mpc-controller/mpc-controller.cpp
  include/mpc-controller/mpc-model2.cpp
  include/mpc-controller/run-mpc-controller.cpp
)

target_link_libraries(mpc_node ${catkin_LIBRARIES} ipopt coinmumps)