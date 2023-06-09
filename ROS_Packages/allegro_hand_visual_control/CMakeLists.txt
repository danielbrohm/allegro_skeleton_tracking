cmake_minimum_required(VERSION 3.0.2)
project(allegro_hand_visual_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  ros_openpose
  OpenCV
)



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES allegro_hand_basic
  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenPose_INCLUDE_DIRS}
  ${Ros_OpenPose_INCLUDE_DIRS}
  #${PROJECT_SOURCE_DIR}/include
)

add_executable(state_publisher_simulation src/state_publisher_simulation.cpp)
target_link_libraries(state_publisher_simulation ${catkin_LIBRARIES})
add_executable(state_publisher_hand src/state_publisher_hand.cpp)
target_link_libraries(state_publisher_hand ${catkin_LIBRARIES})


catkin_install_python(PROGRAMS scripts/kalman_filter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install launch files
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
  FILES_MATCHING PATTERN "*.launch"
)
