cmake_minimum_required(VERSION 3.0.2)
project(chess_master)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
  baxter_interface
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs tf2_ros baxter_interface
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Install Python scripts
catkin_install_python(PROGRAMS
  src/Gripper_Alyssa.py
  src/checkerboard_pose_pub.py
  src/checkerboard_mesh_pub.py
  src/State_Machine_dev.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install meshes
install(DIRECTORY meshes/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes)

## Install launch files
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
