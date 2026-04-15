cmake_minimum_required(VERSION 3.0.2)
project(arm_control)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2_ros
  trajectory_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    roscpp
    rospy
    sensor_msgs
    std_msgs
    tf2_ros
    trajectory_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch config docs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
