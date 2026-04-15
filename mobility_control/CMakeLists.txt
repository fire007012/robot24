cmake_minimum_required(VERSION 3.0.2)
project(mobility_control)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS
    gazebo_msgs
    geometry_msgs
    nav_msgs
    roscpp
    rospy
    tf2_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch config docs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
