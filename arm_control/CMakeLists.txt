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

catkin_install_python(PROGRAMS
  scripts/gripper_cmd_node.py
  scripts/servo_twist_frame_bridge_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config docs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
