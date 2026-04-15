cmake_minimum_required(VERSION 3.0.2)
project(arm_control)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  controller_manager_msgs
  geometry_msgs
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  trajectory_msgs
)

add_action_files(
  FILES
  ExecuteArmGoal.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    controller_manager_msgs
    geometry_msgs
    message_runtime
    roscpp
    rospy
    sensor_msgs
    std_msgs
    tf2_geometry_msgs
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
