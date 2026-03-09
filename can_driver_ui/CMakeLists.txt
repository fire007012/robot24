cmake_minimum_required(VERSION 3.0.2)
project(can_driver_ui)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  can_driver
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/bridge_node.py
  scripts/keyboard_ui.py
  scripts/gui_ui.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config docs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
