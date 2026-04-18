cmake_minimum_required(VERSION 3.0.2)
project(control)

find_package(catkin REQUIRED)

catkin_package()

# Archived vendor/reference package only.
# The formal runtime chain is:
#   mobility_control/base_cmd_node -> wheel_controller -> can_driver(DM backend)
# The old damiao_diff_chassis_node is preserved in source form for traceability,
# but is intentionally not built or installed as an executable.

install(FILES
  README.md
  DM-J10010L-2EC减速电机说明书V1.1.pdf
  调试助手使用说明书（达妙驱动控制协议）V1.4.pdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/archive
)
