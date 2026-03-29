cmake_minimum_required(VERSION 3.10)
project(Eyou_ROS1_Master LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  hardware_interface
  controller_manager
  std_srvs
  can_driver
  Eyou_Canopen_Master
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp hardware_interface controller_manager std_srvs can_driver Eyou_Canopen_Master
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# ── 统一外观节点 ─────────────────────────────────────────
add_executable(hybrid_motor_hw_node
  src/hybrid_motor_hw_node.cpp
  src/hybrid_robot_hw.cpp
  src/hybrid_operational_coordinator.cpp
  src/hybrid_service_gateway.cpp
)
add_dependencies(hybrid_motor_hw_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hybrid_motor_hw_node
  ${catkin_LIBRARIES}
  stdc++fs
)
