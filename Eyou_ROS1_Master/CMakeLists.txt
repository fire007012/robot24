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

if(CATKIN_ENABLE_TESTING)
  find_package(GTest REQUIRED)

  catkin_add_gtest(test_hybrid_operational_coordinator
    tests/test_hybrid_operational_coordinator.cpp
  )
  if(TARGET test_hybrid_operational_coordinator)
    target_include_directories(test_hybrid_operational_coordinator PRIVATE include ${GTEST_INCLUDE_DIRS})
    target_sources(test_hybrid_operational_coordinator PRIVATE
      src/hybrid_operational_coordinator.cpp
    )
    target_link_libraries(test_hybrid_operational_coordinator
      ${catkin_LIBRARIES}
      GTest::GTest
      GTest::Main
    )
  endif()

  catkin_add_gtest(test_hybrid_service_gateway
    tests/test_hybrid_service_gateway.cpp
  )
  if(TARGET test_hybrid_service_gateway)
    target_include_directories(test_hybrid_service_gateway PRIVATE include ${GTEST_INCLUDE_DIRS})
    target_sources(test_hybrid_service_gateway PRIVATE
      src/hybrid_operational_coordinator.cpp
      src/hybrid_service_gateway.cpp
    )
    target_link_libraries(test_hybrid_service_gateway
      ${catkin_LIBRARIES}
      GTest::GTest
      GTest::Main
    )
  endif()
endif()
