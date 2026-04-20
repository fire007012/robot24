cmake_minimum_required(VERSION 3.0.2)
project(vision_pkg)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall)

## =====================
## Third-party paths
## =====================
set(THIRD_PARTY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party)

# OpenCV (指定 4.10 安装路径，跳过系统自带的 4.2)
set(OpenCV_DIR /usr/local/lib/cmake/opencv4)
find_package(OpenCV 4.10 REQUIRED)
set(OPENCV_LIBS ${OpenCV_LIBS})

# GLM (header-only)
set(GLM_INCLUDE_DIR ${THIRD_PARTY_DIR}/include/glm-0.9.9.8)

# Eigen (header-only)
set(EIGEN3_INCLUDE_DIR ${THIRD_PARTY_DIR}/include)

## =====================
## Find catkin and ROS components
## =====================
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  message_generation
)

find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(EGL REQUIRED egl)

## Generate custom messages
add_message_files(FILES Detection.msg ObstacleWarning.msg)
generate_messages(DEPENDENCIES std_msgs)

## Catkin package configuration
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs cv_bridge image_transport message_runtime
)

## =====================
## Include directories
## =====================
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GLM_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${EGL_INCLUDE_DIRS}
)

## =====================
## yolov8_node
## =====================
add_executable(yolov8_node
  src/yolov8.cpp
  src/yolov8_node.cpp
)
target_link_libraries(yolov8_node
  ${catkin_LIBRARIES}
  ${OPENCV_LIBS}
)
add_dependencies(yolov8_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

## =====================
## panorama_node
## =====================
add_executable(panorama_node
  src/fisheye.cpp
  src/panoramasimulator.cpp
  src/panorama.cpp
  src/panorama_node.cpp
)
target_link_libraries(panorama_node
  ${catkin_LIBRARIES}
  ${OPENCV_LIBS}
  ${EGL_LIBRARIES}
  GL
  Threads::Threads
)
add_dependencies(panorama_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

## =====================
## object_pose_node
## =====================
add_executable(object_pose_node
  src/object_pose_node.cpp
)
target_link_libraries(object_pose_node
  ${catkin_LIBRARIES}
  ${OPENCV_LIBS}
)
add_dependencies(object_pose_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

## =====================
## vision_display_node
## =====================
add_executable(vision_display_node
  src/yolov8.cpp
  src/vision_display_node.cpp
)
target_link_libraries(vision_display_node
  ${catkin_LIBRARIES}
  ${OPENCV_LIBS}
)
add_dependencies(vision_display_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

