cmake_minimum_required(VERSION 3.8)
project(hday_slam3d_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(hday_slam REQUIRED)

# Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# point cloud aggregator node
add_executable(point_cloud_aggregator_node point_cloud_aggregator_node.cc)
ament_target_dependencies(
  point_cloud_aggregator_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  hday_slam)
target_include_directories(
  point_cloud_aggregator_node
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
         $<INSTALL_INTERFACE:include>)

# slam system node
add_executable(slam_system_node slam_system_node.cc)
ament_target_dependencies(
  slam_system_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  hday_slam)
target_include_directories(
  slam_system_node PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                          $<INSTALL_INTERFACE:include>)

# Install
install(TARGETS slam_system_node point_cloud_aggregator_node
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_package()
