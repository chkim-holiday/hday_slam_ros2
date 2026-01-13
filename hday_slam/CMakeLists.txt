cmake_minimum_required(VERSION 3.8)
project(hday_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(
  ${PROJECT_NAME} SHARED
  hday/slam3d/slam_system/slam_system.cc
  hday/slam3d/point_cloud_aggregator/point_cloud_aggregator.cc)

target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                         $<INSTALL_INTERFACE:include> ${EIGEN3_INCLUDE_DIRS})

# Install
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

install(
  DIRECTORY hday/
  DESTINATION include/hday
  FILES_MATCHING
  PATTERN "*.h"
  # PATTERN "internal" EXCLUDE
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)

ament_package()
