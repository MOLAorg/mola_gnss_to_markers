# ------------------------------------------------------------------------------
#                             mola_gnss_to_markers
#
# Copyright (C) 2025, Jose Luis Blanco-Claraco, contributors (see git commit history)
# All rights reserved.
# Released under BSD 3-Clause License. See LICENSE.
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.8)
project(mola_gnss_to_markers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mrpt_nav_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(mrpt-topography REQUIRED)

add_executable(mola_gnss_to_marker_node src/mola_gnss_to_marker_node.cpp)
target_include_directories(mola_gnss_to_marker_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(mola_gnss_to_marker_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(
  mola_gnss_to_marker_node
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${mrpt_nav_interfaces_TARGETS}
  ${visualization_msgs_TARGETS}
)

target_link_libraries(mola_gnss_to_marker_node mrpt::topography)

install(TARGETS mola_gnss_to_marker_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
