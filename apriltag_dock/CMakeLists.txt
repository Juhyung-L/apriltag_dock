cmake_minimum_required(VERSION 3.8)
project(apriltag_dock)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(apriltag_dock_msgs REQUIRED)

set(server_dependencies
  rclcpp
  rclcpp_action
  geometry_msgs
  apriltag_msgs
  tf2_ros
  tf2
  nav2_msgs
  nav2_util
  apriltag_dock_msgs
)

set(client_dependencies
  rclcpp
  rclcpp_action
  apriltag_dock_msgs
)

include_directories(
  include
)

add_executable(server src/auto_dock_server.cpp src/find_apriltag.cpp)
ament_target_dependencies(server ${server_dependencies})

add_executable(client src/auto_dock_client.cpp src/main.cpp)
ament_target_dependencies(client ${client_dependencies})

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

INSTALL(TARGETS
  server
  client
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
