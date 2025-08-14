cmake_minimum_required(VERSION 3.8)
project(mobile_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Add executables
add_executable(mobile_robot_controller src/mobile_robot_controller.cpp)
ament_target_dependencies(mobile_robot_controller
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(teleop_keyboard src/teleop_keyboard.cpp)
ament_target_dependencies(teleop_keyboard
  rclcpp
  geometry_msgs
)

add_executable(tray_controller src/tray_controller.cpp)
ament_target_dependencies(tray_controller
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
)

add_executable(tray_joint_relay src/tray_joint_relay.cpp)
ament_target_dependencies(tray_joint_relay
  rclcpp
  sensor_msgs
)

# Install executables
install(TARGETS
  mobile_robot_controller
  teleop_keyboard
  tray_controller
  tray_joint_relay
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install directories
install(DIRECTORY
  config
  launch
  meshes
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
