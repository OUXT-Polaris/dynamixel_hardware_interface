cmake_minimum_required(VERSION 3.5)
project(dynamixel_hardware_interface)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(controller_interface REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_control REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(Boost REQUIRED)

add_library(
  dynamixel_hardware_interface
  SHARED
  src/dynamixel_hardware_interface.cpp
  src/motor_base.cpp
  src/util.cpp
)
target_include_directories(
  dynamixel_hardware_interface
  PRIVATE
  include
  ${Boost_INCLUDE_DIRS}
)
ament_target_dependencies(
  dynamixel_hardware_interface
  dynamixel_sdk
  hardware_interface
  pluginlib
  rclcpp
  ros2_control
  realtime_tools
)

pluginlib_export_plugin_description_file(hardware_interface dynamixel_hardware_interface.xml)

add_library(
  dynamixel_diagnostic_controller
  SHARED
  src/dynamixel_diagnostic_controller.cpp
)
target_include_directories(
  dynamixel_diagnostic_controller
  PRIVATE
  include
  ${Boost_INCLUDE_DIRS}
)
ament_target_dependencies(
  dynamixel_diagnostic_controller
  dynamixel_sdk
  controller_interface
  pluginlib
  rclcpp
  ros2_control
  diagnostic_updater
  diagnostic_msgs
  realtime_tools
)
pluginlib_export_plugin_description_file(controller_interface dynamixel_diagnostic_controller.xml)

install(
  TARGETS dynamixel_hardware_interface dynamixel_diagnostic_controller
  DESTINATION lib
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  dynamixel_hardware_interface
  dynamixel_diagnostic_controller
)
ament_export_dependencies(
  dynamixel_sdk
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  ros2_control
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
