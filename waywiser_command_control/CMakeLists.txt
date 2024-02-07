cmake_minimum_required(VERSION 3.8)
project(waywiser_command_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

include_directories(include)

add_library(emergency_stop_monitor_core SHARED src/rclcpp_components/emergency_stop_monitor_core.cpp)
ament_target_dependencies(emergency_stop_monitor_core geometry_msgs rclcpp rclcpp_components sensor_msgs std_msgs)
rclcpp_components_register_nodes(emergency_stop_monitor_core waywiser_command_control::EmergencyStopMonitor)

add_executable(emergency_stop_monitor src/emergency_stop_monitor.cpp)
target_link_libraries(emergency_stop_monitor emergency_stop_monitor_core)

install(
  TARGETS emergency_stop_monitor_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS emergency_stop_monitor RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights comment the
  # line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips cpplint (only works in a git repo) comment the line
  # when this package is in a git repo and when a copyright and license is added
  # to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
