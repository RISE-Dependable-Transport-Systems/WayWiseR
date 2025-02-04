cmake_minimum_required(VERSION 3.8)
project(waywiser_carla)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(carla_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# C++ targets
add_executable(vehicle_odom_transform src/vehicle_odom_transform.cpp)
ament_target_dependencies(
  vehicle_odom_transform
  rclcpp
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs)

add_executable(emulated_angle_sensor src/emulated_angle_sensor.cpp)
ament_target_dependencies(
  emulated_angle_sensor
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
  std_msgs)

# Add the EmulatedRangeSensor executable
add_executable(emulated_range_sensor src/emulated_range_sensor.cpp)
ament_target_dependencies(
  emulated_range_sensor
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  OpenCV)

# Install targets
install(TARGETS vehicle_odom_transform RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS emulated_angle_sensor RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS emulated_range_sensor RUNTIME DESTINATION lib/${PROJECT_NAME})

# Python package
ament_python_install_package(${PROJECT_NAME}_py PACKAGE_DIR py_scripts)

install(PROGRAMS py_scripts/carla_osm_tile_server.py
        DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS py_scripts/waywiser_twist_to_carla_control.py
        DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS py_scripts/carla_orchestrator.py
        DESTINATION lib/${PROJECT_NAME})

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
