cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0071 NEW)
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
find_package(waywiser REQUIRED)

# C++ targets
add_executable(emulated_angle_sensor cpp_src/emulated_angle_sensor.cpp)
ament_target_dependencies(emulated_angle_sensor
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
  tf2_geometry_msgs
  std_msgs
  waywiser
)

# Add the EmulatedRangeSensor executable
add_executable(emulated_range_sensor_array cpp_src/emulated_range_sensor_array.cpp)
ament_target_dependencies(emulated_range_sensor_array
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  OpenCV
)

# Install targets
install(
  TARGETS emulated_angle_sensor emulated_range_sensor_array
  DESTINATION lib/${PROJECT_NAME}
)

# Python package
ament_python_install_package(${PROJECT_NAME}_py
  PACKAGE_DIR py_src/waywiser_carla_py
)

install(
  PROGRAMS
  py_src/waywiser_carla_py/carla_osm_tile_server.py
  py_src/waywiser_carla_py/waywiser_to_carla_control.py
  py_src/waywiser_carla_py/carla_orchestrator.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
