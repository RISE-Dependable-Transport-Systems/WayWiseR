cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0071 NEW)
project(waywiser_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(joy REQUIRED)
find_package(teleop_twist_joy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(waywiser_twist_safety REQUIRED)
find_package(waywiser_core REQUIRED)
find_package(waywiser REQUIRED)

add_executable(twist_angular_correction cpp_src/twist_angular_correction.cpp)
ament_target_dependencies(twist_angular_correction
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
)

install(
  TARGETS twist_angular_correction
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME}_py
  PACKAGE_DIR py_src/waywiser_teleop_py
)

install(
  PROGRAMS py_src/waywiser_teleop_py/control_tower.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY user_interface/control_tower/
  DESTINATION share/${PROJECT_NAME}/user_interface/control_tower
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_xmllint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
