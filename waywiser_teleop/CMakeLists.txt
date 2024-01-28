cmake_minimum_required(VERSION 3.8)
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
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

add_executable(joy_emergency_stop src/joy_emergency_stop.cpp)
ament_target_dependencies(joy_emergency_stop rclcpp sensor_msgs std_msgs)

add_executable(twist_angular_correction src/twist_angular_correction.cpp)
ament_target_dependencies(twist_angular_correction geometry_msgs rclcpp sensor_msgs std_msgs)

install(TARGETS joy_emergency_stop RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS twist_angular_correction RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_python_install_package(py_scripts)

install(PROGRAMS
  py_scripts/twist_keyboard.py
  DESTINATION lib/${PROJECT_NAME}
)

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
