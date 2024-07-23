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

add_executable(map_to_odom_tf_publisher src/map_to_odom_tf_publisher.cpp)
ament_target_dependencies(map_to_odom_tf_publisher rclcpp tf2_ros tf2_geometry_msgs geometry_msgs nav_msgs)

add_executable(emulated_angle_sensor src/emulated_angle_sensor.cpp)
ament_target_dependencies(emulated_angle_sensor rclcpp tf2 tf2_ros geometry_msgs std_msgs)

install(TARGETS map_to_odom_tf_publisher RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS emulated_angle_sensor RUNTIME DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

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

ament_package()
