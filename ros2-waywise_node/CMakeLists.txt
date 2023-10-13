cmake_minimum_required(VERSION 3.8)
project(ros2-waywise_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(waywise_rover src/waywise_rover.cpp)
ament_target_dependencies(waywise_rover rclcpp std_msgs nav_msgs geometry_msgs tf2_ros)

install(TARGETS
  waywise_rover
  DESTINATION lib/${PROJECT_NAME})

ament_package()
