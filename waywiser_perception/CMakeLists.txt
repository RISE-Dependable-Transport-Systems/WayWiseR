cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0071 NEW)
project(waywiser_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

if(POLICY CMP0125)
  cmake_policy(SET CMP0125 NEW)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_C_STANDARD 23)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
unset(_lib CACHE)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)

include_directories(include)
ament_export_include_directories(include)

add_executable(octomap_node cpp_src/octomap.cpp)
ament_target_dependencies(octomap_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_sensor_msgs
  octomap_msgs
)
target_link_libraries(octomap_node octomap)

install(
  TARGETS octomap_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

add_library(point_cloud_transformer_core SHARED
  cpp_src/rclcpp_components/point_cloud_transformer_core.cpp
)

ament_target_dependencies(point_cloud_transformer_core
  rclcpp
  sensor_msgs
  tf2_ros
  tf2_sensor_msgs
  geometry_msgs
  rclcpp_components
)

rclcpp_components_register_nodes(point_cloud_transformer_core
  waywiser_perception::PointCloudTransformer
)

add_executable(point_cloud_transformer cpp_src/point_cloud_transformer.cpp)
target_link_libraries(point_cloud_transformer point_cloud_transformer_core)

install(
  TARGETS point_cloud_transformer_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS point_cloud_transformer
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME}_py
  PACKAGE_DIR py_src/waywiser_perception_py
)

install(
  PROGRAMS
  py_src/waywiser_perception_py/yolo_node.py
  py_src/waywiser_perception_py/rgbd_to_pointcloud.py
  py_src/waywiser_perception_py/collision_monitor.py
  py_src/waywiser_perception_py/camera_info_publisher.py
  py_src/waywiser_perception_py/rgb_to_grayscale.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_xmllint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
