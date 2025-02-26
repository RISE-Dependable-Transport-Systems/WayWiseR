cmake_minimum_required(VERSION 3.8)
project(waywiser_twist_safety)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(include)

# Generate msgs
set(ROSIDL_FILES)

file(GLOB MSG_FILES "${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg")

foreach(MSG_FILE ${MSG_FILES})
  get_filename_component(MSG_NAME ${MSG_FILE} NAME)
  list(APPEND ROSIDL_FILES msg/${MSG_NAME})
endforeach()

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  ${ROSIDL_FILES}
  DEPENDENCIES
  std_msgs
  builtin_interfaces)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}
                              rosidl_typesupport_cpp)

add_library(emergency_stop_monitor_core SHARED
            cpp_src/rclcpp_components/emergency_stop_monitor_core.cpp)
ament_target_dependencies(
  emergency_stop_monitor_core
  geometry_msgs
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs)
rclcpp_components_register_nodes(emergency_stop_monitor_core
                                 waywiser_twist_safety::EmergencyStopMonitor)
target_link_libraries(emergency_stop_monitor_core "${cpp_typesupport_target}")

add_executable(emergency_stop_monitor cpp_src/emergency_stop_monitor.cpp)
target_link_libraries(emergency_stop_monitor emergency_stop_monitor_core)

install(
  TARGETS emergency_stop_monitor_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS emergency_stop_monitor RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
