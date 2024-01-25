cmake_minimum_required(VERSION 3.8)
project(waywiser_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

file(GLOB MSG_FILES "${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg")
set(ROSIDL_MSG_FILES)

foreach(MSG_FILE ${MSG_FILES})
  get_filename_component(MSG_NAME ${MSG_FILE} NAME)
  list(APPEND ROSIDL_MSG_FILES msg/${MSG_NAME})
endforeach()

rosidl_generate_interfaces(${PROJECT_NAME}
  ${ROSIDL_MSG_FILES}
  DEPENDENCIES geometry_msgs std_msgs
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(py_scripts)

install(PROGRAMS
  py_scripts/yolov8_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
