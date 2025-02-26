cmake_minimum_required(VERSION 3.8)
project(waywiser_agrarsense)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME}_py PACKAGE_DIR py_src)

file(GLOB ROS_BRIDGE_FILES "ros_bridge/*")
install(FILES ${ROS_BRIDGE_FILES} DESTINATION ros_bridge)

install(DIRECTORY launch config ros_bridge DESTINATION share/${PROJECT_NAME})

install(
  PROGRAMS py_src/agrarsense_orchestrator.py py_src/vehicle_tf_publisher.py
           py_src/waywiser_twist_to_agrarsense_control.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
