cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0071 NEW)
project(waywiser_carla)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_C_STANDARD 23)

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
add_executable(emulated_angle_sensor_node cpp_src/emulated_angle_sensor_node.cpp)
ament_target_dependencies(emulated_angle_sensor_node
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
  tf2_geometry_msgs
  std_msgs
  waywiser
)

# Add the EmulatedRangeSensor executable
add_executable(emulated_range_sensor_array_node cpp_src/emulated_range_sensor_array_node.cpp)
target_include_directories(emulated_range_sensor_array_node PUBLIC ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(emulated_range_sensor_array_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
)
target_link_libraries(emulated_range_sensor_array_node ${OpenCV_LIBRARIES})

# Install targets
install(
  TARGETS emulated_angle_sensor_node emulated_range_sensor_array_node
  DESTINATION lib/${PROJECT_NAME}
)

# Python package
ament_python_install_package(${PROJECT_NAME}_py
  PACKAGE_DIR py_src/waywiser_carla_py
)

install(
  PROGRAMS
  py_src/waywiser_carla_py/carla_osm_tile_server_node.py
  py_src/waywiser_carla_py/waywiser_to_carla_control_node.py
  py_src/waywiser_carla_py/carla_orchestrator_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_flake8
    ament_cmake_lint_cmake
    ament_cmake_pep257
    ament_cmake_uncrustify
  )
  set(ament_cmake_xmllint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)

  file(GLOB_RECURSE WAYWISER_CARLA_PYTHON_FILES
    "launch/*.py"
    "py_src/*.py"
  )
  file(GLOB_RECURSE WAYWISER_CARLA_CPP_FILES
    "cpp_src/*.cpp"
    "cpp_src/*.h"
    "cpp_src/*.hpp"
  )

  if(WAYWISER_CARLA_PYTHON_FILES)
    ament_flake8(${WAYWISER_CARLA_PYTHON_FILES})
    ament_pep257(${WAYWISER_CARLA_PYTHON_FILES})
  endif()
  ament_lint_cmake(CMakeLists.txt)
  if(WAYWISER_CARLA_CPP_FILES)
    ament_uncrustify(${WAYWISER_CARLA_CPP_FILES})
  endif()
endif()

ament_package()
