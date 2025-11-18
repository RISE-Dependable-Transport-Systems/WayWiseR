cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0071 NEW)
project(waywiser_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(rclpy REQUIRED)
find_package(urdf REQUIRED)

include_directories(include)

ament_python_install_package(${PROJECT_NAME}_py
  PACKAGE_DIR py_src/waywiser_description_py
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(
  DIRECTORY urdf sdf meshes scripts
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
  py_src/waywiser_description_py/waywiser_description_utils.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
