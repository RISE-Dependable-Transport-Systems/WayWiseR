cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0071 NEW)
project(waywiser)

if(POLICY CMP0125)
  cmake_policy(SET CMP0125 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
unset(_lib CACHE)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

add_library(${PROJECT_NAME}
  cpp_src/waywiser_utils.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} rclcpp)

include_directories(include)

install(
  DIRECTORY discovery
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
  PATTERN "__pycache__" EXCLUDE
)

install(
  DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
  PATTERN "bootstrap" EXCLUDE
  PATTERN "package" EXCLUDE
)

install(
  PROGRAMS
    scripts/configure_env
    scripts/configure_env.bash
    scripts/fastdds_server
    scripts/initialize
    scripts/initialize.bash
    scripts/install_waywiser_debs.bash
    scripts/setup_venv
    scripts/setup_venv.bash
    scripts/zenoh_router
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES ${CMAKE_CURRENT_SOURCE_DIR}/../pyproject.toml
  DESTINATION share/${PROJECT_NAME}/python
)

install(
  FILES ${CMAKE_CURRENT_SOURCE_DIR}/../.env.example
  DESTINATION share/${PROJECT_NAME}
  RENAME waywiser.env.example
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

ament_python_install_package(${PROJECT_NAME}_py
  PACKAGE_DIR py_src/waywiser_py
)

install(
  PROGRAMS py_src/waywiser_py/waywiser_utils.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_environment_hooks(scripts/waywiser_env.bash)
ament_environment_hooks(discovery/scripts/fastdds_client_setup.bash)
ament_environment_hooks(discovery/scripts/zenoh_client_setup.bash)
ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_xmllint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
