cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0071 NEW)
project(waywiser)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

include_directories(include)

file(GLOB DISCOVERY_SETUP_FILES "discovery/*.sh")

install(
  FILES ${DISCOVERY_SETUP_FILES}
  DESTINATION discovery
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME}_py
  PACKAGE_DIR py_src/waywiser_py
)

install(
  PROGRAMS py_src/waywiser_py/waywiser_utils.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_environment_hooks(load_dotenv.bash)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
