cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0071 NEW)
project(waywiser_test_runner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
set(PYTHON_EXECUTABLE ${Python3_EXECUTABLE})
set(PYTHON_INCLUDE_DIRS ${Python3_INCLUDE_DIRS})
set(PYTHON_LIBRARIES ${Python3_LIBRARIES})
set(PYTHON_EXECUTABLE ${Python3_EXECUTABLE} CACHE FILEPATH "Legacy Python executable for ROSIDL" FORCE)
set(PYTHON_INCLUDE_DIRS ${Python3_INCLUDE_DIRS} CACHE PATH "Legacy Python includes for ROSIDL" FORCE)
set(PYTHON_LIBRARIES ${Python3_LIBRARIES} CACHE FILEPATH "Legacy Python library for ROSIDL" FORCE)
unset(_lib CACHE)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

file(GLOB MSG_FILES "${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg")
set(ROSIDL_MSG_FILES)

foreach(MSG_FILE ${MSG_FILES})
  get_filename_component(MSG_NAME ${MSG_FILE} NAME)
  list(APPEND ROSIDL_MSG_FILES msg/${MSG_NAME})
endforeach()

set(PYTHON_EXECUTABLE ${Python3_EXECUTABLE})
set(PYTHON_INCLUDE_DIRS ${Python3_INCLUDE_DIRS})
set(PYTHON_LIBRARIES ${Python3_LIBRARIES})
set(PYTHON_EXECUTABLE ${Python3_EXECUTABLE} CACHE FILEPATH "Legacy Python executable for ROSIDL" FORCE)
set(PYTHON_INCLUDE_DIRS ${Python3_INCLUDE_DIRS} CACHE PATH "Legacy Python includes for ROSIDL" FORCE)
set(PYTHON_LIBRARIES ${Python3_LIBRARIES} CACHE FILEPATH "Legacy Python library for ROSIDL" FORCE)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${ROSIDL_MSG_FILES}
  DEPENDENCIES std_msgs
)

ament_python_install_package(${PROJECT_NAME}_py
  PACKAGE_DIR py_src/waywiser_test_runner_py
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
  py_src/waywiser_test_runner_py/track_test_runner.py
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
