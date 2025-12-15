cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0071 NEW)
project(waywiser_hwbringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(waywiser_perception REQUIRED)

ament_python_install_package(${PROJECT_NAME}_py
  PACKAGE_DIR py_src/waywiser_hwbringup_py
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
  py_src/waywiser_hwbringup_py/urm14_sensor_array_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_flake8
    ament_cmake_pep257
  )
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  set(EXCLUDED_FILES py_src/waywiser_hwbringup_py/dfrobot_ch432t.py)

  # Get all Python files
  file(GLOB_RECURSE ALL_PYTHON_FILES
    "py_src/waywiser_hwbringup_py/*.py"
    "launch/*.py"
  )

  # Remove the excluded files from the list
  foreach(EXCLUDED_FILE ${EXCLUDED_FILES})
    list(FILTER ALL_PYTHON_FILES EXCLUDE REGEX ".*${EXCLUDED_FILE}$")
  endforeach()

  # Run linters with filtered file list
  if(ALL_PYTHON_FILES)
    ament_flake8(${ALL_PYTHON_FILES})
    ament_pep257(${ALL_PYTHON_FILES})
  endif()
endif()

ament_package()
