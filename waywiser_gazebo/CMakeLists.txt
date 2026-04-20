cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0071 NEW)
project(waywiser_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(xacro REQUIRED)

find_program(MAKE_EXECUTABLE make REQUIRED)

set(PX4_AUTOPILOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/external/PX4-Autopilot")
set(PX4_GZ_MODELS_DIR "${PX4_AUTOPILOT_DIR}/Tools/simulation/gz/models")
execute_process(
  COMMAND ${Python3_EXECUTABLE} -c "import site; print(site.getusersitepackages())"
  OUTPUT_VARIABLE PYTHON3_USER_SITE
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

if(DEFINED ENV{PYTHONPATH} AND NOT "$ENV{PYTHONPATH}" STREQUAL "")
  set(PX4_PYTHONPATH "${PYTHON3_USER_SITE}:$ENV{PYTHONPATH}")
else()
  set(PX4_PYTHONPATH "${PYTHON3_USER_SITE}")
endif()

if(EXISTS "${PX4_AUTOPILOT_DIR}/Makefile")
  if(NOT EXISTS "${PX4_GZ_MODELS_DIR}")
    message(
      FATAL_ERROR
      "PX4 Gazebo models are missing. Run "
      "'git -C ${PX4_AUTOPILOT_DIR} submodule update --init --recursive Tools/simulation/gz' "
      "and rebuild."
    )
  endif()

  add_custom_target(
    waywiser_gazebo_px4_sitl ALL
    COMMAND ${CMAKE_COMMAND} -E env
      PYTHONPATH=${PX4_PYTHONPATH}
      PX4_GZ_STANDALONE=1
      HEADLESS=1
      ${MAKE_EXECUTABLE} px4_sitl_default
    WORKING_DIRECTORY ${PX4_AUTOPILOT_DIR}
    USES_TERMINAL
    COMMENT "Building PX4 SITL default target for waywiser_gazebo"
  )
else()
  message(
    WARNING
    "PX4-Autopilot submodule not found at ${PX4_AUTOPILOT_DIR}. "
    "Launch files that depend on PX4 SITL will not work until the submodule is initialized."
  )
endif()

install(
  DIRECTORY launch config worlds
  DESTINATION share/${PROJECT_NAME}
)

if(EXISTS "${PX4_AUTOPILOT_DIR}")
  install(
    DIRECTORY external/PX4-Autopilot
    DESTINATION share/${PROJECT_NAME}/external
    USE_SOURCE_PERMISSIONS
    PATTERN ".git" EXCLUDE
    PATTERN "__pycache__" EXCLUDE
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
