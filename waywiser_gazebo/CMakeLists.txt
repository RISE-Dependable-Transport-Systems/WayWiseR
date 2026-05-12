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

# Set to OFF in CI to skip PX4 SITL and ros_gz_harmonic builds (pass -DWAYWISER_BUILD_SIMULATION=OFF).
option(WAYWISER_BUILD_SIMULATION "Build simulation dependencies (PX4 SITL, ros_gz_harmonic)" ON)

find_program(BASH_EXECUTABLE bash REQUIRED)
find_program(COLCON_EXECUTABLE colcon)
find_program(GIT_EXECUTABLE git REQUIRED)
find_program(MAKE_EXECUTABLE make REQUIRED)
set(PX4_ZENOH_PATCH_FILE "${CMAKE_CURRENT_SOURCE_DIR}/external/px4_zenoh_humble_compat.patch")

set(PX4_AUTOPILOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/external/PX4-Autopilot")
set(PX4_GZ_MODELS_DIR "${PX4_AUTOPILOT_DIR}/Tools/simulation/gz/models")
set(ROS_GZ_HARMONIC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/external/ros_gz_harmonic")
set(ROS_GZ_HARMONIC_BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}/ros_gz_harmonic")
set(PX4_PATCHED_BUILD_SCRIPT "${CMAKE_CURRENT_BINARY_DIR}/build_px4_sitl_with_patch.sh")
get_filename_component(WAYWISER_INSTALL_ROOT "${CMAKE_INSTALL_PREFIX}" DIRECTORY)
set(ROS_GZ_HARMONIC_INSTALL_DIR "${WAYWISER_INSTALL_ROOT}/ros_gz_harmonic")
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

if(WAYWISER_BUILD_SIMULATION AND EXISTS "${ROS_GZ_HARMONIC_DIR}/ros_gz_bridge/package.xml")
  if(COLCON_EXECUTABLE)
    add_custom_target(
      waywiser_gazebo_ros_gz_harmonic ALL
      COMMAND ${CMAKE_COMMAND} -E env
        MAKEFLAGS=
        GZ_VERSION=harmonic
        GZ_SIM_DIR=/usr/lib/x86_64-linux-gnu/cmake/gz-sim8
        GZ_TRANSPORT_DIR=/usr/lib/x86_64-linux-gnu/cmake/gz-transport13
        GZ_MSGS_DIR=/usr/lib/x86_64-linux-gnu/cmake/gz-msgs10
        GZ_SENSORS_DIR=/usr/lib/x86_64-linux-gnu/cmake/gz-sensors8
        ${COLCON_EXECUTABLE}
      --log-base ${ROS_GZ_HARMONIC_BUILD_DIR}/log
      build
      --merge-install
      --symlink-install
      --base-paths ${ROS_GZ_HARMONIC_DIR}
      --build-base ${ROS_GZ_HARMONIC_BUILD_DIR}/build
      --install-base ${ROS_GZ_HARMONIC_INSTALL_DIR}
      --packages-up-to ros_gz_bridge
      --allow-overriding ros_gz_bridge
      --cmake-args -DBUILD_TESTING=OFF
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
      USES_TERMINAL
      COMMENT "Building optional Gazebo Harmonic ros_gz_bridge workspace for PX4 SITL"
    )
  else()
    message(
      WARNING
      "ros_gz_harmonic submodule was found at ${ROS_GZ_HARMONIC_DIR}, but 'colcon' "
      "is not available in PATH. Skipping optional Harmonic bridge build."
    )
  endif()
else()
  message(
    STATUS
    "Optional ros_gz_harmonic submodule not found at ${ROS_GZ_HARMONIC_DIR}. "
    "Skipping Harmonic bridge build."
  )
endif()

if(WAYWISER_BUILD_SIMULATION AND EXISTS "${PX4_AUTOPILOT_DIR}/Makefile")
  if(NOT EXISTS "${PX4_GZ_MODELS_DIR}")
    message(
      FATAL_ERROR
      "PX4 Gazebo models are missing. Run "
      "'git -C ${PX4_AUTOPILOT_DIR} submodule update --init --recursive Tools/simulation/gz' "
      "and rebuild."
    )
  endif()

  configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/scripts/build_px4_sitl_with_patch.sh.in
    ${PX4_PATCHED_BUILD_SCRIPT}
    @ONLY
  )

  add_custom_target(
    waywiser_gazebo_px4_sitl ALL
    COMMAND ${CMAKE_COMMAND} -E env
      PYTHONPATH=${PX4_PYTHONPATH}
      PX4_GZ_STANDALONE=1
      HEADLESS=1
      MAKEFLAGS=$ENV{MAKEFLAGS}
      ${BASH_EXECUTABLE} ${PX4_PATCHED_BUILD_SCRIPT}
    WORKING_DIRECTORY ${PX4_AUTOPILOT_DIR}
    USES_TERMINAL
    COMMENT "Building PX4 SITL Zenoh target for waywiser_gazebo"
  )

  if(TARGET waywiser_gazebo_ros_gz_harmonic)
    add_dependencies(waywiser_gazebo_px4_sitl waywiser_gazebo_ros_gz_harmonic)
  endif()
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

if(WAYWISER_BUILD_SIMULATION AND EXISTS "${PX4_AUTOPILOT_DIR}")
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
  set(AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_flake8
    ament_cmake_lint_cmake
    ament_cmake_pep257
    ament_cmake_uncrustify
  )
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  # Suppress xmllint: the external/ submodules contain thousands of XML files
  # (PX4-Autopilot CycloneDDS configs, JSBSim models, etc.) that do not
  # conform to ROS XML conventions and should never be linted.
  set(ament_cmake_xmllint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)

  ament_flake8(
    launch/gazebo.launch.py
    launch/px4_sitl.launch.py
    launch/spawn.launch.py
  )
  ament_pep257(
    launch/gazebo.launch.py
    launch/px4_sitl.launch.py
    launch/spawn.launch.py
  )
  ament_lint_cmake(CMakeLists.txt)
endif()

ament_package()
