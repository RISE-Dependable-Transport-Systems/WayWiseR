cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0071 NEW)
project(waywiser_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_program(COLCON_EXECUTABLE colcon)

set(COLCON_ALLOW_OVERRIDING_ARGS "")

if(COLCON_EXECUTABLE)
  execute_process(
    COMMAND ${COLCON_EXECUTABLE} build --help
    OUTPUT_VARIABLE COLCON_BUILD_HELP
    ERROR_QUIET
  )

  if(COLCON_BUILD_HELP MATCHES "--allow-overriding")
    set(COLCON_ALLOW_OVERRIDING_ARGS --allow-overriding ros_gz_bridge)
  endif()
endif()

set(ROS_GZ_HARMONIC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/external/ros_gz_harmonic")
set(ROS_GZ_HARMONIC_BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}/ros_gz_harmonic")
get_filename_component(WAYWISER_INSTALL_ROOT "${CMAKE_INSTALL_PREFIX}" DIRECTORY)
set(ROS_GZ_HARMONIC_INSTALL_DIR "${WAYWISER_INSTALL_ROOT}/ros_gz_harmonic")

if(EXISTS "${ROS_GZ_HARMONIC_DIR}/ros_gz_bridge/package.xml")
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
      ${COLCON_ALLOW_OVERRIDING_ARGS}
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

install(
  DIRECTORY launch config worlds
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
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  # Suppress xmllint: external simulation dependencies contain XML files that
  # do not conform to ROS XML conventions and should never be linted.
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
