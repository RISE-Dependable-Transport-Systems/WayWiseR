cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0071 NEW)
project(waywiser_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Python3 COMPONENTS Interpreter REQUIRED)
find_package(xacro REQUIRED)
find_program(BASH_EXECUTABLE bash REQUIRED)
find_program(COLCON_EXECUTABLE colcon)
find_program(GIT_EXECUTABLE git REQUIRED)

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
set(PX4_AUTOPILOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../waywiser_core/external/PX4-Autopilot")
set(PX4_ZENOH_PATCH_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../waywiser_core/external/px4_zenoh.patch")
set(PX4_GZ_MODELS_DIR "${PX4_AUTOPILOT_DIR}/Tools/simulation/gz/models")
set(PX4_PATCHED_BUILD_SCRIPT "${CMAKE_CURRENT_BINARY_DIR}/build_px4_sitl_with_patch.sh")
set(WAYWISER_PX4_BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}/px4_sitl_zenoh_build")
set(WAYWISER_PX4_SITL_DIR
  "${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/px4_sitl_zenoh")
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

if(EXISTS "${PX4_AUTOPILOT_DIR}/Makefile")
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
    COMMENT "Building PX4 SITL Zenoh target into ${WAYWISER_PX4_SITL_DIR}"
  )
else()
  message(
    WARNING
    "PX4-Autopilot submodule not found at ${PX4_AUTOPILOT_DIR}. "
    "PX4 SITL launch files will not work until the submodule is initialized."
  )
endif()

install(
  DIRECTORY launch config worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME}_py
  PACKAGE_DIR py_src/waywiser_gazebo_py
)

install(
  PROGRAMS
  py_src/waywiser_gazebo_py/gazebo_orchestrator_node.py
  py_src/waywiser_gazebo_py/gazebo_osm_tile_server_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(EXISTS "${PX4_AUTOPILOT_DIR}")
  set(PX4_INSTALL_DIR share/${PROJECT_NAME}/px4_sitl_zenoh)

  install(
    DIRECTORY ${WAYWISER_PX4_SITL_DIR}/
    DESTINATION ${PX4_INSTALL_DIR}
    USE_SOURCE_PERMISSIONS
    OPTIONAL
    PATTERN "CMakeFiles" EXCLUDE
    PATTERN "mavlink" EXCLUDE
    PATTERN "msg" EXCLUDE
    PATTERN "OpticalFlow" EXCLUDE
    PATTERN "platforms" EXCLUDE
    PATTERN "src" EXCLUDE
    PATTERN "uORB" EXCLUDE
    PATTERN "__pycache__" EXCLUDE
  )

  install(
    DIRECTORY ${PX4_AUTOPILOT_DIR}/Tools/simulation/gz/models
              ${PX4_AUTOPILOT_DIR}/Tools/simulation/gz/worlds
    DESTINATION ${PX4_INSTALL_DIR}/Tools/simulation/gz
    USE_SOURCE_PERMISSIONS
    PATTERN ".git" EXCLUDE
    PATTERN "__pycache__" EXCLUDE
  )

  install(
    FILES ${PX4_AUTOPILOT_DIR}/ROMFS/px4fmu_common/init.d-posix/px4-rc.gzsim
    DESTINATION ${PX4_INSTALL_DIR}/ROMFS/px4fmu_common/init.d-posix
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_flake8
    ament_cmake_lint_cmake
    ament_cmake_pep257
    ament_cmake_uncrustify
    ament_cmake_cppcheck
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
    launch/gazebo_orchestrator.launch.py
    launch/gazebo_osm_tile_server.launch.py
    py_src/waywiser_gazebo_py/gazebo_osm_tile_server_node.py
  )
  ament_pep257(
    launch/gazebo_orchestrator.launch.py
    launch/gazebo_osm_tile_server.launch.py
    py_src/waywiser_gazebo_py/gazebo_osm_tile_server_node.py
  )
  ament_lint_cmake(CMakeLists.txt)
endif()

ament_package()
