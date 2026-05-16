cmake_minimum_required(VERSION 3.5)
project(px4_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

if(NOT DEFINED PX4_AUTOPILOT_DIR)
  message(FATAL_ERROR "PX4_AUTOPILOT_DIR must point to a PX4-Autopilot checkout.")
endif()

set(PX4_MSGS_SOURCE_DIR "${PX4_AUTOPILOT_DIR}/msg")
set(PX4_SRVS_SOURCE_DIR "${PX4_AUTOPILOT_DIR}/srv")
set(PX4_GENERATED_INTERFACE_DIR "${CMAKE_CURRENT_BINARY_DIR}/interfaces")

if(NOT EXISTS "${PX4_MSGS_SOURCE_DIR}/CMakeLists.txt")
  message(FATAL_ERROR "PX4 message definitions were not found at ${PX4_MSGS_SOURCE_DIR}.")
endif()

file(REMOVE_RECURSE "${PX4_GENERATED_INTERFACE_DIR}")
file(MAKE_DIRECTORY
  "${PX4_GENERATED_INTERFACE_DIR}/msg"
  "${PX4_GENERATED_INTERFACE_DIR}/srv"
)

set(PX4_INTERFACES)

file(GLOB PX4_MSG_FILES
  "${PX4_MSGS_SOURCE_DIR}/*.msg"
  "${PX4_MSGS_SOURCE_DIR}/versioned/*.msg"
)
foreach(PX4_MSG_FILE ${PX4_MSG_FILES})
  get_filename_component(PX4_MSG_NAME "${PX4_MSG_FILE}" NAME)
  configure_file(
    "${PX4_MSG_FILE}"
    "${PX4_GENERATED_INTERFACE_DIR}/msg/${PX4_MSG_NAME}"
    COPYONLY
  )
  list(APPEND PX4_INTERFACES "${PX4_GENERATED_INTERFACE_DIR}:msg/${PX4_MSG_NAME}")
endforeach()

file(GLOB PX4_SRV_FILES
  "${PX4_SRVS_SOURCE_DIR}/*.srv"
  "${PX4_SRVS_SOURCE_DIR}/versioned/*.srv"
)
foreach(PX4_SRV_FILE ${PX4_SRV_FILES})
  get_filename_component(PX4_SRV_NAME "${PX4_SRV_FILE}" NAME)
  configure_file(
    "${PX4_SRV_FILE}"
    "${PX4_GENERATED_INTERFACE_DIR}/srv/${PX4_SRV_NAME}"
    COPYONLY
  )
  list(APPEND PX4_INTERFACES "${PX4_GENERATED_INTERFACE_DIR}:srv/${PX4_SRV_NAME}")
endforeach()

rosidl_generate_interfaces(${PROJECT_NAME}
  ${PX4_INTERFACES}
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
