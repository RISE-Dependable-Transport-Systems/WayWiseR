cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0071 NEW)
project(waywiser_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_C_STANDARD 23)

include(ExternalProject)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Python3 COMPONENTS Interpreter REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS regex)
find_package(rosidl_default_generators REQUIRED)

find_package(waywiser REQUIRED)
find_package(waywiser_description REQUIRED)
find_package(waywiser_twist_safety REQUIRED)

# Qt5
find_package(
  Qt5
  COMPONENTS Core SerialPort Network Widgets
  REQUIRED)
find_package(MAVSDK REQUIRED)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(PX4_AUTOPILOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/external/PX4-Autopilot")
set(PX4_MSGS_SOURCE_DIR "${PX4_AUTOPILOT_DIR}/msg")
set(PX4_MSGS_VENDOR_TEMPLATE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/external/px4_msgs_vendor")

if(EXISTS "${PX4_MSGS_SOURCE_DIR}/CMakeLists.txt")
  ExternalProject_Add(px4_msgs_vendor
    SOURCE_DIR "${PX4_MSGS_VENDOR_TEMPLATE_DIR}"
    BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/px4_msgs_vendor_build"
    INSTALL_DIR "${CMAKE_INSTALL_PREFIX}"
    UPDATE_COMMAND ""
    TEST_COMMAND ""
    CMAKE_ARGS
      -DBUILD_TESTING=OFF
      -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
      -DPython3_EXECUTABLE=${Python3_EXECUTABLE}
      -DPX4_AUTOPILOT_DIR=${PX4_AUTOPILOT_DIR}
  )
else()
  message(FATAL_ERROR
    "PX4-Autopilot submodule is missing at ${PX4_AUTOPILOT_DIR}. Run "
    "'make setup' to initialize the required PX4 checkout."
  )
endif()

# Instead of using find_package(px4_msgs), we manually specify the include directories
# since the external project builds it during the make phase, not configure phase.
include_directories(${CMAKE_INSTALL_PREFIX}/include/px4_msgs/px4_msgs/msg)
include_directories(${CMAKE_INSTALL_PREFIX}/include/px4_msgs)

include_directories(include WayWise/)

file(GLOB MSG_FILES "${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg")
set(ROSIDL_MSG_FILES)

foreach(MSG_FILE ${MSG_FILES})
  get_filename_component(MSG_NAME ${MSG_FILE} NAME)
  list(APPEND ROSIDL_MSG_FILES msg/${MSG_NAME})
endforeach()

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  ${ROSIDL_MSG_FILES}
  DEPENDENCIES
  geometry_msgs
  nav_msgs
  builtin_interfaces
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}
  rosidl_typesupport_cpp
)

set(COMMON_WAYWISE_SOURCES
  WayWise/core/pospoint.cpp
  WayWise/core/vbytearray.cpp
  WayWise/core/coordinatetransforms.h
  WayWise/vehicles/objectstate.cpp
  WayWise/vehicles/vehiclestate.cpp
  WayWise/vehicles/controller/movementcontroller.cpp
  WayWise/vehicles/controller/servocontroller.cpp
  WayWise/logger/logger.cpp
  WayWise/communication/parameterserver.cpp
  WayWise/sensors/camera/gimbal.h
  WayWise/core/simplewatchdog.cpp
)

set(WAYWISE_LOCALIZATION_SOURCES
  WayWise/sensors/gnss/gnssreceiver.cpp
  WayWise/sensors/gnss/rtcm3_simple.cpp
  WayWise/sensors/gnss/rtcmclient.cpp
  WayWise/sensors/gnss/ublox.cpp
  WayWise/sensors/gnss/ubloxrover.cpp
  WayWise/sensors/fusion/sdvpvehiclepositionfuser.cpp
)

set(COMMON_WAYWISE_AP_SOURCES
  WayWise/vehicles/controller/motorcontroller.h
  WayWise/autopilot/purepursuitwaypointfollower.cpp
  WayWise/autopilot/purepursuitwaypointfollower.h
  WayWise/autopilot/waypointfollower.h
  WayWise/autopilot/followpoint.cpp
  WayWise/communication/mavsdkvehicleserver.cpp
  WayWise/communication/jsonstreamparsertcp.cpp
  WayWise/communication/vehicleserver.h
  WayWise/communication/vehicleconnections/vehicleconnection.cpp
  WayWise/communication/mavlinkparameterserver.cpp
  WayWise/routeplanning/routeutils.cpp
  WayWise/routeplanning/routeutils.h
)

set(WAYWISE_CAR_SOURCES
  WayWise/vehicles/carstate.cpp
  WayWise/vehicles/controller/carmovementcontroller.cpp
  WayWise/vehicles/controller/motorcontroller.h
  WayWise/vehicles/controller/vescmotorcontroller.cpp
  WayWise/external/vesc/vescpacket.cpp
  WayWise/external/pi-bno055/i2c_bno055.c
  WayWise/external/VL53L0X/tof.h
  WayWise/external/VL53L0X/tof.c
  WayWise/sensors/imu/bno055orientationupdater.cpp
  WayWise/sensors/imu/imuorientationupdater.cpp
  WayWise/sensors/tof/vl53l0xtofsensor.cpp
  WayWise/sensors/tof/tofsensor.h
)

set(WAYWISE_COPTER_SOURCES
  WayWise/vehicles/copterstate.cpp
)

set(WAYWISE_TRUCK_SOURCES
  WayWise/vehicles/truckstate.cpp
  WayWise/vehicles/trailerstate.cpp
  WayWise/external/pi-as5600/driver_as5600.c
  WayWise/external/pi-as5600/iic.c
  WayWise/external/pi-as5600/driver_as5600_basic.c
  WayWise/external/pi-as5600/raspberrypi4b_driver_as5600_interface.c
  WayWise/sensors/angle/as5600updater.cpp
  WayWise/sensors/angle/anglesensorupdater.cpp
)

add_executable(
  waywiser_localization_node
  cpp_src/waywiser_localization_node.cpp
  cpp_src/waywiser_core_utils.cpp
  cpp_src/localization_component.cpp
  cpp_src/qobject_node.cpp
  ${COMMON_WAYWISE_SOURCES}
  ${WAYWISE_LOCALIZATION_SOURCES}
)

add_executable(
  waywiser_car_node
  cpp_src/waywiser_car_node.cpp
  cpp_src/waywiser_car_node_core.cpp
  cpp_src/waywiser_core_utils.cpp
  cpp_src/car_autopilot_component.cpp
  cpp_src/car_interface_component.cpp
  cpp_src/qobject_node.cpp
  ${COMMON_WAYWISE_SOURCES}
  ${WAYWISE_LOCALIZATION_SOURCES}
  ${WAYWISE_CAR_SOURCES}
  ${COMMON_WAYWISE_AP_SOURCES}
)

add_executable(
  waywiser_truck_node
  cpp_src/waywiser_truck_node.cpp
  cpp_src/waywiser_truck_node_core.cpp
  cpp_src/waywiser_car_node_core.cpp
  cpp_src/waywiser_core_utils.cpp
  cpp_src/truck_autopilot_component.cpp
  cpp_src/truck_interface_component.cpp
  cpp_src/car_autopilot_component.cpp
  cpp_src/car_interface_component.cpp
  cpp_src/qobject_node.cpp
  ${COMMON_WAYWISE_SOURCES}
  ${WAYWISE_LOCALIZATION_SOURCES}
  ${WAYWISE_CAR_SOURCES}
  ${WAYWISE_TRUCK_SOURCES}
  ${COMMON_WAYWISE_AP_SOURCES}
)

add_executable(
  waywiser_copter_node
  cpp_src/waywiser_copter_node.cpp
  cpp_src/waywiser_copter_node_core.cpp
  cpp_src/waywiser_core_utils.cpp
  cpp_src/copter_autopilot_component.cpp
  cpp_src/copter_interface_component.cpp
  cpp_src/qobject_node.cpp
  ${COMMON_WAYWISE_SOURCES}
  ${WAYWISE_LOCALIZATION_SOURCES}
  ${WAYWISE_COPTER_SOURCES}
  ${COMMON_WAYWISE_AP_SOURCES}
)

add_executable(navsatfix_extended_wrapper_node
  cpp_src/navsatfix_extended_wrapper_node.cpp
  cpp_src/waywiser_core_utils.cpp
  ${COMMON_WAYWISE_SOURCES}
)

set(LOCALIZATION_LIBRARIES
  Qt5::Core
  Qt5::SerialPort
  Qt5::Network
  urdf::urdf
  ${cpp_typesupport_target}
)

set(CAR_LIBRARIES
  Qt5::Core
  Qt5::SerialPort
  Qt5::Network
  Qt5::Widgets
  urdf::urdf
  MAVSDK::mavsdk
  Boost::regex
  ${cpp_typesupport_target}
)

set(TRUCK_LIBRARIES
  ${CAR_LIBRARIES}
)

set(COPTER_LIBRARIES
  ${CAR_LIBRARIES}
  ${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__rosidl_typesupport_cpp.so
  ${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__rosidl_typesupport_c.so
  ${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__rosidl_typesupport_introspection_cpp.so
  ${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__rosidl_typesupport_introspection_c.so
  ${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__rosidl_generator_c.so
)

set(NAVSATFIX_EXTENDED_WRAPPER_LIBRARIES
  Qt5::Core
  ${cpp_typesupport_target}
)

target_link_libraries(waywiser_localization_node ${LOCALIZATION_LIBRARIES})
target_link_libraries(waywiser_car_node ${CAR_LIBRARIES})
target_link_libraries(waywiser_truck_node ${TRUCK_LIBRARIES})
target_link_libraries(waywiser_copter_node ${COPTER_LIBRARIES})
target_link_libraries(navsatfix_extended_wrapper_node ${NAVSATFIX_EXTENDED_WRAPPER_LIBRARIES})

function(add_qt_includes target)
  target_include_directories(${target} SYSTEM PRIVATE
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5SerialPort_INCLUDE_DIRS}
    ${Qt5Network_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
  )
endfunction()

add_qt_includes(waywiser_localization_node)
add_qt_includes(waywiser_car_node)
add_qt_includes(waywiser_truck_node)
add_qt_includes(waywiser_copter_node)
add_qt_includes(navsatfix_extended_wrapper_node)

set(LOCALIZATION_DEPENDENCIES
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  waywiser
  waywiser_description
  waywiser_twist_safety
)

set(CAR_DEPENDENCIES
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
  sensor_msgs
  tf2_geometry_msgs
  tf2_ros
  tf2_sensor_msgs
  visualization_msgs
  waywiser
  waywiser_description
  waywiser_twist_safety
)

set(TRUCK_DEPENDENCIES
  ${CAR_DEPENDENCIES}
)

set(COPTER_DEPENDENCIES
  ${CAR_DEPENDENCIES}
)

set(NAVSATFIX_EXTENDED_WRAPPER_DEPENDENCIES
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  urdf
  waywiser
  waywiser_twist_safety
)

ament_target_dependencies(waywiser_localization_node ${LOCALIZATION_DEPENDENCIES})
ament_target_dependencies(waywiser_car_node ${CAR_DEPENDENCIES})
ament_target_dependencies(waywiser_truck_node ${TRUCK_DEPENDENCIES})
ament_target_dependencies(waywiser_copter_node ${COPTER_DEPENDENCIES})
add_dependencies(waywiser_copter_node px4_msgs_vendor)
ament_target_dependencies(
  navsatfix_extended_wrapper_node
  ${NAVSATFIX_EXTENDED_WRAPPER_DEPENDENCIES}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS waywiser_localization_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS waywiser_car_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS waywiser_truck_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS waywiser_copter_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS navsatfix_extended_wrapper_node DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime urdf)
ament_export_include_directories(include)

ament_package()
