cmake_minimum_required(VERSION 3.8)
project(waywiser_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)

# find dependencies
find_package(ament_cmake REQUIRED)
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
  COMPONENTS Core SerialPort Network
  REQUIRED)
find_package(MAVSDK REQUIRED)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

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
  WayWise/sensors/gnss/gnssreceiver.cpp
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
  WayWise/sensors/gnss/rtcm3_simple.cpp
  WayWise/sensors/gnss/rtcmclient.cpp
  WayWise/sensors/gnss/ublox.cpp
  WayWise/sensors/gnss/ubloxrover.cpp
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
  WayWise/sensors/fusion/sdvpvehiclepositionfuser.cpp
  WayWise/sensors/gnss/rtcm3_simple.cpp
  WayWise/sensors/gnss/rtcmclient.cpp
  WayWise/sensors/gnss/ublox.cpp
  WayWise/sensors/gnss/ubloxrover.cpp
  WayWise/sensors/imu/bno055orientationupdater.cpp
  WayWise/sensors/imu/imuorientationupdater.cpp
  WayWise/sensors/tof/vl53l0xtofsensor.cpp
  WayWise/sensors/tof/tofsensor.h
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
  waywiser_car_node
  cpp_src/waywiser_car_node.cpp
  cpp_src/waywiser_car_node_core.cpp
  cpp_src/car_autopilot_component.cpp
  cpp_src/car_interface_component.cpp
  ${COMMON_WAYWISE_SOURCES}
  ${WAYWISE_CAR_SOURCES}
  ${COMMON_WAYWISE_AP_SOURCES}
)

add_executable(
  waywiser_truck_node
  cpp_src/waywiser_truck_node.cpp
  cpp_src/waywiser_truck_node_core.cpp
  cpp_src/waywiser_car_node_core.cpp
  cpp_src/truck_autopilot_component.cpp
  cpp_src/truck_interface_component.cpp
  cpp_src/car_autopilot_component.cpp
  cpp_src/car_interface_component.cpp
  ${COMMON_WAYWISE_SOURCES}
  ${WAYWISE_CAR_SOURCES}
  ${WAYWISE_TRUCK_SOURCES}
  ${COMMON_WAYWISE_AP_SOURCES}
)

set(COMMON_LIBRARIES
  Qt5::Core
  Qt5::SerialPort
  Qt5::Network
  urdf::urdf
  MAVSDK::mavsdk
  Boost::regex
  ${cpp_typesupport_target}
)

target_link_libraries(waywiser_car_node ${COMMON_LIBRARIES})
target_link_libraries(waywiser_truck_node ${COMMON_LIBRARIES})

set(COMMON_DEPENDENCIES
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  sensor_msgs
  tf2_geometry_msgs
  tf2_sensor_msgs
  visualization_msgs
  waywiser
  waywiser_description
  waywiser_twist_safety
)

ament_target_dependencies(waywiser_car_node ${COMMON_DEPENDENCIES})
ament_target_dependencies(waywiser_truck_node ${COMMON_DEPENDENCIES})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS waywiser_car_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS waywiser_truck_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)

ament_package()
