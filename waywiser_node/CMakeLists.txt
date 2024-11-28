cmake_minimum_required(VERSION 3.8)
project(waywiser_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

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

set(COMMON_WAYWISE_SOURCES
    WayWise/core/pospoint.cpp
    WayWise/core/vbytearray.cpp
    WayWise/vehicles/objectstate.cpp
    WayWise/vehicles/vehiclestate.cpp
    WayWise/vehicles/controller/movementcontroller.cpp
    WayWise/vehicles/controller/servocontroller.cpp
    WayWise/sensors/gnss/ublox.cpp
    WayWise/sensors/gnss/ubloxrover.cpp
    WayWise/sensors/gnss/rtcm3_simple.cpp
    WayWise/sensors/gnss/rtcmclient.cpp
    WayWise/logger/logger.cpp
    WayWise/communication/parameterserver.cpp
    WayWise/sensors/camera/gimbal.h
    WayWise/core/simplewatchdog.cpp)

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
    WayWise/communication/mavlinkparameterserver.cpp)

set(WAYWISE_CAR_SOURCES
    WayWise/vehicles/carstate.cpp
    WayWise/vehicles/controller/carmovementcontroller.cpp
    WayWise/vehicles/controller/motorcontroller.h
    WayWise/vehicles/controller/vescmotorcontroller.cpp
    WayWise/external/vesc/vescpacket.cpp
    WayWise/external/pi-bno055/i2c_bno055.c
    WayWise/sensors/fusion/sdvpvehiclepositionfuser.cpp
    WayWise/sensors/gnss/rtcm3_simple.cpp
    WayWise/sensors/gnss/rtcmclient.cpp
    WayWise/sensors/gnss/ublox.cpp
    WayWise/sensors/gnss/ubloxrover.cpp
    WayWise/sensors/imu/bno055orientationupdater.cpp
    WayWise/sensors/imu/imuorientationupdater.cpp)

add_executable(waywise_car src/waywise_car.cpp ${COMMON_WAYWISE_SOURCES}
                           ${WAYWISE_CAR_SOURCES})

add_executable(
  waywise_truck
  src/waywise_truck.cpp
  ${COMMON_WAYWISE_SOURCES}
  ${WAYWISE_CAR_SOURCES}
  WayWise/external/pi-as5600/driver_as5600.c
  WayWise/external/pi-as5600/iic.c
  WayWise/external/pi-as5600/driver_as5600_basic.c
  WayWise/external/pi-as5600/raspberrypi4b_driver_as5600_interface.c
  WayWise/sensors/angle/as5600updater.cpp
  WayWise/sensors/angle/anglesensorupdater.cpp
  WayWise/vehicles/truckstate.cpp
  WayWise/vehicles/trailerstate.cpp)

add_executable(
  waywise_car_autopilot
  src/waywise_car_autopilot.cpp
  ${COMMON_WAYWISE_SOURCES}
  ${COMMON_WAYWISE_AP_SOURCES}
  WayWise/vehicles/carstate.cpp
  WayWise/vehicles/controller/carmovementcontroller.cpp)

add_executable(
  waywise_truck_autopilot
  src/waywise_truck_autopilot.cpp
  ${COMMON_WAYWISE_SOURCES}
  ${COMMON_WAYWISE_AP_SOURCES}
  WayWise/vehicles/carstate.cpp
  WayWise/vehicles/controller/carmovementcontroller.cpp
  WayWise/vehicles/truckstate.cpp
  WayWise/vehicles/trailerstate.cpp)

target_include_directories(waywise_car PRIVATE WayWise/)
target_include_directories(waywise_truck PRIVATE WayWise/)
target_include_directories(waywise_car_autopilot PRIVATE WayWise/)
target_include_directories(waywise_truck_autopilot PRIVATE WayWise/)

target_link_libraries(waywise_car Qt5::Core Qt5::SerialPort Qt5::Network)
target_link_libraries(waywise_truck Qt5::Core Qt5::SerialPort Qt5::Network)
target_link_libraries(
  waywise_car_autopilot
  Qt5::Core
  Qt5::SerialPort
  Qt5::Network
  MAVSDK::mavsdk)
target_link_libraries(
  waywise_truck_autopilot
  Qt5::Core
  Qt5::SerialPort
  Qt5::Network
  MAVSDK::mavsdk)

ament_target_dependencies(
  waywise_car
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf2_ros)
ament_target_dependencies(
  waywise_truck
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf2_ros)
ament_target_dependencies(
  waywise_car_autopilot
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs)
ament_target_dependencies(
  waywise_truck_autopilot
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs)

install(TARGETS waywise_car DESTINATION lib/${PROJECT_NAME})
install(TARGETS waywise_truck DESTINATION lib/${PROJECT_NAME})
install(TARGETS waywise_car_autopilot DESTINATION lib/${PROJECT_NAME})
install(TARGETS waywise_truck_autopilot DESTINATION lib/${PROJECT_NAME})

ament_package()
