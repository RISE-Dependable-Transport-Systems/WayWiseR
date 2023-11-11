cmake_minimum_required(VERSION 3.8)
project(ros2-waywise_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Qt5
find_package(Qt5 COMPONENTS Core SerialPort Network REQUIRED)
find_package(MAVSDK REQUIRED)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_executable(waywise_rover
    src/waywise_rover.cpp
    WayWise/core/pospoint.cpp
    WayWise/core/vbytearray.cpp
    WayWise/vehicles/objectstate.cpp
    WayWise/vehicles/vehiclestate.cpp
    WayWise/vehicles/carstate.cpp
    WayWise/vehicles/controller/motorcontroller.h
    WayWise/vehicles/controller/movementcontroller.cpp
    WayWise/vehicles/controller/servocontroller.cpp
    WayWise/vehicles/controller/carmovementcontroller.cpp
    WayWise/vehicles/controller/vescmotorcontroller.cpp
    WayWise/external/vesc/vescpacket.cpp
    WayWise/autopilot/purepursuitwaypointfollower.cpp
    WayWise/autopilot/purepursuitwaypointfollower.h
    WayWise/autopilot/waypointfollower.h
    WayWise/communication/mavsdkvehicleserver.cpp
    WayWise/communication/jsonstreamparsertcp.cpp
    WayWise/communication/vehicleconnections/vehicleconnection.cpp
    WayWise/communication/mavlinkparameterserver.cpp
    WayWise/communication/parameterserver.cpp
    WayWise/sensors/imu/imuorientationupdater.cpp
    WayWise/sensors/gnss/ublox.cpp
    WayWise/sensors/gnss/ubloxrover.cpp
    WayWise/sensors/gnss/rtcm3_simple.cpp
    WayWise/sensors/gnss/rtcmclient.cpp
    WayWise/sensors/camera/gimbal.h
    WayWise/logger/logger.cpp
    )

target_include_directories(waywise_rover PRIVATE WayWise/)

target_link_libraries(waywise_rover
    Qt5::Core
    Qt5::SerialPort
    Qt5::Network
    MAVSDK::mavsdk
)

ament_target_dependencies(waywise_rover rclcpp std_msgs nav_msgs geometry_msgs tf2_ros)

install(TARGETS
  waywise_rover
  DESTINATION lib/${PROJECT_NAME})

ament_package()
