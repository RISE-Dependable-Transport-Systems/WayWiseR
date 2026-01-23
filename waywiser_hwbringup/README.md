# HW specific dependencies

## URM14 ultrasonic sensor

If running with multiple sensors, make sure to set a unique slave address of each sensor. The default address is `0x0c`. Set for example `0x0d` for the second sensor. This can be done using the function `change_slave_id()`from the `urm14_sensor` module.

## Slamtech lidars

    sudo apt-get install ros-humble-rplidar-ros

## DepthAI camera

    sudo apt-get install ros-humble-depthai-ros

## Intel realsense camera

- Install Intel Realsense SDK and its ROS2 wrapper following the instructions [here](https://github.com/IntelRealSense/realsense-ros?tab=readme-ov-file#installation-on-ubuntu).

## Examples

    ros2 launch waywiser_hwbringup realsense_d435i.launch.py
    ros2 launch waywiser_hwbringup realsense_d435i.launch.py use_sim_time:=true
    ros2 launch waywiser_hwbringup realsense_d435i.launch.py log_level:=debug
    ros2 launch waywiser_hwbringup realsense_d435i.launch.py namespace:=/sensors/drone/camera
    ros2 launch waywiser_hwbringup realsense_d435i.launch.py container:=/sensors/camera/camera_container
    ros2 launch waywiser_hwbringup realsense_d435i.launch.py camera_config:=./src/WayWiseR/waywiser_hwbringup/config/realsense_d435i.yaml
