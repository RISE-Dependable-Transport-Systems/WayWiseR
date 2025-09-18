# HW specific dependencies

## Luxonis OAK-D camera

Make sure to add the `udev` rule for the camera on the host (never in a container):

```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

This is done automatically if passing the a flag to the `make` command when building the workspace, for example: `make all LUXONIS=1`.

## URM14 ultrasonic sensor

If running with multiple sensors, make sure to set a unique slave address of each sensor. The default address is `0x0c`. Set for example `0x0d` for the second sensor. This can be done using the function `change_slave_id()`from the `urm14_sensor` module.

## Slamtech lidars

    sudo apt-get install ros-humble-rplidar-ros

## Intel realsense camera

- Install Intel Realsense SDK and its ROS2 wrapper following the instructions [here](https://github.com/IntelRealSense/realsense-ros?tab=readme-ov-file#installation-on-ubuntu).

## Examples

    ros2 launch waywiser_hwbringup waywiser_car.launch.py
    ros2 launch waywiser_hwbringup waywiser_car.launch.py vehicle_config:=./src/WayWiseR/waywiser_hwbringup/config/rover.yaml
    ros2 launch waywiser_hwbringup waywiser_car.launch.py lidar_config:=./src/WayWiseR/waywiser_hwbringup/config/lidar.yaml
    ros2 launch waywiser_hwbringup waywiser_car.launch.py model:=./src/WayWiseR/waywiser_description/urdf/robot.urdf.xacro
    ros2 launch waywiser_hwbringup waywiser_car.launch.py frame_prefix:=prefix_of_your_choice

    ros2 launch waywiser_hwbringup waywise_car_autopilot.launch.py
    ros2 launch waywiser_hwbringup waywise_car_autopilot.launch.py vehicle_config:=./src/WayWiseR/waywiser_hwbringup/config/rover.yaml
    ros2 launch waywiser_hwbringup waywise_car_autopilot.launch.py use_sim_time:=true

    ros2 launch waywiser_hwbringup realsense_d435i.launch.py
    ros2 launch waywiser_hwbringup realsense_d435i.launch.py use_sim_time:=true
    ros2 launch waywiser_hwbringup realsense_d435i.launch.py log_level:=debug
    ros2 launch waywiser_hwbringup realsense_d435i.launch.py namespace:=/sensors/drone/camera
    ros2 launch waywiser_hwbringup realsense_d435i.launch.py container:=/sensors/camera/camera_container
    ros2 launch waywiser_hwbringup realsense_d435i.launch.py camera_config:=./src/WayWiseR/waywiser_hwbringup/config/realsense_d435i.yaml
