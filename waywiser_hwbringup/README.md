# HW specific dependencies

## Slamtech lidars

    sudo apt-get install ros-humble-rplidar-ros

## Intel realsense camera

- Install Intel Realsense SDK and its ROS2 wrapper following the instructions [here](https://github.com/IntelRealSense/realsense-ros?tab=readme-ov-file#installation-on-ubuntu).

## Examples

    ros2 launch waywiser_hwbringup waywise_car.launch.py
    ros2 launch waywiser_hwbringup waywise_car.launch.py vehicle_config:=./src/WayWiseR/waywiser_hwbringup/config/rover.yaml
    ros2 launch waywiser_hwbringup waywise_car.launch.py lidar_config:=./src/WayWiseR/waywiser_hwbringup/config/lidar.yaml
    ros2 launch waywiser_hwbringup waywise_car.launch.py model:=./src/WayWiseR/waywiser_description/urdf/robot.urdf.xacro
    ros2 launch waywiser_hwbringup waywise_car.launch.py frame_prefix:=prefix_of_your_choice

    ros2 launch waywiser_hwbringup waywise_car_autopilot.launch.py
    ros2 launch waywiser_hwbringup waywise_car_autopilot.launch.py vehicle_config:=./src/WayWiseR/waywiser_hwbringup/config/rover.yaml
    ros2 launch waywiser_hwbringup waywise_car_autopilot.launch.py use_sim_time:=true

    ros2 launch waywiser_hwbringup realsense_d435i.launch.py
    ros2 launch waywiser_hwbringup realsense_d435i.launch.py use_sim_time:=true
    ros2 launch waywiser_hwbringup realsense_d435i.launch.py log_level:=debug
    ros2 launch waywiser_hwbringup realsense_d435i.launch.py namespace:=/sensors/drone/camera
    ros2 launch waywiser_hwbringup realsense_d435i.launch.py container:=/sensors/camera/camera_container
    ros2 launch waywiser_hwbringup realsense_d435i.launch.py camera_config:=./src/WayWiseR/waywiser_hwbringup/config/realsense_d435i.yaml
