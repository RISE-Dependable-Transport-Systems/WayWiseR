## Examples

    ros2 launch waywiser_hwbringup rover.launch.py
    ros2 launch waywiser_hwbringup rover.launch.py rover_config:=$(pwd)/src/WayWiseR/waywiser_hwbringup/config/rover.yaml
    ros2 launch waywiser_hwbringup rover.launch.py lidar_config:=$(pwd)/src/WayWiseR/waywiser_hwbringup/config/lidar.yaml
    ros2 launch waywiser_hwbringup rover.launch.py model:=$(pwd)/src/WayWiseR/waywiser_description/urdf/robot.urdf.xacro
    ros2 launch waywiser_hwbringup rover.launch.py frame_prefix:=prefix_of_your_choice