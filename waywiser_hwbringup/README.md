## HW specific dependencies

### Slamtech lidars

    sudo apt-get install ros-humble-rplidar-ros

### Intel realsense camera

    sudo apt-get install librealsense2-utils librealsense2-dev ros-humble-realsense2-camera ros-humble-realsense2-camera-msgs

## Examples

    ros2 launch waywiser_hwbringup rover.launch.py
    ros2 launch waywiser_hwbringup rover.launch.py rover_config:=$(pwd)/src/WayWiseR/waywiser_hwbringup/config/rover.yaml
    ros2 launch waywiser_hwbringup rover.launch.py lidar_config:=$(pwd)/src/WayWiseR/waywiser_hwbringup/config/lidar.yaml
    ros2 launch waywiser_hwbringup rover.launch.py model:=$(pwd)/src/WayWiseR/waywiser_description/urdf/robot.urdf.xacro
    ros2 launch waywiser_hwbringup rover.launch.py frame_prefix:=prefix_of_your_choice

    ros2 launch waywiser_hwbringup waywise_autopilot.launch.py
    ros2 launch waywiser_hwbringup waywise_autopilot.launch.py rover_config:=$(pwd)/src/WayWiseR/waywiser_hwbringup/config/rover.yaml
    ros2 launch waywiser_hwbringup waywise_autopilot.launch.py use_sim_time:=false
