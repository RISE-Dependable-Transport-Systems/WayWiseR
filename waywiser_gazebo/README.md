## Examples

    ros2 launch waywiser_gazebo gazebo.launch.py
    ros2 launch waywiser_gazebo gazebo.launch.py rover_config:=$(pwd)/src/WayWiseR/waywiser_gazebo/config/ros_gazebo_bridges.yaml
    ros2 launch waywiser_gazebo gazebo.launch.py lidar_config:=$(pwd)/src/WayWiseR/waywiser_gazebo/worlds/car_world.sdf
    ros2 launch waywiser_gazebo gazebo.launch.py model:=$(pwd)/src/WayWiseR/waywiser_description/urdf/robot.urdf.xacro
    ros2 launch waywiser_gazebo gazebo.launch.py frame_prefix:=prefix_of_your_choice
    ros2 launch waywiser_gazebo gazebo.launch.py use_sim_time:=false