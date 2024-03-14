## Examples

    # To launch gazebo world with a robot and a ros2 bridge
    ros2 launch waywiser_gazebo gazebo.launch.py
    ros2 launch waywiser_gazebo gazebo.launch.py gazebo_bridge:=./src/WayWiseR/waywiser_gazebo/config/ros_gazebo_bridges.yaml
    ros2 launch waywiser_gazebo gazebo.launch.py world:=./src/WayWiseR/waywiser_gazebo/worlds/car_world.sdf
    ros2 launch waywiser_gazebo gazebo.launch.py world:=./src/WayWiseR/waywiser_gazebo/worlds/bounded_world.sdf
    ros2 launch waywiser_gazebo gazebo.launch.py model:=./src/WayWiseR/waywiser_description/urdf/robot.urdf.xacro
    ros2 launch waywiser_gazebo gazebo.launch.py frame_prefix:=prefix_of_your_choice
    ros2 launch waywiser_gazebo gazebo.launch.py use_sim_time:=false

    # To spawn models within the world after launching gazebo
    ros2 launch waywiser_gazebo spawn.launch.py model_sdf_paths:='["./src/WayWiseR/waywiser_description/sdf/actor_stand/model.sdf"]'
    ros2 launch waywiser_gazebo spawn.launch.py model_sdf_paths:='["./src/WayWiseR/waywiser_description/sdf/actor_stand/model.sdf", "./src/WayWiseR/waywiser_description/sdf/drone/model.sdf"]'
