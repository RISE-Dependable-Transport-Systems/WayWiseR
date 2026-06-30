## Gazebo simulator setup

- Install [Ignition Fortress](https://gazebosim.org/docs/fortress/install_ubuntu/), by following the offical instructions.

## Examples

    # To launch gazebo world with a ros2 bridge and setup/reset orchestrator
    ros2 launch waywiser_gazebo gazebo_orchestrator.launch.py
    ros2 launch waywiser_gazebo gazebo_orchestrator.launch.py gazebo_bridge:=./src/WayWiseR/waywiser_gazebo/config/ros_gazebo_bridges.yaml
    ros2 launch waywiser_gazebo gazebo_orchestrator.launch.py world:=./src/WayWiseR/waywiser_gazebo/worlds/car_world.sdf
    ros2 launch waywiser_gazebo gazebo_orchestrator.launch.py world:=./src/WayWiseR/waywiser_gazebo/worlds/bounded_world.sdf
    ros2 launch waywiser_gazebo gazebo_orchestrator.launch.py use_sim_time:=false
    ros2 launch waywiser_gazebo gazebo_orchestrator.launch.py launch_gazebo_orchestrator:=false

    # To spawn models through the Gazebo orchestrator
    ros2 launch waywiser_gazebo gazebo_orchestrator.launch.py spawn_on_startup:=true spawn_config_file:=src/WayWiseR/waywiser_gazebo/config/rover_spawn_config.json
    ros2 launch waywiser_gazebo gazebo_orchestrator.launch.py spawn_on_startup:=true spawn_config_file:=src/WayWiseR/waywiser_gazebo/config/drone_spawn_config.json
