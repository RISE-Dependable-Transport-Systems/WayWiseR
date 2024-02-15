## Examples

    ros2 launch waywiser_twist_safety twist_safety.launch.py
    ros2 launch waywiser_twist_safety twist_safety.launch.py use_sim_time:=false
    ros2 launch waywiser_twist_safety twist_safety.launch.py enable_collision_monitor:=true
    ros2 launch waywiser_twist_safety twist_safety.launch.py twist_safety_config:=./src/WayWiseR/waywiser_twist_safety/config/twist_safety.yaml

## Nodes launched by twist_safety.launch.py

1. **onboard_twist_mux**: node that subscribes to topics configured in twist_safety_config file, e.g., teleop_mux_vel, waywise_vel etc., and publishes to onboard_mux_vel by multiplexing between sunscribed topics according to their priorities.
2. **collision_monitor**: node is launched when "enable_collision_monitor" launch argument is set to true. It performs several collision avoidance related tasks using incoming data from the lidar and/or depth camera sensors and prevents potential collisions by reducing the linear speed and even setting it to zero to stop the vehicle.
3. **emergency_stop_monitor**: Upon receiving a boolean 'true' data on the "/emergency_stop" topic, it suspends the input twist commands and publishes a zero-velocity twist command on its output topic. When a boolean 'false' data is received on the "/emergency_stop" topic, the emergency stop is deactivated, allowing the commands to pass through from the input to the output twist topics.

A typical node graph would look as follows:
