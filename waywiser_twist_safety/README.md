## Examples

    ros2 launch waywiser_twist_safety twist_safety.launch.py
    ros2 launch waywiser_twist_safety twist_safety.launch.py use_sim_time:=true
    ros2 launch waywiser_twist_safety twist_safety.launch.py enable_nav2_collision_monitor:=true
    ros2 launch waywiser_twist_safety twist_safety.launch.py twist_safety_config:=./src/WayWiseR/waywiser_twist_safety/config/twist_safety.yaml

## Set/clear emergency_stop from command line

- To set emergency_stop:

  ```
  ros2 topic pub --once /emergency_stop/target_state waywiser_twist_safety/msg/EmergencyStopState "{sender_id : 'command_line' , state : 2}"
  ```

- To clear emergency_stop

  ```
  ros2 topic pub --once /emergency_stop/target_state waywiser_twist_safety/msg/EmergencyStopState "{sender_id : 'command_line' , state : 1}"
  ```

## Nodes launched by twist_safety.launch.py

1. **onboard_twist_mux**: node that subscribes to topics configured in twist_safety_config file, e.g., teleop_mux_vel, waywiser_vel etc., and publishes to onboard_mux_vel by multiplexing between sunscribed topics according to their priorities.
2. **collision_monitor**: node is launched when "enable_nav2_collision_monitor" launch argument is set to true. It performs several collision avoidance related tasks using incoming data from the lidar and/or depth camera sensors and prevents potential collisions by reducing the linear speed and even setting it to zero to stop the vehicle.
3. **emergency_stop_monitor**: Upon receiving a 'waywiser_twist_safety/msg/EmergencyStopState' type message with the state field set to '2' on the "/emergency_stop/target_state" topic, it suspends the input twist commands and publishes a zero-velocity twist command on its output topic. Instead, when the state field is set to '1', the emergency stop is deactivated, allowing the commands to pass through from the input to the output twist topics.

A typical node graph would look as follows:
![collision_monitor](https://github.com/RISE-Dependable-Transport-Systems/WayWiseR/assets/58977950/263c7c78-2cd9-4a8e-a50e-f1ab23c32e6e)
