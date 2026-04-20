## Gazebo setup

- Install [Ignition Fortress](https://gazebosim.org/docs/fortress/install_ubuntu/), by following the offical instructions.

- **Install all dependencies:**

  ```bash
  cd $WAYWISER_WS
  source .venv/bin/activate
  rosdep install --from-paths src/WayWiseR/waywiser_gazebo --ignore-src --rosdistro $ROS_DISTRO -r -y
  ```

- **Build `waywiser_gazebo`:**

  If `waywiser_gazebo` is in your `$WAYWISER_SKIPPED_PACKAGES` list (from the main setup), you need to update that list and build the workspace:

     ```bash
   # Remove waywiser_gazebo from the skipped packages list and persist to .env
   source $WAYWISER_WS/src/WayWiseR/.env
   export WAYWISER_SKIPPED_PACKAGES="$(printf '%s\n' ${WAYWISER_SKIPPED_PACKAGES//\"/} | grep -vx 'waywiser_gazebo' | xargs)"
   if [ -n "$WAYWISER_SKIPPED_PACKAGES" ]; then printf -v WAYWISER_SKIPPED_PACKAGES_ESCAPED '%q' "$WAYWISER_SKIPPED_PACKAGES"; else WAYWISER_SKIPPED_PACKAGES_ESCAPED=; fi
   sed -i "s|^WAYWISER_SKIPPED_PACKAGES=.*|WAYWISER_SKIPPED_PACKAGES=$WAYWISER_SKIPPED_PACKAGES_ESCAPED|" $WAYWISER_WS/src/WayWiseR/.env

   # Build the workspace
   colcon build --symlink-install --packages-up-to waywiser_gazebo
   ```

- Source the workspace:
  ```bash
  source $WAYWISER_WS/install/setup.bash
  ```

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
    ros2 launch waywiser_gazebo spawn.launch.py spawn_config_file:=src/WayWiseR/waywiser_gazebo/config/rover_spawn_config.json
    ros2 launch waywiser_gazebo spawn.launch.py spawn_config_file:=src/WayWiseR/waywiser_gazebo/config/drone_spawn_config.json
