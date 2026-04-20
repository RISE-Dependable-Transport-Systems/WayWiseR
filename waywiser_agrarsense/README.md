## Overview

The waywiser_agrarsense package integrates the [AGRARSENSE](https://agrarsense.frostbit.fi/) simulator with ROS2 and WayWiseR. This package addresses the compatibility gap between AGRARSENSE's ROS1-based plugins and modern ROS2 systems by utilizing a docker container running ROS1-ROS2 bridge. The [py_src](./py_src) contains python-based ros2 nodes to orchestrate the simulator execution, publish vehicle transforms, and relay twist commands from waywiser to the simulator.

## Agrarsense setup

- Install the [Agrarsense](https://agrarsense.frostbit.fi/md_Docs_linux_install.html) simulator, if not already done.

- Either create a symbolic link to the simulator script at "~/workspaces/agrarsense_ws/Agrarsense.sh" or update the script path in the yaml config file.
- Ensure Docker is installed and configured for rootless mode. Follow the official [Docker rootless installation guide](https://docs.docker.com/engine/security/rootless/).

- **Install all dependencies:**

  ```bash
  cd $WAYWISER_WS
  source .venv/bin/activate
  rosdep install --from-paths src/WayWiseR/waywiser_agrarsense --ignore-src --rosdistro $ROS_DISTRO -r -y
  ```

- **Build `waywiser_agrarsense`:**

  If `waywiser_agrarsense` is in your `$WAYWISER_SKIPPED_PACKAGES` list (from the main setup), you need to update that list and build the workspace:

     ```bash
   # Remove waywiser_agrarsense from the skipped packages list and persist to .env
   source $WAYWISER_WS/src/WayWiseR/.env
   export WAYWISER_SKIPPED_PACKAGES="$(printf '%s\n' ${WAYWISER_SKIPPED_PACKAGES//\"/} | grep -vx 'waywiser_agrarsense' | xargs)"
   if [ -n "$WAYWISER_SKIPPED_PACKAGES" ]; then printf -v WAYWISER_SKIPPED_PACKAGES_ESCAPED '%q' "$WAYWISER_SKIPPED_PACKAGES"; else WAYWISER_SKIPPED_PACKAGES_ESCAPED=; fi
   sed -i "s|^WAYWISER_SKIPPED_PACKAGES=.*|WAYWISER_SKIPPED_PACKAGES=$WAYWISER_SKIPPED_PACKAGES_ESCAPED|" $WAYWISER_WS/src/WayWiseR/.env

   # Build the workspace
   colcon build --symlink-install --packages-up-to waywiser_agrarsense
   ```

- Source the workspace:

  ```bash
  source $WAYWISER_WS/install/setup.bash
  ```

- Build the agrarsense-ros bridge docker container natively using the installed symbolic link script:

  ```
  ./install/waywiser_agrarsense/build_docker_container.sh
  ```

## Examples

    ros2 launch waywiser_agrarsense agrarsense.launch.py
    ros2 launch waywiser_agrarsense waywiser_agrarsense_relay.launch.py

## Nodes launched by agrarsense.launch.py

**agrarsense_orchestrator_node**: automates the setup, execution, and management of AGRARSENSE simulations. It starts the AGRARSENSE simulator and ROS bridge processes, and spawns objects based on predefined configurations. Once the objects are spawned and ready, the node signals readiness for scenario execution by other nodes via topic "/agrarsense/simulation_ready". It supports iterative execution of different simulation configurations, waiting for a Bool signal via topic "/agrarsense/end_simulation" to end each simulation before proceeding to the next.

## Nodes launched by waywiser_agrarsense_relay.launch.py

1. **waywiser_twist_to_agrarsense_control_node**: translates ROS2 Twist commands (linear and angular velocities) into control commands for the AGRARSENSE simulator. It utilizes PID control to manage vehicle speed and computes appropriate throttle, brake, and steering values based on the target and current speeds and angular velocities.
2. **vehicle_tf_publisher**: computes and publishes odometry data and corresponding transformations for each spawned vehicle in the Agrarsense simulation. It subscribes to a global transform input topic (providing position and orientation) and calculates the vehicle's relative position, orientation, and velocities based on an initial reference point. The node publishes this odometry data and broadcasts the map_to_odom and odom_to_base_link transforms. This allows other nodes to track the vehicle's movement and pose relative to the global map and local odom frames.
