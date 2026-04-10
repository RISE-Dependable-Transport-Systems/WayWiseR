## Carla setup

> [!IMPORTANT]
> Before proceeding, ensure you have followed the steps in the **[How to install and build](../README.md#how-to-install-and-build-on-ubuntu-2204)** section of the main README.

1. **Clone carla-ros-bridge** (fork from [ros-bridge](https://github.com/carla-simulator/ros-bridge)):

   ```bash
   export CARLA_ROS_BRIDGE_WS=~/carla_ros_bridge_ws     # Update with desired path
   mkdir -p $CARLA_ROS_BRIDGE_WS/src
   git clone --recurse-submodules git@github.com:das-rise/carla-ros-bridge.git $CARLA_ROS_BRIDGE_WS/src/carla-ros-bridge
   ```

2. **Install carla-ros-bridge dependencies:**

   ```bash
   source $WAYWISER_WS/.venv/bin/activate
   cd $CARLA_ROS_BRIDGE_WS
   rosdep install -i --from-path src/carla-ros-bridge --rosdistro $ROS_DISTRO -r -y
   uv pip install -r src/carla-ros-bridge/requirements.txt
   ```

3. **Build carla-ros-bridge:**

   ```bash
   colcon build --symlink-install --base-paths $CARLA_ROS_BRIDGE_WS/src/carla-ros-bridge
   ```

4. **Install waywiser_carla dependencies:**

   ```bash
   # Source the bridge workspace first
   source $CARLA_ROS_BRIDGE_WS/install/setup.bash

   cd $WAYWISER_WS
   rosdep install --from-paths $(colcon list --paths-only | grep "waywiser_carla") --ignore-src --rosdistro $ROS_DISTRO -r -y
   ```

5. **Build waywiser_carla:**

   ```bash
   colcon build --symlink-install --packages-up-to waywiser_carla
   ```

To persist the environment variables and source CARLA_ROS_BRIDGE_WS automatically when activating the virtual environment, run the following command (copy-paste the entire block):

```bash
cat <<EOT >> $WAYWISER_WS/.venv/bin/activate

# Carla ROS Bridge Setup
export CARLA_ROS_BRIDGE_WS=$CARLA_ROS_BRIDGE_WS
if [ -f "$CARLA_ROS_BRIDGE_WS/install/setup.bash" ]; then
source "$CARLA_ROS_BRIDGE_WS/install/setup.bash"
fi
EOT
```

## Examples

    ros2 launch waywiser_carla carla.launch.py
    ros2 launch waywiser_carla waywiser_carla_relay.launch.py
    ros2 launch waywiser_carla carla_osm_tile_server.launch.py

## CARLA OSM Tile Server

The carla_osm_tile_server node implements a TCP/IP based server that generates map tiles from CARLA simulator data and serves them in a format similar to OpenStreetMap (OSM) tile servers, making WayWiseR compatible with OSM-based mapping applications like [ControlTower](https://github.com/das-rise/ControlTower). When the node is run for the first time, it connects to CARLA via ros-bridge and renders a 2D top-view image of the CARLA world, including roads and lane markings from opendrive data. The high-resolution map image is saved locally and is used to generate map tiles on demand.

<https://github.com/user-attachments/assets/edb4115a-6099-4ebc-97ff-f9e2919c92bc>
