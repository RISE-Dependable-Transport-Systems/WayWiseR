## Carla setup

> [!IMPORTANT]
> Before proceeding, ensure you have followed the steps in the **[How to install and build](../README.md#how-to-install-and-build-on-ubuntu-2204)** section of the main README.

1. **Initialize the carla-ros-bridge submodule:**

   The `carla-ros-bridge` is included as a git submodule. To fetch it, run:

   ```bash
   cd $WAYWISER_WS
   git -C src/WayWiseR submodule update --init --recursive waywiser_carla/external/carla-ros-bridge
   ```

2. **Install dependencies:**

   ```bash
   cd $WAYWISER_WS
   source .venv/bin/activate
   rosdep install --from-paths src/WayWiseR/waywiser_carla src/WayWiseR/waywiser_carla/external/carla-ros-bridge --ignore-src --rosdistro $ROS_DISTRO -r -y
   uv pip install -r src/WayWiseR/waywiser_carla/external/carla-ros-bridge/requirements.txt
   ```

3. **Build waywiser_carla and the bridge:**

   If `waywiser_carla` is in your `$WAYWISER_SKIPPED_PACKAGES` list (from the main setup), you need to update that list and build the workspace:

   ```bash
   # Remove waywiser_carla from the skipped packages list and persist to .env
   source $WAYWISER_WS/src/WayWiseR/.env
   export WAYWISER_SKIPPED_PACKAGES="$(printf '%s\n' ${WAYWISER_SKIPPED_PACKAGES//\"/} | grep -vx 'waywiser_carla' | xargs)"
   if [ -n "$WAYWISER_SKIPPED_PACKAGES" ]; then printf -v WAYWISER_SKIPPED_PACKAGES_ESCAPED '%q' "$WAYWISER_SKIPPED_PACKAGES"; else WAYWISER_SKIPPED_PACKAGES_ESCAPED=; fi
   sed -i "s|^WAYWISER_SKIPPED_PACKAGES=.*|WAYWISER_SKIPPED_PACKAGES=$WAYWISER_SKIPPED_PACKAGES_ESCAPED|" $WAYWISER_WS/src/WayWiseR/.env

   # Build the workspace
   colcon build --symlink-install --base-paths src src/WayWiseR/waywiser_carla/external/carla-ros-bridge --packages-up-to waywiser_carla
   ```

4. **Source the workspace:**

   ```bash
   source $WAYWISER_WS/install/setup.bash
   ```

## Examples

    ros2 launch waywiser_carla carla.launch.py
    ros2 launch waywiser_carla waywiser_carla_relay.launch.py
    ros2 launch waywiser_carla carla_osm_tile_server.launch.py

## CARLA OSM Tile Server

The carla_osm_tile_server node implements a TCP/IP based server that generates map tiles from CARLA simulator data and serves them in a format similar to OpenStreetMap (OSM) tile servers, making WayWiseR compatible with OSM-based mapping applications like [ControlTower](https://github.com/das-rise/ControlTower). When the node is run for the first time, it connects to CARLA via ros-bridge and renders a 2D top-view image of the CARLA world, including roads and lane markings from opendrive data. The high-resolution map image is saved locally and is used to generate map tiles on demand.

<https://github.com/user-attachments/assets/edb4115a-6099-4ebc-97ff-f9e2919c92bc>
