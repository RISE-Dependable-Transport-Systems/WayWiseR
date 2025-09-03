> ⚠️ **ROS 2 Version Requirement**
>
> `waywiser_carla` is **officially supported on ROS 2 Humble (native, Python 3.10)**.
> A **custom ROS 2 Jazzy build with Python 3.10** may work, but it is **untested**. The CARLA ROS bridge for UE 4.6 is **not compatible with ROS 2 Jazzy built with Python 3.12** because `distutils` is no longer included.

## Carla setup

- Clone the carla-ros-bridge repo (fork from [ros-bridge](https://github.com/carla-simulator/ros-bridge)):

  ```
  export CARLA_ROS_BRIDGE_WS=~/carla_ros_bridge_ws     #update the environment variable with desired path
  mkdir -p $CARLA_ROS_BRIDGE_WS/src
  git clone --recurse-submodules git@github.com:RISE-Dependable-Transport-Systems/carla-ros-bridge.git $CARLA_ROS_BRIDGE_WS/src/carla-ros-bridge
  ```

- Add CARLA_ROS_BRIDGE_WS environment variable to .bashrc to persist the variable when a new terminal is opened:

  ```
  echo "export CARLA_ROS_BRIDGE_WS=$CARLA_ROS_BRIDGE_WS" >> ~/.bashrc
  ```

- Install carla-ros-bridge specific dependencies from the root directory of the workspace:
  ```
  cd $CARLA_ROS_BRIDGE_WS
  rosdep install -i --from-path src/carla-ros-bridge --rosdistro humble -r -y
  pip install -r src/carla-ros-bridge/requirements.txt
  ```
- Build carla-ros-bridge:
  ```
  colcon build --symlink-install --base-paths $CARLA_ROS_BRIDGE_WS/src/carla-ros-bridge
  ```
- Source carla-ros-bridge from the waywiser workspace before building waywiser_carla:
  ```
  source $CARLA_ROS_BRIDGE_WS/install/local_setup.bash
  ```
- Install waywiser_carla dependencies using rosdep:

  ```
  cd $WAYWISER_WS
  rosdep install --from-paths $(colcon list --paths-only | grep "waywiser_carla") --ignore-src --rosdistro humble -r -y
  ```

- Build waywiser_carla package:
  ```
  colcon build --symlink-install --packages-select waywiser_carla
  ```
- Source the overlay:
  ```
  source install/local_setup.bash
  ```

Note: Both CARLA_ROS_BRIDGE_WS and WAYWISER_WS overlays need to be sourced in every new terminal:

```
cd $WAYWISER_WS
source $CARLA_ROS_BRIDGE_WS/install/local_setup.bash && source install/local_setup.bash
```

## Examples

    ros2 launch waywiser_carla carla.launch.py
    ros2 launch waywiser_carla waywiser_carla_relay.launch.py
    ros2 launch waywiser_carla carla_osm_tile_server.launch.py

## CARLA OSM Tile Server

The carla_osm_tile_server node implements a TCP/IP based server that generates map tiles from CARLA simulator data and serves them in a format similar to OpenStreetMap (OSM) tile servers, making WayWiseR compatible with OSM-based mapping applications like [ControlTower](https://github.com/RISE-Dependable-Transport-Systems/ControlTower). When the node is run for the first time, it connects to CARLA via ros-bridge and renders a 2D top-view image of the CARLA world, including roads and lane markings from opendrive data. The high-resolution map image is saved locally and is used to generate map tiles on demand.

https://github.com/user-attachments/assets/edb4115a-6099-4ebc-97ff-f9e2919c92bc
