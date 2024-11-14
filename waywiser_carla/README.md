## Carla setup

- Clone the carla-ros-bridge repo (fork from [ros-bridge](https://github.com/carla-simulator/ros-bridge)):
  ```
  mkdir -p ~/carla_ros_bridge_ws/src
  cd ~/carla_ros_bridge_ws/src
  git clone --recurse-submodules git@github.com:RISE-Dependable-Transport-Systems/carla-ros-bridge.git
  ```
- Install carla-ros-bridge specific dependencies from the root directory of the workspace:
  ```
  cd ../
  rosdep install -i --from-path src --rosdistro humble -r -y
  pip install -r src/carla-ros-bridge/requirements.txt
  ```
- Build carla-ros-bridge. Currently the packages rviz_carla_plugin, carla_ad_demo, pcl_recorder lead to compilation issues and can be skipped during this step:
  ```
  colcon build --symlink-install --packages-skip rviz_carla_plugin carla_ad_demo pcl_recorder
  ```
- Source carla-ros-bridge from the waywiser workspace before building it:
  ```
  source ~/carla_ros_bridge_ws/install/local_setup.bash
  ```
- Install waywiser dependencies using rosdep:

  ```
  cd waywiser_ws
  rosdep install --from-paths $(colcon list --paths-only | grep "waywiser_carla") --ignore-src --rosdistro humble -r -y
  ```

- Build waywiser_gazebo package:
  ```
  colcon build --symlink-install --packages-select waywiser_carla
  ```
- Source the overlay:
  ```
  source install/local_setup.bash
  ```

Note: Both carla_ros_bridge_ws and waywiser_ws overlays need to be sourced in every new terminal:

```
source ~/carla_ros_bridge_ws/install/local_setup.bash && source install/local_setup.bash
```

## Examples

    ros2 launch waywiser_carla carla.launch.py
    ros2 launch waywiser_carla waywiser_carla_relay.launch.py
    ros2 launch waywiser_carla carla_osm_tile_server.launch.py

## CARLA OSM Tile Server

The carla_osm_tile_server node implements a TCP/IP based server that generates map tiles from CARLA simulator data and serves them in a format similar to OpenStreetMap (OSM) tile servers, making WayWiseR compatible with OSM-based mapping applications like [ControlTower](https://github.com/RISE-Dependable-Transport-Systems/ControlTower). When the node is run for the first time, it connects to CARLA via ros-bridge and renders a 2D top-view image of the CARLA world, including roads and lane markings from opendrive data. The high-resolution map image is saved locally and is used to generate map tiles on demand.

https://github.com/user-attachments/assets/edb4115a-6099-4ebc-97ff-f9e2919c92bc
