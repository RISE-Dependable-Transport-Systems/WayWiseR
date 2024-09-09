## Carla dependencies

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

## Examples

    ros2 launch waywiser_carla carla.launch.py
    ros2 launch waywiser_carla waywiser_carla_relay.launch.py
