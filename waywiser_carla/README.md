## CARLA simulator setup

- Install [CARLA **0.9.15**](https://github.com/carla-simulator/carla/releases/tag/0.9.15) simulator, if not already done.

- Either create a symbolic link to the simulator script at "~/workspaces/carla_ws/CarlaUE4.sh" or update the script path in the carla orchestrator [config](config/carla_orchestrator.yaml) file.

## CARLA OSM Tile Server

The carla_osm_tile_server node implements a TCP/IP based server that generates map tiles from CARLA simulator data and serves them in a format similar to OpenStreetMap (OSM) tile servers, making WayWiseR compatible with OSM-based mapping applications like [ControlTower](https://github.com/das-rise/ControlTower). When the node is run for the first time, it connects to CARLA via ros-bridge and renders a 2D top-view image of the CARLA world, including roads and lane markings from opendrive data. The high-resolution map image is saved locally and is used to generate map tiles on demand.

<https://github.com/user-attachments/assets/edb4115a-6099-4ebc-97ff-f9e2919c92bc>

## Examples

    ros2 launch waywiser_carla carla.launch.py
    ros2 launch waywiser_carla waywiser_carla_relay.launch.py
    ros2 launch waywiser_carla carla_osm_tile_server.launch.py
