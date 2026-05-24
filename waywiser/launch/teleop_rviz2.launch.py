import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    waywiser_teleop_dir = get_package_share_directory('waywiser_teleop')

    # args that can be set from the command line or a default will be used
    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='False', description='Use simulation/Gazebo clock'
    )
    rviz_config_la = DeclareLaunchArgument(
        'rviz_config',
        default_value='',
        description='Full path of rviz display config file or path to their directory',
    )
    teleop_config_la = DeclareLaunchArgument(
        'teleop_config',
        default_value=os.path.join(waywiser_teleop_dir, 'config/teleop.yaml'),
        description='Full path to params file',
    )
    teleop_la = DeclareLaunchArgument(
        'teleop',
        default_value='True',
        description='Launch teleop',
    )
    rviz2_la = DeclareLaunchArgument(
        'rviz2',
        default_value='True',
        description='Launch rviz2',
    )
    control_vehicle_node_fqn_la = DeclareLaunchArgument(
        'control_vehicle_node_fqn',
        default_value='',
        description='Name of the vehicle node to control',
    )
    map_source_la = DeclareLaunchArgument(
        'map_source',
        default_value='',
        description='Control Tower map source: OSM or None',
    )
    osm_tile_server_url_la = DeclareLaunchArgument(
        'osm_tile_server_url',
        default_value='',
        description='Control Tower OSM tile server URL',
    )
    osm_tile_cache_dir_la = DeclareLaunchArgument(
        'osm_tile_cache_dir',
        default_value='',
        description='Control Tower OSM tile cache directory',
    )
    startup_route_file_la = DeclareLaunchArgument(
        'startup_route_file',
        default_value='',
        description='Route file to load in Control Tower at startup',
    )
    vehicle_control_enabled_la = DeclareLaunchArgument(
        'vehicle_control_enabled',
        default_value='',
        description='Enable Control Tower vehicle controls; set False for passive monitoring',
    )
    publish_control_tower_heartbeat_la = DeclareLaunchArgument(
        'publish_control_tower_heartbeat',
        default_value='',
        description='Publish the vehicle-scoped Control Tower heartbeat',
    )

    # create opaque functions to launch nodes using context
    teleop_rviz2_launch_action = OpaqueFunction(function=teleop_rviz2_launch)

    # create launch description
    ld = LaunchDescription()

    # declare launch args
    ld.add_action(use_sim_time_la)
    ld.add_action(rviz_config_la)
    ld.add_action(teleop_config_la)
    ld.add_action(teleop_la)
    ld.add_action(rviz2_la)
    ld.add_action(control_vehicle_node_fqn_la)
    ld.add_action(map_source_la)
    ld.add_action(osm_tile_server_url_la)
    ld.add_action(osm_tile_cache_dir_la)
    ld.add_action(startup_route_file_la)
    ld.add_action(vehicle_control_enabled_la)
    ld.add_action(publish_control_tower_heartbeat_la)

    # start nodes
    ld.add_action(teleop_rviz2_launch_action)

    return ld


def teleop_rviz2_launch(context):
    teleop = (LaunchConfiguration('teleop').perform(context)).lower() == 'true'
    rviz2 = (LaunchConfiguration('rviz2').perform(context)).lower() == 'true'
    nodes = []
    if teleop:
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            get_package_share_directory('waywiser_teleop'),
                            'launch',
                            'teleop.launch.py',
                        )
                    ]
                ),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'teleop_config': LaunchConfiguration('teleop_config'),
                    'control_vehicle_node_fqn': LaunchConfiguration('control_vehicle_node_fqn'),
                    'map_source': LaunchConfiguration('map_source'),
                    'osm_tile_server_url': LaunchConfiguration('osm_tile_server_url'),
                    'osm_tile_cache_dir': LaunchConfiguration('osm_tile_cache_dir'),
                    'startup_route_file': LaunchConfiguration('startup_route_file'),
                    'vehicle_control_enabled': LaunchConfiguration('vehicle_control_enabled'),
                    'publish_control_tower_heartbeat': LaunchConfiguration(
                        'publish_control_tower_heartbeat'
                    ),
                }.items(),
            )
        )
    if rviz2:
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            get_package_share_directory('waywiser_rviz2'),
                            'launch',
                            'rviz.launch.py',
                        )
                    ]
                ),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'rviz_config': LaunchConfiguration('rviz_config'),
                }.items(),
            )
        )

    return nodes
