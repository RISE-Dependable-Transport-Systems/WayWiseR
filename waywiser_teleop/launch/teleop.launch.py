import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import yaml


def generate_launch_description():
    teleop_dir = get_package_share_directory('waywiser_teleop')

    # args that can be set from the command line or a default will be used
    teleop_config_la = DeclareLaunchArgument(
        'teleop_config',
        default_value=os.path.join(teleop_dir, 'config/teleop.yaml'),
        description='Full path to params file',
    )

    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='False', description='Use simulation/Gazebo clock'
    )

    control_vehicle_node_fqn_la = DeclareLaunchArgument(
        'control_vehicle_node_fqn',
        default_value='',
        description='Fully qualified name of the vehicle node to control',
    )
    map_source_la = DeclareLaunchArgument(
        'map_source',
        default_value='',
        description='Control Tower map source: OpenStreetMap, Local OSM server, or None',
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

    # start nodes and use args to set parameters
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        parameters=[
            LaunchConfiguration('teleop_config'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
    )

    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        parameters=[
            LaunchConfiguration('teleop_config'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
        remappings={('/cmd_vel', '/joy_vel')},
    )

    twist_angular_correction_node = Node(
        package='waywiser_teleop',
        executable='twist_angular_correction',
        name='twist_angular_correction',
        parameters=[
            LaunchConfiguration('teleop_config'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
    )

    control_tower_conditional_launch_action = OpaqueFunction(
        function=control_tower_conditional_launch
    )

    # create launch description
    ld = LaunchDescription()

    # declare launch args
    ld.add_action(teleop_config_la)
    ld.add_action(use_sim_time_la)
    ld.add_action(control_vehicle_node_fqn_la)
    ld.add_action(map_source_la)
    ld.add_action(osm_tile_server_url_la)
    ld.add_action(osm_tile_cache_dir_la)
    ld.add_action(startup_route_file_la)
    ld.add_action(vehicle_control_enabled_la)
    ld.add_action(publish_control_tower_heartbeat_la)

    # start nodes
    ld.add_action(joy_node)
    ld.add_action(teleop_twist_joy_node)
    ld.add_action(twist_angular_correction_node)
    ld.add_action(control_tower_conditional_launch_action)

    return ld


def control_tower_conditional_launch(context):
    with open(LaunchConfiguration('teleop_config').perform(context)) as f:
        config_data = yaml.safe_load(f)
        if 'control_tower' in config_data:
            control_tower_params = config_data['control_tower']['ros__parameters']
            enable_control_tower = control_tower_params['enable']
            control_vehicle_node_fqn = LaunchConfiguration('control_vehicle_node_fqn').perform(
                context
            )
            if control_vehicle_node_fqn != '':
                control_tower_params['control_vehicle_node_fqn'] = control_vehicle_node_fqn
            map_source = LaunchConfiguration('map_source').perform(context)
            if map_source != '':
                control_tower_params['map_source'] = map_source
            osm_tile_server_url = LaunchConfiguration('osm_tile_server_url').perform(context)
            if osm_tile_server_url != '':
                control_tower_params['osm_tile_server_url'] = osm_tile_server_url
            osm_tile_cache_dir = LaunchConfiguration('osm_tile_cache_dir').perform(context)
            if osm_tile_cache_dir != '':
                control_tower_params['osm_tile_cache_dir'] = osm_tile_cache_dir
            startup_route_file = LaunchConfiguration('startup_route_file').perform(context)
            if startup_route_file != '':
                control_tower_params['startup_route_file'] = startup_route_file
            vehicle_control_enabled = LaunchConfiguration('vehicle_control_enabled').perform(
                context
            )
            if vehicle_control_enabled != '':
                control_tower_params['vehicle_control_enabled'] = (
                    vehicle_control_enabled.lower() == 'true'
                )
            publish_control_tower_heartbeat = LaunchConfiguration(
                'publish_control_tower_heartbeat'
            ).perform(context)
            if publish_control_tower_heartbeat != '':
                control_tower_params['publish_control_tower_heartbeat'] = (
                    publish_control_tower_heartbeat.lower() == 'true'
                )
            if enable_control_tower:
                if 'DISPLAY' in os.environ:
                    control_tower_node = Node(
                        package='waywiser_teleop',
                        executable='control_tower.py',
                        name='control_tower',
                        output='screen',
                        parameters=[
                            control_tower_params,
                            {
                                'use_sim_time': LaunchConfiguration('use_sim_time'),
                                'map_source': control_tower_params.get(
                                    'map_source', 'OpenStreetMap'
                                ),
                            },
                        ],
                        remappings={},
                    )
                    return [control_tower_node]
                else:
                    log_no_display = LogInfo(
                        msg='No GUI display detected. control_tower_node will not be launched.'
                    )
                    return [log_no_display]

    return []
