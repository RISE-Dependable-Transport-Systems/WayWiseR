import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    waywiser_gazebo_dir = get_package_share_directory('waywiser_gazebo')

    config_la = DeclareLaunchArgument(
        'config',
        default_value=os.path.join(waywiser_gazebo_dir, 'config/gazebo_osm_tile_server.yaml'),
        description='Full path to params file for the Gazebo OSM tile server',
    )
    world_sdf_la = DeclareLaunchArgument(
        'world_sdf',
        default_value='',
        description='Gazebo world SDF used to render the map tile base image',
    )
    base_map_cache_dir_la = DeclareLaunchArgument(
        'base_map_cache_dir',
        default_value='',
        description='Directory where the rendered Gazebo base map PNG is cached',
    )
    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation/Gazebo clock if true',
    )

    gazebo_osm_tile_server_node = Node(
        package='waywiser_gazebo',
        executable='gazebo_osm_tile_server_node.py',
        name='gazebo_osm_tile_server_node',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            LaunchConfiguration('config'),
            {'world_sdf': ParameterValue(LaunchConfiguration('world_sdf'), value_type=str)},
            {
                'base_map_cache_dir': ParameterValue(
                    LaunchConfiguration('base_map_cache_dir'), value_type=str
                )
            },
        ],
        arguments=['--ros-args', '--log-level', 'info'],
        output='screen',
        emulate_tty=True,
    )

    ld = LaunchDescription()
    ld.add_action(config_la)
    ld.add_action(world_sdf_la)
    ld.add_action(base_map_cache_dir_la)
    ld.add_action(use_sim_time_la)
    ld.add_action(gazebo_osm_tile_server_node)
    return ld
