import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    waywiser_carla_dir = get_package_share_directory('waywiser_carla')

    # args that can be set from the command line or a default will be used
    config_la = DeclareLaunchArgument(
        'config',
        default_value=os.path.join(waywiser_carla_dir, 'config/carla_osm_tile_server.yaml'),
        description='Full path to params file for carla',
    )
    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true',
    )

    # start nodes and use args to set parameters
    carla_osm_tile_server_node = Node(
        package='waywiser_carla',
        executable='carla_osm_tile_server.py',
        name='carla_osm_tile_server_node',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            LaunchConfiguration('config'),
        ],
        arguments=['--ros-args', '--log-level', 'info'],
        output='screen',
        emulate_tty=True,
    )

    # create launch description
    ld = LaunchDescription()

    # declare launch args
    ld.add_action(config_la)
    ld.add_action(use_sim_time_la)

    # start nodes
    ld.add_action(carla_osm_tile_server_node)

    return ld
