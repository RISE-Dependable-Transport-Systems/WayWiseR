# launch file to bring up truck nodes

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    hw_bringup_dir = get_package_share_directory('waywiser_hwbringup')

    # args that can be set from the command line or a default will be used
    vehicle_config_la = DeclareLaunchArgument(
        'vehicle_config',
        default_value=os.path.join(hw_bringup_dir, 'config/truck_small_scale.yaml'),
        description='Full path to params file of truck',
    )

    frame_prefix_la = DeclareLaunchArgument(
        'frame_prefix',
        default_value='/',
        description='Prefix to publish robot transforms in',
    )

    # start nodes and use args to set parameters
    waywise_node = Node(
        package='waywiser_node',
        executable='waywise_truck',
        name='waywise_truck_node',
        parameters=[LaunchConfiguration('vehicle_config')],
        remappings=[('/cmd_vel', '/cmd_vel_out')],
        arguments=['--ros-args', '--log-level', 'info'],
    )

    # create launch description
    ld = LaunchDescription()

    # declare launch arg
    ld.add_action(vehicle_config_la)
    ld.add_action(frame_prefix_la)

    # start nodes
    ld.add_action(waywise_node)

    return ld
