# launch file to bring up rover nodes

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    hw_bringup_dir = get_package_share_directory('waywiser_hwbringup')

    # args that can be set from the command line or a default will be used
    rover_config_la = DeclareLaunchArgument(
        'rover_config',
        default_value=os.path.join(hw_bringup_dir, 'config/rover.yaml'),
        description='Full path to params file of rover',
    )

    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='Use simulation/Gazebo clock'
    )

    # start nodes and use args to set parameters
    waywise_autopilot_node = Node(
        package='waywiser_node',
        executable='waywise_autopilot',
        name='waywise_autopilot_node',
        parameters=[
            LaunchConfiguration('rover_config'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
        arguments=['--ros-args', '--log-level', 'info'],
    )

    # create launch description
    ld = LaunchDescription()

    # declare launch arg
    ld.add_action(rover_config_la)
    ld.add_action(use_sim_time_la)

    # start nodes
    ld.add_action(waywise_autopilot_node)

    return ld
