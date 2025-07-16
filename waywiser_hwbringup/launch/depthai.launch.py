import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    waywiser_hwbringup_dir = get_package_share_directory('waywiser_hwbringup')

    # args that can be set from the command line or a default will be used
    depthai_config_la = DeclareLaunchArgument(
        'depthai_config',
        default_value=os.path.join(waywiser_hwbringup_dir, 'config/depthai.yaml'),
        description='Full path to params file for DepthAI node.',
    )

    # start nodes and use args to set parameters
    depthai_node = Node(
        package='waywiser_hwbringup',
        executable='depthai_node.py',
        name='depthai_node',
        parameters=[
            LaunchConfiguration('depthai_config'),
        ],
        arguments=['--ros-args', '--log-level', 'info'],
        output='screen',
        emulate_tty=True,
    )

    # create launch description
    ld = LaunchDescription()

    # declare launch args
    ld.add_action(depthai_config_la)

    # start nodes
    ld.add_action(depthai_node)

    return ld
