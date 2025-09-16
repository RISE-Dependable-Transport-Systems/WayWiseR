import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    hw_bringup_dir = get_package_share_directory('waywiser_hwbringup')

    # Launch argument to specify the YAML config for the human_distance_node
    config_la = DeclareLaunchArgument(
        'human_distance_config',
        default_value=os.path.join(hw_bringup_dir, 'config', 'human_distance.yaml'),
        description='Full path to params file for human_distance_node',
    )

    # Start the human_distance_node with the provided parameter file
    human_distance_node = Node(
        package='waywiser_hwbringup',
        executable='human_distance_node.py',
        name='human_distance_node',
        parameters=[LaunchConfiguration('human_distance_config')],
    )

    # Create and populate the LaunchDescription
    ld = LaunchDescription()
    ld.add_action(config_la)
    ld.add_action(human_distance_node)

    return ld
