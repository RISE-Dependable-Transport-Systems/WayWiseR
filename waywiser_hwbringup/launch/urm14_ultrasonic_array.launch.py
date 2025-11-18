import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    hw_bringup_dir = get_package_share_directory('waywiser_hwbringup')

    # args that can be set from the command line or a default will be used
    config_la = DeclareLaunchArgument(
        'urm14_sensor_array_config',
        default_value=os.path.join(hw_bringup_dir, 'config/urm14_sensor_array.yaml'),
        description='Full path to params file of urm14 sensors',
    )

    # start nodes and use args to set parameters
    urm14_publisher_node = Node(
        package='waywiser_hwbringup',
        executable='urm14_sensor_array_node.py',
        name='urm14_sensor_array_node',
        parameters=[LaunchConfiguration('urm14_sensor_array_config')],
    )

    # create launch description
    ld = LaunchDescription()

    # declare launch arg
    ld.add_action(config_la)

    # start nodes
    ld.add_action(urm14_publisher_node)

    return ld
