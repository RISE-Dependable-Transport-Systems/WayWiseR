# launch file to bring up rover nodes

import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    hw_bringup_dir = get_package_share_directory('ros2-waywise_hwbringup')

    # args that can be set from the command line or a default will be used
    rover_config_la = DeclareLaunchArgument(
        'rover_config', default_value=os.path.join(hw_bringup_dir, 'config/rover.yaml'),
        description='Full path to params file of rover')

    # start nodes and use args to set parameters
    twist_to_ackermann_node = Node(
        package='ros2-waywise_hwbringup',
        executable='twist_to_ackermann',
        name='twist_to_ackermann',
        output='screen'
    )

    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        parameters=[LaunchConfiguration('rover_config')]
    )

    vesc_to_odom_node = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom_node',
        parameters=[LaunchConfiguration('rover_config')]
    )

    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        parameters=[LaunchConfiguration('rover_config')]
    )

    # create launch description
    ld = LaunchDescription()

    # declare launch arg
    ld.add_action(rover_config_la)

    # start nodes
    ld.add_action(twist_to_ackermann_node)
    ld.add_action(ackermann_to_vesc_node)
    ld.add_action(vesc_to_odom_node)
    ld.add_action(vesc_driver_node)

    return ld
