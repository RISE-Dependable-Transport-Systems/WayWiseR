import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    waywiser_carla_dir = get_package_share_directory('waywiser_carla')

    # args that can be set from the command line or a default will be used
    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='Use simulation clock'
    )
    config_la = DeclareLaunchArgument(
        'config',
        default_value=os.path.join(waywiser_carla_dir, 'config/truck_full_scale.yaml'),
        description='Full path to params file of node',
    )

    # create nodes
    emulated_angle_sensor = Node(
        package='waywiser_carla',
        executable='emulated_angle_sensor',
        name='emulated_angle_sensor_node',
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
    ld.add_action(use_sim_time_la)
    ld.add_action(config_la)

    # start nodes
    ld.add_action(emulated_angle_sensor)

    return ld
