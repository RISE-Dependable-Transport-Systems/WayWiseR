import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    waywiser_test_runner_dir = get_package_share_directory('waywiser_test_runner')
    waywiser_hwbringup_dir = get_package_share_directory('waywiser_hwbringup')

    # args that can be set from the command line or a default will be used
    vehicle_config_la = DeclareLaunchArgument(
        'vehicle_config',
        default_value=os.path.join(waywiser_hwbringup_dir, 'config/truck_small_scale.yaml'),
        description='Full path to params file of vehicle',
    )
    track_test_runner_config_la = DeclareLaunchArgument(
        'track_test_runner_config',
        default_value=os.path.join(
            waywiser_test_runner_dir, 'config/truck_small_scale_track_test_runner.yaml'
        ),
        description='Full path to params file for track_test_runner',
    )

    # create nodes
    track_test_runner_node = Node(
        package='waywiser_test_runner',
        executable='track_test_runner.py',
        name='track_test_runner_node',
        output='screen',
        parameters=[LaunchConfiguration('track_test_runner_config')],
        sigterm_timeout=['30'],
    )

    # create launch description
    ld = LaunchDescription()

    # declare launch args
    ld.add_action(vehicle_config_la)
    ld.add_action(track_test_runner_config_la)

    # start nodes
    ld.add_action(track_test_runner_node)

    return ld
