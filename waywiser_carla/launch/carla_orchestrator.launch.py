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
        default_value=os.path.join(waywiser_carla_dir, 'config/carla_orchestrator.yaml'),
        description='Full path to params file for carla orchestrator',
    )
    ego_vehicle_role_name_la = DeclareLaunchArgument(
        'ego_vehicle_role_name',
        default_value='truck',
        description='Name of the ego vehicle',
    )

    # create nodes
    carla_orchestrator = Node(
        package='waywiser_carla',
        executable='carla_orchestrator.py',
        name='carla_orchestrator_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config'),
            {'ego_vehicle_role_name': LaunchConfiguration('ego_vehicle_role_name')},
        ],
        sigterm_timeout=['30'],
    )

    # create launch description
    ld = LaunchDescription()

    # declare launch args
    ld.add_action(config_la)
    ld.add_action(ego_vehicle_role_name_la)

    # start nodes
    ld.add_action(carla_orchestrator)

    return ld
